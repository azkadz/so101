"""Drive the real SO-101 follower from /leader/joint_states.

Runs alongside so101_sim_node: both subscribe to the same leader topic.
Publishes the real arm's state on /real/joint_states (the sim owns /follower/joint_states).

read_only:=true  -> torque off, logs raw ticks at 5 Hz (use it to measure zero_ticks / limits).
"""
import math

JOINTS = ['shoulder_pan', 'shoulder_lift', 'elbow_flex', 'wrist_flex', 'wrist_roll', 'gripper']
ADDR_TORQUE_ENABLE, ADDR_GOAL_POS, ADDR_TORQUE_LIMIT, ADDR_PRESENT_POS = 40, 42, 48, 56
TICKS_PER_RAD = 4096 / (2 * math.pi)


def leader_to_ticks(q, zero_ticks, sign, grip_ticks):
    """q: 5 joint angles (rad, leader convention) + gripper 0..1 -> 6 follower ticks."""
    ticks = [round(z + s * a * TICKS_PER_RAD) for a, z, s in zip(q[:5], zero_ticks, sign)]
    g = min(max(q[5], 0.0), 1.0)
    ticks.append(round(grip_ticks[0] + g * (grip_ticks[1] - grip_ticks[0])))
    return ticks


def clamp(ticks, limits):
    """limits: flat [min1, max1, ..., min6, max6]."""
    return [min(max(t, limits[2 * i]), limits[2 * i + 1]) for i, t in enumerate(ticks)]


def ramp(start, goal, frac):
    frac = min(max(frac, 0.0), 1.0)
    return [round(s + frac * (g - s)) for s, g in zip(start, goal)]


def main():
    import rclpy
    from rclpy.node import Node
    from sensor_msgs.msg import JointState
    import scservo_sdk as sdk

    class FollowerNode(Node):
        def __init__(self):
            super().__init__('follower_node')
            p = self.declare_parameter
            self.port_name = p('port', '/dev/ttyACM1').value
            self.read_only = p('read_only', False).value
            self.zero = list(p('zero_ticks', [2048] * 5).value)
            self.sign = list(p('sign', [1.0] * 5).value)
            self.grip = list(p('grip_ticks', [0, 0]).value)          # [tick at leader 0, tick at leader 1]
            self.limits = list(p('tick_limits', [0] * 12).value)      # measured, required
            self.ramp_s = p('ramp_s', 2.0).value
            self.grip_torque = p('gripper_torque_limit', 500).value   # 0..1000

            self.port = sdk.PortHandler(self.port_name)
            self.pkt = sdk.PacketHandler(0)
            if not (self.port.openPort() and self.port.setBaudRate(1_000_000)):
                raise RuntimeError(f'cannot open {self.port_name}')
            self.reader = sdk.GroupSyncRead(self.port, self.pkt, ADDR_PRESENT_POS, 2)
            self.writer = sdk.GroupSyncWrite(self.port, self.pkt, ADDR_GOAL_POS, 2)
            for i in range(1, 7):
                self.reader.addParam(i)

            self.pub = self.create_publisher(JointState, '/real/joint_states', 10)
            if self.read_only:
                self.set_torque(0)
                self.create_timer(0.2, self.log_ticks)
                self.get_logger().info('read_only: torque off, logging ticks')
                return
            if not any(self.limits) or self.grip == [0, 0]:
                raise RuntimeError('set tick_limits and grip_ticks (measure them with read_only:=true)')

            self.goal = None
            self.start = self.read_ticks()
            self.t0 = None
            self.pkt.write2ByteTxRx(self.port, 6, ADDR_TORQUE_LIMIT, self.grip_torque)
            self.set_torque(1)
            self.create_subscription(JointState, '/leader/joint_states', self.on_leader, 10)
            self.create_timer(0.01, self.step)  # 100 Hz
            self.get_logger().info(f'torque on, holding {self.start}; waiting for leader')

        def set_torque(self, on):
            for i in range(1, 7):
                self.pkt.write1ByteTxRx(self.port, i, ADDR_TORQUE_ENABLE, on)

        def read_ticks(self):
            if self.reader.txRxPacket() != sdk.COMM_SUCCESS:
                return None
            return [self.reader.getData(i, ADDR_PRESENT_POS, 2) for i in range(1, 7)]

        def log_ticks(self):
            self.get_logger().info(str(self.read_ticks()))

        def on_leader(self, msg):
            pos = dict(zip(msg.name, msg.position))
            q = [pos[j] for j in JOINTS]  # match by name, not index
            self.goal = clamp(leader_to_ticks(q, self.zero, self.sign, self.grip), self.limits)
            if self.t0 is None:
                self.t0 = self.get_clock().now()

        def step(self):
            if self.goal is None or self.start is None:
                return
            frac = (self.get_clock().now() - self.t0).nanoseconds / 1e9 / self.ramp_s
            cmd = ramp(self.start, self.goal, frac)
            self.writer.clearParam()
            for i, t in enumerate(cmd, start=1):
                self.writer.addParam(i, [sdk.SCS_LOBYTE(t), sdk.SCS_HIBYTE(t)])
            self.writer.txPacket()
            ticks = self.read_ticks()
            if ticks:
                m = JointState()
                m.header.stamp = self.get_clock().now().to_msg()
                m.name = JOINTS
                m.position = [s * (t - z) / TICKS_PER_RAD for t, z, s in zip(ticks[:5], self.zero, self.sign)]
                m.position.append((ticks[5] - self.grip[0]) / ((self.grip[1] - self.grip[0]) or 1))
                self.pub.publish(m)

    rclpy.init()
    node = None
    try:
        node = FollowerNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.set_torque(0)  # go limp, never hold a stale pose
            node.port.closePort()
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
