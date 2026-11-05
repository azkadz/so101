import math

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from scservo_sdk import PortHandler, PacketHandler

PRESENT_POSITION = 56  # STS3215 register, 2 bytes, 0..4095 ticks per turn
TICKS_TO_RAD = 2 * math.pi / 4096

ARM_IDS = [1, 2, 3, 4, 5]
ARM_NAMES = ['shoulder_pan', 'shoulder_lift', 'elbow_flex', 'wrist_flex', 'wrist_roll']
GRIPPER_ID, GRIPPER_MIN, GRIPPER_SPAN = 6, 1544, 1144


def open_bus(port_name, baud):
    port = PortHandler(port_name)
    if not port.openPort():
        raise RuntimeError(f'could not open {port_name}')
    port.setBaudRate(baud)
    return port, PacketHandler(0)


class LeaderNode(Node):
    def __init__(self):
        super().__init__('so101_leader')
        self.declare_parameter('port', '/dev/ttyACM0')
        self.declare_parameter('baud', 1000000)
        self.declare_parameter('rate_hz', 100.0)
        # ticks at the pose that should map to the Panda's "home" keyframe (ids 1..5)
        self.declare_parameter('zero_ticks', [1971, 2095, 1973, 3135, 2028])

        self.port, self.ph = open_bus(self.get_parameter('port').value, self.get_parameter('baud').value)
        self.pub = self.create_publisher(JointState, '/leader/joint_states', 10)
        self.create_timer(1.0 / self.get_parameter('rate_hz').value, self.tick)

    def read(self, sid):
        ticks, result, _error = self.ph.read2ByteTxRx(self.port, sid, PRESENT_POSITION)
        if result != 0:
            self.get_logger().warning(f'servo {sid}: comm={result}', throttle_duration_sec=1.0)
            return None
        return ticks

    def tick(self):
        zero = self.get_parameter('zero_ticks').value
        pos = []
        for sid, z in zip(ARM_IDS, zero):
            t = self.read(sid)
            if t is None:
                return
            pos.append((t - z) * TICKS_TO_RAD)
        g = self.read(GRIPPER_ID)
        if g is None:
            return
        pos.append((g - GRIPPER_MIN) / GRIPPER_SPAN)

        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = ARM_NAMES + ['gripper']
        msg.position = pos
        self.pub.publish(msg)

    def destroy_node(self):
        self.port.closePort()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = LeaderNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


def print_ticks(args=None):
    """Hold the leader in the Panda-home pose, run this, paste the list into zero_ticks."""
    port, ph = open_bus('/dev/ttyACM0', 1000000)
    ticks = [ph.read2ByteTxRx(port, sid, PRESENT_POSITION)[0] for sid in ARM_IDS]
    print(f'zero_ticks:="{ticks}"')
    port.closePort()


if __name__ == '__main__':
    main()
