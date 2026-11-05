import time

import mujoco
import mujoco.viewer
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState

# leader joint -> (panda actuator, home offset rad).  joint3/joint5 stay at home.
MAP = {
    'shoulder_pan':  ('actuator1', 0.0),
    'shoulder_lift': ('actuator2', 0.0),
    'elbow_flex':    ('actuator4', -1.57079),
    'wrist_flex':    ('actuator6', 1.57079),
    'wrist_roll':    ('actuator7', -0.7853),
}
PANDA_JOINTS = [f'joint{i}' for i in range(1, 8)]


class PandaSimNode(Node):
    def __init__(self):
        super().__init__('panda_sim')
        self.declare_parameter('model_path', 'franka_emika_panda/scene.xml')
        self.declare_parameter('sign', [-1.0, 1.0, -1.0, -1.0, -1.0])  # pan, lift, elbow, wflex, wroll

        self.m = mujoco.MjModel.from_xml_path(self.get_parameter('model_path').value)
        self.d = mujoco.MjData(self.m)
        mujoco.mj_resetDataKeyframe(self.m, self.d, self.m.key('home').id)
        self.act = {name: self.m.actuator(name).id for name in [f'actuator{i}' for i in range(1, 9)]}
        self.lo, self.hi = self.m.actuator_ctrlrange.T
        self.qadr = [self.m.joint(j).qposadr[0] for j in PANDA_JOINTS]

        self.create_subscription(JointState, '/leader/joint_states', self.on_leader, 10)
        self.pub = self.create_publisher(JointState, '/follower/joint_states', 10)

        self.viewer = mujoco.viewer.launch_passive(self.m, self.d, show_left_ui=False, show_right_ui=False)
        self.create_timer(self.m.opt.timestep, self.step)

    def on_leader(self, msg):
        sign = self.get_parameter('sign').value
        pos = dict(zip(msg.name, msg.position))
        for k, (name, (actuator, home)) in enumerate(MAP.items()):
            if name in pos:
                i = self.act[actuator]
                self.d.ctrl[i] = np.clip(home + sign[k] * pos[name], self.lo[i], self.hi[i])
        if 'gripper' in pos:
            i = self.act['actuator8']
            self.d.ctrl[i] = np.clip(pos['gripper'] * 255.0, self.lo[i], self.hi[i])

    def step(self):
        if not self.viewer.is_running():
            rclpy.shutdown()
            return
        mujoco.mj_step(self.m, self.d)
        self.viewer.sync()
        out = JointState()
        out.header.stamp = self.get_clock().now().to_msg()
        out.name = PANDA_JOINTS
        out.position = [float(self.d.qpos[a]) for a in self.qadr]
        self.pub.publish(out)


def main(args=None):
    rclpy.init(args=args)
    node = PandaSimNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        node.viewer.close()
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()