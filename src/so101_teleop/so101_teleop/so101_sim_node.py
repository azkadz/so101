"""Simulated SO-101 follower: leader joints -> identical MJCF joints, 1:1 (same kinematics, no IK)."""
import os
from ament_index_python.packages import get_package_share_directory
import mujoco
import mujoco.viewer
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from so101_teleop.so101_scene import load_model

ARM = ["shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex", "wrist_roll"]
G_CLOSED, G_OPEN = -0.17, 1.745  # MJCF gripper angle; -0.17 over-closes so the servo squeezes the cube


class SO101Sim(Node):
    def __init__(self):
        super().__init__("so101_sim_node")
        p = lambda n, v: self.declare_parameter(n, v).value
        path = p("model_path", os.path.expanduser(os.path.join(get_package_share_directory("so101_teleop"), "models", "so101", "pick_place.xml")))
        self.sign = np.array(p("sign", [1.0] * 5))
        self.offset = np.array(p("offset", [0.0] * 5))
        self.m = load_model(path)
        self.d = mujoco.MjData(self.m)
        self.qadr = [self.m.joint(n).qposadr[0] for n in ARM + ["gripper"]]
        self.act = [self.m.actuator(n).id for n in ARM + ["gripper"]]
        self.d.qpos[self.qadr[:5]] = self.offset  # start where the leader's zero pose maps
        self.d.ctrl[self.act[:5]] = self.offset
        mujoco.mj_forward(self.m, self.d)
        self.cube = self.m.body("cube").id
        self.pads = {self.m.geom("fixed_pad").id, self.m.geom("moving_pad").id}
        self.zone = self.m.site("bin_zone").id
        self.state = "idle"

        self.viewer = mujoco.viewer.launch_passive(self.m, self.d, key_callback=getattr(self, "on_key", None))
        self.pub = self.create_publisher(JointState, "/follower/joint_states", 10)
        self.create_subscription(JointState, "/leader/joint_states", self.on_leader, 10)
        self.k = 0
        self.create_timer(self.m.opt.timestep, self.step)

    def on_leader(self, msg):
        idx = {n: i for i, n in enumerate(msg.name)}
        q = np.array([msg.position[idx[n]] for n in ARM])
        g = float(np.clip(msg.position[idx["gripper"]], 0.0, 1.0))
        ctrl = np.r_[self.sign * q + self.offset, G_CLOSED + g * (G_OPEN - G_CLOSED)]
        lo, hi = self.m.actuator_ctrlrange[self.act].T
        self.d.ctrl[self.act] = np.clip(ctrl, lo, hi)

    def step(self):
        mujoco.mj_step(self.m, self.d)
        self.k += 1
        if self.k % 5 == 0:   # 100 Hz, same rate as the leader
            self.publish()
        if self.k % 8 == 0:   # ~60 Hz is enough for the viewer
            self.viewer.sync()
        if self.k % 50 == 0:  # 10 Hz
            self.check_grasp()

    def publish(self):
        """Published in the leader's convention (same names, same zero), so PlotJuggler overlays 1:1."""
        q = self.d.qpos[self.qadr]
        arm = (q[:5] - self.offset) / self.sign
        g = (q[5] - G_CLOSED) / (G_OPEN - G_CLOSED)
        js = JointState(name=ARM + ["gripper"], position=np.r_[arm, g].tolist())
        js.header.stamp = self.get_clock().now().to_msg()
        self.pub.publish(js)

    def check_grasp(self):
        touching = set()
        for c in self.d.contact[: self.d.ncon]:
            g1, g2 = c.geom1, c.geom2
            if self.m.geom_bodyid[g1] == self.cube:
                touching.add(g2)
            elif self.m.geom_bodyid[g2] == self.cube:
                touching.add(g1)
        cube = self.d.xpos[self.cube]
        grasped = self.pads <= touching and cube[2] > 0.02
        in_zone = np.all(np.abs(cube - self.d.site_xpos[self.zone]) < self.m.site_size[self.zone])
        new = "grasped" if grasped else "placed" if in_zone else "idle" if self.state == "idle" else "dropped"
        if new != self.state:
            self.get_logger().info(f"{new}  cube=({cube[0]:.3f}, {cube[1]:.3f}, {cube[2]:.3f})")
            self.state = new


def main():
    rclpy.init()
    rclpy.spin(SO101Sim())


if __name__ == "__main__":
    main()
