import os
import mujoco
import mujoco.viewer
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from geometry_msgs.msg import PoseStamped
from so101_teleop.panda_ik import PandaIK


class PandaIKSim(Node):
    def __init__(self):
        super().__init__("panda_ik_sim_node")
        p = lambda n, v: self.declare_parameter(n, v).value
        path = p("model_path", os.path.expanduser(
            "~/wsp/playground/mujoco_menagerie/franka_emika_panda/pick_place.xml"))
        self.gripper_invert = p("gripper_invert", False)
        self.m = mujoco.MjModel.from_xml_path(path)
        self.d = mujoco.MjData(self.m)
        mujoco.mj_resetDataKeyframe(self.m, self.d, self.m.key("pick_home").id)
        mujoco.mj_forward(self.m, self.d)
        self.ik = PandaIK(self.m)
        self.q_cmd = self.d.qpos[self.ik.qadr].copy()
        self.target = None
        self.names = [f"joint{i}" for i in range(1, 8)] + ["finger_joint1"]
        self.jadr = [self.m.joint(n).qposadr[0] for n in self.names]
        self.cube = self.m.body("cube").id
        self.fingers = {self.m.body("left_finger").id, self.m.body("right_finger").id}
        self.zone = self.m.site("bin_zone").id
        self.state = "idle"

        self.viewer = mujoco.viewer.launch_passive(self.m, self.d)
        self.pub_js = self.create_publisher(JointState, "/follower/joint_states", 10)
        self.pub_tcp = self.create_publisher(PoseStamped, "/follower/tcp_pose", 10)
        self.create_subscription(PoseStamped, "/target_pose", self.on_target, 10)
        self.create_subscription(JointState, "/leader/joint_states", self.on_leader, 10)
        self.k = 0
        self.create_timer(self.m.opt.timestep, self.step)

    def on_target(self, msg):
        p, o = msg.pose.position, msg.pose.orientation
        self.target = (np.array([p.x, p.y, p.z]), np.array([o.w, o.x, o.y, o.z]))

    def on_leader(self, msg):
        g = msg.position[msg.name.index("gripper")]
        g = 1.0 - g if self.gripper_invert else g
        self.d.ctrl[7] = 255.0 * float(np.clip(g, 0.0, 1.0))  # 0 = closed, 255 = open (menagerie convention)

    def step(self):
        if self.target is not None:
            self.q_cmd = self.ik.solve(self.d, self.q_cmd, *self.target, iters=1)
            self.d.ctrl[:7] = self.q_cmd
            self.d.mocap_pos[0], self.d.mocap_quat[0] = self.target
        mujoco.mj_step(self.m, self.d)
        self.k += 1
        if self.k % 5 == 0:   # 100 Hz, same rate as the leader
            self.publish()
        if self.k % 8 == 0:   # ~60 Hz is enough for the viewer
            self.viewer.sync()
        if self.k % 50 == 0:  # 10 Hz
            self.check_grasp()

    def publish(self):
        now = self.get_clock().now().to_msg()
        js = JointState(name=self.names, position=self.d.qpos[self.jadr].tolist())
        js.header.stamp = now
        self.pub_js.publish(js)
        p, R = self.ik.tcp(self.d)
        q = np.zeros(4)
        mujoco.mju_mat2Quat(q, R.flatten())
        tcp = PoseStamped()
        tcp.header.stamp, tcp.header.frame_id = now, "panda_link0"
        tcp.pose.position.x, tcp.pose.position.y, tcp.pose.position.z = map(float, p)
        tcp.pose.orientation.w, tcp.pose.orientation.x, tcp.pose.orientation.y, tcp.pose.orientation.z = map(float, q)
        self.pub_tcp.publish(tcp)

    def check_grasp(self):
        touching = set()
        for c in self.d.contact[: self.d.ncon]:
            b1, b2 = self.m.geom_bodyid[c.geom1], self.m.geom_bodyid[c.geom2]
            if self.cube in (b1, b2):
                touching.add(b2 if b1 == self.cube else b1)
        cube = self.d.xpos[self.cube]
        rel = np.abs(cube - self.d.site_xpos[self.zone])
        grasped = self.fingers <= touching and cube[2] > 0.03
        in_zone = np.all(rel < self.m.site_size[self.zone])
        new = "grasped" if grasped else ("placed" if in_zone else self.state if self.state == "idle" else "dropped")
        if new != self.state:
            self.get_logger().info(f"{new}  cube=({cube[0]:.3f}, {cube[1]:.3f}, {cube[2]:.3f})")
            self.state = new


def main():
    rclpy.init()
    rclpy.spin(PandaIKSim())


if __name__ == "__main__":
    main()
