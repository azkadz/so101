import os
from ament_index_python.packages import get_package_share_directory
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from geometry_msgs.msg import PoseStamped
from std_srvs.srv import Trigger
from so101_teleop.retarget import SO101Retarget

ARM = ["shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex", "wrist_roll"]


class RetargetNode(Node):
    def __init__(self):
        super().__init__("retarget_node")
        p = lambda n, v: self.declare_parameter(n, v).value
        xml = p("so101_xml", os.path.expanduser(
            os.path.join(get_package_share_directory("so101_teleop"), "models", "so101", "so101_new_calib.xml")))
        self.rt = SO101Retarget(xml, p("sign", [1.0] * 5), p("offset", [0.0] * 5),
                                p("scale", 2.0), p("track_orientation", True))
        self.viewer = None
        self.last_q = None
        if p("show_so101", False):  # mirror of the leader in its own MJCF, for checking sign/offset
            import mujoco.viewer
            self.viewer = mujoco.viewer.launch_passive(self.rt.m, self.rt.d)
        self.pub = self.create_publisher(PoseStamped, "/target_pose", 10)
        self.create_subscription(JointState, "/leader/joint_states", self.cb, 10)
        self.create_service(Trigger, "~/reanchor", self.reanchor)

    def reanchor(self, _req, res):
        self.rt.reanchor(self.last_q)
        res.success = True
        return res

    def cb(self, msg):
        idx = {n: i for i, n in enumerate(msg.name)}
        self.last_q = [msg.position[idx[n]] for n in ARM]
        pos, quat = self.rt.update(self.last_q)
        out = PoseStamped()
        out.header.stamp = msg.header.stamp  # keep the leader read time, so latency can be measured end to end
        out.header.frame_id = "panda_link0"
        out.pose.position.x, out.pose.position.y, out.pose.position.z = map(float, pos)
        out.pose.orientation.w, out.pose.orientation.x, out.pose.orientation.y, out.pose.orientation.z = map(float, quat)
        self.pub.publish(out)
        if self.viewer is not None:
            self.viewer.sync()


def main():
    rclpy.init()
    rclpy.spin(RetargetNode())


if __name__ == "__main__":
    main()