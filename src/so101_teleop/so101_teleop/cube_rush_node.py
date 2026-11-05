"""Cube Rush on the simulated SO-101 follower. SPACE in the viewer starts a round."""
import mujoco
import rclpy
from std_msgs.msg import String
from so101_teleop.so101_sim_node import SO101Sim
from so101_teleop.cube_rush import CubeRush

SPACE = 32


class CubeRushNode(SO101Sim):
    start_requested = False

    def __init__(self):
        super().__init__()
        self.game = CubeRush(self.m, self.d)
        # game events go on a topic, so a bag of a round is already split into per-cube episodes
        self.pub_event = self.create_publisher(String, "/cube_rush/event", 10)

    def on_key(self, key):  # viewer thread: only set a flag, the sim thread does the work
        if key == SPACE:
            self.start_requested = True

    def check_grasp(self):  # called at 10 Hz by SO101Sim.step
        if self.start_requested:
            self.start_requested = False
            event = self.game.start()
        else:
            event = self.game.update()
        if event:
            self.get_logger().info(event)
            self.pub_event.publish(String(data=event))
        self.viewer.set_texts((mujoco.mjtFont.mjFONT_BIG, mujoco.mjtGridPos.mjGRID_TOPLEFT, *self.game.hud()))


def main():
    rclpy.init()
    rclpy.spin(CubeRushNode())


if __name__ == "__main__":
    main()
