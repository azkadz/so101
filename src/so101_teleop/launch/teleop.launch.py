"""Leader -> MuJoCo sim + real follower + USB camera.

ros2 launch so101_teleop teleop.launch.py              # everything
ros2 launch so101_teleop teleop.launch.py real:=false  # sim only
ros2 launch so101_teleop teleop.launch.py sim:=false   # real follower only
ros2 launch so101_teleop teleop.launch.py cam:=false   # no USB camera
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

CAM = '/dev/v4l/by-id/usb-Sonix_Technology_Co.__Ltd._USB2.0_CAM1_USB2.0_CAM1-video-index0'


def generate_launch_description():
    share = get_package_share_directory('so101_teleop')
    arms = LaunchConfiguration('arms')
    return LaunchDescription([
        DeclareLaunchArgument('sim', default_value='true'),
        DeclareLaunchArgument('real', default_value='true'),
        DeclareLaunchArgument('arms', default_value=os.path.join(share, 'config', 'arms.yaml')),
        DeclareLaunchArgument('cam', default_value='true'),
        DeclareLaunchArgument('cam_device', default_value=CAM),
        # Publishes /camera/image_raw at ~27 fps. by-id path survives replugging.
        Node(package='v4l2_camera', executable='v4l2_camera_node', name='camera', namespace='camera',
             parameters=[{'video_device': LaunchConfiguration('cam_device'),
                          'image_size': [640, 480], 'output_encoding': 'yuv422_yuy2'}],
             condition=IfCondition(LaunchConfiguration('cam'))),
        Node(package='so101_teleop', executable='leader_node', name='leader_node',
             parameters=[arms], output='screen'),
        Node(package='so101_teleop', executable='so101_sim_node', name='so101_sim_node',
             additional_env={'XDG_SESSION_TYPE': 'x11'}, output='screen',
             condition=IfCondition(LaunchConfiguration('sim'))),
        Node(package='so101_teleop', executable='follower_node', name='follower_node',
             parameters=[arms], output='screen',
             condition=IfCondition(LaunchConfiguration('real'))),
    ])