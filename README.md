# SO-101 Teleoperation

ROS 2 teleoperation for the [SO-101](https://github.com/huggingface/lerobot) robotic arm. A physical SO-101 leader publishes joint commands that can be consumed by:

- a second physical SO-101 follower;
- a MuJoCo simulation of the SO-101; or
- an optional Cartesian retargeting and MuJoCo simulation pipeline for a Franka Panda.

The package also provides a USB camera launch configuration, ROS bag recording tools, a live servo-range calibration helper, and a scripted MuJoCo pick-and-place test.

![Recorded teleoperation demo](media/demo_01.gif)

## Features

- Read six joints from an SO-101 leader at 100 Hz.
- Convert servo ticks to named ROS `sensor_msgs/msg/JointState` messages.
- Drive a physical follower over the STS3215 serial bus with:
  - per-joint zero offsets and signs;
  - measured tick limits;
  - gripper calibration;
  - a configurable startup ramp; and
  - torque shutdown on exit.
- Mirror the leader directly into an SO-101 MuJoCo model.
- Retarget SO-101 motion to a Panda end-effector pose and solve Panda inverse kinematics.
- Detect grasp, drop, and placement events in the MuJoCo pick-and-place scene.
- Record leader, follower, simulated follower, and camera topics to an MCAP ROS bag.
- Render a recorded bag as a side-by-side camera/simulation video with joint tracking plots.

## Repository layout

```text
.
├── src/so101_teleop/
│   ├── so101_teleop/       # ROS 2 nodes and retargeting/simulation logic
│   ├── config/              # arm calibration and retargeting parameters
│   ├── launch/              # combined teleoperation launch file
│   ├── models/so101/        # SO-101 MJCF and mesh assets
│   └── test/                # unit, lint, XML, and simulation tests
├── tools/
│   ├── bag_to_video.py      # MCAP bag -> demo video and tracking plot
│   └── track_range.py       # live servo min/max calibration helper
├── bags/                    # example recording (when present locally)
└── media/                   # generated demo media (when present locally)
```

## Requirements

### Software

- Linux with Python 3
- ROS 2 with `rclpy`, `sensor_msgs`, `geometry_msgs`, `std_msgs`, and `std_srvs`
- `colcon` and `ament_python`
- `v4l2_camera` for the USB camera
- Python packages:
  - `mujoco`
  - `numpy`
  - `matplotlib`
  - `opencv-python` (required by `tools/bag_to_video.py`)
  - the Python package providing `scservo_sdk` (required for physical arms)

The package manifest lists the ROS dependencies and test dependencies. MuJoCo and `scservo_sdk` are intentionally treated as pip/vendor dependencies because they do not have rosdep keys in this project.

### Hardware

For physical teleoperation:

- one SO-101 leader arm connected to the leader computer/bus;
- optionally, one SO-101 follower arm;
- a USB camera, if camera capture is enabled; and
- reliable power for both arms.

The default serial devices are `/dev/ttyACM0` for the leader and `/dev/ttyACM1` for the follower. Device names and permissions vary by machine; use stable `/dev/serial/by-id/` paths where possible.

> **Safety:** Test with the follower arm mechanically clear of people and obstacles. Start with low speed/current and verify joint directions and limits before enabling torque. The follower node disables torque when it exits, but an emergency stop or power disconnect should remain available.

## Installation

Clone the repository into a ROS 2 workspace and install the ROS dependencies:

```bash
mkdir -p ~/so101_ws/src
cd ~/so101_ws/src
git clone https://github.com/azkadz/so101.git
cd ..
rosdep install --from-paths src --ignore-src -r -y
```

Install the Python dependencies in the environment used by ROS 2:

```bash
python3 -m pip install mujoco numpy matplotlib opencv-python scservo-sdk
```

If your vendor distributes `scservo_sdk` under a different package name, install that distribution instead and verify the import:

```bash
python3 -c "import mujoco, scservo_sdk; print('dependencies OK')"
```

Build and source the workspace:

```bash
cd ~/so101_ws
colcon build --symlink-install --packages-select so101_teleop
source install/setup.bash
```

## Quick start

The combined launch file starts the leader, optional SO-101 simulation, optional physical follower, and optional USB camera:

```bash
ros2 launch so101_teleop teleop.launch.py
```

The default starts all four components:

| Component | Executable | Enabled by default |
|---|---|---:|
| Leader reader | `leader_node` | Always |
| SO-101 MuJoCo follower | `so101_sim_node` | `sim:=true` |
| Physical follower | `follower_node` | `real:=true` |
| USB camera | `v4l2_camera_node` | `cam:=true` |

Useful launch variants:

```bash
# Physical leader -> SO-101 MuJoCo simulation; no physical follower.
ros2 launch so101_teleop teleop.launch.py real:=false

# Physical leader -> physical follower; no SO-101 MuJoCo viewer.
ros2 launch so101_teleop teleop.launch.py sim:=false

# Disable camera capture.
ros2 launch so101_teleop teleop.launch.py cam:=false

# Use a different camera device.
ros2 launch so101_teleop teleop.launch.py cam_device:=/dev/video2

# Use a different arm calibration file.
ros2 launch so101_teleop teleop.launch.py arms:=/absolute/path/to/arms.yaml
```

`sim:=false` and `real:=false` only disable the corresponding consumers. The leader node still expects a connected leader arm because it is the source of `/leader/joint_states`.

## Calibration

The checked-in [arms.yaml](src/so101_teleop/config/arms.yaml) contains calibration for one specific SO-101 pair. Do not assume those values apply to another assembly.

### 1. Find the serial ports

```bash
ls -l /dev/serial/by-id/
ls -l /dev/ttyACM*
```

Set `leader_node.port` and `follower_node.port` in a local copy of `arms.yaml` if the defaults do not match your machine.

### 2. Measure follower ranges

With the follower connected and safely supported, disable torque and run:

```bash
python3 tools/track_range.py /dev/ttyACM1
```

Move every joint through its intended range. Hold the desired zero/L pose when stopping with `Ctrl-C`. Copy the reported values into:

- `follower_node.zero_ticks`
- `follower_node.tick_limits`
- `follower_node.grip_ticks`

The follower refuses to run in torque-enabled mode when `tick_limits` or `grip_ticks` are unset.

### 3. Measure the leader zero pose

Hold the leader in the pose that should correspond to the MuJoCo/Panda home pose, then run:

```bash
ros2 run so101_teleop print_ticks
```

Copy the printed five-servo list into `leader_node.zero_ticks`. The gripper is normalized separately to approximately `0.0` (closed) through `1.0` (open).

### 4. Check directions and limits

Start simulation only:

```bash
ros2 launch so101_teleop teleop.launch.py real:=false cam:=false
```

Move one leader joint at a time and confirm that the simulated arm moves in the expected direction. For a physical follower, first use `follower_node.read_only:=true` or keep the follower disconnected until signs, zeros, and limits have been checked.

## ROS interfaces

### Joint-state topics

| Topic | Type | Publisher | Description |
|---|---|---|---|
| `/leader/joint_states` | `sensor_msgs/msg/JointState` | `leader_node` | Leader commands in the common SO-101 convention |
| `/follower/joint_states` | `sensor_msgs/msg/JointState` | `so101_sim_node` or `panda_*_sim_node` | Simulated follower state |
| `/real/joint_states` | `sensor_msgs/msg/JointState` | `follower_node` | Measured physical follower state |
| `/camera/image_raw` | `sensor_msgs/msg/Image` | `v4l2_camera_node` | USB camera frames |

The SO-101 joint names are:

```text
shoulder_pan, shoulder_lift, elbow_flex, wrist_flex, wrist_roll, gripper
```

The gripper position is normalized to `0.0..1.0`; the other five positions are radians relative to the configured zero pose.

### Cartesian retargeting topics/services

| Interface | Type | Description |
|---|---|---|
| `/target_pose` | `geometry_msgs/msg/PoseStamped` | Retargeted Panda end-effector target |
| `/follower/tcp_pose` | `geometry_msgs/msg/PoseStamped` | Panda simulation TCP pose |
| `/<node>/reanchor` | `std_srvs/srv/Trigger` | Re-anchor retargeting to the latest leader pose |

## Running the Panda retargeting pipeline

The Panda path is composed of two nodes:

1. `retarget_node` converts the five SO-101 arm joints into `/target_pose`.
2. `panda_ik_sim_node` solves the Panda IK target and displays the MuJoCo Panda scene.

Run the leader, retargeter, and Panda simulator in separate terminals:

```bash
ros2 run so101_teleop leader_node \
  --ros-args --params-file src/so101_teleop/config/arms.yaml

ros2 run so101_teleop retarget_node \
  --ros-args --params-file src/so101_teleop/config/retarget.yaml

ros2 run so101_teleop panda_ik_sim_node \
  --ros-args -p model_path:=/absolute/path/to/pick_place.xml
```

The default Panda model path in `panda_ik_sim_node` points to a local MuJoCo Menagerie checkout. Supply `model_path` explicitly when using a different checkout or scene. `retarget.yaml` controls joint signs, offsets, scale, and whether orientation tracking is enabled.

To re-anchor the Cartesian mapping to the latest leader pose:

```bash
ros2 service call /retarget_node/reanchor std_srvs/srv/Trigger "{}"
```

## Recording and replay analysis

Record the topics needed by the video renderer:

```bash
mkdir -p bags/demo_02
ros2 bag record -s mcap \
  -o bags/demo_02 \
  /leader/joint_states \
  /real/joint_states \
  /follower/joint_states \
  /camera/image_raw
```

Stop recording with `Ctrl-C`, then render the bag:

```bash
python3 tools/bag_to_video.py bags/demo_02
```

This writes:

```text
bags/demo_02.mp4
bags/demo_02_tracking.png
```

The renderer re-simulates the SO-101 from recorded leader commands, places the camera view beside the MuJoCo view, and adds leader-versus-real joint plots. It expects the topic names and six joint names listed above. The bag must contain at least one recognized topic; missing topics are shown as empty data in the output.

The included demo is approximately 25 seconds long and contains leader, real follower, simulated follower, and camera topics:

```bash
python3 tools/bag_to_video.py bags/demo_01
```

## Other executables

The package installs these console scripts:

| Command | Purpose |
|---|---|
| `leader_node` | Read the physical leader and publish joint states |
| `print_ticks` | Print leader zero-pose ticks |
| `follower_node` | Drive and read a physical follower |
| `so101_sim_node` | Direct SO-101 MuJoCo simulation |
| `retarget_node` | SO-101 joint-to-Panda Cartesian retargeting |
| `panda_ik_sim_node` | Panda MuJoCo simulation with iterative IK |
| `panda_sim_node` | Legacy/direct joint-mapped Panda simulation |
| `latency` | Latency measurement utility |
| `cube_rush_node` | Cube-rush experiment node |

The launch file currently uses `leader_node`, `so101_sim_node`, `follower_node`, and `v4l2_camera_node`. The other executables are intended for focused experiments and debugging.

## Testing

Run the package tests after building:

```bash
source install/setup.bash
colcon test --packages-select so101_teleop
colcon test-result --verbose
```

Run the headless scripted Panda pick-and-place check directly when the required Panda scene is available:

```bash
python3 src/so101_teleop/test/test_pick_place_scripted.py \
  /absolute/path/to/pick_place.xml
```

The test verifies that the IK controller lifts the cube and leaves it inside the bin. MuJoCo viewer-based nodes should be run from a graphical session with a working display.

## Configuration reference

### `config/arms.yaml`

- `leader_node.zero_ticks`: five leader servo ticks defining the zero pose.
- `follower_node.port`: follower serial device.
- `follower_node.zero_ticks`: five follower zero ticks.
- `follower_node.tick_limits`: flattened `[min, max]` pairs for all six servos.
- `follower_node.grip_ticks`: follower gripper ticks at normalized positions `0` and `1`.

Additional follower parameters can be supplied in a custom parameter file:

- `sign`: per-joint direction multiplier.
- `ramp_s`: startup command ramp duration in seconds.
- `gripper_torque_limit`: gripper torque limit from `0` to `1000`.
- `read_only`: disable torque and print raw ticks.

### `config/retarget.yaml`

- `sign`: direction multipliers for the five arm joints.
- `offset`: Cartesian retargeting joint offsets.
- `scale`: retargeting scale.
- `track_orientation`: enable orientation tracking.

## Troubleshooting

### Cannot open a serial port

Check that the arm is powered, the selected device exists, and the user has serial permissions:

```bash
ls -l /dev/ttyACM*
groups
```

Stop other processes using the port. If needed, add the user to the system's serial device group and log in again.

### The follower moves in the wrong direction

Stop torque, correct the relevant `sign` value in the local calibration file, and verify the zero pose again. Do not compensate for an incorrect direction by widening limits.

### The follower does not start

Ensure `tick_limits` and `grip_ticks` are populated. The node intentionally raises an error when required safety calibration is missing.

### The camera does not start

List available V4L2 devices and pass the correct path:

```bash
v4l2-ctl --list-devices
ros2 launch so101_teleop teleop.launch.py cam_device:=/dev/video2
```

Or disable it with `cam:=false`.

### MuJoCo cannot open a viewer

Run from a graphical desktop session and check `DISPLAY`/Wayland configuration. The direct simulations use passive MuJoCo viewers and are not headless by default. The scripted test and bag renderer do not require an interactive viewer.

## License and attribution

The ROS 2 package is released under the MIT License. The SO-101 model assets include their own license file at [models/so101/LICENSE](src/so101_teleop/models/so101/LICENSE). See the individual upstream projects for the licenses of ROS 2, MuJoCo, the SO-101 design, and the servo SDK.
