# Chess-Playing Arm

A simulated robot arm that plays chess against a human. A Franka Panda (7-DOF) sits next to a chessboard in **Gazebo Harmonic**, an overhead depth camera watches the board, **MoveIt 2** plans the arm motions, and a Python node decides the arm's moves with `python-chess`, then commands a C++ pick-and-place action server to physically move the pieces.

**Target stack:** Ubuntu 24.04 · ROS 2 Jazzy · Gazebo Harmonic (gz-sim 8) · MoveIt 2

> The Gazebo/ROS 2 bring-up was ported from Gazebo Classic and has not been verified on every setup. See [Known issues](#known-issues).

---

## How it works

```
 Overhead depth camera (Gazebo, /realsense/depth/*)
        │
        ├── testmodel.py ──────► detects the board, saves config/chessboard_poses.json
        │                        (centre of each of the 64 squares)
        │
        └── chess_managment.py ─► sees the human's move (square occupancy),
                │                 updates python-chess, picks the arm's reply,
                │                 sends a PickAndPlace action goal
                ▼
        pick_and_place_node (C++, MoveIt 2) ──► arm_controller ──► Panda in Gazebo
                │
                └── gz_link_attacher.py ──► attaches/detaches the piece to the gripper
```

Turn flow:

1. The human moves a piece on the board in Gazebo.
2. `chess_managment.py` compares the camera's point-cloud occupancy with the expected board and works out the human move.
3. It chooses the arm's reply and sends a goal (`source_sqr`, `target_sqr`, `piece_id`) to the `pick_and_place` action.
4. For captures, the captured piece is first moved to the graveyard beside the board.
5. The terminal prints the board after every move.

---

## Repository layout

| Folder | What it is |
|---|---|
| `moveit_config_pkg/` | MoveIt config for the Panda, the Gazebo world, chess piece models, and all the application nodes in `src/` |
| `franka_description/` | Panda URDF/xacro, meshes, and the `PickAndPlace` action definition |
| `linkattacher_msgs/` | `AttachLink` / `DetachLink` service definitions (Gazebo Harmonic link-attacher port) |
| `robotic_arms_control/` | **Gazebo launch files**, controller configs, URDFs, and generic arm-control examples |
| `ROS2-Ultimate-guide-for-Custom-Robotic-Arms-and-Panda-7-DOF/` | Separate learning notes, notebook and GIFs for custom arms |

Key files:

| File | Purpose |
|---|---|
| `robotic_arms_control/launch/gazebo_bringup.launch.py` | **Main launch file.** Starts Gazebo, the bridges, controllers, MoveIt, RViz and the support nodes |
| `moveit_config_pkg/src/chess_managment.py` | **Main chess node.** Game logic, human-move detection, action client |
| `moveit_config_pkg/src/pick_place_gazebo.cpp` | C++ `pick_and_place` action server (MoveIt) |
| `moveit_config_pkg/src/testmodel.py` | Board detection and calibration; writes `chessboard_poses.json` |
| `moveit_config_pkg/src/pice_spawn.py` | Spawns the 32 pieces on their start squares |
| `moveit_config_pkg/src/gz_link_attacher.py` | Grasp attach/detach via Gazebo `DetachableJoint` |
| `moveit_config_pkg/config/chessboard_poses.json` | Square centres in the camera frame |
| `moveit_config_pkg/config/initial_board_layout.json` | Square → piece ID map (`a2` → `w_pawn_1`, …) |
| `franka_description/action/PickAndPlace.action` | Goal: `source_sqr`, `target_sqr`, `piece_id` |

---

## Installation

### 1. System dependencies

```bash
sudo apt update
sudo apt install ros-jazzy-ros-gz ros-jazzy-gz-ros2-control ros-jazzy-ros-gz-interfaces \
                 ros-jazzy-ros2-controllers ros-jazzy-moveit ros-jazzy-cv-bridge \
                 ros-jazzy-tf2-ros ros-jazzy-xacro
```

### 2. Python dependencies

```bash
pip3 install python-chess "numpy<2" opencv-python --break-system-packages
```

`numpy<2` matters: `cv_bridge` is built against NumPy 1.x and crashes (`exit -11`) with NumPy 2.x.

### 3. Build

```bash
mkdir -p ~/arm_ws/src && cd ~/arm_ws/src
git clone https://github.com/Adithya-cc/chess-playing-arm.git .
cd ~/arm_ws
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install
source install/setup.bash
```

Make sure the Python nodes are executable (once):

```bash
chmod +x src/moveit_config_pkg/src/*.py
```

> Clone into `~/arm_ws/src` — a few scripts fall back to `~/arm_ws/src/moveit_config_pkg` when the package share directory isn't found. See [Known issues](#known-issues) for hardcoded paths you may need to edit.

---

## Running

Use two terminals, both with the workspace sourced.

### Terminal 1 — Gazebo launch

```bash
ros2 launch robotic_arms_control gazebo_bringup.launch.py
```

This starts:

- Gazebo with `world/world.sdf` (board, overhead camera, ground)
- `ros_gz_bridge` for the camera, `/clock`, and the per-piece attach/detach topics, plus the spawn service
- `robot_state_publisher`, the Panda (entity name `cobot`), and the `joint_state_broadcaster`, `arm_controller` and `gripper_controller`
- Static TFs for the camera, MoveIt `move_group`, and RViz
- `gz_link_attacher.py`, `pick_and_place_node` (the action server), `testmodel.py` (board calibration) and `pice_spawn.py` (spawns the pieces)

Launch arguments:

| Argument | Default | Meaning |
|---|---|---|
| `with_rviz` | `true` | Open RViz with the MoveIt config |
| `gui` | `true` | `false` runs Gazebo headless (server only) |
| `x`, `y`, `z` | `0.0` | Spawn pose of the arm |

Example: `ros2 launch robotic_arms_control gazebo_bringup.launch.py with_rviz:=false gui:=false`

Wait until the board is detected (`testmodel.py` logs the calibration and `chessboard_poses.json` is written) and the pieces have spawned.

### Terminal 2 — Chess management

```bash
ros2 run moveit_config_pkg chess_managment.py
```

`chess_managment.py` is commented out at the bottom of `gazebo_bringup.launch.py` so you can restart it on its own. To start it with everything else, uncomment this block:

```python
ld.add_action(Node(package='moveit_config_pkg', executable='chess_managment.py',
                   output='screen'))
```

Then make a move for the human side in Gazebo. The node detects it, replies, and calls the `pick_and_place` action.

### Testing the arm without the chess logic

```bash
ros2 action send_goal /pick_and_place franka_description/action/PickAndPlace \
  "{source_sqr: 'e2', target_sqr: 'e4', piece_id: 'w_pawn_5'}" --feedback
```

---

## Useful topics, actions and files

| Name | Type | Notes |
|---|---|---|
| `/pick_and_place` | `franka_description/action/PickAndPlace` | Served by `pick_and_place_node` |
| `/realsense/depth/image_rect_raw` | `sensor_msgs/Image` | RGB image (name kept from the Classic setup) |
| `/realsense/depth/image_raw` | `sensor_msgs/Image` | Depth image |
| `/realsense/depth/points` | `sensor_msgs/PointCloud2` | Used for piece-occupancy detection |
| `/realsense/depth/camera_info` | `sensor_msgs/CameraInfo` | Camera intrinsics |
| `/arm_controller/follow_joint_trajectory` | `FollowJointTrajectory` | Arm trajectories |

Camera frame: the camera sits at world `(0, 0, 2.5)` looking straight down. `chessboard_poses.json` stores square centres in the camera optical frame as `[x_right, y_down, depth]`, so they must be converted before use as world coordinates:

```
world_x = -y_opt
world_y = -x_opt
world_z = 2.5 - depth
```

---

## Known issues

- **Hardcoded paths.** `pick_place_gazebo.cpp` has `json_path_` set to `/home/adithya-con006/arm_ws/src/moveit_config_pkg/config/chessboard_poses.json`, and `chess_managment.py` first tries `/home/adithya-si3510/arm_ws/src/...`. Change these to your own workspace path (or switch to `get_package_share_directory`) and rebuild.
- **Camera-frame coordinates.** The pose JSON is in the camera frame (see above). The piece spawner and the C++ action server must both apply the camera → world conversion, or pieces spawn and the arm moves to the wrong place. A symptom is the action finishing `ABORTED: Failed to resolve square coordinates from JSON`.
- **Optical-frame TF.** In `gazebo_bringup.launch.py` the `camera_to_optical_tf` static transform currently uses an identity rotation. A standard optical frame uses the quaternion `(-0.5, 0.5, -0.5, 0.5)` relative to `realsense_link`.
- **`chessboard_poses.json` is overwritten at install.** `install(DIRECTORY config ...)` copies the source copy over the one written at runtime when you rebuild. Re-run board calibration after rebuilding.
- **Pieces fall or drift.** Check the `PIECE_Z_OFFSETS` in `pice_spawn.py` and the board's height in `world.sdf`.
- **Gazebo/ROS versions.** The Gazebo Harmonic port targets Jazzy and, per the `linkattacher_msgs` notes, has not been run against a live Gazebo on every setup. Expect to debug launch ordering (controllers spawning after the robot, bridges starting before the world).
- **Large binary files in the repo** (`*.whl`, `panda_ppo_model.zip`) are not needed for the chess pipeline and make cloning slow.

---

## Troubleshooting

| Symptom | Fix |
|---|---|
| `testmodel.py` / `chess_managment.py` crashes with exit code -11 | `pip3 install "numpy<2"` |
| `chess_managment.py` never sends a goal | Check that the board is calibrated, `pick_and_place_node` is up (`ros2 action list`), and the camera topics publish (`ros2 topic hz /realsense/depth/points`) |
| Pieces spawn at the wrong place / in a wall | Apply the camera → world conversion in `pice_spawn.py` |
| Action `ABORTED: Failed to resolve square coordinates from JSON` | Wrong `json_path_` in the C++ node, or JSON missing/empty. Re-run calibration |
| `/world/default/create` not available | `pice_spawn.py` automatically falls back to `ros2 run ros_gz_sim create` |

---

## Credits

- [Franka Emika](https://github.com/frankaemika/franka_ros2) Panda description
- [MoveIt 2](https://moveit.ai/) and [Gazebo](https://gazebosim.org/)
- [python-chess](https://python-chess.readthedocs.io/)
