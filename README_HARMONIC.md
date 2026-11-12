# Gazebo Classic -> Gazebo Harmonic port (ROS 2 Jazzy)

Target: Ubuntu 24.04, ROS 2 Jazzy, Gazebo Harmonic.  **Not run against a live Gazebo** - see "Things to check".

## Install
    sudo apt install ros-jazzy-ros-gz ros-jazzy-gz-ros2-control ros-jazzy-ros-gz-interfaces \
                     ros-jazzy-ros2-controllers ros-jazzy-moveit
    # remove the Classic IFRA_LinkAttacher / ros2_linkattacher from your workspace (it needs gazebo_ros)

## Build
    cd ~/arm_ws && colcon build --symlink-install
    source install/setup.bash
    ros2 launch robotic_arms_control gazebo_bringup.launch.py        # gui:=false for headless server

## What changed
| Classic | Harmonic |
|---|---|
| gazebo_ros `gazebo.launch.py`, `GAZEBO_MODEL_PATH` | `ros_gz_sim gz_sim.launch.py`, `GZ_SIM_RESOURCE_PATH` |
| `spawn_entity.py`, `/spawn_entity` | `ros_gz_sim create` (robot), bridged `/world/default/create` (pieces) |
| `gazebo_ros2_control/GazeboSystem` | `gz_ros2_control/GazeboSimSystem` + `libgz_ros2_control-system.so`; controller yaml path via `$(find moveit_config_pkg)` (no more `/home/...`) |
| `libgazebo_ros_camera.so` depth plugin | `rgbd_camera` sensor + `ros_gz_bridge` (same ROS topic names, `config/gz_bridge.yaml`) |
| `libgazebo_link_attacher.so` + `linkattacher_msgs` + `ros2_linkattacher` | `DetachableJoint` per piece + `gz_link_attacher.py` (same `/ATTACHLINK` `/DETACHLINK` services) + local `linkattacher_msgs` |
| `model://sun`, `ground_plane`, `table` | defined inline in `world/world.world` |
| `<script>Gazebo/White</script>` materials | plain ambient/diffuse colours |
| world SDF 1.5 | SDF 1.9 with Physics / UserCommands / SceneBroadcaster / Sensors systems |

## Link attacher
`pick_place_gazebo.cpp` is unchanged: it still calls `/ATTACHLINK` / `/DETACHLINK` with
`("cobot","panda_leftfinger", <piece>, "link")`.
Chain: service -> `std_msgs/Empty` on `/gz_link_attacher/<piece>/attach|detach` -> ros_gz_bridge -> `gz::sim::systems::DetachableJoint`
(declared once per piece in `config/panda.gz_attachers.xacro`, names read from `initial_board_layout.json`).

Manual test:
    ros2 service call /ATTACHLINK linkattacher_msgs/srv/AttachLink \
      "{model1_name: cobot, link1_name: panda_leftfinger, model2_name: w_pawn_1, link2_name: link}"

Limits: the robot link is fixed in the xacro (`panda_leftfinger`); pieces not in the layout JSON (e.g. promoted queens) need adding to it.

## Things to check on first run
1. DetachableJoint starts *attached* when the child exists; `gz_link_attacher.py` sends detach to every piece for the first 90 s.  Pieces should sit still on the board; if one follows the gripper, raise `startup_detach_seconds`.
2. Point-cloud axes: `testmodel.py` negates x/y from the cloud (written for the Classic optical frame).  Compare one square's pose in `chessboard_poses.json` with the true board position; if mirrored, change the sign flips.
3. Hard-coded `/home/adithya-si3510/arm_ws/...` paths remain in `pick_place_gazebo.cpp`, `pick_and_place_client.cpp`, `chess_managment.py` (unchanged).
4. `bazu.urdf` only had its plugin names renamed; it is not used by the chess pipeline.
5. Removed: torsional-friction block in `panda_arm.urdf` (not valid under gz-sim), dead commented Classic blocks.
