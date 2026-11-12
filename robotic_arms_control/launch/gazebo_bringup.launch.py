"""Gazebo Harmonic (gz-sim 8) + ROS 2 Jazzy bring-up for the chess cobot.

Ported from the Gazebo Classic launch file:
  gazebo_ros gazebo.launch.py  -> ros_gz_sim gz_sim.launch.py
  gazebo_ros spawn_entity.py   -> ros_gz_sim create
  gazebo_ros2_control          -> gz_ros2_control (configured in the URDF)
  libgazebo_link_attacher      -> DetachableJoint + gz_link_attacher.py
"""
import json
import os
import tempfile

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    AppendEnvironmentVariable, DeclareLaunchArgument, IncludeLaunchDescription,
    RegisterEventHandler, TimerAction
)
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder
import yaml


def _bridge_config(moveit_cfg_pkg):
    """gz_bridge.yaml + one attach/detach topic pair per chess piece."""
    with open(os.path.join(moveit_cfg_pkg, 'config', 'gz_bridge.yaml')) as f:
        entries = yaml.safe_load(f)
    with open(os.path.join(moveit_cfg_pkg, 'config', 'initial_board_layout.json')) as f:
        pieces = sorted(set(json.load(f).values()))
    for p in pieces:
        for kind in ('attach', 'detach'):
            topic = f'/gz_link_attacher/{p}/{kind}'
            entries.append({
                'ros_topic_name': topic,
                'gz_topic_name': topic,
                'ros_type_name': 'std_msgs/msg/Empty',
                'gz_type_name': 'gz.msgs.Empty',
                'direction': 'ROS_TO_GZ',
            })
    out = os.path.join(tempfile.gettempdir(), 'chess_cobot_gz_bridge.yaml')
    with open(out, 'w') as f:
        yaml.safe_dump(entries, f)
    return out


def generate_launch_description():
    ld = LaunchDescription()

    # --- share dirs ---
    moveit_cfg_pkg = get_package_share_directory('moveit_config_pkg')
    ros_gz_sim_share = get_package_share_directory('ros_gz_sim')
    world_file = os.path.join(moveit_cfg_pkg, 'world', 'world.sdf')

    # --- Gazebo resource lookup (replaces GAZEBO_MODEL_PATH) ---
    #  * models/ : model://chessboard, model://w_pawn, model://realsense_camera ...
    #  * parents of the ROS share dirs: resolves package://franka_description/meshes/...
    resource_dirs = [
        os.path.join(moveit_cfg_pkg, 'models'),
        os.path.dirname(get_package_share_directory('franka_description')),
        os.path.dirname(get_package_share_directory('robotic_arms_control')),
    ]
    for d in resource_dirs:
        ld.add_action(AppendEnvironmentVariable('GZ_SIM_RESOURCE_PATH', d))

    # --- args ---
    ld.add_action(DeclareLaunchArgument('with_rviz', default_value='true'))
    ld.add_action(DeclareLaunchArgument('gui', default_value='true'))
    ld.add_action(DeclareLaunchArgument('x', default_value='0.0'))
    ld.add_action(DeclareLaunchArgument('y', default_value='0.0'))
    ld.add_action(DeclareLaunchArgument('z', default_value='0.0'))

    # --- MoveIt config ---
    moveit_config = (
        MoveItConfigsBuilder('custom_robot', package_name='moveit_config_pkg')
        .robot_description(
            file_path='config/panda.urdf.xacro',
            mappings={
                'initial_positions_file': os.path.join(
                    moveit_cfg_pkg, 'config', 'initial_positions.yaml'),
            },
        )
        .robot_description_semantic(file_path='config/panda.srdf')
        .trajectory_execution(file_path='config/moveit_controllers.yaml')
        .robot_description_kinematics(file_path='config/kinematics.yaml')
        .planning_pipelines(pipelines=["ompl", "chomp", "pilz_industrial_motion_planner"])
        .planning_scene_monitor(
            publish_robot_description=True,
            publish_robot_description_semantic=True,
            publish_planning_scene=True
        )
        .to_moveit_configs()
    )

    # --- Gazebo Sim (-r: run immediately, -s: server only when gui:=false) ---
    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(ros_gz_sim_share, 'launch', 'gz_sim.launch.py')),
        launch_arguments={
            'gz_args': [
                PythonExpression(["'-r -s ' if '", LaunchConfiguration('gui'),
                                  "' == 'false' else '-r '"]),
                world_file,
            ],
            'on_exit_shutdown': 'true',
        }.items(),
    )
    ld.add_action(gz_sim)

    # --- ROS <-> Gazebo bridges ---
    bridge = Node(
        package='ros_gz_bridge', executable='parameter_bridge',
        parameters=[{'config_file': _bridge_config(moveit_cfg_pkg)}],
        output='screen',
    )
    # spawn service (isolated in its own node so a problem here cannot take
    # the camera/attacher bridge down; pice_spawn.py falls back to the CLI)
    spawn_service_bridge = Node(
        package='ros_gz_bridge', executable='parameter_bridge',
        name='spawn_service_bridge',
        arguments=['/world/default/create@ros_gz_interfaces/srv/SpawnEntity'
                   '@gz.msgs.EntityFactory@gz.msgs.Boolean'],
        output='screen',
    )
    ld.add_action(bridge)
    ld.add_action(spawn_service_bridge)

    # --- RSP ---
    ld.add_action(Node(
        package='robot_state_publisher', executable='robot_state_publisher',
        parameters=[moveit_config.robot_description, {'use_sim_time': True}],
        output='screen',
    ))

    # --- Spawn robot (entity name must stay "cobot": the C++ node uses it) ---
    spawn = Node(
        package='ros_gz_sim', executable='create',
        arguments=[
            '-world', 'default',
            '-name', 'cobot',
            '-topic', 'robot_description',
            '-x', LaunchConfiguration('x'),
            '-y', LaunchConfiguration('y'),
            '-z', LaunchConfiguration('z'),
        ],
        output='screen',
    )
    ld.add_action(TimerAction(period=5.0, actions=[spawn]))

    # --- ros2_control controllers (controller_manager lives inside gz) ---
    def spawner(name):
        return Node(
            package='controller_manager', executable='spawner',
            arguments=[name, '--controller-manager', '/controller_manager',
                       '--controller-manager-timeout', '60'],
            output='screen',
        )

    jsb, arm, grip = (spawner('joint_state_broadcaster'),
                      spawner('arm_controller'), spawner('gripper_controller'))
    ld.add_action(RegisterEventHandler(
        OnProcessExit(target_action=spawn, on_exit=[jsb, arm, grip])))

    # --- static TFs for the overhead camera ---
    ld.add_action(Node(
        package='tf2_ros', executable='static_transform_publisher',
        name='world_to_camera_tf',
        arguments=['--x', '0.0', '--y', '0.0', '--z', '2.5',
                   '--roll', '0.0', '--pitch', '1.5708', '--yaw', '0.0',
                   '--frame-id', 'world', '--child-frame-id', 'realsense_link'],
    ))
    ld.add_action(Node(
        package='tf2_ros', executable='static_transform_publisher',
        name='camera_to_optical_tf',
        arguments=['--x', '0.0', '--y', '0.0', '--z', '0.0',
                   '--roll', '0.0', '--pitch', '0.0', '--yaw', '0.0',
                   '--frame-id', 'realsense_link',
                   '--child-frame-id', 'camera_depth_optical_frame'],
    ))

    # --- MoveIt ---
    mg_params = moveit_config.to_dict()
    mg_params.update({'use_sim_time': True})
    ld.add_action(Node(
        package='moveit_ros_move_group', executable='move_group',
        output='screen', parameters=[mg_params],
        arguments=['--ros-args', '--log-level', 'info'],
    ))
    ld.add_action(Node(
        package='rviz2', executable='rviz2', name='rviz2', output='screen',
        arguments=['-d', os.path.join(moveit_cfg_pkg, 'config', 'moveit.rviz')],
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.planning_pipelines,
            moveit_config.robot_description_kinematics,
            {'use_sim_time': True},
        ],
        condition=IfCondition(LaunchConfiguration('with_rviz')),
    ))

    # --- application nodes ---
    ld.add_action(Node(package='moveit_config_pkg', executable='gz_link_attacher.py',
                       output='screen'))
    ld.add_action(Node(package='moveit_config_pkg', executable='pick_and_place_node',
                       output='screen'))
    ld.add_action(Node(package='moveit_config_pkg', executable='testmodel.py',
                       output='screen'))
    ld.add_action(Node(package='moveit_config_pkg', executable='pice_spawn.py',
                       output='screen'))
    # ld.add_action(Node(package='moveit_config_pkg', executable='chess_managment.py',
    #                    output='screen'))
    return ld
