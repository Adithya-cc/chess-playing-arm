#!/usr/bin/env python3

import json
import os
import subprocess
from ament_index_python.packages import get_package_share_directory
from ros_gz_interfaces.srv import SpawnEntity
import rclpy
from rclpy.node import Node

# Fallback board layout mapping square names to model folder names
DEFAULT_BOARD_LAYOUT = {
    # White Pieces
    'a1': 'w_rook',
    'b1': 'w_knight',
    'c1': 'w_bishop',
    'd1': 'w_queen',
    'e1': 'w_king',
    'f1': 'w_bishop',
    'g1': 'w_knight',
    'h1': 'w_rook',
    'a2': 'w_pawn',
    'b2': 'w_pawn',
    'c2': 'w_pawn',
    'd2': 'w_pawn',
    'e2': 'w_pawn',
    'f2': 'w_pawn',
    'g2': 'w_pawn',
    'h2': 'w_pawn',
    # Black Pieces
    'a8': 'b_rook',
    'b8': 'b_knight',
    'c8': 'b_bishop',
    'd8': 'b_queen',
    'e8': 'b_king',
    'f8': 'b_bishop',
    'g8': 'b_knight',
    'h8': 'b_rook',
    'a7': 'b_pawn',
    'b7': 'b_pawn',
    'c7': 'b_pawn',
    'd7': 'b_pawn',
    'e7': 'b_pawn',
    'f7': 'b_pawn',
    'g7': 'b_pawn',
    'h7': 'b_pawn',
}

# Vertical offsets (meters) to adjust piece base placement above board surface
PIECE_Z_OFFSETS = {
    'pawn': 0.02,
    'rook': 0.025,
    'knight': 0.028,
    'bishop': 0.03,
    'queen': 0.035,
    'king': 0.04,
}


class ChessPieceSpawner(Node):

  def __init__(self):
    super().__init__('chess_piece_spawner')

    # Gazebo Harmonic spawn service, bridged by ros_gz_bridge:
    #   /world/<world>/create  (ros_gz_interfaces/srv/SpawnEntity)
    self.declare_parameter('world_name', 'default')
    self.world_name = self.get_parameter('world_name').value
    self.service_name = f'/world/{self.world_name}/create'
    self.client = self.create_client(SpawnEntity, self.service_name)
    # If the service bridge is unavailable, fall back to `ros_gz_sim create`.
    self.use_service = False
    for _ in range(30):
      if self.client.wait_for_service(timeout_sec=1.0):
        self.use_service = True
        break
      self.get_logger().info(f'Waiting for {self.service_name} ...')
    if self.use_service:
      self.get_logger().info('Connected to Gazebo spawn service.')
    else:
      self.get_logger().warn(
          f'{self.service_name} not available; falling back to '
          '`ros2 run ros_gz_sim create`.')

  def get_package_path(self):
    """Utility to locate moveit_config_pkg directory safely."""
    try:
      return get_package_share_directory('moveit_config_pkg')
    except Exception:
      return os.path.expanduser('~/arm_ws/src/moveit_config_pkg')

  def load_poses_from_json(self):
    """Reads chessboard_poses.json from moveit_config_pkg/config/."""
    pkg_path = self.get_package_path()
    json_path = os.path.join(pkg_path, 'config', 'chessboard_poses.json')

    if not os.path.exists(json_path):
      self.get_logger().error(f'Poses JSON file not found at: {json_path}')
      return None

    try:
      with open(json_path, 'r') as f:
        board_poses = json.load(f)
      self.get_logger().info(f'Successfully loaded poses from {json_path}')
      return board_poses
    except Exception as e:
      self.get_logger().error(f'Failed to read Poses JSON file: {e}')
      return None

  def load_board_layout_from_json(self):
    """Reads initial_board_layout.json from moveit_config_pkg/config/."""
    pkg_path = self.get_package_path()
    json_path = os.path.join(pkg_path, 'config', 'initial_board_layout.json')

    if not os.path.exists(json_path):
      self.get_logger().warn(
          f'Layout JSON not found at {json_path}. Falling back to default'
          ' layout.'
      )
      return DEFAULT_BOARD_LAYOUT

    try:
      with open(json_path, 'r') as f:
        board_layout = json.load(f)
      self.get_logger().info(f'Successfully loaded layout from {json_path}')
      return board_layout
    except Exception as e:
      self.get_logger().error(
          f'Failed to read Layout JSON file: {e}. Using default.'
      )
      return DEFAULT_BOARD_LAYOUT

  def spawn_piece(self, piece_name, model_dir_name, x, y, z):
    """Loads the piece SDF and spawns it into Gazebo Harmonic."""
    pkg_path = self.get_package_path()
    model_path = os.path.join(
        pkg_path, 'models', model_dir_name, 'model.sdf'
    )

    if not os.path.exists(model_path):
      model_path = os.path.join(
          pkg_path, 'models', model_dir_name, 'model.urdf'
      )

    if not os.path.exists(model_path):
      self.get_logger().error(
          f'Model file not found for: {model_dir_name} at {model_path}'
      )
      return False

    with open(model_path, 'r') as f:
      model_xml = f.read()

    if not self.use_service:
      return self.spawn_piece_cli(piece_name, model_path, x, y, z)

    req = SpawnEntity.Request()
    req.entity_factory.name = piece_name
    req.entity_factory.allow_renaming = False
    req.entity_factory.sdf = model_xml
    req.entity_factory.pose.position.x = float(x)
    req.entity_factory.pose.position.y = float(y)
    req.entity_factory.pose.position.z = float(z)
    req.entity_factory.pose.orientation.w = 1.0

    future = self.client.call_async(req)
    rclpy.spin_until_future_complete(self, future)

    if future.result() is not None and future.result().success:
      self.get_logger().info(
          f"Successfully spawned '{piece_name}' at ({x:.3f}, {y:.3f}, {z:.3f})"
      )
      return True
    else:
      self.get_logger().error(f"Failed to spawn '{piece_name}'")
      return False

  def spawn_piece_cli(self, piece_name, model_path, x, y, z):
    """Fallback: spawn through the ros_gz_sim `create` executable."""
    cmd = [
        'ros2', 'run', 'ros_gz_sim', 'create',
        '-world', self.world_name, '-name', piece_name,
        '-file', model_path,
        '-x', str(float(x)), '-y', str(float(y)), '-z', str(float(z)),
    ]
    ok = subprocess.run(cmd, capture_output=True).returncode == 0
    if ok:
      self.get_logger().info(f"Spawned '{piece_name}' (ros_gz_sim create) x:{x}, y:{y}, z:{z}")
    else:
      self.get_logger().error(f"Failed to spawn '{piece_name}'")
    return ok

  def spawn_all_pieces(self):
      """Reads JSON layout & poses, spawns pieces using indexed JSON names."""
      board_poses = self.load_poses_from_json()
      board_layout = self.load_board_layout_from_json()

      if board_poses is None or board_layout is None:
        self.get_logger().error(
            'Aborting piece spawn due to missing JSON data.'
        )
        return
      CAM_Z = 2.5  # must match world_to_camera_tf z

      for square, piece_unique_id in board_layout.items():
        if square in board_poses and board_poses[square] is not None:
          x_opt, y_opt, depth = board_poses[square]

          # camera-optical -> world (camera at (0, 0, CAM_Z) looking straight down)
          x = x_opt
          y = y_opt
          z = CAM_Z - depth          # 2.5 - 1.475 = 1.025 (board surface)

          model_folder = '_'.join(piece_unique_id.split('_')[:2])
          piece_type = model_folder.split('_')[1]
          spawn_z = z + PIECE_Z_OFFSETS.get(piece_type, 0.02)

          self.spawn_piece(piece_unique_id, model_folder, x, y, spawn_z)


def main():
  rclpy.init()
  spawner = ChessPieceSpawner()

  # Load JSON layout + poses automatically and spawn numbered pieces
  spawner.spawn_all_pieces()

  spawner.destroy_node()
  rclpy.shutdown()


if __name__ == '__main__':
  main()