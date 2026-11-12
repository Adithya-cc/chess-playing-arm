#!/usr/bin/env python3
import json
import os
from pprint import pprint

from ament_index_python.packages import get_package_share_directory
import cv2
from cv_bridge import CvBridge
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image, PointCloud2
from sensor_msgs_py import point_cloud2


class ChessboardDictionaryDetector(Node):

  def __init__(self):
    super().__init__('chessboard_dictionary_detector')

    self.pattern_size = (7, 7)  # 7x7 inner corners

    # Files (a to h across columns) and Ranks (8 to 1 down rows)
    self.files = ['a', 'b', 'c', 'd', 'e', 'f', 'g', 'h']
    self.ranks = ['8', '7', '6', '5', '4', '3', '2', '1']

    self.bridge = CvBridge()
    self.latest_pc = None

    # Graveyard Default Position Configuration
    self.graveyard_base_pose = [0.209654, -0.288512, 0.1800]

    self.create_subscription(
        PointCloud2,
        '/realsense/depth/points',
        self.pc_callback,
        qos_profile_sensor_data,
    )

    self.create_subscription(
        Image,
        '/realsense/depth/image_rect_raw',
        self.color_callback,
        qos_profile_sensor_data,
    )

    self.get_logger().info('Chessboard 3D Dictionary Detector Started.')

  def pc_callback(self, msg: PointCloud2):
    self.latest_pc = msg

  def save_poses_to_json(self, chessboard_poses):
    """Saves the detected 3D poses dictionary as JSON in moveit_config_pkg/config/."""
    try:
      try:
        pkg_path = get_package_share_directory('moveit_config_pkg')
      except Exception:
        pkg_path = os.path.expanduser('~/arm_ws/src/moveit_config_pkg')

      config_dir = os.path.join(pkg_path, 'config')
      os.makedirs(config_dir, exist_ok=True)

      json_path = os.path.join(config_dir, 'chessboard_poses.json')

      # Convert tuples to lists for standard JSON formatting
      json_data = {
          k: list(v) if v is not None else None
          for k, v in chessboard_poses.items()
      }

      with open(json_path, 'w') as f:
        json.dump(json_data, f, indent=4)

      self.get_logger().info(
          f'SUCCESSFULLY saved board poses to: {json_path}'
      )
      return True

    except Exception as e:
      self.get_logger().error(f'Failed to save JSON: {e}')
      return False

  def color_callback(self, msg: Image):
    if self.latest_pc is None:
      return

    try:
      frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
    except Exception as e:
      self.get_logger().error(f'CvBridge Error: {e}')
      return

    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    ret, corners = cv2.findChessboardCorners(gray, self.pattern_size, None)

    if ret:
      criteria = (
          cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER,
          30,
          0.001,
      )
      corners = cv2.cornerSubPix(gray, corners, (11, 11), (-1, -1), criteria)

      cols, rows = self.pattern_size  # 7, 7
      inner_grid = corners.reshape((rows, cols, 2))

      # ------------------------------------------------------------------
      # STEP 1: Extrapolate to full 9x9 grid points (8x8 squares)
      # ------------------------------------------------------------------
      full_grid = np.zeros((9, 9, 2), dtype=np.float32)
      full_grid[1:8, 1:8] = inner_grid

      for r in range(1, 8):
        step_x_left = inner_grid[r - 1, 1] - inner_grid[r - 1, 0]
        full_grid[r, 0] = inner_grid[r - 1, 0] - step_x_left

        step_x_right = inner_grid[r - 1, 6] - inner_grid[r - 1, 5]
        full_grid[r, 8] = inner_grid[r - 1, 6] + step_x_right

      for c in range(0, 9):
        step_y_top = full_grid[2, c] - full_grid[1, c]
        full_grid[0, c] = full_grid[1, c] - step_y_top

        step_y_bottom = full_grid[7, c] - full_grid[6, c]
        full_grid[8, c] = full_grid[7, c] + step_y_bottom

      # ------------------------------------------------------------------
      # STEP 2: Draw outer border
      # ------------------------------------------------------------------
      border_pts = np.array(
          [full_grid[0, 0], full_grid[0, 8], full_grid[8, 8], full_grid[8, 0]],
          dtype=np.int32,
      ).reshape((-1, 1, 2))

      cv2.polylines(
          frame, [border_pts], isClosed=True, color=(0, 255, 0), thickness=3
      )

      # ------------------------------------------------------------------
      # STEP 3: Store 3D Coordinates into Dictionary (Rank 8 -> Rank 1)
      # ------------------------------------------------------------------
      pc_array = point_cloud2.read_points_numpy(
          self.latest_pc, field_names=('x', 'y', 'z')
      )
      pc_grid = pc_array.reshape(
          (self.latest_pc.height, self.latest_pc.width, 3)
      )

      chessboard_poses = {}

      # Outer Loop: Ranks ('8' down to '1')
      for r in range(8):
        # Inner Loop: Files ('a' across to 'h')
        for c in range(8):
          # Access transposed grid indices [c, r] to align image axes
          p1 = full_grid[c, r]
          p2 = full_grid[c + 1, r]
          p3 = full_grid[c + 1, r + 1]
          p4 = full_grid[c, r + 1]

          center_u = int((p1[0] + p2[0] + p3[0] + p4[0]) / 4.0)
          center_v = int((p1[1] + p2[1] + p3[1] + p4[1]) / 4.0)

          square_name = f'{self.files[c]}{self.ranks[r]}'

          center_u = max(0, min(center_u, self.latest_pc.width - 1))
          center_v = max(0, min(center_v, self.latest_pc.height - 1))

          point_3d = pc_grid[center_v, center_u]
          x, y, z = point_3d[0], point_3d[1], point_3d[2]

          if not (np.isnan(z) or np.isinf(z)):
            chessboard_poses[square_name] = (
                round(-float(x), 4),
                round(-float(y), 4),
                round(float(z), 4),
            )
          else:
            chessboard_poses[square_name] = None

          # Visual Feedback
          cv2.circle(frame, (center_u, center_v), 2, (0, 0, 255), -1)
          cv2.putText(
              frame,
              square_name,
              (center_u - 10, center_v + 4),
              cv2.FONT_HERSHEY_SIMPLEX,
              0.35,
              (255, 255, 255),
              1,
          )

      # ------------------------------------------------------------------
      # STEP 4: Add GRAVEYARD_BASE to Dictionary
      # ------------------------------------------------------------------
      chessboard_poses['GRAVEYARD_BASE'] = self.graveyard_base_pose

      # ------------------------------------------------------------------
      # STEP 5: Output & Exit Node
      # ------------------------------------------------------------------
      print('\n' + '=' * 50)
      print('CHESSBOARD SQUARES 3D POSES (Camera Frame):')
      print('=' * 50)
      pprint(chessboard_poses)

      cv2.waitKey(1)

      if self.save_poses_to_json(chessboard_poses):
        rclpy.shutdown()
        return


def main(args=None):
  rclpy.init(args=args)
  node = ChessboardDictionaryDetector()
  try:
    rclpy.spin(node)
  except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
    pass
  finally:
    if rclpy.ok():
      node.destroy_node()
      rclpy.shutdown()
    cv2.destroyAllWindows()


if __name__ == '__main__':
  main()