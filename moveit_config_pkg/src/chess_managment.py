#!/usr/bin/env python3

import json
import os

os.environ["QT_LOGGING_RULES"] = "qt.qpa.fonts.warning=false"
os.environ["QT_QPA_FONTDIR"] = "/usr/share/fonts/truetype"

from action_msgs.msg import GoalStatus
from ament_index_python.packages import get_package_share_directory
import chess
import cv2
from cv_bridge import CvBridge
from franka_description.action import PickAndPlace
import numpy as np
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image, PointCloud2
from sensor_msgs_py import point_cloud2


class CompleteChessEngineVisionNode(Node):

  def __init__(self):
    super().__init__('chess_engine_vision_node')

    self.pattern_size = (7, 7)
    self.files = ['a', 'b', 'c', 'd', 'e', 'f', 'g', 'h']
    self.ranks = ['8', '7', '6', '5', '4', '3', '2', '1']

    self.bridge = CvBridge()
    self.latest_pc = None

    self.height_threshold = 0.015

    self.board = chess.Board()
    self.active_move = None

    self.waiting_for_user_move = True
    self.robot_executing_move = False
    self.pending_robot_move = None
    self.current_feedback_stage = 'Initializing'

    # Captured Piece Graveyard Tracking
    self.graveyard_start_x = 0.209654
    self.graveyard_y = -0.288512
    self.graveyard_x_step = 0.057965
    self.captured_black_count = 0

    self.board_layout = self.load_board_layout_from_json()

    self._action_client = ActionClient(self, PickAndPlace, 'pick_and_place')

    self.is_calibrated = False
    self.saved_full_grid = None
    self.saved_board_z_plane = None

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

    self.get_logger().info('Chess Engine Perception Node Online!')
    self.calculate_and_set_next_move()

  def pc_callback(self, msg: PointCloud2):
    self.latest_pc = msg

  def load_board_layout_from_json(self):
    json_path = (
        '/home/adithya-si3510/arm_ws/src/moveit_config_pkg/config/initial_board_layout.json'
    )
    if not os.path.exists(json_path):
      try:
        pkg_path = get_package_share_directory('moveit_config_pkg')
        json_path = os.path.join(
            pkg_path, 'config', 'initial_board_layout.json'
        )
      except Exception:
        pass

    if os.path.exists(json_path):
      try:
        with open(json_path, 'r') as f:
          layout = json.load(f)
        self.get_logger().info(f'Loaded board layout from {json_path}')
        return layout
      except Exception as e:
        self.get_logger().error(f'Failed to read layout JSON: {e}')
    return {}

  def print_terminal_board_state(self):
    """Prints current ASCII board and piece tracking in terminal."""
    print('\n' + '=' * 45)
    print('          CURRENT BOARD STATE')
    print('=' * 45)
    print(self.board)
    print('=' * 45)
    print('PIECE LAYOUT TRACKING:')
    for sq in sorted(self.board_layout.keys()):
      print(f'  {sq.upper()}: {self.board_layout[sq]}')
    print('=' * 45 + '\n')

  def calculate_graveyard_x(self):
    """Calculates offset for the off-board graveyard position."""
    return self.graveyard_start_x - (
        self.captured_black_count * self.graveyard_x_step
    )

  def execute_pick_and_place_client(self, src: str, dst: str):
    """Evaluates capture vs standard move and sends action goal."""
    target_piece = self.board_layout.get(dst, None)
    move_obj = chess.Move.from_uci(f'{src}{dst}')
    is_capture = (
        target_piece is not None and target_piece != 'unknown_piece'
    ) or self.board.is_capture(move_obj)

    if is_capture:
      captured_id = self.board_layout.get(dst, 'b_piece')
      graveyard_x = self.calculate_graveyard_x()

      self.get_logger().warn(
          f'CAPTURE DETECTED! Removing Black Piece {captured_id} from {dst} ->'
          f' GRAVEYARD (X: {graveyard_x:.4f}, Y: {self.graveyard_y:.4f})'
      )

      # Step 1: Tell action server to move captured piece to GRAVEYARD
      self.send_goal_to_action_server(
          src=dst,
          dst='GRAVEYARD',
          piece_id=captured_id,
          next_phase=lambda: self.execute_primary_move(src, dst),
      )
    else:
      self.execute_primary_move(src, dst)

  def execute_primary_move(self, src: str, dst: str):
    piece_id = self.board_layout.get(src, 'unknown_piece')
    self.send_goal_to_action_server(
        src=src,
        dst=dst,
        piece_id=piece_id,
        next_phase=self.handle_client_success,
    )

  def send_goal_to_action_server(
      self, src: str, dst: str, piece_id: str, next_phase
  ):
    if not self._action_client.wait_for_server(timeout_sec=3.0):
      self.get_logger().error("Action server '/pick_and_place' not available.")
      self.robot_executing_move = False
      self.waiting_for_user_move = True
      return

    # Lock state immediately to prevent rapid frame-triggering during execution
    self.robot_executing_move = True
    self.waiting_for_user_move = False

    goal_msg = PickAndPlace.Goal()
    goal_msg.source_sqr = src
    goal_msg.target_sqr = dst
    goal_msg.piece_id = piece_id

    self.get_logger().info(
        f"Sending Goal -> Src: '{src}' | Dst: '{dst}' | Piece: '{piece_id}'"
    )

    self.pending_robot_move = (src, dst, piece_id)
    self.current_next_phase = next_phase

    send_goal_future = self._action_client.send_goal_async(
        goal_msg, feedback_callback=self.action_feedback_callback
    )
    send_goal_future.add_done_callback(self.goal_response_callback)

  def goal_response_callback(self, future):
    goal_handle = future.result()
    if not goal_handle.accepted:
      self.get_logger().error('Goal rejected by action server!')
      self.robot_executing_move = False
      self.waiting_for_user_move = True
      return

    get_result_future = goal_handle.get_result_async()
    get_result_future.add_done_callback(self.action_result_callback)

  def action_feedback_callback(self, feedback_msg):
    feedback = feedback_msg.feedback
    self.current_feedback_stage = (
        f'{feedback.current_stage} ({feedback.percentage_complete:.1f}%)'
    )

  def action_result_callback(self, future):
    result_response = future.result()
    if result_response.status == GoalStatus.STATUS_SUCCEEDED:
      self.get_logger().info('Goal Execution Succeeded!')
      if hasattr(self, 'current_next_phase') and self.current_next_phase:
        self.current_next_phase()
    else:
      self.get_logger().error(
          'Goal Execution Failed! Halting execution state.'
      )
      self.robot_executing_move = False
      self.waiting_for_user_move = False

  def handle_client_success(self):
    src, dst, piece_id = self.pending_robot_move

    # If this move was placing a piece into the graveyard
    if dst == 'GRAVEYARD':
      if src in self.board_layout:
        del self.board_layout[src]
      self.captured_black_count += 1
      self.get_logger().info(
          f'Black piece moved to graveyard. Total captures:'
          f' {self.captured_black_count}'
      )
      return

    # Standard board move mapping update
    if src in self.board_layout:
      del self.board_layout[src]
    self.board_layout[dst] = piece_id

    move_uci = f'{src}{dst}'
    try:
      self.board.push_san(move_uci)
    except Exception:
      self.board.push(chess.Move.from_uci(move_uci))

    self.robot_executing_move = False
    self.pending_robot_move = None

    self.print_terminal_board_state()
    self.calculate_and_set_next_move()

  def calculate_and_set_next_move(self):
    if self.board.is_game_over():
      self.get_logger().info('GAME OVER!')
      self.active_move = None
      return

    if self.board.turn == chess.WHITE:
      legal_moves = list(self.board.legal_moves)
      if legal_moves:
        # Prioritize captures if available, otherwise pick first legal move
        captures = [m for m in legal_moves if self.board.is_capture(m)]
        next_move = captures[0] if captures else legal_moves[0]

        next_src = chess.square_name(next_move.from_square)
        next_dst = chess.square_name(next_move.to_square)

        self.active_move = (next_src, next_dst)
        self.waiting_for_user_move = True
        self.get_logger().info(
            f'ROBOT (WHITE) TARGET MOVE: {next_src.upper()} ->'
            f' {next_dst.upper()}'
        )
    else:
      self.active_move = None
      self.waiting_for_user_move = True
      self.get_logger().info("HUMAN (BLACK) TURN: Waiting for player's move...")

  def color_callback(self, msg: Image):
    if self.latest_pc is None:
      return

    try:
      frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
    except Exception:
      return

    if not self.is_calibrated:
      gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
      ret, corners = cv2.findChessboardCorners(gray, self.pattern_size, None)

      if ret:
        criteria = (
            cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER,
            30,
            0.001,
        )
        corners = cv2.cornerSubPix(gray, corners, (11, 11), (-1, -1), criteria)
        cols, rows = self.pattern_size
        inner_grid = corners.reshape((rows, cols, 2))

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

        self.saved_full_grid = full_grid
        pc_array = point_cloud2.read_points_numpy(
            self.latest_pc, field_names=('x', 'y', 'z')
        )
        pc_grid = pc_array.reshape(
            (self.latest_pc.height, self.latest_pc.width, 3)
        )

        self.saved_board_z_plane = np.nanmedian(
            pc_grid[
                inner_grid[:, :, 1].astype(int),
                inner_grid[:, :, 0].astype(int),
                2,
            ]
        )
        self.is_calibrated = True
        self.get_logger().info('BOARD CALIBRATED!')
        self.print_terminal_board_state()
      else:
        cv2.imshow('Chess Engine Vision', frame)
        cv2.waitKey(1)
        return

    pc_array = point_cloud2.read_points_numpy(
        self.latest_pc, field_names=('x', 'y', 'z')
    )
    pc_grid = pc_array.reshape(
        (self.latest_pc.height, self.latest_pc.width, 3)
    )

    board_occupancy = {}

    for r in range(8):
      for c in range(8):
        p1 = self.saved_full_grid[r, c]
        p2 = self.saved_full_grid[r, c + 1]
        p3 = self.saved_full_grid[r + 1, c + 1]
        p4 = self.saved_full_grid[r + 1, c]
        pts = np.array([p1, p2, p3, p4], dtype=np.int32)
        square_name = f'{self.files[c]}{self.ranks[r]}'

        center_u = int((p1[0] + p2[0] + p3[0] + p4[0]) / 4.0)
        center_v = int((p1[1] + p2[1] + p3[1] + p4[1]) / 4.0)
        center_u = max(2, min(center_u, self.latest_pc.width - 3))
        center_v = max(2, min(center_v, self.latest_pc.height - 3))

        roi_depth = pc_grid[
            center_v - 2 : center_v + 3, center_u - 2 : center_u + 3, 2
        ]
        valid_depths = roi_depth[~np.isnan(roi_depth) & ~np.isinf(roi_depth)]

        color = (255, 255, 255)
        thickness = 1
        if self.active_move:
          src, dst = self.active_move
          if square_name == src:
            color = (255, 0, 0)
            thickness = 3
          elif square_name == dst:
            color = (0, 255, 0)
            thickness = 3

        if len(valid_depths) > 0:
          board_occupancy[square_name] = (
              self.saved_board_z_plane - np.min(valid_depths)
          ) > self.height_threshold
        else:
          board_occupancy[square_name] = False

        cv2.polylines(frame, [pts], True, color, thickness)

    # TURN PROCESSING
    is_robot_turn = self.board.turn == chess.WHITE

    if self.waiting_for_user_move and not self.robot_executing_move:
      if is_robot_turn and self.active_move:
        src, dst = self.active_move
        self.execute_pick_and_place_client(src, dst)

      elif not is_robot_turn:
        for move in self.board.legal_moves:
          src = chess.square_name(move.from_square)
          dst = chess.square_name(move.to_square)

          if not board_occupancy.get(src, True) and board_occupancy.get(
              dst, False
          ):
            self.get_logger().info(
                f'HUMAN MOVED: {src.upper()} -> {dst.upper()}'
            )

            # Update json layout tracking for human move
            p_id = self.board_layout.get(src, 'b_piece')
            if src in self.board_layout:
              del self.board_layout[src]
            self.board_layout[dst] = p_id

            self.board.push(move)
            self.print_terminal_board_state()
            self.calculate_and_set_next_move()
            break

    cv2.imshow('Chess Engine Vision', frame)
    cv2.waitKey(1)


def main(args=None):
  rclpy.init(args=args)
  node = CompleteChessEngineVisionNode()
  try:
    rclpy.spin(node)
  except KeyboardInterrupt:
    pass
  finally:
    node.destroy_node()
    cv2.destroyAllWindows()
    rclpy.shutdown()


if __name__ == '__main__':
  main()