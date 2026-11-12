#!/usr/bin/env python3

import cv2
from cv_bridge import CvBridge
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
import chess

COLUMNS = ['a', 'b', 'c', 'd', 'e', 'f', 'g', 'h']
ROWS = ['8', '7', '6', '5', '4', '3', '2', '1']


class ChessSquareDetectionNode(Node):

    def __init__(self):
        super().__init__('chess_square_detection_node')

        self.depth_sub = self.create_subscription(
            Image,
            '/realsense/depth/image_rect_raw',
            self.depth_callback,
            10,
        )

        self.bridge = CvBridge()
        self.latest_depth = None

        # Homography and Calibration
        self.board_detected = False
        self.H_matrix = None  # Perspective transformation matrix
        self.square_polygons = {}  # Holds true image coordinate polygons for all 64 squares

        # Distance from camera to empty board plane in mm
        self.board_plane_depth_mm = 650.0

        # State tracking
        self.board = chess.Board()
        self.current_turn = 'WHITE'  # Alternates between WHITE and BLACK
        self.move_in_progress = None  # Stores (source_square, target_square)
        self.waiting_for_move = True

        self.get_logger().info('Chess Square Perception System Online!')

    def depth_callback(self, msg):
        try:
            # 16-bit depth image (millimeters)
            self.latest_depth = self.bridge.imgmsg_to_cv2(
                msg, desired_encoding='passthrough'
            )

            # Detect the physical chessboard corners if not calibrated yet
            if not self.board_detected and self.latest_depth is not None:
                self.detect_chessboard_corners(self.latest_depth)

            # Run detection loop
            if self.board_detected and self.latest_depth is not None:
                self.process_frame_and_check_moves()

        except Exception as e:
            self.get_logger().error(f'Error: {e}')

    def detect_chessboard_corners(self, depth_frame):
        """Locates the 4 outer corners of the physical board using depth thresholding."""
        # Ensure image is single-channel 8-bit unsigned integer (CV_8UC1)
        if len(depth_frame.shape) == 3:
            depth_gray = cv2.cvtColor(depth_frame, cv2.COLOR_BGR2GRAY)
        else:
            depth_gray = depth_frame

        # Convert to 8-bit single channel explicitly
        depth_8u = cv2.normalize(
            depth_gray, None, 0, 255, cv2.NORM_MINMAX, dtype=cv2.CV_8U
        )

        # Apply OTSU thresholding on single channel
        _, thresh = cv2.threshold(
            depth_8u, 0, 255, cv2.THRESH_BINARY_INV + cv2.THRESH_OTSU
        )

        contours, _ = cv2.findContours(
            thresh, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE
        )

        board_contour = None
        max_area = 0

        for cnt in contours:
            area = cv2.contourArea(cnt)
            if area > 10000:  # Filter out noise
                peri = cv2.arcLength(cnt, True)
                approx = cv2.approxPolyDP(cnt, 0.02 * peri, True)
                if len(approx) == 4 and area > max_area:
                    board_contour = approx
                    max_area = area

        if board_contour is not None:
            pts = board_contour.reshape(4, 2)
            src_pts = self.order_points(pts)

            # Destination canonical 800x800 square plane
            dst_pts = np.array(
                [[0, 0], [800, 0], [800, 800], [0, 800]], dtype=np.float32
            )

            # Calculate Perspective Homography Matrix
            self.H_matrix, _ = cv2.findHomography(src_pts, dst_pts)
            self.H_inv = np.linalg.inv(self.H_matrix)

            # Build exact 64-square pixel regions mapped back to raw image
            sq_w, sq_h = 100, 100
            for r_idx, row in enumerate(ROWS):
                for c_idx, col in enumerate(COLUMNS):
                    sq_name = f'{col}{row}'

                    box_canonical = np.array(
                        [
                            [c_idx * sq_w, r_idx * sq_h],
                            [(c_idx + 1) * sq_w, r_idx * sq_h],
                            [(c_idx + 1) * sq_w, (r_idx + 1) * sq_h],
                            [c_idx * sq_w, (r_idx + 1) * sq_h],
                        ],
                        dtype=np.float32,
                    )

                    pts_reshaped = np.array([box_canonical])
                    img_pts = cv2.perspectiveTransform(
                        pts_reshaped, self.H_inv
                    )
                    self.square_polygons[sq_name] = np.int32(img_pts[0])

            self.board_detected = True
            self.get_logger().info(
                ' Physical Chessboard detected and all 64 squares calibrated!'
            )

    def order_points(self, pts):
        """Sorts 4 corners: top-left, top-right, bottom-right, bottom-left."""
        rect = np.zeros((4, 2), dtype=np.float32)
        s = pts.sum(axis=1)
        rect[0] = pts[np.argmin(s)]
        rect[2] = pts[np.argmax(s)]

        diff = np.diff(pts, axis=1)
        rect[1] = pts[np.argmin(diff)]
        rect[3] = pts[np.argmax(diff)]
        return rect

    def get_square_occupancy(self, square_name: str) -> bool:
        """Evaluates whether a piece is inside the specific detected square boundary using depth data."""
        if square_name not in self.square_polygons or self.latest_depth is None:
            return False

        poly = self.square_polygons[square_name]

        # Mask depth image to ONLY the specific detected square ROI
        mask = np.zeros(self.latest_depth.shape[:2], dtype=np.uint8)
        cv2.fillPoly(mask, [poly], 255)

        depths = self.latest_depth[mask == 255]
        valid_depths = depths[depths > 0]

        if len(valid_depths) == 0:
            return False

        measured_depth = np.median(valid_depths)
        height_above_board = self.board_plane_depth_mm - measured_depth

        # Piece present if height exceeds 15mm threshold
        return height_above_board > 15.0

    def process_frame_and_check_moves(self):
        # Render visual frame
        depth_vis = cv2.normalize(
            self.latest_depth, None, 0, 255, cv2.NORM_MINMAX, dtype=cv2.CV_8U
        )
        display_frame = cv2.applyColorMap(depth_vis, cv2.COLORMAP_JET)

        # 1. Draw detected square boundaries
        for sq_name, poly in self.square_polygons.items():
            occupied = self.get_square_occupancy(sq_name)

            # Draw green polygon if piece present, white if empty
            border_color = (0, 255, 0) if occupied else (255, 255, 255)
            cv2.polylines(
                display_frame, [poly], True, border_color, 1, cv2.LINE_AA
            )

            # Label square name
            cx, cy = int(np.mean(poly[:, 0])), int(np.mean(poly[:, 1]))
            cv2.putText(
                display_frame,
                sq_name,
                (cx - 10, cy + 4),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.3,
                (255, 255, 255),
                1,
            )

        # 2. Check for completed moves
        if self.waiting_for_move and self.move_in_progress:
            src, dst = self.move_in_progress
            src_has_piece = self.get_square_occupancy(src)
            dst_has_piece = self.get_square_occupancy(dst)

            # Move completion condition: Source cleared AND Destination filled
            if not src_has_piece and dst_has_piece:
                self.get_logger().info(
                    f' MOVE COMPLETE: [{src.upper()}] to [{dst.upper()}].'
                )

                # Switch active turn to next player
                self.current_turn = (
                    'BLACK' if self.current_turn == 'WHITE' else 'WHITE'
                )
                self.move_in_progress = None
                self.get_logger().info(
                    f' NEXT TURN: Player [{self.current_turn}]'
                )

        # 3. Status Display Banner
        turn_text = f"CURRENT TURN: {self.current_turn}"
        cv2.rectangle(
            display_frame, (0, 0), (display_frame.shape[1], 35), (0, 0, 0), -1
        )
        cv2.putText(
            display_frame,
            turn_text,
            (15, 23),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.6,
            (0, 255, 255),
            2,
        )

        cv2.imshow("Detected Chess Squares", display_frame)
        cv2.waitKey(1)


def main(args=None):
    rclpy.init(args=args)
    node = ChessSquareDetectionNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        cv2.destroyAllWindows()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()