"""
Human Follower Node — Detects and follows a human using depth camera.

Detection strategy (in order of priority):
1. OpenCV HOG Person Detector — classical gradient-based, works on synthetic meshes
2. Depth-based shape detection — finds tall vertical objects of human height

On real hardware, MediaPipe can be used instead (works great on real humans).
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge
import cv2
import numpy as np


class HumanFollowerNode(Node):
    def __init__(self):
        super().__init__('human_follower')

        # --- Parameters ---
        self.declare_parameter('follow_distance', 1.0)
        self.declare_parameter('max_linear_speed', 0.4)
        self.declare_parameter('max_angular_speed', 0.8)
        self.declare_parameter('linear_gain', 0.5)
        self.declare_parameter('angular_gain', 2.0)
        self.declare_parameter('dead_zone', 0.05)
        self.declare_parameter('detection_timeout', 3.0)
        self.declare_parameter('min_detect_range', 0.5)
        self.declare_parameter('max_detect_range', 3.0)
        self.declare_parameter('use_sim_time', True)

        self.follow_distance = self.get_parameter('follow_distance').value
        self.max_linear_speed = self.get_parameter('max_linear_speed').value
        self.max_angular_speed = self.get_parameter('max_angular_speed').value
        self.linear_gain = self.get_parameter('linear_gain').value
        self.angular_gain = self.get_parameter('angular_gain').value
        self.dead_zone = self.get_parameter('dead_zone').value
        self.detection_timeout = self.get_parameter('detection_timeout').value
        self.min_detect_range = self.get_parameter('min_detect_range').value
        self.max_detect_range = self.get_parameter('max_detect_range').value

        # --- Subscribers ---
        self.create_subscription(Image, '/camera/camera/color/image_raw', self.rgb_callback, 10)
        self.create_subscription(Image, '/camera/camera/aligned_depth_to_color/image_raw', self.depth_callback, 10)
        self.create_subscription(CameraInfo, '/camera/camera/color/camera_info', self.camera_info_callback, 10)

        # --- Publisher ---
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        # --- Internal state ---
        self.bridge = CvBridge()
        self.depth_image = None
        self.fx = None
        self.fy = None
        self.cx_intr = None
        self.cy_intr = None
        self.last_detection_time = self.get_clock().now()

        # --- OpenCV HOG Person Detector ---
        self.hog = cv2.HOGDescriptor()
        self.hog.setSVMDetector(cv2.HOGDescriptor_getDefaultPeopleDetector())

        self.get_logger().info('Human Follower Node started (HOG + Depth fallback)')
        self.get_logger().info(
            f'  follow_distance={self.follow_distance}m, '
            f'max_lin={self.max_linear_speed}m/s, '
            f'max_ang={self.max_angular_speed}rad/s'
        )

        # Diagnostic counters
        self.rgb_count = 0
        self.depth_count = 0
        self.hog_detections = 0
        self.depth_detections = 0
        self.create_timer(3.0, self._status_timer)

    def _status_timer(self):
        """Periodic status log for debugging."""
        has_intrinsics = self.fx is not None
        has_depth = self.depth_image is not None
        self.get_logger().info(
            f'[STATUS] intrinsics={has_intrinsics}, '
            f'rgb={self.rgb_count}, depth={self.depth_count}, '
            f'hog_det={self.hog_detections}, depth_det={self.depth_detections}'
        )

    def camera_info_callback(self, msg: CameraInfo):
        """Extract camera intrinsics (once)."""
        if self.fx is None:
            self.fx = msg.k[0]
            self.fy = msg.k[4]
            self.cx_intr = msg.k[2]
            self.cy_intr = msg.k[5]
            self.get_logger().info(
                f'Camera intrinsics received: fx={self.fx:.1f}, fy={self.fy:.1f}, '
                f'cx={self.cx_intr:.1f}, cy={self.cy_intr:.1f}'
            )

    def depth_callback(self, msg: Image):
        """Store latest depth image."""
        try:
            self.depth_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='32FC1')
            self.depth_count += 1
        except Exception as e:
            self.get_logger().error(f'Depth conversion error: {e}', throttle_duration_sec=5.0)

    def rgb_callback(self, msg: Image):
        """Main processing loop — detect human, compute follow command."""
        self.rgb_count += 1

        # Wait for intrinsics and depth
        if self.fx is None or self.depth_image is None:
            return

        try:
            frame = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        except Exception as e:
            self.get_logger().error(f'RGB conversion error: {e}', throttle_duration_sec=5.0)
            return

        twist = Twist()
        human_cx = None
        human_depth = None

        # --- Strategy 1: OpenCV HOG Person Detection ---
        human_cx, human_depth = self._detect_hog(frame)

        # --- Strategy 2: Depth-based shape detection (fallback) ---
        if human_cx is None:
            human_cx, human_depth = self._detect_depth_shape()

        # --- Compute control if human found ---
        if human_cx is not None and human_depth is not None and human_depth > 0:
            # Angular: center the human in the image
            image_center_x = frame.shape[1] / 2.0
            angular_error = (image_center_x - human_cx) / image_center_x

            twist.angular.z = float(np.clip(
                angular_error * self.angular_gain,
                -self.max_angular_speed,
                self.max_angular_speed,
            ))

            # Linear: maintain follow distance
            linear_error = human_depth - self.follow_distance
            if linear_error > self.dead_zone:
                twist.linear.x = float(np.clip(
                    linear_error * self.linear_gain,
                    0.0,
                    self.max_linear_speed,
                ))
            else:
                twist.linear.x = 0.0

            self.last_detection_time = self.get_clock().now()

            self.get_logger().info(
                f'FOLLOWING: depth={human_depth:.2f}m, ang_err={angular_error:.2f} '
                f'-> lin={twist.linear.x:.2f}, ang={twist.angular.z:.2f}',
                throttle_duration_sec=1.0,
            )
        else:
            # Human not found
            elapsed = (self.get_clock().now() - self.last_detection_time).nanoseconds / 1e9
            if elapsed > self.detection_timeout:
                self.get_logger().warn(
                    f'Human lost for {elapsed:.1f}s — stopping.',
                    throttle_duration_sec=5.0,
                )

        self.cmd_vel_pub.publish(twist)

    def _detect_hog(self, frame):
        """Detect human using OpenCV HOG person detector. Returns (cx, depth) or (None, None)."""
        # Resize for faster detection
        small = cv2.resize(frame, (320, 240))
        scale_x = frame.shape[1] / 320.0
        scale_y = frame.shape[0] / 240.0

        boxes, weights = self.hog.detectMultiScale(
            small,
            winStride=(8, 8),
            padding=(4, 4),
            scale=1.05,
        )

        if len(boxes) == 0:
            return None, None

        self.hog_detections += 1

        # Pick the detection with highest confidence
        best_idx = np.argmax(weights)
        x, y, w, h = boxes[best_idx]

        # Scale back to original resolution
        cx = int((x + w / 2) * scale_x)
        cy = int((y + h / 2) * scale_y)

        depth = self._sample_depth(cx, cy)
        if depth is not None and depth > 0:
            self.get_logger().info(
                f'HOG detected human at pixel ({cx},{cy}), depth={depth:.2f}m',
                throttle_duration_sec=2.0,
            )
            return cx, depth

        return None, None

    def _detect_depth_shape(self):
        """
        Detect tall vertical objects in the depth image (human-shaped).
        Looks for a cluster of valid depth pixels in the upper portion of the image
        that spans a significant vertical extent (like a standing person).
        Returns (cx, depth) or (None, None).
        """
        if self.depth_image is None:
            return None, None

        h, w = self.depth_image.shape[:2]
        depth = self.depth_image.copy()

        # Mask: only keep pixels in valid range
        valid_mask = np.isfinite(depth) & (depth > self.min_detect_range) & (depth < self.max_detect_range)

        # Ignore bottom 1/3 (mostly ground)
        valid_mask[int(h * 0.7):, :] = False

        if np.count_nonzero(valid_mask) < 50:
            return None, None

        # Find columns with many valid depth pixels (vertical extent = human-like)
        col_counts = np.sum(valid_mask, axis=0)  # how many valid pixels per column

        # A human should span at least 15% of the image height in valid pixels
        min_height_pixels = int(h * 0.10)

        # Find columns with enough vertical pixels
        human_cols = np.where(col_counts > min_height_pixels)[0]
        if len(human_cols) == 0:
            return None, None

        self.depth_detections += 1

        # Centroid of the human columns
        cx = int(np.mean(human_cols))

        # Average depth at the human columns (upper portion)
        col_depths = []
        for col in human_cols:
            col_data = depth[:int(h * 0.7), col]
            valid_col = col_data[np.isfinite(col_data) & (col_data > self.min_detect_range) & (col_data < self.max_detect_range)]
            if len(valid_col) > 0:
                col_depths.append(np.median(valid_col))

        if len(col_depths) == 0:
            return None, None

        human_depth = float(np.median(col_depths))

        self.get_logger().info(
            f'DEPTH detected human-shape at col {cx}, depth={human_depth:.2f}m, '
            f'cols={len(human_cols)}',
            throttle_duration_sec=2.0,
        )
        return cx, human_depth

    def _sample_depth(self, cx: int, cy: int):
        """Sample depth in a small window, return median of valid values."""
        if self.depth_image is None:
            return None
        h, w = self.depth_image.shape[:2]
        half = 5
        x0 = max(0, cx - half)
        x1 = min(w, cx + half + 1)
        y0 = max(0, cy - half)
        y1 = min(h, cy + half + 1)

        window = self.depth_image[y0:y1, x0:x1]
        valid = window[np.isfinite(window) & (window > 0)]
        if valid.size == 0:
            return None
        return float(np.median(valid))


def main(args=None):
    rclpy.init(args=args)
    node = HumanFollowerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        stop = Twist()
        node.cmd_vel_pub.publish(stop)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
