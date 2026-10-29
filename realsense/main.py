#!/usr/bin/env python3
"""Run YOLO on RealSense ROS images and publish the best box.

Run after sourcing ROS 2, with cv_bridge, OpenCV, NumPy, rclpy,
Ultralytics, CUDA/TensorRT, and best.engine available in this Python environment.
"""

import os

# Avoid Ultralytics silently installing packages while loading a TensorRT engine.
os.environ.setdefault('YOLO_AUTOINSTALL', 'False')

import cv2
import numpy as np
import rclpy
import torch
from cv_bridge import CvBridge
from geometry_msgs.msg import Vector3
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image
from ultralytics import YOLO

MODEL_PATH = '/home/ghost/realsense_ws/best.engine'
OUTPUT_TOPIC = '/bb_vals'
COLOR_TOPIC = '/camera/color/image_raw'
DEPTH_TOPIC = '/camera/aligned_depth_to_color/image_raw'
CONFIDENCE_THRESHOLD = 0.25
DEPTH_PATCH_RADIUS = 5


class RealSenseYoloPublisher(Node):
    def __init__(self):
        super().__init__('realsense_yolo_publisher')
        self.publisher = self.create_publisher(Vector3, OUTPUT_TOPIC, 10)
        self.bridge = CvBridge()
        self.color_image = None
        self.color_sequence = 0
        self.depth_image = None
        self.depth_encoding = None
        self.color_subscription = self.create_subscription(
            Image, COLOR_TOPIC, self.color_callback, qos_profile_sensor_data
        )
        self.depth_subscription = self.create_subscription(
            Image, DEPTH_TOPIC, self.depth_callback, qos_profile_sensor_data
        )
        self.get_logger().info(f'Loading TensorRT model: {MODEL_PATH}')
        if not os.path.isfile(MODEL_PATH):
            raise FileNotFoundError(f'Model not found: {MODEL_PATH}')
        if not torch.cuda.is_available():
            self.get_logger().warning(
                'PyTorch reports CUDA unavailable; best.engine requires a compatible '
                'NVIDIA TensorRT/CUDA environment and may not run here.'
            )
        self.model = YOLO(MODEL_PATH, task='detect')

    def color_callback(self, msg):
        self.color_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        self.color_sequence += 1

    def depth_callback(self, msg):
        self.depth_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
        self.depth_encoding = msg.encoding

    @staticmethod
    def get_distance(depth_image, center_x, center_y, depth_encoding):
        """Return the median valid depth in meters around the box center."""
        height, width = depth_image.shape[:2]
        if not (0 <= center_x < width and 0 <= center_y < height):
            return None

        x0 = max(0, center_x - DEPTH_PATCH_RADIUS)
        x1 = min(width, center_x + DEPTH_PATCH_RADIUS + 1)
        y0 = max(0, center_y - DEPTH_PATCH_RADIUS)
        y1 = min(height, center_y + DEPTH_PATCH_RADIUS + 1)
        depth = depth_image[y0:y1, x0:x1]
        valid = depth[depth > 0]
        if valid.size == 0:
            return None
        if depth_encoding == '16UC1':
            scale = 0.001
        elif depth_encoding == '32FC1':
            scale = 1.0
        else:
            raise ValueError(f'Unsupported depth encoding: {depth_encoding}')
        return float(np.median(valid)) * scale

    def process(self, image, depth_image, depth_encoding):
        result = self.model(
            image,
            conf=CONFIDENCE_THRESHOLD,
            augment=False,
            verbose=False,
        )[0]
        annotated = result.plot()

        if len(result.boxes) == 0:
            cv2.putText(annotated, 'No detection', (12, 30),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 180, 255), 2)
            return annotated

        confidences = result.boxes.conf.cpu().numpy()
        best_index = int(np.argmax(confidences))
        x1, y1, x2, y2 = result.boxes.xyxy.cpu().numpy()[best_index]
        x1, y1, x2, y2 = map(int, (x1, y1, x2, y2))
        center_x = (x1 + x2) // 2
        center_y = (y1 + y2) // 2
        distance_m = (
            self.get_distance(depth_image, center_x, center_y, depth_encoding)
            if depth_image is not None else None
        )

        if distance_m is None:
            label = 'Depth unavailable'
        else:
            msg = Vector3()
            msg.x = float(x1)
            msg.y = float(x2)
            msg.z = distance_m
            self.publisher.publish(msg)
            label = f'x1={x1} x2={x2} depth={distance_m:.2f} m'

        cv2.putText(annotated, label,
                     (10, 30),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
        return annotated


def main():
    rclpy.init()
    node = None
    window_name = 'RealSense YOLO (press q to quit)'

    try:
        node = RealSenseYoloPublisher()
        node.get_logger().info(
            f'Subscribed to {COLOR_TOPIC} and {DEPTH_TOPIC}; '
            f'publishing Vector3 to {OUTPUT_TOPIC} (x=box left, y=box right, z=depth m)'
        )

        cv2.namedWindow(window_name, cv2.WINDOW_NORMAL)
        processed_sequence = 0
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.01)
            if (node.color_image is not None and
                    node.color_sequence != processed_sequence):
                processed_sequence = node.color_sequence
                preview = node.process(
                    node.color_image, node.depth_image, node.depth_encoding
                )
                cv2.imshow(window_name, preview)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break
    except KeyboardInterrupt:
        pass
    finally:
        cv2.destroyAllWindows()
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
