#!/usr/bin/env python3
"""Run YOLO on a directly connected RealSense and publish the best box.

Run after sourcing ROS 2, with pyrealsense2, OpenCV, NumPy, rclpy,
Ultralytics, CUDA/TensorRT, and best.engine available in this Python environment.
"""

import os

# Avoid Ultralytics silently installing packages while loading a TensorRT engine.
os.environ.setdefault('YOLO_AUTOINSTALL', 'False')

import cv2
import numpy as np
import pyrealsense2 as rs
import rclpy
import torch
from geometry_msgs.msg import Vector3
from rclpy.node import Node
from ultralytics import YOLO

MODEL_PATH = '/home/ghost/realsense_ws/best.engine'
OUTPUT_TOPIC = '/bb_vals'
CONFIDENCE_THRESHOLD = 0.25
WIDTH = 640
HEIGHT = 480
FPS = 30
DEPTH_PATCH_RADIUS = 5


class RealSenseYoloPublisher(Node):
    def __init__(self):
        super().__init__('realsense_yolo_publisher')
        self.publisher = self.create_publisher(Vector3, OUTPUT_TOPIC, 10)
        self.get_logger().info(f'Loading TensorRT model: {MODEL_PATH}')
        if not os.path.isfile(MODEL_PATH):
            raise FileNotFoundError(f'Model not found: {MODEL_PATH}')
        if not torch.cuda.is_available():
            self.get_logger().warning(
                'PyTorch reports CUDA unavailable; best.engine requires a compatible '
                'NVIDIA TensorRT/CUDA environment and may not run here.'
            )
        self.model = YOLO(MODEL_PATH, task='detect')

    @staticmethod
    def get_distance(depth_frame, center_x, center_y, depth_scale):
        """Return the median valid depth in meters around the box center."""
        width = depth_frame.get_width()
        height = depth_frame.get_height()
        if not (0 <= center_x < width and 0 <= center_y < height):
            return None

        x0 = max(0, center_x - DEPTH_PATCH_RADIUS)
        x1 = min(width, center_x + DEPTH_PATCH_RADIUS + 1)
        y0 = max(0, center_y - DEPTH_PATCH_RADIUS)
        y1 = min(height, center_y + DEPTH_PATCH_RADIUS + 1)
        depth = np.asanyarray(depth_frame.get_data())[y0:y1, x0:x1]
        valid = depth[depth > 0]
        if valid.size == 0:
            return None
        return float(np.median(valid)) * depth_scale

    def process(self, color_frame, depth_frame, depth_scale):
        image = np.asanyarray(color_frame.get_data())
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
        distance_m = self.get_distance(
            depth_frame, center_x, center_y, depth_scale
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
    pipeline = rs.pipeline()
    started = False
    window_name = 'RealSense YOLO (press q to quit)'

    try:
        node = RealSenseYoloPublisher()
        config = rs.config()
        config.enable_stream(rs.stream.color, WIDTH, HEIGHT, rs.format.bgr8, FPS)
        config.enable_stream(rs.stream.depth, WIDTH, HEIGHT, rs.format.z16, FPS)
        profile = pipeline.start(config)
        started = True
        align_to_color = rs.align(rs.stream.color)
        depth_scale = profile.get_device().first_depth_sensor().get_depth_scale()
        node.get_logger().info(
            f'RealSense started at {WIDTH}x{HEIGHT}@{FPS}; depth scale={depth_scale}; '
            f'publishing Vector3 to {OUTPUT_TOPIC} (x=box left, y=box right, z=depth m)'
        )

        cv2.namedWindow(window_name, cv2.WINDOW_NORMAL)
        while rclpy.ok():
            frames = pipeline.wait_for_frames()
            aligned = align_to_color.process(frames)
            color_frame = aligned.get_color_frame()
            depth_frame = aligned.get_depth_frame()
            if not color_frame or not depth_frame:
                rclpy.spin_once(node, timeout_sec=0)
                continue

            preview = node.process(color_frame, depth_frame, depth_scale)
            cv2.imshow(window_name, preview)
            rclpy.spin_once(node, timeout_sec=0)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break
    except KeyboardInterrupt:
        pass
    finally:
        if started:
            pipeline.stop()
        cv2.destroyAllWindows()
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
