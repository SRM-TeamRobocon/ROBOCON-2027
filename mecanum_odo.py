#!/usr/bin/env python3

import sys
import math
import signal
import threading
from collections import deque

import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from matplotlib.patches import Polygon

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from std_msgs.msg import Float32MultiArray


MAX_POINTS = 5000
FIELD_X_MAX = 6000
FIELD_Y_MAX = 10000
ROBOT_RADIUS_MM = 200

x_data = deque(maxlen=MAX_POINTS)
y_data = deque(maxlen=MAX_POINTS)

latest_x, latest_y, latest_angle_deg = 0.0, 0.0, 0.0
data_lock = threading.Lock()


class JetsonRobotController(Node):

    def __init__(self):
        super().__init__('jetson_robot_controller')
        self.joy_sub = self.create_subscription(Joy, 'joy', self.joy_callback, 10)
        self.odom_sub = self.create_subscription(Float32MultiArray, 'teensy_odom', self.odom_callback, 10)
        self.power_pub = self.create_publisher(Float32MultiArray, 'target_rpm', 10)
        self.last_joy_msg_time = 0.0
        self.get_logger().info("Jetson Mecanum Controller & Visualization node started.")

    def joy_callback(self, msg):

        self.last_joy_msg_time = self.get_clock().now().nanoseconds / 1e9

        lx = msg.axes[0] if len(msg.axes) > 0 else 0.0
        ly = msg.axes[1] if len(msg.axes) > 1 else 0.0
        rx = msg.axes[3] if len(msg.axes) > 3 else (msg.axes[2] if len(msg.axes) > 2 else 0.0)

        raw_lx = lx * 127.0
        raw_ly = ly * 127.0
        raw_rx = rx * 127.0

        DEADZONE = 0

        joy_lx = raw_lx if abs(raw_lx) >= DEADZONE else 0.0
        joy_ly = raw_ly if abs(raw_ly) >= DEADZONE else 0.0
        joy_rx = raw_rx if abs(raw_rx) >= DEADZONE else 0.0

        forward = joy_ly
        strafe = joy_lx
        turn = joy_rx

        # Mecanum inverse kinematics
        fl = (forward + strafe + turn) / 127.0
        fr = (forward - strafe - turn) / 127.0
        bl = (forward - strafe + turn) / 127.0
        br = (forward + strafe - turn) / 127.0

        max_power = max(abs(fl), abs(fr), abs(bl), abs(br))

        if max_power > 1.0:
            fl /= max_power
            fr /= max_power
            bl /= max_power
            br /= max_power

        power_msg = Float32MultiArray()
        power_msg.data = [float(fl), float(fr), float(bl), float(br)]

        self.power_pub.publish(power_msg)

    def odom_callback(self, msg):

        global latest_x, latest_y, latest_angle_deg

        if len(msg.data) >= 3:
            with data_lock:
                latest_x = msg.data[0]
                latest_y = msg.data[1]
                latest_angle_deg = msg.data[2]

                x_data.append(latest_x)
                y_data.append(latest_y)


fig, ax = plt.subplots(figsize=(8, 10))

path_line, = ax.plot([], [], "b-", linewidth=1.5, label="Odometry Path")

robot_triangle = Polygon([[0, 0], [0, 0], [0, 0]], facecolor="red", edgecolor="darkred", zorder=5, label="Robot (Triangle)")
ax.add_patch(robot_triangle)

ax.set_xlabel("X (mm)")
ax.set_ylabel("Y (mm)")
ax.set_title("ROBOCON - Real-Time Robot Path & Orientation Tracker")
ax.legend(loc="upper right")
ax.grid(True, linestyle="--", alpha=0.5)

ax.set_xlim(0, FIELD_X_MAX)
ax.set_ylim(0, FIELD_Y_MAX)
ax.set_aspect("equal", adjustable="box")

position_text = fig.text(
    0.5,
    0.02,
    "X: 0.0 mm   Y: 0.0 mm   Angle: 0.0 deg",
    ha="center",
    va="bottom",
    fontsize=12,
    bbox=dict(boxstyle="round", facecolor="white", alpha=0.8)
)

fig.subplots_adjust(bottom=0.15)


def update(frame):

    global latest_x, latest_y, latest_angle_deg

    with data_lock:
        local_x_data = list(x_data)
        local_y_data = list(y_data)
        current_x = latest_x
        current_y = latest_y
        current_angle = latest_angle_deg

    if local_x_data and local_y_data:

        path_line.set_data(local_x_data, local_y_data)

        theta = math.radians(current_angle)

        x_nose = current_x + ROBOT_RADIUS_MM * math.cos(theta)
        y_nose = current_y + ROBOT_RADIUS_MM * math.sin(theta)

        x_left = current_x + (ROBOT_RADIUS_MM * 0.7) * math.cos(theta + math.radians(135))
        y_left = current_y + (ROBOT_RADIUS_MM * 0.7) * math.sin(theta + math.radians(135))

        x_right = current_x + (ROBOT_RADIUS_MM * 0.7) * math.cos(theta - math.radians(135))
        y_right = current_y + (ROBOT_RADIUS_MM * 0.7) * math.sin(theta - math.radians(135))

        robot_triangle.set_xy([[x_nose, y_nose], [x_left, y_left], [x_right, y_right]])

        position_text.set_text(f"X: {current_x:.1f} mm   Y: {current_y:.1f} mm   Angle: {current_angle:.1f} deg")

        fig.canvas.draw_idle()

    return path_line, robot_triangle, position_text


def spin_ros_node(node):

    try:
        rclpy.spin(node)
    except Exception as e:
        print(f"[ROS Thread Error]: {e}")


if __name__ == '__main__':

    rclpy.init(args=sys.argv)

    ros_node = JetsonRobotController()

    ros_thread = threading.Thread(target=spin_ros_node, args=(ros_node,), daemon=True)
    ros_thread.start()

    ani = FuncAnimation(fig, update, interval=50, cache_frame_data=False, blit=False)

    signal.signal(signal.SIGINT, signal.SIG_DFL)

    try:
        plt.show()
    finally:
        ros_node.destroy_node()
        rclpy.shutdown()