#!/usr/bin/env python3

import sys
import math
import signal
import threading
from collections import deque

import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from matplotlib.patches import Polygon
from matplotlib.widgets import TextBox

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from std_msgs.msg import Float32MultiArray

# =============================================================================
# CONFIGURATION
# =============================================================================

MAX_POINTS = 5000

FIELD_X_MAX = 6000
FIELD_Y_MAX = 10000

ROBOT_RADIUS_MM = 200

# Maximum target speed for each wheel
MAX_RPM = 250.0

# Joystick deadzone after scaling to [-127, 127]
DEADZONE = 0


# =============================================================================
# SHARED ODOMETRY DATA
# =============================================================================

x_data = deque(maxlen=MAX_POINTS)
y_data = deque(maxlen=MAX_POINTS)

latest_x = 0.0
latest_y = 0.0
latest_angle_deg = 0.0

odom_origin_x = None
odom_origin_y = None
odom_origin_angle = None

data_lock = threading.Lock()


# =============================================================================
# ROS 2 CONTROLLER NODE
# =============================================================================


class JetsonRobotController(Node):

    def __init__(self):
        super().__init__("jetson_robot_controller")
        # YAW CORRECTION
        self.current_yaw = 0.0
        self.target_yaw = 0.0
        self.target_yaw_initialized = False
        self.last_yaw_error = 0.0

        self.yaw_kp = 0.0
        self.yaw_kd = 0.0

        # Subscribers
        self.joy_sub = self.create_subscription(Joy, "joy", self.joy_callback, 10)

        self.odom_sub = self.create_subscription(
            Float32MultiArray, "teensy_odom", self.odom_callback, 10
        )

        # Publishes actual target RPM
        # Wheel order: [FL, FR, BL, BR]
        self.rpm_pub = self.create_publisher(Float32MultiArray, "target_rpm", 10)

        self.get_logger().info("Jetson Controller & Visualization node started.")

    # =========================================================================
    # JOYSTICK CALLBACK + MECANUM IK
    # =========================================================================

    # def joy_callback(self, msg):
    #     lx = msg.axes[0] if len(msg.axes) > 0 else 0.0
    #     ly = msg.axes[1] if len(msg.axes) > 1 else 0.0
    #     rx = msg.axes[3] if len(msg.axes) > 3 else 0.0

    #     def axis_to_255(axis):
    #         if abs(axis) < 0.08:
    #             axis = 0.0
    #         return int(round((axis + 1.0) * 127.5))

    #     lx_255 = axis_to_255(lx)
    #     ly_255 = axis_to_255(ly)
    #     rx_255 = axis_to_255(rx)

    #     strafe = lx_255 - 127
    #     forward = ly_255 - 127
    #     turn = rx_255 - 127

    #     DEADZONE = 5

    #     if abs(strafe) <= DEADZONE:
    #         strafe = 0

    #     if abs(forward) <= DEADZONE:
    #         forward = 0

    #     if abs(turn) <= DEADZONE:
    #         turn = 0

    #     if forward == 0 and strafe == 0 and turn == 0:
    #         rpm_msg = Float32MultiArray()
    #         rpm_msg.data = [0.0, 0.0, 0.0, 0.0]
    #         self.rpm_pub.publish(rpm_msg)
    #         return

    #     fl = forward + strafe + turn
    #     fr = forward - strafe - turn
    #     bl = forward - strafe + turn
    #     br = forward + strafe - turn

    #     max_magnitude = max(
    #         abs(fl),
    #         abs(fr),
    #         abs(bl),
    #         abs(br),
    #         127
    #     )

    #     fl_rpm = -(fl / max_magnitude) * MAX_RPM
    #     fr_rpm = (fr / max_magnitude) * MAX_RPM
    #     bl_rpm = -(bl / max_magnitude) * MAX_RPM
    #     br_rpm = (br / max_magnitude) * MAX_RPM

    #     rpm_msg = Float32MultiArray()
    #     rpm_msg.data = [
    #         float(fl_rpm),
    #         float(fr_rpm),
    #         float(bl_rpm),
    #         float(br_rpm)
    #     ]

    #     self.rpm_pub.publish(rpm_msg)

    def joy_callback(self, msg):

        lx = -msg.axes[0] if len(msg.axes) > 0 else 0.0
        ly = -msg.axes[1] if len(msg.axes) > 1 else 0.0
        rx = msg.axes[2] if len(msg.axes) > 2 else 0.0

        DEADZONE = 0.08

        if abs(lx) < DEADZONE:
            lx = 0.0

        if abs(ly) < DEADZONE:
            ly = 0.0

        if abs(rx) < DEADZONE:
            rx = 0.0

        lx_255 = int(round((lx + 1.0) * 127.5))
        ly_255 = int(round((ly + 1.0) * 127.5))
        rx_255 = int(round((rx + 1.0) * 127.5))

        lx_255 = max(0, min(255, lx_255))
        ly_255 = max(0, min(255, ly_255))
        rx_255 = max(0, min(255, rx_255))

        strafe = lx_255 - 127
        forward = ly_255 - 127
        turn = rx_255 - 127

        # ============================== YAW CORRECTION ==============================

        with data_lock:
            current_yaw = latest_angle_deg

        if not self.target_yaw_initialized:
            self.target_yaw = current_yaw
            self.target_yaw_initialized = True

        if abs(turn) > 1:

            # Manual rotation
            self.target_yaw = current_yaw

        else:

            yaw_error = self.target_yaw - current_yaw

            while yaw_error > 180:
                yaw_error -= 360

            while yaw_error < -180:
                yaw_error += 360

            yaw_derivative = yaw_error - self.last_yaw_error

            yaw_correction = yaw_error * self.yaw_kp + yaw_derivative * self.yaw_kd

            yaw_correction = max(-50, min(50, yaw_correction))

            turn = int(yaw_correction)

            self.last_yaw_error = yaw_error

        # print(
        #     "RAW:",
        #     lx, ly, rx,
        #     "MAPPED:",
        #     lx_255, ly_255, rx_255,
        #     "MOTION:",
        #     strafe, forward, turn
        # )

        if abs(strafe) <= 5:
            strafe = 0

        if abs(forward) <= 5:
            forward = 0

        if abs(rx_255 - 127) <= 5 and abs(turn) <= 1:
            turn = 0

        if forward == 0 and strafe == 0 and turn == 0:
            self.rpm_pub.publish(Float32MultiArray(data=[0.0, 0.0, 0.0, 0.0]))
            return

        # fl = forward + strafe + turn
        # fr = forward - strafe - turn
        # bl = forward - strafe + turn
        # br = forward + strafe - turn

        fl = forward - strafe + turn
        bl = forward + strafe + turn
        br = forward - strafe - turn
        fr = forward + strafe - turn

        max_value = max(abs(fl), abs(fr), abs(bl), abs(br), 127)

        scale = MAX_RPM / max_value

        rpm_msg = Float32MultiArray()

        rpm_msg.data = [
            float(-fl * scale),
            float(fr * scale),
            float(-bl * scale),
            float(br * scale),
        ]

        self.rpm_pub.publish(rpm_msg)

    # =========================================================================
    # ODOMETRY CALLBACK
    # =========================================================================

    def odom_callback(self, msg):

        global latest_x, latest_y, latest_angle_deg
        global odom_origin_x, odom_origin_y, odom_origin_angle

        if len(msg.data) >= 3:

            with data_lock:

                if odom_origin_x is None:
                    odom_origin_x = msg.data[0]
                    odom_origin_y = msg.data[1]
                    odom_origin_angle = msg.data[2]

                    latest_x = 0.0
                    latest_y = 0.0
                    latest_angle_deg = 0.0

                else:
                    latest_x = msg.data[0] - odom_origin_x
                    latest_y = msg.data[1] - odom_origin_y

                    latest_angle_deg = msg.data[2] - odom_origin_angle

                    while latest_angle_deg > 180:
                        latest_angle_deg -= 360

                    while latest_angle_deg < -180:
                        latest_angle_deg += 360

                x_data.append(latest_x)
                y_data.append(latest_y)


# =============================================================================
# MATPLOTLIB VISUALIZATION
# =============================================================================

fig, ax = plt.subplots(figsize=(8, 10))

# ============================== YAW PID UI ==============================

kp_ax = plt.axes([0.15, 0.93, 0.12, 0.04])
kd_ax = plt.axes([0.40, 0.93, 0.12, 0.04])

kp_box = TextBox(kp_ax, "Yaw Kp: ", initial="1.2")
kd_box = TextBox(kd_ax, "Yaw Kd: ", initial="0.0")


def update_kp(text):
    try:
        ros_node.yaw_kp = float(text)
        print(f"Yaw Kp = {ros_node.yaw_kp}")
    except ValueError:
        print("Invalid Kp")


def update_kd(text):
    try:
        ros_node.yaw_kd = float(text)
        print(f"Yaw Kd = {ros_node.yaw_kd}")
    except ValueError:
        print("Invalid Kd")


kp_box.on_submit(update_kp)
kd_box.on_submit(update_kd)

(path_line,) = ax.plot([], [], "b-", linewidth=1.5, label="Odometry Path")

robot_triangle = Polygon(
    [[0, 0], [0, 0], [0, 0]],
    facecolor="red",
    edgecolor="darkred",
    zorder=5,
    label="Robot",
)

ax.add_patch(robot_triangle)

ax.set_xlabel("X (mm)")
ax.set_ylabel("Y (mm)")
ax.set_title("ROBOCON - Real-Time Robot Path & Orientation Tracker")

ax.legend(loc="upper right")
ax.grid(True, linestyle="--", alpha=0.5)

ax.set_xlim(-FIELD_X_MAX, FIELD_X_MAX)
ax.set_ylim(-FIELD_Y_MAX, FIELD_Y_MAX)
ax.set_aspect("equal", adjustable="box")

position_text = fig.text(
    0.5,
    0.02,
    "X: 0.0 mm   Y: 0.0 mm   Angle: 0.0 deg",
    ha="center",
    va="bottom",
    fontsize=12,
    bbox=dict(boxstyle="round", facecolor="white", alpha=0.8),
)

fig.subplots_adjust(bottom=0.15)


# =============================================================================
# VISUALIZATION UPDATE
# =============================================================================


def update(frame):

    with data_lock:

        local_x_data = list(x_data)
        local_y_data = list(y_data)

        current_x = latest_x
        current_y = latest_y
        current_angle = latest_angle_deg

    if local_x_data and local_y_data:

        # Update odometry path
        path_line.set_data(local_x_data, local_y_data)

        # Robot orientation
        theta = math.radians(current_angle)

        # Front of robot
        x_nose = current_x + ROBOT_RADIUS_MM * math.cos(theta)
        y_nose = current_y + ROBOT_RADIUS_MM * math.sin(theta)

        # Rear corners
        rear_radius = ROBOT_RADIUS_MM * 0.7

        x_left = current_x + rear_radius * math.cos(theta + math.radians(135))
        y_left = current_y + rear_radius * math.sin(theta + math.radians(135))

        x_right = current_x + rear_radius * math.cos(theta - math.radians(135))
        y_right = current_y + rear_radius * math.sin(theta - math.radians(135))

        robot_triangle.set_xy([[x_nose, y_nose], [x_left, y_left], [x_right, y_right]])

        position_text.set_text(
            f"X: {current_x:.1f} mm   "
            f"Y: {current_y:.1f} mm   "
            f"Angle: {current_angle:.1f} deg   "
            f"Target: {ros_node.target_yaw:.1f} deg"
        )

    return path_line, robot_triangle, position_text


# =============================================================================
# ROS SPIN THREAD
# =============================================================================


def spin_ros_node(node):

    try:
        rclpy.spin(node)

    except Exception as e:
        print(f"[ROS Thread Error]: {e}")


# =============================================================================
# MAIN
# =============================================================================

if __name__ == "__main__":

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

        if rclpy.ok():
            rclpy.shutdown()
