#!/usr/bin/env python3

import sys
import math
import signal
import threading
from collections import deque

import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from matplotlib.patches import Polygon
from matplotlib.widgets import TextBox, Button

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray, Bool


MAX_POINTS = 5000

FIELD_X_MAX = 6000
FIELD_Y_MAX = 11000

ROBOT_RADIUS_MM = 200

x_data = deque(maxlen=MAX_POINTS)
y_data = deque(maxlen=MAX_POINTS)

latest_x = 0.0
latest_y = 0.0
latest_angle_deg = 0.0

plot_x = 0.0
plot_y = 0.0

error_x_display = 0.0
error_y_display = 0.0
error_distance_display = 0.0

data_lock = threading.Lock()

TARGET_X_MM = 0.0
TARGET_Y_MM = 0.0
TARGET_THETA_DEG = 0.0

DEADBAND_MM = 50.0
MAX_CONTROL_DISTANCE_MM = 3000.0

MIN_CMD = 0.15
MAX_CMD = 1.0

MAX_RPM = 250.0

target_reached = False
target_set = False

startup_origin_x = None
startup_origin_y = None




class JetsonRobotController(Node):

    def __init__(self):
        super().__init__('jetson_robot_controller')

        self.odom_sub = self.create_subscription(Float32MultiArray, 'teensy_odom', self.odom_callback, 10)
        self.target_pose_sub = self.create_subscription(Float32MultiArray, '/target_pose', self.target_pose_callback, 10)

        self.rpm_pub = self.create_publisher(Float32MultiArray, 'target_rpm', 10)
        self.target_reached_pub = self.create_publisher(Bool, 'target_reached', 10)

        self.get_logger().info("Jetson Controller & Visualization node started.")

        self.control_timer = self.create_timer(0.05, self.autonomous_control)
        self.target_reached = False
        
    def publish_target_reached(self, reached):
        msg = Bool()
        msg.data = reached
        self.target_reached_pub.publish(msg)

    def target_pose_callback(self, msg):
        global TARGET_X_MM, TARGET_Y_MM, TARGET_THETA_DEG
        global target_set

        if len(msg.data) < 3:
            self.get_logger().error(
                "Invalid target pose: expected [x, y, theta]."
            )
            return

        x = float(msg.data[0])
        y = float(msg.data[1])
        theta = float(msg.data[2])

        if not all(math.isfinite(v) for v in (x, y, theta)):
            self.get_logger().error(
                "Invalid target pose: values must be finite."
            )
            return

        TARGET_X_MM = x
        TARGET_Y_MM = y
        TARGET_THETA_DEG = theta
        target_set = True

        self.target_reached = False
        self.publish_target_reached(False)

        self.get_logger().info(
            f"NEW TARGET POSE | X={x:.1f} mm, "
            f"Y={y:.1f} mm, THETA={theta:.1f} deg"
        )

    def odom_callback(self, msg):

        global latest_x, latest_y, latest_angle_deg
        global plot_x, plot_y
        global startup_origin_x, startup_origin_y

        if len(msg.data) < 3:
            return

        raw_x = float(msg.data[0])
        raw_y = float(msg.data[1])
        yaw = float(msg.data[2])

        with data_lock:

            if startup_origin_x is None:
                startup_origin_x = raw_x
                startup_origin_y = raw_y

                self.get_logger().info(
                    f"ODOMETRY START ORIGIN | X={raw_x:.1f} mm Y={raw_y:.1f} mm"
                )

            latest_x = raw_x - startup_origin_x
            latest_y = raw_y - startup_origin_y

            plot_x = latest_x
            plot_y = latest_y
            latest_angle_deg = yaw

            x_data.append(plot_x)
            y_data.append(plot_y)

    def publish_zero_rpm(self):

        rpm_msg = Float32MultiArray()
        rpm_msg.data = [0.0, 0.0, 0.0, 0.0]

        self.rpm_pub.publish(rpm_msg)

    def distance_to_speed(self, distance):

        if distance <= DEADBAND_MM:
            return 0.0

        if distance >= MAX_CONTROL_DISTANCE_MM:
            return MAX_CMD

        ratio = (distance - DEADBAND_MM) / (MAX_CONTROL_DISTANCE_MM - DEADBAND_MM)

        return MIN_CMD + ratio * (MAX_CMD - MIN_CMD)

    def autonomous_control(self):

        global TARGET_X_MM, TARGET_Y_MM
        global target_set
        global error_x_display, error_y_display, error_distance_display

        if not target_set:
            self.publish_zero_rpm()
            self.publish_target_reached(False)
            self.target_reached = False

            with data_lock:
                error_x_display = 0.0
                error_y_display = 0.0
                error_distance_display = 0.0

            return

        with data_lock:
            x = latest_x
            y = latest_y
            theta_deg = latest_angle_deg

        theta = math.radians(theta_deg)

        error_x = TARGET_X_MM - x
        error_y = TARGET_Y_MM - y
        distance = math.sqrt(error_x ** 2 + error_y ** 2)

        with data_lock:
            error_x_display = error_x
            error_y_display = error_y
            error_distance_display = distance

        if distance <= DEADBAND_MM:

            self.publish_zero_rpm()

            if not self.target_reached:
                self.get_logger().info(
                    f"TARGET REACHED | X={x:.1f} mm Y={y:.1f} mm "
                    f"| Error={distance:.1f} mm"
                )

            self.target_reached = True
            self.publish_target_reached(True)
            return

        self.target_reached = False
        self.publish_target_reached(False)

        forward_error = error_x * math.cos(theta) + error_y * math.sin(theta)
        strafe_error = -error_x * math.sin(theta) + error_y * math.cos(theta)

        direction_forward = forward_error / distance
        direction_strafe = strafe_error / distance

        speed = self.distance_to_speed(distance)

        forward = -direction_forward * speed
        strafe = -direction_strafe * speed
        turn = 0.0

        fl = forward + strafe + turn
        fr = forward - strafe - turn
        bl = forward - strafe + turn
        br = forward + strafe - turn

        # fl = forward + strafe + turn;
        # fr = forward + strafe + turn;
        # bl = forward - strafe + turn;
        # br = forward + strafe - turn;

        max_magnitude = max(abs(fl), abs(fr), abs(bl), abs(br), 1.0)

        fl /= max_magnitude
        fr /= max_magnitude
        bl /= max_magnitude
        br /= max_magnitude

        fl_rpm = -fl * MAX_RPM
        fr_rpm = fr * MAX_RPM
        bl_rpm = -bl * MAX_RPM
        br_rpm = br * MAX_RPM

        rpm_msg = Float32MultiArray()
        rpm_msg.data = [float(fl_rpm), float(fr_rpm), float(bl_rpm), float(br_rpm)]

        self.rpm_pub.publish(rpm_msg)


fig, ax = plt.subplots(figsize=(8, 10))

path_line, = ax.plot([], [], "b-", linewidth=1.5, label="Odometry Path")

robot_triangle = Polygon(
    [[0, 0], [0, 0], [0, 0]],
    facecolor="red",
    edgecolor="darkred",
    zorder=5,
    label="Robot"
)

ax.add_patch(robot_triangle)

ax.set_xlabel("X (mm)")
ax.set_ylabel("Y (mm)")
ax.set_title("ROBOCON - Real-Time Robot Path & Orientation Tracker", pad=18)

ax.legend(loc="upper right")
ax.grid(True, linestyle="--", alpha=0.5)

ax.set_xlim(-FIELD_X_MAX, FIELD_X_MAX)
ax.set_ylim(-FIELD_Y_MAX, FIELD_Y_MAX)
ax.set_aspect("equal", adjustable="box")


status_text = fig.text(
    0.5,
    0.985,
    "WAITING FOR TARGET",
    ha="center",
    va="top",
    fontsize=14,
    fontweight="bold"
)


axbox_x = plt.axes([0.18, 0.075, 0.20, 0.04])
axbox_y = plt.axes([0.48, 0.075, 0.20, 0.04])

text_box_x = TextBox(axbox_x, "Target X: ", initial="0")
text_box_y = TextBox(axbox_y, "Target Y: ", initial="0")


button_ax = plt.axes([0.72, 0.075, 0.16, 0.04])
set_target_button = Button(button_ax, "SET TARGET")


error_text = fig.text(
    0.5,
    0.047,
    "Error X: 0.0 mm   Error Y: 0.0 mm   Distance: 0.0 mm",
    ha="center",
    va="bottom",
    fontsize=11,
    bbox=dict(boxstyle="round", facecolor="white", alpha=0.8)
)


position_text = fig.text(
    0.5,
    0.018,
    "X: 0.0 mm   Y: 0.0 mm   Angle: 0.0 deg   Target: (0.0, 0.0)",
    ha="center",
    va="bottom",
    fontsize=11,
    bbox=dict(boxstyle="round", facecolor="white", alpha=0.8)
)

fig.subplots_adjust(bottom=0.15, top=0.92)


def set_target(event):

    global TARGET_X_MM, TARGET_Y_MM, target_set

    try:
        new_x = float(text_box_x.text)
        new_y = float(text_box_y.text)

        TARGET_X_MM = new_x
        TARGET_Y_MM = new_y
        target_set = True

        ros_node.target_reached = False
        ros_node.publish_target_reached(False)

        ros_node.get_logger().info(
            f"NEW TARGET | X={TARGET_X_MM:.1f} mm Y={TARGET_Y_MM:.1f} mm"
        )

    except ValueError:

        ros_node.get_logger().error(
            "Invalid target. Enter numeric X and Y values."
        )


set_target_button.on_clicked(set_target)


def update(frame):

    with data_lock:

        local_x_data = list(x_data)
        local_y_data = list(y_data)

        current_x = plot_x
        current_y = plot_y
        current_angle = latest_angle_deg

        current_error_x = error_x_display
        current_error_y = error_y_display
        current_error_distance = error_distance_display

    if local_x_data and local_y_data:
        path_line.set_data(local_x_data, local_y_data)

    theta = math.radians(current_angle)

    x_nose = current_x + ROBOT_RADIUS_MM * math.cos(theta)
    y_nose = current_y + ROBOT_RADIUS_MM * math.sin(theta)

    rear_radius = ROBOT_RADIUS_MM * 0.7

    x_left = current_x + rear_radius * math.cos(theta + math.radians(135))
    y_left = current_y + rear_radius * math.sin(theta + math.radians(135))

    x_right = current_x + rear_radius * math.cos(theta - math.radians(135))
    y_right = current_y + rear_radius * math.sin(theta - math.radians(135))

    robot_triangle.set_xy([
        [x_nose, y_nose],
        [x_left, y_left],
        [x_right, y_right]
    ])

    if ros_node.target_reached:
        status_text.set_text("TARGET REACHED")
    elif target_set:
        status_text.set_text("MOVING TO TARGET")
    else:
        status_text.set_text("WAITING FOR TARGET")

    error_text.set_text(
        f"Error X: {current_error_x:.1f} mm   "
        f"Error Y: {current_error_y:.1f} mm   "
        f"Distance: {current_error_distance:.1f} mm"
    )

    position_text.set_text(
        f"X: {current_x:.1f} mm   "
        f"Y: {current_y:.1f} mm   "
        f"Angle: {current_angle:.1f} deg   "
        f"Target: ({TARGET_X_MM:.1f}, {TARGET_Y_MM:.1f})"
    )

    return path_line, robot_triangle, status_text, error_text, position_text


def spin_ros_node(node):

    try:
        rclpy.spin(node)
    except Exception as e:
        print(f"[ROS Thread Error]: {e}")


if __name__ == '__main__':

    rclpy.init(args=sys.argv)

    ros_node = JetsonRobotController()

    ros_thread = threading.Thread(
        target=spin_ros_node,
        args=(ros_node,),
        daemon=True
    )

    ros_thread.start()

    ani = FuncAnimation(
        fig,
        update,
        interval=50,
        cache_frame_data=False,
        blit=False
    )

    signal.signal(signal.SIGINT, signal.SIG_DFL)

    try:
        plt.show()
    finally:
        ros_node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()
