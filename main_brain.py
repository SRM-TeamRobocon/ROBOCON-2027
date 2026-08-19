#!/usr/bin/env python3

import math
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from std_msgs.msg import Float32, Float32MultiArray

# ============================== INDEX MAP ==============================

W1 = 0
W2 = 1
W3 = 2
ARM = 0
KFS_WRIST = 1
KFS_GRIP = 2
STAFF_WRIST1 = 3
ARUCO = 4
STAFF_GRIP1 = 5
STAFF_GRIP2 = 6
CONVEYOR = 7

# ============================== PS4 MAP ==============================

PS4_MAP = {
    "SQUARE": 0,
    "X": 1,
    "O": 2,
    "TRIANGLE": 3,
    "L1": 4,
    "R1": 5,
    "L2": 6,
    "R2": 7,
    "SHARE": 8,
    "OPTIONS": 9,
    "R3": 10,
    "L3": 11,
    "PS": 12,
    "TRACKPAD": 13
}

# ============================== HELPERS ==============================

def clamp(v, mn, mx):
    return max(mn, min(mx, v))

def apply_deadband(v, db=0.08):
    return 0.0 if abs(v) < db else v

def expo(v, e=1.5):
    return math.copysign(abs(v) ** e, v)

def get_dpad(msg):
    return {
        "UP": msg.axes[7] > 0.5,
        "DOWN": msg.axes[7] < -0.5,
        "LEFT": msg.axes[6] > 0.5,
        "RIGHT": msg.axes[6] < -0.5
    }

# ============================== MAIN NODE ==============================

class BrainNode(Node):

    def __init__(self):

        super().__init__('brain')

        # ============================== INPUT ==============================

        self.axes = [0.0] * 8
        self.buttons = [0] * 16
        self.last_buttons = [0] * 16
        self.joy_msg = None

        # ============================== DRIVE CONFIG ==============================

        self.MAX_PWM = 12000
        self.MAX_RPM = 250
        self.MAX_LINEAR = 375
        self.MAX_OMEGA = 5.0

        # ============================== YAW ==============================

        self.current_yaw = 0.0
        self.target_yaw = 0.0
        self.target_yaw_initialized = False
        self.last_yaw_error = 0.0
        self.last_manual_omega = 0.0

        self.yaw_kp = 0.08
        self.yaw_kd = 0.002

        # ============================== MODES ==============================

        # 0 -> FULL CONTROL
        # 1 -> Aruco CONTROL

        self.mode = 0

        # ============================== ARM ==============================

        self.arm_angle = 0.0
        self.arm_step_up = 15.0
        self.arm_step_down= 10.0

        # ============================== KFS ==============================

        self.kfs_wrist_toggle = False
        self.kfs_grip_toggle = False
        self.kfswrist = 40
        self.kfsgrip = 30


        self.aruco = 0

        # ============================== VALVES ==============================

        self.valve1 = 0
        self.valve2 = 0

        # ============================== STAFF ==============================

        # self.staffwrist_toggle = False
        self.staff_wrist = 80
        self.staffgrip = 30

        # ============================== DPAD EDGE ==============================

        self.last_dpad_up = False
        self.last_dpad_down = False

        # ============================== SUBSCRIBERS ==============================

        self.create_subscription(Joy, '/robot1/joy', self.joy_cb, 10)
        self.create_subscription(Float32, 'imu_yaw', self.imu_cb, 10)

        # ============================== PUBLISHERS ==============================

        self.rpm_pub = self.create_publisher(Float32MultiArray, 'wheel_rpm_cmd', 10)
        self.actuator_pub = self.create_publisher(Float32MultiArray, 'actuator_cmd', 10)

        # ============================== LOOP ==============================

        self.create_timer(0.02, self.loop)

    # ============================== CALLBACKS ==============================

    def joy_cb(self, msg):
        self.joy_msg = msg
        self.axes = msg.axes
        self.buttons = msg.buttons

    def imu_cb(self, msg):
        self.current_yaw = msg.data

    # ============================== BUTTON HELPERS ==============================

    def btn(self, name):
        return self.buttons[PS4_MAP[name]]

    def btn_edge(self, name):
        i = PS4_MAP[name]
        return self.buttons[i] and not self.last_buttons[i]

    # ============================== IK ==============================

    def ik(self, vx, vy, omega):

        R = 32.2
        r = 6.3

        # vx=-vx
        # vy=-vy
        # w=-w
        w1 = (-math.sin(0) * -vx + math.cos(0) * vy + R * omega) / r
        w2 = (-math.sin(2 * math.pi / 3) * -vx + math.cos(2 * math.pi / 3) * vy + R * omega) / r
        w3 = (-math.sin(4 * math.pi / 3) * -vx + math.cos(4 * math.pi / 3) * vy + R * omega) / r

        return (
            w1 * 60 / (2 * math.pi),
            w2 * 60 / (2 * math.pi),
            w3 * 60 / (2 * math.pi)
        )

    # ============================== MAIN LOOP ==============================

    def loop(self):

        if self.joy_msg is None:
            return

        dpad = get_dpad(self.joy_msg)

        # ============================== MODE SWITCH ==============================

        if self.btn_edge("TRACKPAD"):

            self.mode += 1

            if self.mode > 1:
                self.mode = 0

            names = ["FULL CONTROL", "Aruco ONLY"]

            print(f"\nMODE -> {names[self.mode]}\n")

        # ============================== TRANSLATION INPUT ==============================

        raw_x = expo(apply_deadband(self.axes[0]), 1.5)
        raw_y = expo(apply_deadband(self.axes[1]), 1.5)

        mag = math.sqrt(raw_x**2 + raw_y**2)

        if mag > 1.0:
            raw_x /= mag
            raw_y /= mag

        vy = raw_x * self.MAX_LINEAR
        vx = raw_y * self.MAX_LINEAR

        # ============================== ROTATION INPUT ==============================

        rot_input = expo(apply_deadband(self.axes[2]), 1.4)
        manual_omega = rot_input * self.MAX_OMEGA

        rotation_deadband = 0.2
        rotating = abs(manual_omega) > rotation_deadband

        # ============================== INITIAL YAW LOCK ==============================

        if not self.target_yaw_initialized:
            self.target_yaw = self.current_yaw
            self.target_yaw_initialized = True

        # ============================== MANUAL ROTATION ==============================

        if rotating:

            omega = manual_omega
            self.target_yaw = self.current_yaw

        # ============================== YAW HOLD PID ==============================

        else:

            yaw_error = self.target_yaw - self.current_yaw

            while yaw_error > 180:
                yaw_error -= 360

            while yaw_error < -180:
                yaw_error += 360

            yaw_der = (yaw_error - self.last_yaw_error)/0.02

            omega = yaw_error * self.yaw_kp + yaw_der * self.yaw_kd
            omega = clamp(omega, -2.2, 2.2)

            self.last_yaw_error = yaw_error

        self.last_manual_omega = manual_omega

        # ============================== IK ==============================

        r1, r2, r3 = self.ik(vx, vy, omega)

        # ============================== RPM CLAMP ==============================

        r1 = clamp(r1, -self.MAX_RPM, self.MAX_RPM)
        r2 = clamp(r2, -self.MAX_RPM, self.MAX_RPM)
        r3 = clamp(r3, -self.MAX_RPM, self.MAX_RPM)

        # ============================== RPM -> PWM ==============================

        p1 = clamp((r1 / self.MAX_RPM) * self.MAX_PWM, -self.MAX_PWM, self.MAX_PWM)
        p2 = clamp((r2 / self.MAX_RPM) * self.MAX_PWM, -self.MAX_PWM, self.MAX_PWM)
        p3 = clamp((r3 / self.MAX_RPM) * self.MAX_PWM, -self.MAX_PWM, self.MAX_PWM)

        # ============================== PUBLISH DRIVE ==============================

        drive_msg = Float32MultiArray()

        drive_msg.data = [float(r1), float(r2), float(r3)]

        self.rpm_pub.publish(drive_msg)

        # ============================== ARM CONTROL ==============================

        if self.mode in [0,1]:

            if dpad["UP"] and not self.last_dpad_up:
                self.arm_angle += self.arm_step_up

            if dpad["DOWN"] and not self.last_dpad_down:
                self.arm_angle -= self.arm_step_down

        # ============================== STAFF CONTROLS ==============================

        if self.mode in [0,1]:

            # if self.btn_edge("TRIANGLE"):
            #     self.staffwrist_toggle = not self.staffwrist_toggle

            if self.btn("SQUARE"):
                self.staffgrip += 1

            if self.btn("O"):
                self.staffgrip -= 1


        # ============================== VALVE CONTROLS ==============================

        if self.mode in [1]:
            if dpad["LEFT"]:
                self.valve1 = 1 - self.valve1
                self.valve2 = 1 - self.valve2

        # ============================== KFS CONTROLS ==============================

        if self.mode in [0]:

            if self.btn_edge("L1"):
                self.kfswrist += 35

            if self.btn_edge("R1"):
                self.kfswrist -= 25

            if self.btn_edge("R2"):
                self.kfsgrip += 30

            if self.btn_edge("L2"):
                self.kfsgrip -= 30

            self.kfswrist = clamp(self.kfswrist, 35, 145)
            self.kfsgrip = clamp(self.kfsgrip,0,100)

        # ============================== Aruco CONTROLS ==============================

        if self.mode in [1]:

            if self.btn_edge("L2"):
                self.aruco = 0

            if self.btn_edge("R2"):
                self.aruco = 180

            if self.btn_edge("L1"):
                self.aruco = 60

            if self.btn_edge("R1"):
                self.aruco = 120

            self.aruco = clamp(self.aruco, 0, 180)

        # ============================== CLAMPS ==============================

        self.staffgrip = clamp(self.staffgrip, 20, 95)

        # ============================== STAFF WRIST ==============================

        if self.mode in [0,1]:
            if self.btn("TRIANGLE"):
                self.staff_wrist -= 5 
            if self.btn("X"):
                self.staff_wrist += 5
            self.staff_wrist = clamp(self.staff_wrist,10,90)
        
        # ============================== CONVEYOR ==============================

        conveyor_speed = 0.0

        if self.mode in [0]:

            if dpad["LEFT"]:
                conveyor_speed = -1.0

            elif dpad["RIGHT"]:
                conveyor_speed = 1.0

        # ============================== ACTUATOR MESSAGE ==============================

        actuator_msg = Float32MultiArray()

        actuator_msg.data = [
            float(self.arm_angle),
            float(self.kfswrist),
            float(self.kfsgrip),
            float(self.staff_wrist),
            float(self.aruco),
            float(self.staffgrip),
            float(self.staffgrip),
            float(conveyor_speed),
            float(self.valve1),
            float(self.valve2)
        ]

        self.actuator_pub.publish(actuator_msg)

        # ============================== SAVE BUTTON STATES ==============================

        self.last_dpad_up = dpad["UP"]
        self.last_dpad_down = dpad["DOWN"]

        self.last_buttons = list(self.buttons)

# ============================== MAIN ==============================

def main(args=None):

    rclpy.init(args=args)

    node = BrainNode()

    try:
        rclpy.spin(node)

    except KeyboardInterrupt:
        node.get_logger().info("Shutting down...")

    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()