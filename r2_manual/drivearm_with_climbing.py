#!/usr/bin/env python3

#     _  _  _  _  _  _            _     _  _  _     _           _                _  _  _   _  _  _  _       _  _  _    _           _  _              _  _  _  _           _    
#    (_)(_)(_)(_)(_)(_)          (_) _ (_)(_)(_) _ (_)       _ (_)              (_)(_)(_)_(_)(_)(_)(_)_  _ (_)(_)(_) _(_)_       _(_)(_)            (_)(_)(_)(_) _       (_)   
#    (_)            (_)          (_)(_)         (_)(_)    _ (_)                    (_)  (_)          (_)(_)         (_) (_)_   _(_)  (_)               (_)   (_)(_)_     (_)   
#    (_) _  _       (_)          (_)(_)            (_) _ (_)                       (_)  (_)          (_)(_)               (_)_(_)    (_)               (_)   (_)  (_)_   (_)   
#    (_)(_)(_)      (_)          (_)(_)            (_)(_) _                        (_)  (_)          (_)(_)                 (_)      (_)               (_)   (_)    (_)_ (_)   
#    (_)            (_)          (_)(_)          _ (_)   (_) _              _      (_)  (_)          (_)(_)          _      (_)      (_)               (_)   (_)      (_)(_)   
#    (_)            (_)_  _  _  _(_)(_) _  _  _ (_)(_)      (_) _          (_)  _  (_)  (_)_  _  _  _(_)(_) _  _  _ (_)     (_)      (_) _  _  _  _  _ (_) _ (_)         (_)   
#    (_)              (_)(_)(_)(_)     (_)(_)(_)   (_)         (_)          (_)(_)(_)     (_)(_)(_)     (_)(_)(_)        (_)      (_)(_)(_)(_)(_)(_)(_)(_)(_)            (_)   

import os
os.environ.setdefault("FASTDDS_BUILTIN_TRANSPORTS", "UDPv4")

import time
import threading
import numpy as np
import pygame
import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, Float32MultiArray, Int32MultiArray, String

# ================= DRIVE & ARM PARAMETERS =================
MAX_RPM = 700.0
CONTROL_HZ = 100.0

L1 = 40.0
L2 = 35.0

CROSS_BTN = 0      # PS4 Cross button index
TRIANGLE_BTN = 3   # PS4 Triangle button index


# ================= HELPERS =================
def clamp(val, min_val, max_val):
    return max(min_val, min(max_val, val))


def is_valid_workspace(x, y):
    if not (-75 <= x <= 75 and -75 <= y <= 75):
        return False

    if (-70 <= x <= 0 and -70 <= y <= 0):
        return False

    if (0 <= x <= 70 and -70 <= y <= -30):
        return False

    return True


# ================= IK =================
def computeIK(x, y):
    r = np.sqrt(x * x + y * y)

    if r > (L1 + L2) or r < abs(L1 - L2):
        return None

    cos_t2 = (
        (x * x + y * y - L1 * L1 - L2 * L2)
        / (2 * L1 * L2)
    )
    cos_t2 = clamp(cos_t2, -1.0, 1.0)

    if x < 35:
        t2 = np.arccos(cos_t2)
    else:
        t2 = -np.arccos(cos_t2)

    k1 = L1 + L2 * np.cos(t2)
    k2 = L2 * np.sin(t2)

    t1 = np.arctan2(y, x) - np.arctan2(k2, k1)

    return t1, -t2


# ================= ROS NODE =================
class OmniController(Node):
    def __init__(self):
        super().__init__('omni_controller')

        # ================= PUBLISHERS =================
        self.publisher = self.create_publisher(
            Float32MultiArray,
            'tuning_commands',
            10
        )

        self.arm_pub = self.create_publisher(
            Int32MultiArray,
            'arm_angles',
            10
        )

        self.climb_state_pub = self.create_publisher(
            String,
            '/climb_state',
            10
        )

        self.climb_status_pub = self.create_publisher(
            Bool,
            'climb_status',
            10
        )

        # ================= PS4 DRIVE STATE =================
        self.prev_btn_cross = False
        self.climb_status = False
        self.prev_btn_triangle = False
        self.target_rpms = [0.0, 0.0, 0.0, 0.0]

        # ================= ARM SEQUENCE STATE =================
        self.state_lock = threading.Lock()
        self.sequence_running = False
        self.final_idle_delay = 1.0

        # ================= PICKUPPLACE SEQUENCE =================
        # format: [x, y, s1, s2, delay_after_step]
        # self.pickupplace_sequence = [
        #     [75, 0, 100, 120, 1.0],
        #     [0, 75, 100, 120, 2.0],
        #     [-29, 55, 120, 120, 2.0],
        #     [-29, 55, 120, 75, 2.0],
        #     [0, 75, 120, 75, 2.0],
        #     [75, 0, 120, 75, 2.0],
        #     [40, -20, 120, 75, 2.0],
        #     [40, -20, 120, 130, 1.5],
        #     [75, 0, 100, 130, 2.0]
        # ]
        self.pickupplace_sequence = [
            # [0, 75, 100, 120, 1.0],
            [75, 0, 100, 120, 2.0],
            [55, -29, 120, 120, 2.0],
            [55, -29, 120, 75, 2.0],
            [75, 0, 120, 75, 2.0],
            [0, 75, 120, 75, 2.0],
            [-20, 40, 120, 75, 2.0],
            [-20, 40, 120, 130, 1.5],
            [0, 75, 100, 130, 2.0]
        ]

    # ==========================================================
    # ARM STARTUP INITIALIZATION (0, 0 DEGREES)
    # ==========================================================
    def initialize_arm(self, s1=100, s2=140):
        for _ in range(20):
            rclpy.spin_once(self, timeout_sec=0.05)
            if self.arm_pub.get_subscription_count() > 0:
                break

        self.publish_angles(0.0, 0.0, s1, s2)
        rclpy.spin_once(self, timeout_sec=0.0)
        self.get_logger().info("Initialized arm at 0 deg, 0 deg")

    # ==========================================================
    # OMNI DRIVE KINEMATICS & COMMANDS
    # ==========================================================
    def compute_omni_x_kinematics(self, joy_x, joy_y, joy_w):
        val = [
            -joy_x + joy_y - joy_w,
             joy_x + joy_y - joy_w,
             joy_x + joy_y + joy_w,
            -joy_x + joy_y + joy_w
        ]

        peak = max(abs(v) for v in val)

        if peak > 100.0:
            val = [v * 100.0 / peak for v in val]

        return [(v / 100.0) * MAX_RPM for v in val]

    def publish_commands(self):
        msg = Float32MultiArray()

        msg.data = [
            float(self.target_rpms[0]),
            float(self.target_rpms[1]),
            float(self.target_rpms[2]),
            float(self.target_rpms[3]),

            9.0, 140.0, 0.0,
            9.0, 140.0, 0.0,
            9.0, 140.0, 0.0,
            9.0, 140.0, 0.0,

            1.0,
            1.0,
            1.0,
            1.0
        ]
        self.publisher.publish(msg)

    def publish_climb_state(self, state):
        msg = String()
        msg.data = state
        self.climb_state_pub.publish(msg)

    def publish_climb_status(self, status):
        msg = Bool()
        msg.data = bool(status)
        self.climb_status_pub.publish(msg)

    # ==========================================================
    # SEQUENCE TRIGGER & EXECUTION
    # ==========================================================
    def trigger_pickupplace(self):
        with self.state_lock:
            if self.sequence_running:
                return False
            self.sequence_running = True

        self.publish_climb_state("pickupplace")
        self.get_logger().info("Starting PICKUPPLACE sequence")
        threading.Thread(
            target=self.run_pickupplace_sequence,
            daemon=True
        ).start()
        return True

    def publish_angles(self, t1_deg, t2_deg, s1, s2):
        msg = Int32MultiArray()

        a1 = int(round(t1_deg))
        a2 = -int(round(t2_deg))
        a1 = 0 if a1 == 0 else a1
        a2 = 0 if a2 == 0 else a2

        msg.data = [
            a1,
            a2,
            int(round(s1)),
            int(round(s2))
        ]

        self.arm_pub.publish(msg)

        self.get_logger().info(
            f"Published to arm_angles: t1={a1}, t2={a2}, s1={s1}, s2={s2}"
        )

    def run_pickupplace_sequence(self):
        try:
            for step in self.pickupplace_sequence:
                x, y, s1, s2, delay_time = step

                if not is_valid_workspace(x, y):
                    self.get_logger().error(
                        f"Invalid workspace point: {x}, {y}"
                    )
                    continue

                result = computeIK(x, y)

                if result is None:
                    self.get_logger().error(
                        f"Unreachable point: {x}, {y}"
                    )
                    continue

                t1, t2 = result
                t1_deg = np.degrees(t1)
                t2_deg = np.degrees(t2)

                self.publish_angles(t1_deg, t2_deg, s1, s2)

                self.get_logger().info(f"Waiting {delay_time} sec")
                time.sleep(delay_time)

            time.sleep(self.final_idle_delay)
            self.publish_climb_state("idle")
            self.get_logger().info("Pickupplace sequence complete")
        finally:
            with self.state_lock:
                self.sequence_running = False


# ================= MAIN =================
def main():
    rclpy.init()

    node = OmniController()

    # Initialize arm angles to 0°, 0° on startup
    node.initialize_arm()

    pygame.init()
    pygame.joystick.init()

    joystick = None

    if pygame.joystick.get_count() > 0:
        joystick = pygame.joystick.Joystick(0)
        joystick.init()

        print("PS4 Controller Connected")
        print(f"Controller: {joystick.get_name()}")
        print(f"Axes: {joystick.get_numaxes()}")
        print(f"Buttons: {joystick.get_numbuttons()}")
    else:
        print("No PS4 Controller Found")

    clock = pygame.time.Clock()
    running = True

    try:
        while running:
            pygame.event.pump()

            rclpy.spin_once(node, timeout_sec=0.0)

            if joystick is not None:
                lx = joystick.get_axis(0)
                ly = joystick.get_axis(1)

                if joystick.get_numaxes() > 2:
                    rx = joystick.get_axis(2)
                else:
                    rx = 0.0

                joy_x = -lx * 100.0 if abs(lx) > 0.05 else 0.0
                joy_y = +ly * 100.0 if abs(ly) > 0.05 else 0.0
                joy_w = rx * 100.0 if abs(rx) > 0.05 else 0.0

                node.target_rpms = node.compute_omni_x_kinematics(
                    joy_x,
                    joy_y,
                    joy_w
                )

                num_buttons = joystick.get_numbuttons()

                btn_cross = (
                    joystick.get_button(CROSS_BTN)
                    if num_buttons > CROSS_BTN
                    else 0
                )

                if btn_cross and not node.prev_btn_cross:
                    node.climb_status = not node.climb_status
                    node.publish_climb_status(node.climb_status)
                    print(
                        f"\nCross pressed: climb_status is {node.climb_status}"
                    )

                node.prev_btn_cross = bool(btn_cross)

                btn_triangle = (
                    joystick.get_button(TRIANGLE_BTN)
                    if num_buttons > TRIANGLE_BTN
                    else 0
                )

                if btn_triangle and not node.prev_btn_triangle:
                    node.trigger_pickupplace()

                node.prev_btn_triangle = bool(btn_triangle)

                print(
                    f"\r"
                    f"X: {joy_x:+6.1f}% | "
                    f"Y: {joy_y:+6.1f}% | "
                    f"W: {joy_w:+6.1f}% | "
                    f"RPM: "
                    f"{node.target_rpms[0]:+6.1f}, "
                    f"{node.target_rpms[1]:+6.1f}, "
                    f"{node.target_rpms[2]:+6.1f}, "
                    f"{node.target_rpms[3]:+6.1f}",
                    end="",
                    flush=True
                )

            else:
                node.target_rpms = [0.0, 0.0, 0.0, 0.0]

            node.publish_commands()

            clock.tick(CONTROL_HZ)

    except KeyboardInterrupt:
        pass

    finally:
        print("\nStopping controller...")

        node.target_rpms = [0.0, 0.0, 0.0, 0.0]

        for _ in range(5):
            node.publish_commands()
            rclpy.spin_once(node, timeout_sec=0.001)

        pygame.quit()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()