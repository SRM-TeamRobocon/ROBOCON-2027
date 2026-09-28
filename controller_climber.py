#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Joy
from std_msgs.msg import Float32MultiArray


class JetsonRackController(Node):

    def __init__(self):
        super().__init__('jetson_rack_controller')

        # ============================================================
        # Publisher
        # ============================================================

        self.pub_cmd = self.create_publisher(
            Float32MultiArray,
            '/rpm_sp',
            10
        )

        # ============================================================
        # Subscriber
        # ============================================================

        self.create_subscription(
            Joy,
            '/joy',
            self.joy_cb,
            10
        )

        # Publish at 50 Hz
        self.create_timer(0.02, self.tick)

        # ============================================================
        # PS5 / joy_node mapping
        # ============================================================

        # Axes
        self.AXIS_LEFT_X = 0
        self.AXIS_LEFT_Y = 1
        self.AXIS_RIGHT_X = 2
        self.AXIS_L2 = 3
        self.AXIS_R2 = 4
        self.AXIS_RIGHT_Y = 5

        # Buttons
        self.BTN_SQUARE = 0
        self.BTN_CROSS = 1
        self.BTN_CIRCLE = 2
        self.BTN_TRIANGLE = 3
        self.BTN_L1 = 4
        self.BTN_R1 = 5

        # ============================================================
        # Rack commands
        #
        #  0  = no movement
        # +1  = direction 1
        # -1  = direction 2
        # ============================================================

        self.rp1_cmd = 0.0
        self.rp2_cmd = 0.0

        # Left Y for traction
        self.left_y = 0.0

        # Previous button/trigger states
        self.prev_buttons = [0] * 6
        self.prev_l2 = False
        self.prev_r2 = False

        self.get_logger().info(
            'Jetson Rack Controller started'
        )

    # ================================================================
    # JOY CALLBACK
    # ================================================================

    def joy_cb(self, msg):

        axes = msg.axes
        buttons = msg.buttons

        # ------------------------------------------------------------
        # Left Y → traction
        # ------------------------------------------------------------

        if len(axes) > self.AXIS_LEFT_Y:
            self.left_y = axes[self.AXIS_LEFT_Y] * 128.0

            # Deadzone
            if abs(self.left_y) < 5:
                self.left_y = 0.0

        # ------------------------------------------------------------
        # Button states
        # ------------------------------------------------------------

        def pressed(index):
            return (
                len(buttons) > index and
                buttons[index] == 1
            )
        
                # ------------------------------------------------------------
        # L2 / R2 trigger states
        # ------------------------------------------------------------

        l2_pressed = False

        if len(axes) > self.AXIS_L2:
            l2_pressed = axes[self.AXIS_L2] < 0.0

        r2_pressed = False

        if len(axes) > self.AXIS_R2:
            r2_pressed = axes[self.AXIS_R2] < 0.0

        # ------------------------------------------------------------
        # TRIANGLE
        #
        # Both racks → direction 2
        # ------------------------------------------------------------

        if pressed(self.BTN_TRIANGLE) and not self.prev_buttons[self.BTN_TRIANGLE]:
            self.rp1_cmd = -1.0
            self.rp2_cmd = -1.0

            self.get_logger().info(
                'TRIANGLE: RP1 + RP2 direction 2'
            )

        # ------------------------------------------------------------
        # CROSS
        #
        # Both racks → direction 1
        # ------------------------------------------------------------

        if pressed(self.BTN_CROSS) and not self.prev_buttons[self.BTN_CROSS]:
            self.rp1_cmd = 1.0
            self.rp2_cmd = 1.0

            self.get_logger().info(
                'CROSS: RP1 + RP2 direction 1'
            )

       # ------------------------------------------------------------
        # L1 → RP1 direction 1 ONLY
        # ------------------------------------------------------------

        if pressed(self.BTN_L1) and not self.prev_buttons[self.BTN_L1]:
            self.rp1_cmd = 1.0
            self.rp2_cmd = 0.0
            self.get_logger().info(
                'L1: RP1 direction 1'
            )


        # ------------------------------------------------------------
        # L2 → RP1 direction 2 ONLY
        # ------------------------------------------------------------

        if l2_pressed and not self.prev_l2:
            self.rp1_cmd = -1.0
            self.rp2_cmd = 0.0
            self.get_logger().info(
                'L2: RP1 direction 2'
            )


        # ------------------------------------------------------------
        # R1 → RP2 direction 1 ONLY
        # ------------------------------------------------------------

        if pressed(self.BTN_R1) and not self.prev_buttons[self.BTN_R1]:
            self.rp1_cmd = 0.0
            self.rp2_cmd = 1.0
            self.get_logger().info(
                'R1: RP2 direction 1'
            )


        # ------------------------------------------------------------
        # R2 → RP2 direction 2 ONLY
        # ------------------------------------------------------------

        if r2_pressed and not self.prev_r2:
            self.rp1_cmd = 0.0
            self.rp2_cmd = -1.0
            self.get_logger().info(
                'R2: RP2 direction 2'
            )

        # ------------------------------------------------------------
        # CIRCLE
        #
        # Stop active rack(s).
        #
        # Sending 0 tells Teensy to capture the current position
        # and start PID holding it.
        # ------------------------------------------------------------

        if pressed(self.BTN_CIRCLE) and not self.prev_buttons[self.BTN_CIRCLE]:

            self.rp1_cmd = 0.0
            self.rp2_cmd = 0.0

            self.get_logger().info(
                'CIRCLE: Stop + PID hold'
            )

        # ------------------------------------------------------------
        # Save previous states
        # ------------------------------------------------------------

        for i in range(min(len(buttons), 6)):
            self.prev_buttons[i] = buttons[i]

        self.prev_l2 = l2_pressed
        self.prev_r2 = r2_pressed

    # ================================================================
    # PUBLISH
    # ================================================================

    def tick(self):

        msg = Float32MultiArray()

        msg.data = [
            float(self.rp1_cmd),
            float(self.rp2_cmd),
            float(self.left_y),
            0.0
        ]

        self.pub_cmd.publish(msg)


def main(args=None):

    rclpy.init(args=args)

    node = JetsonRackController()

    try:
        rclpy.spin(node)

    except KeyboardInterrupt:
        pass

    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()