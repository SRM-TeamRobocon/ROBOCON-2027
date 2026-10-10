#!/usr/bin/env python3

import rclpy

from rclpy.node import Node

from sensor_msgs.msg import Joy
from std_msgs.msg import Float32MultiArray


# ============================================================
# EASY-TO-CHANGE SETTINGS
# ============================================================

# ---------------- Topics ----------------

JOY_TOPIC = '/joy'

RPM_TOPIC = '/rpm_sp'

SENSOR_TOPIC = '/sensor_values'

LIMIT_SWITCH_TOPIC = '/limit_switch'


# ---------------- Publish rate ----------------

PUBLISH_PERIOD = 0.02       # 50 Hz


# ---------------- Sensor array indices ----------------

FRONT_RACK_INDEX = 0

BACK_RACK_INDEX = 1

FORWARD_LIDAR_INDEX = 2

BACKWARD_LIDAR_INDEX = 3

YAW_INDEX = 4

ROLL_INDEX = 5


# ---------------- Rack automatic stop distances ----------------

FRONT_RACK_STOP_DISTANCE = 300.0

BACK_RACK_STOP_DISTANCE = 300.0


# ---------------- Joystick settings ----------------

JOYSTICK_SCALE = 128.0

LEFT_X_DEADZONE = 5.0

LEFT_Y_DEADZONE = 5.0


# ---------------- Rack commands ----------------

RACK_DIRECTION_1 = 1.0

RACK_DIRECTION_2 = -1.0

RACK_STOP = 0.0


# ---------------- Traction limits ----------------

TRACTION_MAX = 128.0


class JetsonRackController(Node):

    def __init__(self):

        super().__init__(
            'jetson_rack_controller'
        )

        # ============================================================
        # Publisher
        # ============================================================

        self.pub_cmd = self.create_publisher(
            Float32MultiArray,
            RPM_TOPIC,
            10
        )

        # ============================================================
        # Subscribers
        # ============================================================

        self.create_subscription(
            Joy,
            JOY_TOPIC,
            self.joy_cb,
            10
        )

        # ------------------------------------------------------------
        # Sensor values subscriber
        # ------------------------------------------------------------

        self.create_subscription(
            Float32MultiArray,
            SENSOR_TOPIC,
            self.sensor_values,
            10
        )

        # ------------------------------------------------------------
        # Limit switch subscriber
        # ------------------------------------------------------------

        self.create_subscription(
            Float32MultiArray,
            LIMIT_SWITCH_TOPIC,
            self.limit_switch_values,
            10
        )

        # Publish at 50 Hz

        self.create_timer(
            PUBLISH_PERIOD,
            self.tick
        )

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
        # 0  = no movement
        # +1 = direction 1
        # -1 = direction 2
        # ============================================================

        self.rp1_cmd = 0.0

        self.rp2_cmd = 0.0

        # ============================================================
        # LIMIT SWITCH STATE
        #
        # True = pressed
        # False = not pressed
        # ============================================================

        self.rp1_limit = False

        self.rp2_limit = False

        # ============================================================
        # TRACTION
        #
        # M1 + M3 = LEFT
        # M2 + M4 = RIGHT
        # ============================================================

        self.left_y = 0.0

        self.left_x = 0.0

        self.left_traction = 0.0

        self.right_traction = 0.0

        # ============================================================
        # SENSOR VALUES
        # ============================================================

        self.front_rack_distance = 0.0

        self.back_rack_distance = 0.0

        self.forward_lidar = 0.0

        self.backward_lidar = 0.0

        self.yaw = 0.0

        self.roll = 0.0

        # ============================================================
        # TRIANGLE AUTO-STOP STATE
        # ============================================================

        self.triangle_active = False

        # Previous button/trigger states

        self.prev_buttons = [0] * 6

        self.prev_l2 = False

        self.prev_r2 = False

        self.get_logger().info(
            'Jetson Rack Controller started'
        )

    # ================================================================
    # LIMIT SWITCH CALLBACK
    # ================================================================

    def limit_switch_values(self, msg):

        if len(msg.data) < 2:
            self.get_logger().warn(
                '/limit_switch requires 2 values'
            )
            return

        # ------------------------------------------------------------
        # Read Teensy limit switch states
        #
        # [0] = RP1
        # [1] = RP2
        # ------------------------------------------------------------

        self.rp1_limit = (
            msg.data[0] > 0.5
        )

        self.rp2_limit = (
            msg.data[1] > 0.5
        )

        # ------------------------------------------------------------
        # RP1
        #
        # Only stop if RP1 is moving toward direction 1.
        # Direction 2 remains allowed.
        # ------------------------------------------------------------

        if (
            self.rp1_limit
            and self.rp1_cmd == RACK_DIRECTION_1
        ):

            self.rp1_cmd = RACK_STOP

            self.get_logger().info(
                'RP1 LIMIT: Stop + PID hold'
            )

        # ------------------------------------------------------------
        # RP2
        #
        # Only stop if RP2 is moving toward direction 1.
        # Direction 2 remains allowed.
        # ------------------------------------------------------------

        if (
            self.rp2_limit
            and self.rp2_cmd == RACK_DIRECTION_1
        ):

            self.rp2_cmd = RACK_STOP

            self.get_logger().info(
                'RP2 LIMIT: Stop + PID hold'
            )

    # ================================================================
    # SENSOR VALUES CALLBACK
    # ================================================================

    def sensor_values(self, msg):

        if len(msg.data) <= ROLL_INDEX:

            self.get_logger().warn(
                f'/sensor_values requires at least 6 values, '
                f'but received {len(msg.data)}'
            )

            return

        # ------------------------------------------------------------
        # Read sensor values
        # ------------------------------------------------------------

        self.front_rack_distance = (
            msg.data[FRONT_RACK_INDEX]
        )

        self.back_rack_distance = (
            msg.data[BACK_RACK_INDEX]
        )

        self.forward_lidar = (
            msg.data[FORWARD_LIDAR_INDEX]
        )

        self.backward_lidar = (
            msg.data[BACKWARD_LIDAR_INDEX]
        )

        self.yaw = (
            msg.data[YAW_INDEX]
        )

        self.roll = (
            msg.data[ROLL_INDEX]
        )

        # ------------------------------------------------------------
        # TRIANGLE AUTOMATIC STOP
        # ------------------------------------------------------------

        if self.triangle_active:

            front_reached = (
                self.front_rack_distance >=
                FRONT_RACK_STOP_DISTANCE
            )

            back_reached = (
                self.back_rack_distance >=
                BACK_RACK_STOP_DISTANCE
            )

            if front_reached and back_reached:

                self.rp1_cmd = RACK_STOP

                self.rp2_cmd = RACK_STOP

                self.triangle_active = False

                self.get_logger().info(
                    'TRIANGLE AUTO STOP: '
                    f'Front={self.front_rack_distance:.2f}, '
                    f'Back={self.back_rack_distance:.2f} '
                    '-> Stop + PID hold'
                )

    # ================================================================
    # JOY CALLBACK
    # ================================================================

    def joy_cb(self, msg):

        axes = msg.axes

        buttons = msg.buttons

        # ------------------------------------------------------------
        # Left X → traction LEFT / RIGHT
        # ------------------------------------------------------------

        if len(axes) > self.AXIS_LEFT_X:

            self.left_x = (
                axes[self.AXIS_LEFT_X] *
                JOYSTICK_SCALE
            )

            if abs(self.left_x) < LEFT_X_DEADZONE:
                self.left_x = 0.0

        # ------------------------------------------------------------
        # Left Y → traction FORWARD / BACKWARD
        # ------------------------------------------------------------

        if len(axes) > self.AXIS_LEFT_Y:

            self.left_y = (
                axes[self.AXIS_LEFT_Y] *
                JOYSTICK_SCALE
            )

            if abs(self.left_y) < LEFT_Y_DEADZONE:
                self.left_y = 0.0

        # ------------------------------------------------------------
        # Differential traction
        #
        # LEFT  = Y + X
        # RIGHT = Y - X
        # ------------------------------------------------------------

        left = (
            self.left_y +
            self.left_x
        )

        right = (
            self.left_y -
            self.left_x
        )

        max_value = max(
            abs(left),
            abs(right)
        )

        if max_value > TRACTION_MAX:

            left = (
                left / max_value
            ) * TRACTION_MAX

            right = (
                right / max_value
            ) * TRACTION_MAX

        self.left_traction = left

        self.right_traction = right

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

            l2_pressed = (
                axes[self.AXIS_L2] < 0.0
            )

        r2_pressed = False

        if len(axes) > self.AXIS_R2:

            r2_pressed = (
                axes[self.AXIS_R2] < 0.0
            )

        # ============================================================
        # TRIANGLE
        #
        # Both racks → direction 2
        # ============================================================

        if (
            pressed(self.BTN_TRIANGLE)
            and not self.prev_buttons[
                self.BTN_TRIANGLE
            ]
        ):

            self.rp1_cmd = RACK_DIRECTION_2

            self.rp2_cmd = RACK_DIRECTION_2

            self.triangle_active = True

            self.get_logger().info(
                'TRIANGLE: RP1 + RP2 direction 2 | '
                'Auto-stop enabled'
            )

        # ============================================================
        # CROSS
        #
        # Both racks → direction 1
        # ============================================================

        if (
            pressed(self.BTN_CROSS)
            and not self.prev_buttons[
                self.BTN_CROSS
            ]
        ):

            self.rp1_cmd = RACK_DIRECTION_1

            self.rp2_cmd = RACK_DIRECTION_1

            self.triangle_active = False

            self.get_logger().info(
                'CROSS: RP1 + RP2 direction 1'
            )

        # ============================================================
        # L1 → RP1 direction 1 ONLY
        # ============================================================

        if (
            pressed(self.BTN_L1)
            and not self.prev_buttons[
                self.BTN_L1
            ]
        ):

            if self.rp1_limit:

                self.rp1_cmd = RACK_STOP

                self.get_logger().info(
                    'L1: RP1 limit active -> Stop + PID hold'
                )

            else:

                self.rp1_cmd = RACK_DIRECTION_1

                self.get_logger().info(
                    'L1: RP1 direction 1'
                )

            self.rp2_cmd = RACK_STOP

            self.triangle_active = False

        # ============================================================
        # L2 → RP1 direction 2 ONLY
        # ============================================================

        if (
            l2_pressed
            and not self.prev_l2
        ):

            self.rp1_cmd = RACK_DIRECTION_2

            self.rp2_cmd = RACK_STOP

            self.triangle_active = False

            self.get_logger().info(
                'L2: RP1 direction 2'
            )

        # ============================================================
        # R1 → RP2 direction 1 ONLY
        # ============================================================

        if (
            pressed(self.BTN_R1)
            and not self.prev_buttons[
                self.BTN_R1
            ]
        ):

            self.rp1_cmd = RACK_STOP

            if self.rp2_limit:

                self.rp2_cmd = RACK_STOP

                self.get_logger().info(
                    'R1: RP2 limit active -> Stop + PID hold'
                )

            else:

                self.rp2_cmd = RACK_DIRECTION_1

                self.get_logger().info(
                    'R1: RP2 direction 1'
                )

            self.triangle_active = False

        # ============================================================
        # R2 → RP2 direction 2 ONLY
        # ============================================================

        if (
            r2_pressed
            and not self.prev_r2
        ):

            self.rp1_cmd = RACK_STOP

            self.rp2_cmd = RACK_DIRECTION_2

            self.triangle_active = False

            self.get_logger().info(
                'R2: RP2 direction 2'
            )

        # ============================================================
        # CIRCLE
        #
        # Stop both racks + PID hold
        # ============================================================

        if (
            pressed(self.BTN_CIRCLE)
            and not self.prev_buttons[
                self.BTN_CIRCLE
            ]
        ):

            self.rp1_cmd = RACK_STOP

            self.rp2_cmd = RACK_STOP

            self.triangle_active = False

            self.get_logger().info(
                'CIRCLE: Stop + PID hold'
            )

        # ------------------------------------------------------------
        # Save previous states
        # ------------------------------------------------------------

        for i in range(
            min(len(buttons), 6)
        ):

            self.prev_buttons[i] = buttons[i]

        self.prev_l2 = l2_pressed

        self.prev_r2 = r2_pressed

    # ================================================================
    # PUBLISH
    # ================================================================

    def tick(self):

        # ------------------------------------------------------------
        # Safety enforcement from latest limit state
        #
        # Direction 1 is blocked while switch is active.
        # Direction 2 is still allowed.
        # ------------------------------------------------------------

        if (
            self.rp1_limit
            and self.rp1_cmd == RACK_DIRECTION_1
        ):

            self.rp1_cmd = RACK_STOP

        if (
            self.rp2_limit
            and self.rp2_cmd == RACK_DIRECTION_1
        ):

            self.rp2_cmd = RACK_STOP

        msg = Float32MultiArray()

        msg.data = [

            float(self.rp1_cmd),

            float(self.rp2_cmd),

            # M1 + M3
            float(self.left_traction),

            # M2 + M4
            float(self.right_traction)
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