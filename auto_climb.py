#!/usr/bin/env python3

import rclpy

from rclpy.node import Node

from sensor_msgs.msg import Joy
from std_msgs.msg import Float32MultiArray


# ============================================================
# TOP-LEVEL SETTINGS
# ============================================================

# ============================================================
# TOPICS
# ============================================================

JOY_TOPIC = '/joy'

RPM_TOPIC = '/rpm_sp'

SENSOR_TOPIC = '/sensor_values'

LIMIT_SWITCH_TOPIC = '/limit_switch'


# ============================================================
# PUBLISH RATE
# ============================================================

PUBLISH_PERIOD = 0.02       # 50 Hz


# ============================================================
# SENSOR ARRAY
#
# sensor_values:
#
# [0] = RP1 lidar
# [1] = RP2 lidar
# [2] = forward lidar
# [3] = backward lidar
# [4] = yaw
# [5] = roll
# ============================================================

RP1_LIDAR_INDEX = 0

RP2_LIDAR_INDEX = 1

FORWARD_LIDAR_INDEX = 2

BACKWARD_LIDAR_INDEX = 3

YAW_INDEX = 4

ROLL_INDEX = 5


# ============================================================
# AUTONOMOUS CLIMB SETTINGS
# ============================================================

# ------------------------------------------------------------
# STAGE 1
#
# Bot moves forward until lidar [2] reaches this value.
# ------------------------------------------------------------

CLIMB_START_LIDAR_INDEX = FORWARD_LIDAR_INDEX

CLIMB_START_DISTANCE = 500.0

CLIMB_FORWARD_SPEED = 80.0


# ------------------------------------------------------------
# STAGE 2
#
# Racks move according to Triangle logic.
#
# RP1 uses lidar [0]
# RP2 uses lidar [1]
# ------------------------------------------------------------

RP1_RACK_STOP_DISTANCE = 300.0

RP2_RACK_STOP_DISTANCE = 300.0

RACK_CLIMB_COMMAND = -1.0


# ------------------------------------------------------------
# STAGE 3
#
# After both racks have stopped, bot moves forward again.
#
# Final check uses lidar [0].
# ------------------------------------------------------------

FINAL_FORWARD_LIDAR_INDEX = RP1_LIDAR_INDEX

FINAL_FORWARD_DISTANCE = 500.0

FINAL_FORWARD_SPEED = 80.0


# ============================================================
# RACK COMMANDS
# ============================================================

RACK_DIRECTION_1 = 1.0

RACK_DIRECTION_2 = -1.0

RACK_STOP = 0.0


# ============================================================
# TRACTION
# ============================================================

TRACTION_MAX = 128.0

TRACTION_STOP = 0.0


# ============================================================
# MANUAL JOYSTICK
# ============================================================

JOYSTICK_SCALE = 128.0

LEFT_X_DEADZONE = 5.0

LEFT_Y_DEADZONE = 5.0


class JetsonRackController(Node):

    def __init__(self):

        super().__init__(
            'jetson_rack_controller'
        )

        # ========================================================
        # PUBLISHER
        # ========================================================

        self.pub_cmd = self.create_publisher(
            Float32MultiArray,
            RPM_TOPIC,
            10
        )

        # ========================================================
        # JOY SUBSCRIBER
        # ========================================================

        self.create_subscription(
            Joy,
            JOY_TOPIC,
            self.joy_cb,
            10
        )

        # ========================================================
        # SENSOR SUBSCRIBER
        # ========================================================

        self.create_subscription(
            Float32MultiArray,
            SENSOR_TOPIC,
            self.sensor_values,
            10
        )

        # ========================================================
        # LIMIT SWITCH SUBSCRIBER
        # ========================================================

        self.create_subscription(
            Float32MultiArray,
            LIMIT_SWITCH_TOPIC,
            self.limit_switch_values,
            10
        )

        # ========================================================
        # 50 Hz TIMER
        # ========================================================

        self.create_timer(
            PUBLISH_PERIOD,
            self.tick
        )

        # ========================================================
        # PS5 AXES
        # ========================================================

        self.AXIS_LEFT_X = 0
        self.AXIS_LEFT_Y = 1
        self.AXIS_RIGHT_X = 2
        self.AXIS_L2 = 3
        self.AXIS_R2 = 4
        self.AXIS_RIGHT_Y = 5

        # ========================================================
        # PS5 BUTTONS
        # ========================================================

        self.BTN_SQUARE = 0
        self.BTN_CROSS = 1
        self.BTN_CIRCLE = 2
        self.BTN_TRIANGLE = 3
        self.BTN_L1 = 4
        self.BTN_R1 = 5

        # ========================================================
        # RACK COMMANDS
        # ========================================================

        self.rp1_cmd = 0.0
        self.rp2_cmd = 0.0

        # ========================================================
        # TRACTION
        # ========================================================

        self.left_y = 0.0
        self.left_x = 0.0

        self.left_traction = 0.0
        self.right_traction = 0.0

        # ========================================================
        # SENSOR VALUES
        # ========================================================

        self.sensor_data = [0.0] * 6

        self.front_rack_distance = 0.0
        self.back_rack_distance = 0.0

        self.forward_lidar = 0.0
        self.backward_lidar = 0.0

        self.yaw = 0.0
        self.roll = 0.0

        # ========================================================
        # LIMIT SWITCHES
        # ========================================================

        self.rp1_limit = False
        self.rp2_limit = False

        # ========================================================
        # AUTONOMOUS CLIMB
        #
        # 0 = IDLE
        # 1 = MOVE TO CLIMB START
        # 2 = RACK CLIMB
        # 3 = FINAL FORWARD
        # 4 = COMPLETE
        # ========================================================

        self.climb_stage = 0

        self.climb_active = False

        self.rp1_climb_done = False
        self.rp2_climb_done = False

        # ========================================================
        # PREVIOUS BUTTON STATES
        # ========================================================

        self.prev_buttons = [0] * 6

        self.prev_l2 = False
        self.prev_r2 = False

        self.get_logger().info(
            'Jetson Rack Controller started'
        )

    # ============================================================
    # SENSOR VALUES
    # ============================================================

    def sensor_values(self, msg):

        if len(msg.data) < 6:

            self.get_logger().warn(
                '/sensor_values requires 6 values'
            )

            return

        # Store complete sensor array

        for i in range(6):
            self.sensor_data[i] = msg.data[i]

        # --------------------------------------------------------
        # Individual values
        # --------------------------------------------------------

        self.front_rack_distance = (
            msg.data[RP1_LIDAR_INDEX]
        )

        self.back_rack_distance = (
            msg.data[RP2_LIDAR_INDEX]
        )

        self.forward_lidar = (
            msg.data[FORWARD_LIDAR_INDEX]
        )

        self.backward_lidar = (
            msg.data[BACKWARD_LIDAR_INDEX]
        )

        self.yaw = msg.data[YAW_INDEX]

        self.roll = msg.data[ROLL_INDEX]

        # ========================================================
        # AUTONOMOUS CLIMB
        # ========================================================

        if not self.climb_active:
            return

        # ========================================================
        # STAGE 1
        #
        # Bot moves forward until lidar [2]
        # reaches CLIMB_START_DISTANCE
        # ========================================================

        if self.climb_stage == 1:

            if (
                self.sensor_data[
                    CLIMB_START_LIDAR_INDEX
                ]
                >= CLIMB_START_DISTANCE
            ):

                # Stop bot

                self.left_traction = TRACTION_STOP
                self.right_traction = TRACTION_STOP

                # Start racks

                self.rp1_cmd = RACK_CLIMB_COMMAND
                self.rp2_cmd = RACK_CLIMB_COMMAND

                self.rp1_climb_done = False
                self.rp2_climb_done = False

                self.climb_stage = 2

                self.get_logger().info(
                    'CLIMB STAGE 1 COMPLETE -> '
                    'Starting rack climb'
                )

            else:

                # Keep moving forward

                self.left_traction = CLIMB_FORWARD_SPEED

                self.right_traction = CLIMB_FORWARD_SPEED

        # ========================================================
        # STAGE 2
        #
        # RP1 and RP2 stop independently.
        # ========================================================

        elif self.climb_stage == 2:

            # ----------------------------------------------------
            # RP1
            # ----------------------------------------------------

            if not self.rp1_climb_done:

                if (
                    self.sensor_data[
                        RP1_LIDAR_INDEX
                    ]
                    >= RP1_RACK_STOP_DISTANCE
                ):

                    self.rp1_cmd = RACK_STOP

                    self.rp1_climb_done = True

                    self.get_logger().info(
                        'RP1 reached target -> '
                        'Stop + PID hold'
                    )

            # ----------------------------------------------------
            # RP2
            # ----------------------------------------------------

            if not self.rp2_climb_done:

                if (
                    self.sensor_data[
                        RP2_LIDAR_INDEX
                    ]
                    >= RP2_RACK_STOP_DISTANCE
                ):

                    self.rp2_cmd = RACK_STOP

                    self.rp2_climb_done = True

                    self.get_logger().info(
                        'RP2 reached target -> '
                        'Stop + PID hold'
                    )

            # ----------------------------------------------------
            # BOTH RACKS DONE
            # ----------------------------------------------------

            if (
                self.rp1_climb_done
                and self.rp2_climb_done
            ):

                self.rp1_cmd = RACK_STOP
                self.rp2_cmd = RACK_STOP

                self.left_traction = TRACTION_STOP
                self.right_traction = TRACTION_STOP

                self.climb_stage = 3

                self.get_logger().info(
                    'BOTH RACKS COMPLETE -> '
                    'Starting final forward movement'
                )

        # ========================================================
        # STAGE 3
        #
        # Bot moves forward until sensor [0]
        # reaches FINAL_FORWARD_DISTANCE
        # ========================================================

        elif self.climb_stage == 3:

            # Racks remain held

            self.rp1_cmd = RACK_STOP
            self.rp2_cmd = RACK_STOP

            if (
                self.sensor_data[
                    FINAL_FORWARD_LIDAR_INDEX
                ]
                >= FINAL_FORWARD_DISTANCE
            ):

                # Stop entire bot

                self.left_traction = TRACTION_STOP
                self.right_traction = TRACTION_STOP

                self.climb_stage = 4

                self.climb_active = False

                self.get_logger().info(
                    'CLIMB COMPLETE -> '
                    'Bot stopped'
                )

            else:

                # Continue forward

                self.left_traction = FINAL_FORWARD_SPEED

                self.right_traction = FINAL_FORWARD_SPEED

        # ========================================================
        # STAGE 4
        # ========================================================

        elif self.climb_stage == 4:

            self.rp1_cmd = RACK_STOP
            self.rp2_cmd = RACK_STOP

            self.left_traction = TRACTION_STOP
            self.right_traction = TRACTION_STOP

    # ============================================================
    # LIMIT SWITCH CALLBACK
    # ============================================================

    def limit_switch_values(self, msg):

        if len(msg.data) < 2:
            return

        self.rp1_limit = (
            msg.data[0] > 0.5
        )

        self.rp2_limit = (
            msg.data[1] > 0.5
        )

        # --------------------------------------------------------
        # During autonomous climbing
        #
        # If a limit switch activates, that rack is considered
        # finished immediately.
        # --------------------------------------------------------

        if self.climb_active:

            if self.rp1_limit:

                self.rp1_cmd = RACK_STOP

                self.rp1_climb_done = True

                self.get_logger().info(
                    'RP1 LIMIT SWITCH -> '
                    'Stop + PID hold'
                )

            if self.rp2_limit:

                self.rp2_cmd = RACK_STOP

                self.rp2_climb_done = True

                self.get_logger().info(
                    'RP2 LIMIT SWITCH -> '
                    'Stop + PID hold'
                )

            # If both racks are done, move to final stage

            if (
                self.climb_stage == 2
                and self.rp1_climb_done
                and self.rp2_climb_done
            ):

                self.rp1_cmd = RACK_STOP
                self.rp2_cmd = RACK_STOP

                self.climb_stage = 3

                self.get_logger().info(
                    'BOTH RACKS COMPLETE -> '
                    'Starting final forward movement'
                )

    # ============================================================
    # JOY CALLBACK
    # ============================================================

    def joy_cb(self, msg):

        axes = msg.axes
        buttons = msg.buttons

        # ========================================================
        # TRACTION
        # ========================================================

        if len(axes) > self.AXIS_LEFT_X:

            self.left_x = (
                axes[self.AXIS_LEFT_X]
                * JOYSTICK_SCALE
            )

            if abs(self.left_x) < LEFT_X_DEADZONE:
                self.left_x = 0.0

        if len(axes) > self.AXIS_LEFT_Y:

            self.left_y = (
                axes[self.AXIS_LEFT_Y]
                * JOYSTICK_SCALE
            )

            if abs(self.left_y) < LEFT_Y_DEADZONE:
                self.left_y = 0.0

        left = self.left_y + self.left_x

        right = self.left_y - self.left_x

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

        # ========================================================
        # BUTTON FUNCTION
        # ========================================================

        def pressed(index):

            return (
                len(buttons) > index
                and buttons[index] == 1
            )

        # ========================================================
        # L2 / R2
        # ========================================================

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

        # ========================================================
        # TRIANGLE
        #
        # START ENTIRE CLIMBING PROCESS
        # ========================================================

        if (
            pressed(self.BTN_TRIANGLE)
            and not self.prev_buttons[
                self.BTN_TRIANGLE
            ]
        ):

            # ----------------------------------------------------
            # Start autonomous climb
            # ----------------------------------------------------

            self.climb_active = True

            self.climb_stage = 1

            self.rp1_climb_done = False
            self.rp2_climb_done = False

            # Racks initially stopped

            self.rp1_cmd = RACK_STOP
            self.rp2_cmd = RACK_STOP

            # Start moving forward

            self.left_traction = CLIMB_FORWARD_SPEED

            self.right_traction = CLIMB_FORWARD_SPEED

            self.get_logger().info(
                'TRIANGLE -> AUTONOMOUS CLIMB STARTED'
            )

        # ========================================================
        # MANUAL CONTROLS
        #
        # These are ignored while autonomous climb is active.
        # ========================================================

        if not self.climb_active:

            # ----------------------------------------------------
            # CROSS
            # ----------------------------------------------------

            if (
                pressed(self.BTN_CROSS)
                and not self.prev_buttons[
                    self.BTN_CROSS
                ]
            ):

                self.rp1_cmd = RACK_DIRECTION_1

                self.rp2_cmd = RACK_DIRECTION_1

                self.get_logger().info(
                    'CROSS: RP1 + RP2 direction 1'
                )

            # ----------------------------------------------------
            # L1
            # ----------------------------------------------------

            if (
                pressed(self.BTN_L1)
                and not self.prev_buttons[
                    self.BTN_L1
                ]
            ):

                if self.rp1_limit:

                    self.rp1_cmd = RACK_STOP

                else:

                    self.rp1_cmd = RACK_DIRECTION_1

                self.rp2_cmd = RACK_STOP

            # ----------------------------------------------------
            # L2
            # ----------------------------------------------------

            if (
                l2_pressed
                and not self.prev_l2
            ):

                self.rp1_cmd = RACK_DIRECTION_2

                self.rp2_cmd = RACK_STOP

            # ----------------------------------------------------
            # R1
            # ----------------------------------------------------

            if (
                pressed(self.BTN_R1)
                and not self.prev_buttons[
                    self.BTN_R1
                ]
            ):

                self.rp1_cmd = RACK_STOP

                if self.rp2_limit:

                    self.rp2_cmd = RACK_STOP

                else:

                    self.rp2_cmd = RACK_DIRECTION_1

            # ----------------------------------------------------
            # R2
            # ----------------------------------------------------

            if (
                r2_pressed
                and not self.prev_r2
            ):

                self.rp1_cmd = RACK_STOP

                self.rp2_cmd = RACK_DIRECTION_2

            # ----------------------------------------------------
            # CIRCLE
            # ----------------------------------------------------

            if (
                pressed(self.BTN_CIRCLE)
                and not self.prev_buttons[
                    self.BTN_CIRCLE
                ]
            ):

                self.rp1_cmd = RACK_STOP

                self.rp2_cmd = RACK_STOP

                self.left_traction = TRACTION_STOP
                self.right_traction = TRACTION_STOP

                self.get_logger().info(
                    'CIRCLE: Stop + PID hold'
                )

        # ========================================================
        # SAVE PREVIOUS BUTTON STATES
        # ========================================================

        for i in range(
            min(len(buttons), 6)
        ):

            self.prev_buttons[i] = buttons[i]

        self.prev_l2 = l2_pressed

        self.prev_r2 = r2_pressed

    # ============================================================
    # PUBLISH
    # ============================================================

    def tick(self):

        # ========================================================
        # LIMIT SWITCH SAFETY
        # ========================================================

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

        # ========================================================
        # PUBLISH /rpm_sp
        #
        # [RP1, RP2, LEFT, RIGHT]
        # ========================================================

        msg = Float32MultiArray()

        msg.data = [

            float(self.rp1_cmd),

            float(self.rp2_cmd),

            float(self.left_traction),

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