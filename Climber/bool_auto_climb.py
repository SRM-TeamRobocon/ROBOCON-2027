#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from std_msgs.msg import Float32MultiArray, Bool

# ============================================================
# SETTINGS
# ============================================================

JOY_TOPIC = '/joy'
RPM_TOPIC = '/rpm_sp'
SENSOR_TOPIC = '/sensor_values'
LIMIT_SWITCH_TOPIC = '/limit_switch'
CLIMB_STATE_TOPIC = '/climb_state'

PUBLISH_PERIOD = 0.02

FRONT_RACK_INDEX = 0
BACK_RACK_INDEX = 1
FORWARD_LIDAR_INDEX = 2
BACKWARD_LIDAR_INDEX = 3
YAW_INDEX = 4
ROLL_INDEX = 5

FORWARD_LIDAR_STOP_DISTANCE = 130.0
FRONT_RACK_STOP_DISTANCE = 300.0
BACK_RACK_STOP_DISTANCE = 300.0
FINAL_FORWARD_LIDAR_STOP_DISTANCE = 130.0

CLIMB_FORWARD_SPEED = -20.0
FINAL_FORWARD_SPEED = -20.0

JOYSTICK_SCALE = 128.0
LEFT_X_DEADZONE = 5.0
LEFT_Y_DEADZONE = 5.0

RACK_DIRECTION_1 = 1.0
RACK_DIRECTION_2 = -1.0
RACK_STOP = 0.0

TRACTION_MAX = 128.0


class JetsonRackController(Node):

    def __init__(self):
        super().__init__('jetson_rack_controller')

        self.pub_cmd = self.create_publisher(
            Float32MultiArray,
            RPM_TOPIC,
            10
        )

        self.create_subscription(
            Joy,
            JOY_TOPIC,
            self.joy_cb,
            10
        )

        self.create_subscription(
            Float32MultiArray,
            SENSOR_TOPIC,
            self.sensor_values,
            10
        )

        self.create_subscription(
            Float32MultiArray,
            LIMIT_SWITCH_TOPIC,
            self.limit_switch_values,
            10
        )

        self.create_subscription(
            Bool,
            CLIMB_STATE_TOPIC,
            self.climb_state_cb,
            10
        )

        self.create_timer(
            PUBLISH_PERIOD,
            self.tick
        )

        # PS5 axes
        self.AXIS_LEFT_X = 0
        self.AXIS_LEFT_Y = 1
        self.AXIS_RIGHT_X = 2
        self.AXIS_L2 = 3
        self.AXIS_R2 = 4
        self.AXIS_RIGHT_Y = 5

        # PS5 buttons
        self.BTN_SQUARE = 0
        self.BTN_CROSS = 1
        self.BTN_CIRCLE = 2
        self.BTN_TRIANGLE = 3
        self.BTN_L1 = 4
        self.BTN_R1 = 5

        # Rack commands
        self.rp1_cmd = 0.0
        self.rp2_cmd = 0.0

        # Limit switches
        self.rp1_limit = False
        self.rp2_limit = False

        # Traction
        self.left_y = 0.0
        self.left_x = 0.0
        self.left_traction = 0.0
        self.right_traction = 0.0

        # Sensor values
        self.front_rack_distance = 0.0
        self.back_rack_distance = 0.0
        self.forward_lidar = 0.0
        self.backward_lidar = 0.0
        self.yaw = 0.0
        self.roll = 0.0

        # Autonomous sequence:
        # 0 = Idle
        # 1 = Initial forward movement
        # 2 = Both racks direction 2
        # 3 = Slow forward movement
        # 4 = RP2 direction 1 until RP2 limit
        # 5 = Move forward until lidar [1] < 140
        # 6 = RP1 direction 1 until RP1 limit
        # 7 = Move forward until lidar [2] <= 140

        self.triangle_active = False
        self.triangle_stage = 0

        self.prev_buttons = [0] * 6
        self.prev_l2 = False
        self.prev_r2 = False

        self.get_logger().info(
            'Jetson Rack Controller started'
        )

    # ============================================================
    # START AUTONOMOUS CLIMB
    # ============================================================

    def start_autonomous_climb(self, source):

        if self.triangle_active:
            return

        self.triangle_active = True
        self.triangle_stage = 1

        self.rp1_cmd = RACK_STOP
        self.rp2_cmd = RACK_STOP

        self.left_traction = CLIMB_FORWARD_SPEED
        self.right_traction = CLIMB_FORWARD_SPEED

        self.get_logger().info(
            f'{source}: Autonomous sequence started. '
            'Moving forward until lidar [2] <= '
            f'{FORWARD_LIDAR_STOP_DISTANCE}.'
        )

    # ============================================================
    # CLIMB STATE CALLBACK
    # /climb_state TRUE triggers the same sequence as TRIANGLE.
    # FALSE does not cancel an ongoing sequence.
    # ============================================================

    def climb_state_cb(self, msg):

        if msg.data:
            self.start_autonomous_climb('/climb_state=True')

    # ============================================================
    # LIMIT SWITCH CALLBACK
    # ============================================================

    def limit_switch_values(self, msg):

        if len(msg.data) < 2:
            self.get_logger().warn(
                '/limit_switch requires 2 values'
            )
            return

        self.rp1_limit = msg.data[0] > 0.5
        self.rp2_limit = msg.data[1] > 0.5

        # Block direction 1 if RP1 limit is active.
        if (
            self.rp1_limit
            and self.rp1_cmd == RACK_DIRECTION_1
        ):
            self.rp1_cmd = RACK_STOP

            self.get_logger().info(
                'RP1 LIMIT: Stop + PID hold'
            )

        # Block direction 1 if RP2 limit is active.
        if (
            self.rp2_limit
            and self.rp2_cmd == RACK_DIRECTION_1
        ):
            self.rp2_cmd = RACK_STOP

            self.get_logger().info(
                'RP2 LIMIT: Stop + PID hold'
            )

    # ============================================================
    # SENSOR CALLBACK
    # ============================================================

    def sensor_values(self, msg):

        if len(msg.data) <= ROLL_INDEX:
            self.get_logger().warn(
                f'/sensor_values requires at least 6 values, '
                f'but received {len(msg.data)}'
            )
            return

        self.front_rack_distance = msg.data[FRONT_RACK_INDEX]
        self.back_rack_distance = msg.data[BACK_RACK_INDEX]

        self.forward_lidar = msg.data[FORWARD_LIDAR_INDEX]
        self.backward_lidar = msg.data[BACKWARD_LIDAR_INDEX]

        self.yaw = msg.data[YAW_INDEX]
        self.roll = msg.data[ROLL_INDEX]

        if not self.triangle_active:
            return

        # ========================================================
        # STAGE 1
        # Move forward until lidar [2] <= 130.
        # ========================================================

        if self.triangle_stage == 1:

            if (
                self.forward_lidar
                <= FORWARD_LIDAR_STOP_DISTANCE
            ):

                self.left_traction = 0.0
                self.right_traction = 0.0

                self.rp1_cmd = RACK_DIRECTION_2
                self.rp2_cmd = RACK_DIRECTION_2

                self.triangle_stage = 2

                self.get_logger().info(
                    'STAGE 1 COMPLETE: '
                    f'Lidar [2]={self.forward_lidar:.2f} '
                    f'<= {FORWARD_LIDAR_STOP_DISTANCE}. '
                    'Traction stopped; both racks moving '
                    'in direction 2.'
                )

            else:

                self.left_traction = CLIMB_FORWARD_SPEED
                self.right_traction = CLIMB_FORWARD_SPEED

        # ========================================================
        # STAGE 2
        # Move both racks in direction 2.
        # Stop when both rack lidar thresholds are reached.
        # ========================================================

        elif self.triangle_stage == 2:

            front_reached = (
                self.front_rack_distance
                >= FRONT_RACK_STOP_DISTANCE
            )

            back_reached = (
                self.back_rack_distance
                >= BACK_RACK_STOP_DISTANCE
            )

            if front_reached and back_reached:

                self.rp1_cmd = RACK_STOP
                self.rp2_cmd = RACK_STOP

                self.left_traction = 0.0
                self.right_traction = 0.0

                self.triangle_stage = 3

                self.get_logger().info(
                    'STAGE 2 COMPLETE: Both racks stopped. '
                    'Starting slow forward movement until '
                    f'lidar [0] <= '
                    f'{FINAL_FORWARD_LIDAR_STOP_DISTANCE}.'
                )

        # ========================================================
        # STAGE 3
        # Move forward slowly until lidar [0] reaches threshold.
        # Then proceed to Stage 4.
        # ========================================================

        elif self.triangle_stage == 3:

            self.rp1_cmd = RACK_STOP
            self.rp2_cmd = RACK_STOP

            if (
                self.front_rack_distance
                <= FINAL_FORWARD_LIDAR_STOP_DISTANCE
            ):

                self.left_traction = 0.0
                self.right_traction = 0.0

                self.rp1_cmd = RACK_STOP
                self.rp2_cmd = RACK_DIRECTION_1

                self.triangle_stage = 4

                self.get_logger().info(
                    'STAGE 3 COMPLETE: '
                    f'Lidar [0]={self.front_rack_distance:.2f} '
                    f'<= {FINAL_FORWARD_LIDAR_STOP_DISTANCE}. '
                    'Traction stopped; Stage 4 started: '
                    'RP2 direction 1 (same as R1), until RP2 limit.'
                )

            else:

                self.left_traction = FINAL_FORWARD_SPEED
                self.right_traction = FINAL_FORWARD_SPEED

        # ========================================================
        # STAGE 4
        # Execute R1 movement: RP2 direction 1 only.
        # Advance when RP2 limit switch activates.
        # ========================================================

        elif self.triangle_stage == 4:

            self.left_traction = 0.0
            self.right_traction = 0.0

            self.rp1_cmd = RACK_STOP

            if self.rp2_limit:

                self.rp2_cmd = RACK_STOP
                self.triangle_stage = 5

                self.get_logger().info(
                    'STAGE 4 COMPLETE: RP2 limit switch activated. '
                    'RP2 stopped; Stage 5 forward movement starting.'
                )

            else:

                self.rp2_cmd = RACK_DIRECTION_1

        # ========================================================
        # STAGE 5
        # Move the robot forward until lidar [1] < 140.
        # Speed is CLIMB_FORWARD_SPEED (-20).
        # ========================================================

        elif self.triangle_stage == 5:

            self.rp1_cmd = RACK_STOP
            self.rp2_cmd = RACK_STOP

            if self.back_rack_distance < 140.0:

                self.left_traction = 0.0
                self.right_traction = 0.0

                self.rp1_cmd = RACK_DIRECTION_1
                self.rp2_cmd = RACK_STOP

                self.triangle_stage = 6

                self.get_logger().info(
                    'STAGE 5 COMPLETE: '
                    f'Lidar [1]={self.back_rack_distance:.2f} < 140. '
                    'Traction stopped; Stage 6 started: '
                    'RP1 direction 1 (same as L1), until RP1 limit.'
                )

            else:

                self.left_traction = CLIMB_FORWARD_SPEED
                self.right_traction = CLIMB_FORWARD_SPEED

        # ========================================================
        # STAGE 6
        # Execute L1 movement: RP1 direction 1 only.
        # Advance when RP1 limit switch activates.
        # ========================================================

        elif self.triangle_stage == 6:

            self.left_traction = 0.0
            self.right_traction = 0.0

            self.rp2_cmd = RACK_STOP

            if self.rp1_limit:

                self.rp1_cmd = RACK_STOP
                self.triangle_stage = 7

                self.get_logger().info(
                    'STAGE 6 COMPLETE: RP1 limit switch activated. '
                    'RP1 stopped; Stage 7 forward movement starting.'
                )

            else:

                self.rp1_cmd = RACK_DIRECTION_1

        # ========================================================
        # STAGE 7
        # Move forward until lidar [2] <= 140.
        # ========================================================

        elif self.triangle_stage == 7:

            self.rp1_cmd = RACK_STOP
            self.rp2_cmd = RACK_STOP

            if self.forward_lidar <= 140.0:

                self.left_traction = 0.0
                self.right_traction = 0.0

                self.triangle_active = False
                self.triangle_stage = 0

                self.get_logger().info(
                    'STAGE 7 COMPLETE: '
                    f'Lidar [2]={self.forward_lidar:.2f} <= 140. '
                    'Traction stopped; both racks held. '
                    'Autonomous sequence complete.'
                )

            else:

                self.left_traction = CLIMB_FORWARD_SPEED
                self.right_traction = CLIMB_FORWARD_SPEED

    # ============================================================
    # JOYSTICK CALLBACK
    # ============================================================

    def joy_cb(self, msg):

        axes = msg.axes
        buttons = msg.buttons

        if not self.triangle_active:

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

        def pressed(index):

            return (
                len(buttons) > index
                and buttons[index] == 1
            )

        l2_pressed = False

        if len(axes) > self.AXIS_L2:
            l2_pressed = axes[self.AXIS_L2] < 0.0

        r2_pressed = False

        if len(axes) > self.AXIS_R2:
            r2_pressed = axes[self.AXIS_R2] < 0.0

        # TRIANGLE: Same autonomous sequence as /climb_state=True.

        if (
            pressed(self.BTN_TRIANGLE)
            and not self.prev_buttons[self.BTN_TRIANGLE]
        ):

            self.start_autonomous_climb('TRIANGLE')

        # CROSS: Both racks direction 1.

        if (
            pressed(self.BTN_CROSS)
            and not self.prev_buttons[self.BTN_CROSS]
        ):

            self.rp1_cmd = RACK_DIRECTION_1
            self.rp2_cmd = RACK_DIRECTION_1

            self.triangle_active = False
            self.triangle_stage = 0

            self.get_logger().info(
                'CROSS: RP1 + RP2 direction 1'
            )

        # L1: RP1 direction 1 only.

        if (
            pressed(self.BTN_L1)
            and not self.prev_buttons[self.BTN_L1]
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
            self.triangle_stage = 0

        # L2: RP1 direction 2 only.

        if l2_pressed and not self.prev_l2:

            self.rp1_cmd = RACK_DIRECTION_2
            self.rp2_cmd = RACK_STOP

            self.triangle_active = False
            self.triangle_stage = 0

            self.get_logger().info(
                'L2: RP1 direction 2'
            )

        # R1: RP2 direction 1 only.

        if (
            pressed(self.BTN_R1)
            and not self.prev_buttons[self.BTN_R1]
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
            self.triangle_stage = 0

        # R2: RP2 direction 2 only.

        if r2_pressed and not self.prev_r2:

            self.rp1_cmd = RACK_STOP
            self.rp2_cmd = RACK_DIRECTION_2

            self.triangle_active = False
            self.triangle_stage = 0

            self.get_logger().info(
                'R2: RP2 direction 2'
            )

        # CIRCLE: Stop both racks and hold position.

        if (
            pressed(self.BTN_CIRCLE)
            and not self.prev_buttons[self.BTN_CIRCLE]
        ):

            self.rp1_cmd = RACK_STOP
            self.rp2_cmd = RACK_STOP

            self.left_traction = 0.0
            self.right_traction = 0.0

            self.triangle_active = False
            self.triangle_stage = 0

            self.get_logger().info(
                'CIRCLE: Stop + PID hold'
            )

        for i in range(min(len(buttons), 6)):
            self.prev_buttons[i] = buttons[i]

        self.prev_l2 = l2_pressed
        self.prev_r2 = r2_pressed

    # ============================================================
    # PUBLISH
    # ============================================================

    def tick(self):

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