#!/usr/bin/env python3

import rclpy
import time
from rclpy.node import Node

from sensor_msgs.msg import Joy
from std_msgs.msg import Float32, Float32MultiArray

# ============================================================
# SETTINGS
# ============================================================

JOY_TOPIC = '/joy'
RPM_TOPIC = '/rpm_sp'

SENSOR_TOPIC = '/sensor_values'
LIMIT_SWITCH_TOPIC = '/limit_switch'
ENCODER_TOPIC = '/encoder_ticks'

RP1_TARGET_TOPIC = '/rp1_target_ticks'
RP2_TARGET_TOPIC = '/rp2_target_ticks'

RP1_THRESHOLD_TOPIC = '/rp1_threshold'
RP2_THRESHOLD_TOPIC = '/rp2_threshold'

PUBLISH_PERIOD = 0.02

# Rack target configuration
DEFAULT_RP1_TARGET_TICKS = 3250.0
DEFAULT_RP2_TARGET_TICKS = 3250.0

DEFAULT_RP1_THRESHOLD_TICKS = 0.0
DEFAULT_RP2_THRESHOLD_TICKS = 0.0

# Sensor indexes
FRONT_RACK_INDEX = 0
BACK_RACK_INDEX = 1
FORWARD_LIDAR_INDEX = 2
BACKWARD_LIDAR_INDEX = 3
FIFTH_LIDAR_INDEX = 4
YAW_INDEX = 5
ROLL_INDEX = 6

FRONT_RACK_STOP_DISTANCE = 370.0
BACK_RACK_STOP_DISTANCE = 370.0

# Joystick
JOYSTICK_SCALE = 128.0

LEFT_X_DEADZONE = 5.0
LEFT_Y_DEADZONE = 5.0

# Rack commands
RACK_DIRECTION_1 = 1.0
RACK_DIRECTION_2 = -1.0
RACK_STOP = 0.0

RACK_RELATIVE_DIRECTION_2 = -2.0

# Mecanum
TRACTION_MAX = 128.0

# Automated Square process
PROCESS_REVERSE_VELOCITY = 30.0
PROCESS_LIDAR_THRESHOLD = 390.0
PROCESS_FINAL_REVERSE_SECONDS = 3.0

# ============================================================
# NODE
# ============================================================

class JetsonRackController(Node):

    def __init__(self):

        super().__init__('jetson_rack_controller')

        # ====================================================
        # RACK TARGETS
        # ====================================================

        self.rp1_target_ticks = DEFAULT_RP1_TARGET_TICKS
        self.rp2_target_ticks = DEFAULT_RP2_TARGET_TICKS

        self.rp1_threshold_ticks = DEFAULT_RP1_THRESHOLD_TICKS
        self.rp2_threshold_ticks = DEFAULT_RP2_THRESHOLD_TICKS

        # ====================================================
        # PUBLISHER
        # ====================================================

        self.pub_cmd = self.create_publisher(
            Float32MultiArray,
            RPM_TOPIC,
            10
        )

        # ====================================================
        # SUBSCRIBERS
        # ====================================================

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
            Float32MultiArray,
            ENCODER_TOPIC,
            self.encoder_ticks_callback,
            10
        )

        self.create_subscription(
            Float32,
            RP1_TARGET_TOPIC,
            self.rp1_target_callback,
            10
        )

        self.create_subscription(
            Float32,
            RP2_TARGET_TOPIC,
            self.rp2_target_callback,
            10
        )

        self.create_subscription(
            Float32,
            RP1_THRESHOLD_TOPIC,
            self.rp1_threshold_callback,
            10
        )

        self.create_subscription(
            Float32,
            RP2_THRESHOLD_TOPIC,
            self.rp2_threshold_callback,
            10
        )

        # ====================================================
        # TIMER
        # ====================================================

        self.create_timer(
            PUBLISH_PERIOD,
            self.tick
        )

        # ====================================================
        # PS5 AXIS MAPPING
        # ====================================================

        self.AXIS_LEFT_X = 0
        self.AXIS_LEFT_Y = 1
        self.AXIS_RIGHT_X = 2

        self.AXIS_L2 = 3
        self.AXIS_R2 = 4

        self.AXIS_RIGHT_Y = 5

        # ====================================================
        # PS5 BUTTON MAPPING
        # ====================================================

        self.BTN_SQUARE = 0
        self.BTN_CROSS = 1
        self.BTN_CIRCLE = 2
        self.BTN_TRIANGLE = 3
        self.BTN_L1 = 4
        self.BTN_R1 = 5

        # ====================================================
        # RACK COMMANDS
        # ====================================================

        self.rp1_cmd = RACK_STOP
        self.rp2_cmd = RACK_STOP

        self.rp1_limit = False
        self.rp2_limit = False

        self.rp1_pending_relative = False
        self.rp2_pending_relative = False

        # ====================================================
        # MECANUM DRIVE
        # ====================================================

        self.left_x = 0.0
        self.left_y = 0.0
        self.right_x = 0.0

        self.mecanum_speeds = [
            0.0,
            0.0,
            0.0,
            0.0
        ]

        # ====================================================
        # SENSOR VALUES
        # ====================================================

        self.front_rack_distance = 0.0
        self.back_rack_distance = 0.0

        self.forward_lidar = 0.0
        self.backward_lidar = 0.0
        self.fifth_lidar = 0.0

        self.yaw = 0.0
        self.roll = 0.0

        self.triangle_active = False

        # ====================================================
        # AUTOMATED SQUARE PROCESS
        # ====================================================

        # idle -> stage1_reverse -> stage2_rp1
        # -> stage3_reverse -> stage4_rp2
        # -> stage5_reverse -> stage5_extend -> complete

        self.process_state = 'idle'
        self.stage5_reverse_start_time = None

        # ====================================================
        # JOYSTICK STATE
        # ====================================================

        self.prev_buttons = [0] * 6

        self.prev_l2 = False
        self.prev_r2 = False

        # ====================================================
        # ENCODER VALUES
        # ====================================================

        self.encoder_rp1 = 0.0
        self.encoder_rp2 = 0.0

        # ====================================================
        # RELATIVE MOVEMENT TRACKING
        # ====================================================

        self.rp1_move_start = 0.0
        self.rp2_move_start = 0.0

        self.rp1_target_active = False
        self.rp2_target_active = False

        self.rp1_target_reached = False
        self.rp2_target_reached = False

        # ====================================================
        # CAPTURE CLIMB POSITION
        # ====================================================

        self.capture_climb_request = 0.0

        # ====================================================
        # STARTUP
        # ====================================================

        self.get_logger().info(
            '========================================'
        )

        self.get_logger().info(
            'JETSON RACK + MECANUM CONTROLLER STARTED'
        )

        self.get_logger().info(
            f'RP1 Target={self.rp1_target_ticks:.0f} | '
            f'Threshold={self.rp1_threshold_ticks:.0f}'
        )

        self.get_logger().info(
            f'RP2 Target={self.rp2_target_ticks:.0f} | '
            f'Threshold={self.rp2_threshold_ticks:.0f}'
        )

        self.get_logger().info(
            'Mecanum drive enabled: M1 M2 M3 M4'
        )

        self.get_logger().info(
            'RPM message: 7 values'
        )

        self.get_logger().info(
            '========================================'
        )

    # ========================================================
    # TARGET CALLBACKS
    # ========================================================

    def rp1_target_callback(self, msg):

        value = float(msg.data)

        if value <= 0:
            self.get_logger().warn(
                f'Invalid RP1 target: {value}'
            )
            return

        self.rp1_target_ticks = value

        self.get_logger().info(
            f'RP1 TARGET UPDATED: {self.rp1_target_ticks:.0f}'
        )

    def rp2_target_callback(self, msg):

        value = float(msg.data)

        if value <= 0:
            self.get_logger().warn(
                f'Invalid RP2 target: {value}'
            )
            return

        self.rp2_target_ticks = value

        self.get_logger().info(
            f'RP2 TARGET UPDATED: {self.rp2_target_ticks:.0f}'
        )

    def rp1_threshold_callback(self, msg):

        value = float(msg.data)

        if value < 0:
            self.get_logger().warn(
                f'Invalid RP1 threshold: {value}'
            )
            return

        self.rp1_threshold_ticks = value

        self.get_logger().info(
            f'RP1 THRESHOLD UPDATED: {value:.0f}'
        )

    def rp2_threshold_callback(self, msg):

        value = float(msg.data)

        if value < 0:
            self.get_logger().warn(
                f'Invalid RP2 threshold: {value}'
            )
            return

        self.rp2_threshold_ticks = value

        self.get_logger().info(
            f'RP2 THRESHOLD UPDATED: {value:.0f}'
        )

    # ========================================================
    # LIMIT SWITCH CALLBACK
    # ========================================================

    def limit_switch_values(self, msg):

        if len(msg.data) < 2:
            self.get_logger().warn(
                '/limit_switch requires 2 values'
            )
            return

        self.rp1_limit = msg.data[0] > 0.5
        self.rp2_limit = msg.data[1] > 0.5

        if (
            self.rp1_limit and
            self.rp1_cmd == RACK_DIRECTION_1
        ):
            self.rp1_cmd = RACK_STOP

            self.get_logger().info(
                'RP1 LIMIT ACTIVATED: Stop + PID hold'
            )

        if (
            self.rp2_limit and
            self.rp2_cmd == RACK_DIRECTION_1
        ):
            self.rp2_cmd = RACK_STOP

            self.get_logger().info(
                'RP2 LIMIT ACTIVATED: Stop + PID hold'
            )

        if (
            self.process_state == 'stage5_extend' and
            self.rp1_limit and self.rp2_limit
        ):
            self.finish_automated_process()

    # ========================================================
    # AUTOMATED PROCESS COMPLETE
    # ========================================================

    def finish_automated_process(self):

        self.process_state = 'complete'

        self.rp1_cmd = RACK_STOP
        self.rp2_cmd = RACK_STOP

        self.rp1_pending_relative = False
        self.rp2_pending_relative = False

        self.rp1_target_active = False
        self.rp2_target_active = False

        self.get_logger().info(
            '========================================\n'
            '       AUTOMATED PROCESS COMPLETE\n'
            'Both limit switches activated.\n'
            'RP1 stopped + PID hold.\n'
            'RP2 stopped + PID hold.\n'
            'Traction stopped.\n'
            '========================================'
        )

    # ========================================================
    # ENCODER CALLBACK
    # ========================================================

    def encoder_ticks_callback(self, msg):

        if len(msg.data) < 2:
            return

        self.encoder_rp1 = float(msg.data[0])
        self.encoder_rp2 = float(msg.data[1])

        # ====================================================
        # RP1 COMPLETION TRACKING
        # ====================================================

        if (
            self.rp1_target_active and
            not self.rp1_target_reached
        ):

            travelled = abs(
                self.encoder_rp1 - self.rp1_move_start
            )

            required_travel = (
                self.rp1_target_ticks -
                self.rp1_threshold_ticks
            )

            if required_travel <= 0:
                self.get_logger().warn(
                    'RP1 required travel <= 0. Check target/threshold.'
                )
                self.rp1_target_active = False
                self.rp1_cmd = RACK_STOP
                return

            if travelled >= required_travel:

                self.rp1_target_reached = True
                self.rp1_target_active = False

                self.rp1_cmd = RACK_STOP

                self.get_logger().info(
                    '\n'
                    '========================================\n'
                    '          RP1 TARGET REACHED\n'
                    '========================================\n'
                    f'Start Encoder  : {self.rp1_move_start:.0f}\n'
                    f'Current Encoder: {self.encoder_rp1:.0f}\n'
                    f'Travelled      : {travelled:.0f} ticks\n'
                    f'Required       : {required_travel:.0f} ticks\n'
                    'Status         : PID HOLD\n'
                    '========================================'
                )

                if self.process_state == 'stage2_rp1':

                    self.process_state = 'stage3_reverse'

                    self.rp1_cmd = RACK_STOP
                    self.rp2_cmd = RACK_STOP

                    self.rp1_pending_relative = False
                    self.rp2_pending_relative = False

                    self.get_logger().info(
                        '========================================\n'
                        '       STAGE 2 COMPLETED\n'
                        'RP1 stopped + PID hold.\n'
                        'STAGE 3: Reverse until fifth LiDAR > 390\n'
                        '========================================'
                    )

        # ====================================================
        # RP2 COMPLETION TRACKING
        # ====================================================

        if (
            self.rp2_target_active and
            not self.rp2_target_reached
        ):

            travelled = abs(
                self.encoder_rp2 - self.rp2_move_start
            )

            required_travel = (
                self.rp2_target_ticks -
                self.rp2_threshold_ticks
            )

            if required_travel <= 0:
                self.get_logger().warn(
                    'RP2 required travel <= 0. Check target/threshold.'
                )
                self.rp2_target_active = False
                self.rp2_cmd = RACK_STOP
                return

            if travelled >= required_travel:

                self.rp2_target_reached = True
                self.rp2_target_active = False

                self.rp2_cmd = RACK_STOP

                if self.process_state == 'stage4_rp2':

                    self.process_state = 'stage5_reverse'

                    self.stage5_reverse_start_time = time.monotonic()

                    self.rp1_cmd = RACK_STOP
                    self.rp2_cmd = RACK_STOP

                    self.rp1_pending_relative = False
                    self.rp2_pending_relative = False

                    self.get_logger().info(
                        '========================================\n'
                        '       STAGE 4 COMPLETED\n'
                        'RP2 stopped + PID hold.\n'
                        'STAGE 5: Reverse for 5 seconds\n'
                        'Then both racks direction 1 until\n'
                        'both limit switches activate.\n'
                        '========================================'
                    )

                self.get_logger().info(
                    '\n'
                    '========================================\n'
                    '          RP2 TARGET REACHED\n'
                    '========================================\n'
                    f'Start Encoder  : {self.rp2_move_start:.0f}\n'
                    f'Current Encoder: {self.encoder_rp2:.0f}\n'
                    f'Travelled      : {travelled:.0f} ticks\n'
                    f'Required       : {required_travel:.0f} ticks\n'
                    'Status         : PID HOLD\n'
                    '========================================'
                )

    # ========================================================
    # SENSOR CALLBACK
    # ========================================================

    def sensor_values(self, msg):

        if len(msg.data) <= ROLL_INDEX:
            self.get_logger().warn(
                '/sensor_values requires 7 values: 5 LiDARs, yaw, roll'
            )
            return

        self.front_rack_distance = msg.data[FRONT_RACK_INDEX]
        self.back_rack_distance = msg.data[BACK_RACK_INDEX]

        self.forward_lidar = msg.data[FORWARD_LIDAR_INDEX]
        self.backward_lidar = msg.data[BACKWARD_LIDAR_INDEX]
        self.fifth_lidar = msg.data[FIFTH_LIDAR_INDEX]

        # ====================================================
        # STAGE 1 -> STAGE 2
        # ====================================================

        if (
            self.process_state == 'stage1_reverse' and
            self.backward_lidar > PROCESS_LIDAR_THRESHOLD
        ):

            self.process_state = 'stage2_rp1'

            self.rp1_cmd = RACK_STOP
            self.rp2_cmd = RACK_STOP

            self.rp1_pending_relative = True
            self.rp2_pending_relative = False

            self.rp1_target_active = False
            self.rp2_target_active = False

            self.rp1_target_reached = False

            self.get_logger().info(
                '========================================\n'
                '       STAGE 1 COMPLETED\n'
                f'Backward LiDAR [3]: {self.backward_lidar:.2f}\n'
                'Traction stopped. Starting RP1 movement.\n'
                '========================================'
            )

        # ====================================================
        # STAGE 3 -> STAGE 4
        # ====================================================

        if (
            self.process_state == 'stage3_reverse' and
            self.fifth_lidar > PROCESS_LIDAR_THRESHOLD
        ):

            self.process_state = 'stage4_rp2'

            self.rp1_cmd = RACK_STOP
            self.rp2_cmd = RACK_STOP

            self.rp1_pending_relative = False
            self.rp2_pending_relative = True

            self.rp1_target_active = False
            self.rp2_target_active = False

            self.rp2_target_reached = False

            self.get_logger().info(
                '========================================\n'
                '       STAGE 3 COMPLETED\n'
                f'Fifth LiDAR [4]: {self.fifth_lidar:.2f}\n'
                'Traction stopped. Starting RP2 movement.\n'
                '========================================'
            )

        self.yaw = msg.data[YAW_INDEX]
        self.roll = msg.data[ROLL_INDEX]

        # ====================================================
        # TRIANGLE AUTO STOP
        # ====================================================

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
                    '========================================\n'
                    '       TRIANGLE AUTO STOP\n'
                    '========================================\n'
                    f'Front Distance: {self.front_rack_distance:.2f}\n'
                    f'Back Distance : {self.back_rack_distance:.2f}\n'
                    'Both racks stopped.\n'
                    '========================================'
                )

    # ========================================================
    # MECANUM INVERSE KINEMATICS
    # ========================================================

    def inverse_kinematics(self, vx, vy, omega):

        m1 = vx - vy - omega
        m2 = vx + vy + omega
        m3 = vx + vy - omega
        m4 = vx - vy + omega

        speeds = [m1, m2, m3, m4]

        max_speed = max(
            abs(value) for value in speeds
        )

        if max_speed > TRACTION_MAX:

            scale = TRACTION_MAX / max_speed

            speeds = [
                value * scale for value in speeds
            ]

        return speeds

    # ========================================================
    # JOYSTICK CALLBACK
    # ========================================================

    def joy_cb(self, msg):

        axes = msg.axes
        buttons = msg.buttons

        # ====================================================
        # MECANUM INPUTS
        # ====================================================

        if len(axes) > self.AXIS_LEFT_X:

            self.left_x = (
                axes[self.AXIS_LEFT_X] * JOYSTICK_SCALE
            )

            if abs(self.left_x) < LEFT_X_DEADZONE:
                self.left_x = 0.0

        if len(axes) > self.AXIS_LEFT_Y:

            self.left_y = (
                axes[self.AXIS_LEFT_Y] * JOYSTICK_SCALE
            )

            if abs(self.left_y) < LEFT_Y_DEADZONE:
                self.left_y = 0.0

        self.right_x = 0.0

        if len(axes) > self.AXIS_RIGHT_X:

            self.right_x = (
                axes[self.AXIS_RIGHT_X] * JOYSTICK_SCALE
            )

            if abs(self.right_x) < LEFT_X_DEADZONE:
                self.right_x = 0.0

        vx = -self.left_y
        vy = self.left_x
        omega = self.right_x

        self.mecanum_speeds = self.inverse_kinematics(
            vx,
            vy,
            omega
        )

        # ====================================================
        # BUTTON HELPER
        # ====================================================

        def pressed(index):

            return (
                len(buttons) > index and
                buttons[index] == 1
            )

        # ====================================================
        # L2/R2 TRIGGERS
        # ====================================================

        l2_pressed = (
            len(axes) > self.AXIS_L2 and
            axes[self.AXIS_L2] < 0.0
        )

        r2_pressed = (
            len(axes) > self.AXIS_R2 and
            axes[self.AXIS_R2] < 0.0
        )

        # ====================================================
        # SQUARE: START AUTOMATED PROCESS
        # ====================================================

        if (
            pressed(self.BTN_SQUARE) and
            not self.prev_buttons[self.BTN_SQUARE] and
            self.process_state in ('idle', 'complete')
        ):

            rp1_required_move = (
                self.rp1_target_ticks -
                self.rp1_threshold_ticks
            )

            rp2_required_move = (
                self.rp2_target_ticks -
                self.rp2_threshold_ticks
            )

            if rp1_required_move <= 0 or rp2_required_move <= 0:

                self.get_logger().warn(
                    'Both rack targets must be greater than their thresholds'
                )

            else:

                self.rp1_cmd = RACK_STOP
                self.rp2_cmd = RACK_STOP

                self.rp1_pending_relative = False
                self.rp2_pending_relative = False

                self.rp1_target_active = False
                self.rp2_target_active = False

                self.rp1_target_reached = False
                self.rp2_target_reached = False

                self.stage5_reverse_start_time = None

                self.triangle_active = False

                self.process_state = 'stage1_reverse'

                self.get_logger().info(
                    '========================================\n'
                    '     SQUARE AUTOMATED PROCESS STARTED\n'
                    'STAGE 1: Reverse until LiDAR [3] > 390\n'
                    'STAGE 2: RP1 relative encoder movement\n'
                    f'RP1 target: {self.rp1_target_ticks:.0f}\n'
                    f'RP1 threshold: {self.rp1_threshold_ticks:.0f}\n'
                    'STAGE 3: Reverse until LiDAR [4] > 390\n'
                    'STAGE 4: RP2 relative encoder movement\n'
                    f'RP2 target: {self.rp2_target_ticks:.0f}\n'
                    f'RP2 threshold: {self.rp2_threshold_ticks:.0f}\n'
                    'STAGE 5: Reverse 5 seconds, then extend both racks\n'
                    '========================================'
                )

        # Lock manual rack controls while the automated process runs.
        if self.process_state in (
            'stage1_reverse', 'stage2_rp1', 'stage3_reverse',
            'stage4_rp2', 'stage5_reverse', 'stage5_extend', 'complete'
        ):

            for i in range(min(len(buttons), 6)):
                self.prev_buttons[i] = buttons[i]

            self.prev_l2 = l2_pressed
            self.prev_r2 = r2_pressed

            return

        # ====================================================
        # TRIANGLE: BOTH RACKS DIRECTION 2
        # ====================================================

        if (
            pressed(self.BTN_TRIANGLE) and
            not self.prev_buttons[self.BTN_TRIANGLE]
        ):

            self.rp1_pending_relative = False
            self.rp2_pending_relative = False

            self.rp1_target_active = False
            self.rp2_target_active = False

            self.rp1_cmd = RACK_DIRECTION_2
            self.rp2_cmd = RACK_DIRECTION_2

            self.triangle_active = True

            self.get_logger().info(
                'TRIANGLE: Both racks direction 2'
            )

        # ====================================================
        # CROSS: BOTH RACKS DIRECTION 1
        # ====================================================

        if (
            pressed(self.BTN_CROSS) and
            not self.prev_buttons[self.BTN_CROSS]
        ):

            self.rp1_pending_relative = False
            self.rp2_pending_relative = False

            self.rp1_target_active = False
            self.rp2_target_active = False

            self.rp1_cmd = RACK_DIRECTION_1
            self.rp2_cmd = RACK_DIRECTION_1

            self.triangle_active = False

            self.get_logger().info(
                'CROSS: Both racks direction 1'
            )

        # ====================================================
        # L1: RP1 CONTINUOUS DIRECTION 1
        # ====================================================

        if (
            pressed(self.BTN_L1) and
            not self.prev_buttons[self.BTN_L1]
        ):

            self.rp2_cmd = RACK_STOP
            self.rp2_pending_relative = False

            self.rp1_target_active = False
            self.rp1_pending_relative = False

            if self.rp1_limit:

                self.rp1_cmd = RACK_STOP

                self.get_logger().info(
                    'L1: RP1 limit active'
                )

            else:

                self.rp1_cmd = RACK_DIRECTION_1

                self.get_logger().info(
                    'L1: RP1 direction 1'
                )

            self.triangle_active = False

        # ====================================================
        # L2: RP1 RELATIVE DIRECTION 2
        # ====================================================

        if l2_pressed and not self.prev_l2:

            required_move = (
                self.rp1_target_ticks -
                self.rp1_threshold_ticks
            )

            if required_move <= 0:

                self.get_logger().warn(
                    'RP1 target must be greater than threshold'
                )

            else:

                self.rp2_cmd = RACK_STOP
                self.rp2_pending_relative = False
                self.rp2_target_active = False

                self.rp1_cmd = RACK_STOP
                self.rp1_pending_relative = True

                self.rp1_target_active = False
                self.rp1_target_reached = False

                self.triangle_active = False

                self.get_logger().info(
                    '\n'
                    '========================================\n'
                    '         RP1 MOVEMENT REQUESTED\n'
                    '========================================\n'
                    f'Current Encoder: {self.encoder_rp1:.0f}\n'
                    f'Target          : {self.rp1_target_ticks:.0f}\n'
                    f'Threshold       : {self.rp1_threshold_ticks:.0f}\n'
                    f'Required Move   : {required_move:.0f} ticks\n'
                    '========================================'
                )

        # ====================================================
        # R1: RP2 CONTINUOUS DIRECTION 1
        # ====================================================

        if (
            pressed(self.BTN_R1) and
            not self.prev_buttons[self.BTN_R1]
        ):

            self.rp1_cmd = RACK_STOP
            self.rp1_pending_relative = False

            self.rp2_target_active = False
            self.rp2_pending_relative = False

            if self.rp2_limit:

                self.rp2_cmd = RACK_STOP

                self.get_logger().info(
                    'R1: RP2 limit active'
                )

            else:

                self.rp2_cmd = RACK_DIRECTION_1

                self.get_logger().info(
                    'R1: RP2 direction 1'
                )

            self.triangle_active = False

        # ====================================================
        # R2: RP2 RELATIVE DIRECTION 2
        # ====================================================

        if r2_pressed and not self.prev_r2:

            required_move = (
                self.rp2_target_ticks -
                self.rp2_threshold_ticks
            )

            if required_move <= 0:

                self.get_logger().warn(
                    'RP2 target must be greater than threshold'
                )

            else:

                self.rp1_cmd = RACK_STOP
                self.rp1_pending_relative = False
                self.rp1_target_active = False

                self.rp2_cmd = RACK_STOP
                self.rp2_pending_relative = True

                self.rp2_target_active = False
                self.rp2_target_reached = False

                self.triangle_active = False

                self.get_logger().info(
                    '\n'
                    '========================================\n'
                    '         RP2 MOVEMENT REQUESTED\n'
                    '========================================\n'
                    f'Current Encoder: {self.encoder_rp2:.0f}\n'
                    f'Target          : {self.rp2_target_ticks:.0f}\n'
                    f'Threshold       : {self.rp2_threshold_ticks:.0f}\n'
                    f'Required Move   : {required_move:.0f} ticks\n'
                    '========================================'
                )

        # ====================================================
        # CIRCLE: STOP BOTH + PID HOLD
        # ====================================================

        if (
            pressed(self.BTN_CIRCLE) and
            not self.prev_buttons[self.BTN_CIRCLE]
        ):

            self.rp1_cmd = RACK_STOP
            self.rp2_cmd = RACK_STOP

            self.rp1_pending_relative = False
            self.rp2_pending_relative = False

            self.rp1_target_active = False
            self.rp2_target_active = False

            self.triangle_active = False

            self.get_logger().info(
                'CIRCLE: Stop both racks + PID hold'
            )

        # ====================================================
        # UPDATE BUTTON STATES
        # ====================================================

        for i in range(min(len(buttons), 6)):
            self.prev_buttons[i] = buttons[i]

        self.prev_l2 = l2_pressed
        self.prev_r2 = r2_pressed

    # ========================================================
    # PUBLISH COMMANDS
    # ========================================================

    def tick(self):

        # ====================================================
        # LIMIT SWITCH PROTECTION
        # ====================================================

        if (
            self.rp1_limit and
            self.rp1_cmd == RACK_DIRECTION_1
        ):
            self.rp1_cmd = RACK_STOP

        if (
            self.rp2_limit and
            self.rp2_cmd == RACK_DIRECTION_1
        ):
            self.rp2_cmd = RACK_STOP

        # ====================================================
        # STAGE 5 TIMER AND FINAL LIMIT-SWITCH COMPLETION
        # ====================================================

        if (
            self.process_state == 'stage5_reverse' and
            self.stage5_reverse_start_time is not None and
            (time.monotonic() - self.stage5_reverse_start_time)
            >= PROCESS_FINAL_REVERSE_SECONDS
        ):

            self.process_state = 'stage5_extend'

            self.rp1_cmd = (
                RACK_STOP if self.rp1_limit else RACK_DIRECTION_1
            )

            self.rp2_cmd = (
                RACK_STOP if self.rp2_limit else RACK_DIRECTION_1
            )

            self.get_logger().info(
                '========================================\n'
                '       STAGE 5 REVERSE COMPLETED\n'
                '5 seconds elapsed.\n'
                'Both racks moving direction 1\n'
                'until both limit switches activate.\n'
                '========================================'
            )

        # Check both switches here as well,
        # in case they were already active.

        if (
            self.process_state == 'stage5_extend' and
            self.rp1_limit and self.rp2_limit
        ):
            self.finish_automated_process()

        # ====================================================
        # AUTOMATED PROCESS TRACTION CONTROL
        # ====================================================

        if self.process_state in (
            'stage1_reverse', 'stage3_reverse', 'stage5_reverse'
        ):

            traction_speeds = self.inverse_kinematics(
                PROCESS_REVERSE_VELOCITY,
                0.0,
                0.0
            )

        elif self.process_state in (
            'stage2_rp1', 'stage4_rp2', 'stage5_extend', 'complete'
        ):

            traction_speeds = [0.0, 0.0, 0.0, 0.0]

        else:

            traction_speeds = self.mecanum_speeds

        # ====================================================
        # PUBLISH SEVEN VALUES
        #
        # [RP1, RP2, M1, M2, M3, M4, CAPTURE_REQUEST]
        # ====================================================

        msg = Float32MultiArray()

        msg.data = [
            float(self.rp1_cmd),
            float(self.rp2_cmd),

            float(traction_speeds[0]),  # M1
            float(traction_speeds[1]),  # M2
            float(traction_speeds[2]),  # M3
            float(traction_speeds[3]),  # M4

            float(self.capture_climb_request)
        ]

        self.pub_cmd.publish(msg)

        # ====================================================
        # START RP1 RELATIVE MOVEMENT AFTER NEUTRAL CYCLE
        # ====================================================

        if self.rp1_pending_relative:

            self.rp1_pending_relative = False

            self.rp1_move_start = self.encoder_rp1
            self.rp1_target_active = True
            self.rp1_target_reached = False

            self.rp1_cmd = RACK_RELATIVE_DIRECTION_2

            self.get_logger().info(
                f'RP1 RELATIVE MOVE STARTED at '
                f'{self.rp1_move_start:.0f} ticks'
            )

        # ====================================================
        # START RP2 RELATIVE MOVEMENT AFTER NEUTRAL CYCLE
        # ====================================================

        if self.rp2_pending_relative:

            self.rp2_pending_relative = False

            self.rp2_move_start = self.encoder_rp2
            self.rp2_target_active = True
            self.rp2_target_reached = False

            self.rp2_cmd = RACK_RELATIVE_DIRECTION_2

            self.get_logger().info(
                f'RP2 RELATIVE MOVE STARTED at '
                f'{self.rp2_move_start:.0f} ticks'
            )


# ============================================================
# MAIN
# ============================================================

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