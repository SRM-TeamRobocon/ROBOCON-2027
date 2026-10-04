#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray, Bool
from geometry_msgs.msg import Vector3


class AlignmentNode(Node):

    def __init__(self):
        super().__init__('alignment_node')
        
        # Subscribers
        self.bb_sub = self.create_subscription(Vector3, 'bb_vals', self.callback, 10)
        self.start_align_sub = self.create_subscription(Bool, 'start_align', self.start_align_callback, 10)
        
        # Publishers
        self.error_pub = self.create_publisher(Float32MultiArray, 'target_rpm', 10)
        self.align_confirmed_pub = self.create_publisher(Bool, 'align_confirmed', 10)

        # Threshold Parameters
        self.min_length = 1.1
        self.max_length = 1.15
        self.margin1 = 220
        self.margin2 = 450
        self.width = 640

        # State Variables
        self.align_confirmed = False
        self.start_align = False

        self.get_logger().info("Alignment node initialized and waiting for start signal.")

    def start_align_callback(self, msg: Bool):
        """Updates internal enable flag and resets completion status on rising edge."""
        previous_state = self.start_align
        self.start_align = msg.data

        if self.start_align and not previous_state:
            self.align_confirmed = False
            self.publish_align_status(False)
            self.get_logger().info("Alignment task started.")
        elif not self.start_align and previous_state:
            self.stop_motors()
            self.publish_align_status(False)
            self.get_logger().info("Alignment task stopped.")

    def map_value(self, val: float, in_min: float, in_max: float, out_min: float, out_max: float) -> float:
        """Linear mapping utility with strict output clamping."""
        if in_max == in_min:
            return out_min
        mapped = (val - in_min) * (out_max - out_min) / (in_max - in_min) + out_min
        return max(out_min, min(out_max, mapped))

    def stop_motors(self):
        """Helper to safely stop all four wheels."""
        power_msg = Float32MultiArray()
        power_msg.data = [0.0, 0.0, 0.0, 0.0]
        self.error_pub.publish(power_msg)

    def publish_align_status(self, confirmed: bool):
        """Helper to publish alignment status."""
        msg = Bool()
        msg.data = confirmed
        self.align_confirmed_pub.publish(msg)

    def callback(self, msg: Vector3):
        # 1. Gate execution: Do nothing if alignment isn't active
        if not self.start_align:
            return

        x1, x2, length = msg.x, msg.y, msg.z

        # 2. Zero signal guard: stop all motors immediately
        if x1 == 0.0 and x2 == 0.0 and length == 0.0:
            self.stop_motors()
            return

        xdifference = 0.0
        lengthDifference = 0.0

        # 3. Distance error calculation
        if length < self.min_length:
            lengthDifference = self.min_length - length    # Target too far -> Move Forward (+)
        elif length > self.max_length:
            lengthDifference = self.max_length - length    # Target too close -> Move Backward (-)

        # 4. Lateral error calculation
        if x1 < self.margin1:
            xdifference = self.margin1 - x1               # Target on left -> Strafe Left (-)
        elif x2 > self.margin2:
            xdifference = self.margin2 - x2               # Target on right -> Strafe Right (+)

        # 5. Velocity / Command Mapping (-250 to 250 RPM range)
        forward = self.map_value(lengthDifference, -0.5, 0.5, -250.0, 250.0)
        strafe = -self.map_value(xdifference, -150.0, 150.0, -100.0, 100.0)
        turn = 0.0

        # 6. Mecanum Kinematics
        fl = -(forward + strafe + turn)
        fr = forward - strafe - turn
        bl = -(forward - strafe + turn)
        br = forward + strafe - turn

        # 7. Power Normalization
        max_power = max(abs(fl), abs(fr), abs(bl), abs(br), 250.0)
        if max_power > 250.0:
            fl = (fl / max_power) * 250.0
            fr = (fr / max_power) * 250.0
            bl = (bl / max_power) * 250.0
            br = (br / max_power) * 250.0

        # 8. Check if motor outputs are within the 1.0 threshold
        if abs(fl) <= 1.0 and abs(fr) <= 1.0 and abs(bl) <= 1.0 and abs(br) <= 1.0:
            self.stop_motors()
            if not self.align_confirmed:
                self.align_confirmed = True
                self.publish_align_status(True)
                self.get_logger().info("Target aligned! Motor commands within absolute threshold of 1.0.")
            else:
                self.publish_align_status(True)
            return

        # Target drifted outside tolerance: reset alignment confirmation
        if self.align_confirmed:
            self.align_confirmed = False
            self.publish_align_status(False)

        # 9. Publish target RPM
        power_msg = Float32MultiArray()
        power_msg.data = [float(fl), float(fr), float(bl), float(br)]
        self.error_pub.publish(power_msg)


def main(args=None):
    rclpy.init(args=args)
    node = AlignmentNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.stop_motors()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()