#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
from geometry_msgs.msg import Vector3


class AlignmentNode(Node):

    def __init__(self):
        super().__init__('alignment_node')
        self.bb_sub = self.create_subscription(Vector3, 'bb_vals', self.callback, 10)
        self.error_pub = self.create_publisher(Float32MultiArray, 'target_rpm', 10)
        self.get_logger().info("alignment node started.")

        self.min_length = 1.1
        self.max_length = 1.15
        self.margin1 = 220
        self.margin2 = 450
        self.width = 640
        
    def mapValue(self, input, input_lowerange, input_upperrange, output_lowerrnage, output_upperrange):
    
        mapped_value = (input - input_lowerange) *(output_upperrange - output_lowerrnage) /(input_upperrange - input_lowerange)+ output_lowerrnage
        return max(output_lowerrnage, min(output_upperrange, mapped_value))
    
    def callback(self, msg):
        x1, x2, length = msg.x, msg.y, msg.z

        # Zero signal guard: stop all motors immediately
        if x1 == 0.0 and x2 == 0.0 and length == 0.0:
            power_msg = Float32MultiArray()
            power_msg.data = [0.0, 0.0, 0.0, 0.0]
            self.error_pub.publish(power_msg)
            return

        xdifference = 0.0
        lengthDifference = 0.0

        # Distance error (Length check)
        if length < self.min_length:
            lengthDifference = self.min_length - length    # Target too far -> Move Forward (+)
        elif length > self.max_length:
            lengthDifference = self.max_length - length    # Target too close -> Move Backward (-)

        # Lateral error (X alignment check)
        if x1 < self.margin1:
            xdifference = self.margin1 - x1               # Target on left -> Strafe Left (-)
        elif x2 > self.margin2:
            xdifference = self.margin2 - x2               # Target on right -> Strafe Right (+)

        # Command Mapping (-250 to 250 RPM range)
        forward = self.mapValue(lengthDifference, -0.5, 0.5, -250, 250)
        strafe = self.mapValue(xdifference, -150, 150, -100, 100)
        strafe = -strafe
        turn = 0.0

        # --- Corrected Mecanum Inverse Kinematics ---
        # FL = +Forward + Strafe + Turn
        # FR = +Forward - Strafe - Turn
        # BL = +Forward - Strafe + Turn
        # BR = +Forward + Strafe - Turn
        fl = -(forward + strafe + turn)
        fr = forward - strafe - turn
        bl = -(forward - strafe + turn)
        br = forward + strafe - turn

        # Power normalization (prevents commands from overflowing past max RPM)
        max_power = max(abs(fl), abs(fr), abs(bl), abs(br), 250.0)
        if max_power > 250.0:
            fl = (fl / max_power) * 250.0
            fr = (fr / max_power) * 250.0
            bl = (bl / max_power) * 250.0
            br = (br / max_power) * 250.0

        power_msg = Float32MultiArray()
        power_msg.data = [float(fl), float(fr), float(bl), float(br)]
        print(f"xdiff{xdifference},ydiff{lengthDifference}")
        self.error_pub.publish(power_msg)


def main(args=None):
    rclpy.init(args=args)
    node = AlignmentNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()