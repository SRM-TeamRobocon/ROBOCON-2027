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
        self.max_length = 1.2
        self.margin = 150
        self.margin1 = 220
        self.margin2 = 460
        self.width = 640
        
    def mapValue( self, input, input_lowerange, input_upperrange, output_lowerrnage, output_upperrange):
    
        mapped_value = (input - input_lowerange) *(output_upperrange - output_lowerrnage) /(input_upperrange - input_lowerange)+ output_lowerrnage
        return max(output_lowerrnage, min(output_upperrange, mapped_value))
    
    def callback(self, msg):

        xdifference = 0.0
        lengthDifference = 0.0

        

        x1, x2 , length = msg.x, msg.y, msg.z

        if x1 == 0.0 and x2 == 0.0 and length == 0.0:
            power_msg = Float32MultiArray()
            power_msg.data = [0.0, 0.0, 0.0, 0.0]
            self.error_pub.publish(power_msg)
            print("No target detected. Publishing zero RPM [0, 0, 0, 0].")
            return
          #  centre = self.width // 2

        if length < self.min_length:
            lengthDifference = float(self.min_length - length) 
        elif length > self.max_length:
            lengthDifference = float(self.max_length - length) 

            # X alignment
        if x1 < self.margin1:
            xdifference = float(x1 - self.margin1) 
        elif x2 > self.margin2:
            xdifference = float(x2 - self.margin2) 

        # strafe_norm = xdifference / 170.0
        
        # # lengthDifference max cap ~0.55 -> scale to [-1.0, 1.0]
        # forward_norm = lengthDifference / 0.55

        # # Assign motion axes (Inverted strafe to move TOWARD target)
        # forward = forward_norm
        # strafe = -strafe_norm  
        # turn = 0.0
        # print(f"Forward: {forward}, Strafe: {strafe}, Turn: {turn}")

        # Define maximum allowed RPM limit
        # MAX_RPM = 250.0
        forward = self.mapValue(lengthDifference, -4, 4, -250, 250)
        strafe = self.mapValue(xdifference, -125, 125, -250, 250)
        turn = 0

        # Mecanum inverse kinematics (Normalized mixing)
        fl = forward + strafe + turn
        fr = forward - strafe - turn
        bl = forward - strafe + turn
        br = forward + strafe - turn

        # 1. Cap relative outputs to [-1.0, 1.0] to preserve motion trajectory
        # max_power = max(abs(fl), abs(fr), abs(bl), abs(br))
        # if max_power > 1.0:
        #     fl /= max_power
        #     fr /= max_power
        #     bl /= max_power
        #     br /= max_power

        # # 2. Scale normalized values to the [-250, 250] RPM range
        # fl_rpm = fl * MAX_RPM
        # fr_rpm = fr * MAX_RPM
        # bl_rpm = bl * MAX_RPM
        # br_rpm = br * MAX_RPM

        # 3. Publish actual RPM commands
        power_msg = Float32MultiArray()
        power_msg.data = [float(fl), float(fr), float(bl), float(br)]
        # print(power_msg.data)
        print(f"xdiff{xdifference}, ydiff{lengthDifference}")

        self.error_pub.publish(power_msg)



def main(args=None):
    rclpy.init(args=args)
    node = AlignmentNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()