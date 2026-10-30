import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray, Bool

class statemachine(Node):
    def __init__(self):
        super().__init__('statemachine')

        #publishers

        self.odo_pub = self.create_publisher(Float32MultiArray, 'target_pose', 10)
        self.climb_state_pub = self.create_publisher(Bool, 'climb_state', 10)
        self.allignment_start_pub = self.create_publisher(Bool, 'start_align', 10)

        #subscriptions
        self.odo_reached_sub = self.create_subscription(Bool, 'target_reached', self.odo_callback, 10)
        self.climb_done_sub = self.create_subscription(Bool, 'climb_done', self.climb_callback, 10)
        self.align_done_sub = self.create_subscription(Bool, 'align_confirmed', self.align_callback, 10)

        self.timer = self.create_timer(0.1, self.fsm)
    def odo_callback(self, msg):
        self.target_reached = msg.data
    def climb_callback(self, msg):
        self.climb_done = msg.data
    def align_callback(self, msg):
        self.align_done = msg.data


    def fsm(self):
            # State machine logic running periodically via timer
            if self.state == 'SEND_TARGET':
                odotargets = [1000.0, 2000.0, 90.0]
                self.odo_pub.publish(Float32MultiArray(data=odotargets))
                self.get_logger().info("Target published. Waiting for target to be reached...")
                self.state = 'WAIT_FOR_TARGET'

            elif self.state == 'WAIT_FOR_TARGET':
                if self.target_reached:
                    self.get_logger().info("Target reached, starting alignment.")
                    self.alignment_start_pub.publish(Bool(data=True))
                    self.state = 'WAIT_FOR_ALIGNMENT'

            elif self.state == 'WAIT_FOR_ALIGNMENT':
                if self.align_done:
                    self.get_logger().info("Alignment completed, starting climbing.")
                    self.climb_state_pub.publish(Bool(data=True))
                    self.state = 'WAIT_FOR_CLIMB'

            elif self.state == 'WAIT_FOR_CLIMB':
                if self.climb_done:
                    self.get_logger().info("Climbing completed. Shutting down state machine.")
                    self.timer.cancel()
                    rclpy.shutdown()

def main(args=None):
    rclpy.init(args=args)
    node = statemachine()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()