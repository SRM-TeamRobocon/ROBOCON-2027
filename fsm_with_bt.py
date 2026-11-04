import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray, Bool
import py_trees
import py_trees.console as console

# --- 1. Define Behavior Tree Leaf Nodes ---

class SendTargetAction(py_trees.behaviour.Behaviour):
    def __init__(self, ros_node):
        super().__init__("Send Target")
        self.ros_node = ros_node
        self.sent = False

    def initialise(self):
        self.sent = False

    def update(self):
        if not self.sent:
            odotargets = [1000.0, 2000.0, 90.0]
            self.ros_node.odo_pub.publish(Float32MultiArray(data=odotargets))
            self.ros_node.get_logger().info("Target published.")
            self.sent = True
        return py_trees.common.Status.SUCCESS

class CheckTargetReached(py_trees.behaviour.Behaviour):
    def __init__(self, ros_node):
        super().__init__("Check Target Reached")
        self.ros_node = ros_node

    def update(self):
        if self.ros_node.target_reached:
            self.ros_node.get_logger().info("Target reached condition met.")
            return py_trees.common.Status.SUCCESS
        return py_trees.common.Status.RUNNING

class StartAlignmentAction(py_trees.behaviour.Behaviour):
    def __init__(self, ros_node):
        super().__init__("Start Alignment")
        self.ros_node = ros_node
        self.sent = False

    def initialise(self):
        self.sent = False

    def update(self):
        if not self.sent:
            self.ros_node.alignment_start_pub.publish(Bool(data=True))
            self.ros_node.get_logger().info("Alignment command published.")
            self.sent = True
        return py_trees.common.Status.SUCCESS

class CheckAlignmentDone(py_trees.behaviour.Behaviour):
    def __init__(self, ros_node):
        super().__init__("Check Alignment Done")
        self.ros_node = ros_node

    def update(self):
        if self.ros_node.align_done:
            self.ros_node.get_logger().info("Alignment done condition met.")
            return py_trees.common.Status.SUCCESS
        return py_trees.common.Status.RUNNING

class StartClimbAction(py_trees.behaviour.Behaviour):
    def __init__(self, ros_node):
        super().__init__("Start Climb")
        self.ros_node = ros_node
        self.sent = False

    def initialise(self):
        self.sent = False

    def update(self):
        if not self.sent:
            self.ros_node.climb_state_pub.publish(Bool(data=True))
            self.ros_node.get_logger().info("Climb command published.")
            self.sent = True
        return py_trees.common.Status.SUCCESS

class CheckClimbDone(py_trees.behaviour.Behaviour):
    def __init__(self, ros_node):
        super().__init__("Check Climb Done")
        self.ros_node = ros_node

    def update(self):
        if self.ros_node.climb_done:
            self.ros_node.get_logger().info("Climb completed successfully.")
            return py_trees.common.Status.SUCCESS
        return py_trees.common.Status.RUNNING


# --- 2. Main ROS2 Node containing the Behavior Tree ---

class TreeStateMachineNode(Node):
    def __init__(self):
        super().__init__('tree_statemachine')

        # Subscription state flags updated via callbacks
        self.target_reached = False
        self.align_done = False
        self.climb_done = False

        # Publishers
        self.odo_pub = self.create_publisher(Float32MultiArray, 'target_pose', 10)
        self.alignment_start_pub = self.create_publisher(Bool, 'start_align', 10)
        self.climb_state_pub = self.create_publisher(Bool, 'climb_state', 10)

        # Subscriptions
        self.create_subscription(Bool, 'target_reached', self.odo_callback, 10)
        self.create_subscription(Bool, 'align_confirmed', self.align_callback, 10)
        self.create_subscription(Bool, 'climb_done', self.climb_callback, 10)

        # Build Behavior Tree
        self.tree = self.create_behavior_tree()

        # Timer to tick the behavior tree regularly (e.g., 10 Hz)
        self.timer = self.create_timer(0.1, self.tick_tree)

    def odo_callback(self, msg):
        self.target_reached = msg.data

    def align_callback(self, msg):
        self.align_done = msg.data

    def climb_callback(self, msg):
        self.climb_done = msg.data

    def create_behavior_tree(self):
        # Root Sequence: executes children strictly from left to right
        root = py_trees.composites.Sequence(name="Main Sequence", memory=True)

        # Build sub-steps
        root.add_children([
            SendTargetAction(self),
            CheckTargetReached(self),
            StartAlignmentAction(self),
            CheckAlignmentDone(self),
            StartClimbAction(self),
            CheckClimbDone(self)
        ])

        behavior_tree = py_trees.trees.BehaviourTree(root)
        behavior_tree.setup(timeout=15.0)
        return behavior_tree

    def tick_tree(self):
        self.tree.tick()
        # If the entire root sequence succeeds, shut down
        if self.tree.root.status == py_trees.common.Status.SUCCESS:
            self.get_logger().info("Behavior Tree finished successfully. Shutting down.")
            self.timer.cancel()
            rclpy.shutdown()

def main(args=None):
    rclpy.init(args=args)
    node = TreeStateMachineNode()
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