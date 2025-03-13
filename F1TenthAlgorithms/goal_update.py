# Built-in Imports
# None

# Custom Library Imports
# None

# ROS Imports
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, Point

# Global Variables
# None

class GoalUpdater(Node):
    """
    A node that updates a list of goal positions.

    Topics subscribed to:
        /goal_pose:
            geometry_msgs/msg/PoseStamped
                Published by the F1Tenth RViz simulator.
                Contains the automatically generated goal positions.

        /target_pos:
            geometry_msgs/msg/Point
                A custom topic allowing manual addition of goal positions via the console.
    
    This node appends received goal positions to a list (`targetPos_Fix_m`), which can later be used 
    for navigation or path planning.
    """
    def __init__(self):
        """
        Initialise the GoalUpdater node and create subscriptions to the 'goal_pose' and 'target_pos' topics.
        """
        super().__init__('goal_position_updater')

        # List to store target positions (goal coordinates)
        self.targetPos_Fix_m = []

        # Subscribe to 'goal_pose' (automatic goal input)
        self.goal_pose_subscription = self.create_subscription(PoseStamped,
                                                    'goal_pose', 
                                                    self.goal_update_callback,
                                                    10)
        
        # Subscribe to 'target_pos' (manual goal input)
        self.target_pos_subscription = self.create_subscription(Point,
                                                    'target_pos',
                                                    self.target_update_callback,
                                                    10)

    def goal_update_callback(self, msg:PoseStamped):
        """
        Callback function for the 'goal_pose' topic.

        Extracts the goal position from the PoseStamped message and adds it to the target list.

        Parameters:
            msg (PoseStamped):
                Message containing the position of the goal.
        """
        self.target_update_callback(msg.pose.position)

        
    def target_update_callback(self, msg:Point):
        """
        Callback function for the 'target_pos' topic.

        Receives a manually specified goal position and appends it to the target list.

        Parameters:
            msg (Point):
                The received manual goal position.
                Contains x and y coordinates.
        """
        targetPos_Fix_m = self.targetPos_Fix_m
        targetPos_Fix_m.append((msg.x, msg.y))

def main(args=None):
    """
    Entry point for the GoalUpdater node.

    Initialises the ROS2 node, starts listening for goal updates, and keeps it running.
    """
    rclpy.init(args=args)

    goal_updater = GoalUpdater()

    rclpy.spin(goal_updater)

    # Destroy the node explicitly
    goal_updater.destroy_node()

    rclpy.shutdown()

if __name__ == '__main__':
    main()