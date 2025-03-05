import rclpy
from rclpy.node import Node

from geometry_msgs.msg import PoseStamped

class GoalUpdateListener(Node):
    """
    A subscriber node that listens to the 'goal_pose' topic and logs the goal position.
    
    This node listens for PoseStamped messages, published by the F1Tenth RViz simulator.
    When a message is received, it extracts and logs the position from the pose.
    """
    def __init__(self):
        """
        Initialise the GoalUpdateListener node and create a subscription to the 'goal_pose' topic.
        """
        super().__init__('goal_update_listener')
        self.subscription = self.create_subscription(PoseStamped,
                                                    'goal_pose', 
                                                    self.listener_callback,
                                                    10)

    def listener_callback(self, msg:PoseStamped):
        """
        Callback function that processes incoming PoseStamped messages.
        
        Parameters:
            msg (PoseStamped):
                The message containing the goal pose.
        """
        self.get_logger().info(f'goal pose position: {msg.pose.position}')


def main(args=None):
    """
    Entry point for the GoalUpdateListener node.
    """
    rclpy.init(args=args)

    goal_update_listener = GoalUpdateListener()

    rclpy.spin(goal_update_listener)

    goal_update_listener.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()