import math

from F1TenthAlgorithms import *

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from tf_transformations import euler_from_quaternion
from ackermann_msgs.msg import AckermannDriveStamped
from geometry_msgs.msg import PoseStamped, Point


class SDM_Navigator(Node):
    """
    Simple drive mode navigator.

    Topics subscribed to:
        /ego_racecar/odom:
            nav_msgs/msg/Odometry

    Topics published to:
        /drive:
            ackermann_msgs/msg/AckermannDriveStamped
    """

    def __init__(self):
        super().__init__('sdm_navigator')

        self.targetPos_Fix_m = [-10.0, 0.0]  # TODO

        # subscription to receive the car's odometry data from the
        # `/ego_racecar/odom` topic
        # calls `odom_callback` function when a value is published
        self.odomSubscription = self.create_subscription(
            Odometry,
            '/ego_racecar/odom',
            self.odom_callback,
            10
        )

        # create publisher to send driving instructions to the car
        # via the `/drive` topic
        self.drivePublisher = self.create_publisher(
            AckermannDriveStamped,
            '/drive',
            10
        )

        # GOAL UPDATER
        # Subscribe to 'goal_pose' (automatic goal input)
        self.goal_pose_subscription = self.create_subscription(
            PoseStamped,
            '/goal_pose',
            self.goal_update_callback,
            10)

        # Subscribe to 'target_pos' (manual goal input)
        self.target_pos_subscription = self.create_subscription(
            Point,
            '/target_pos',
            self.target_update_callback,
            10)

    def odom_callback(self, msg: Odometry) -> None:
        """
        Callback function for odometry message callback.

        Calculates new driving instructions and publishes them to `/drive`.

        Parameters:
            odomMessage:
                The car's odometry message received by the subscription.
        """

        # extract pose position and orientation from odometry message
        posePos_Fix_m = msg.pose.pose.position
        orientation_Fix_rad = (
            msg.pose.pose.orientation.x,
            msg.pose.pose.orientation.y,
            msg.pose.pose.orientation.z,
            msg.pose.pose.orientation.w
        )  # using tuple since euler_from_quaternion requires it to be iterable

        # get target position
        targetPos_Fix_m = self.targetPos_Fix_m

        # convert pose position into a list for easier readability when used
        # alongside `targetPos_Fix_m`.
        currentPos_Fix_m = [posePos_Fix_m.x, posePos_Fix_m.y]

        # get heading (yaw) from orientation quaternion
        (_, _, currentHeading_Fix_rad) = euler_from_quaternion(
            orientation_Fix_rad
        )

        # calculate desired velocity
        velocityDesired_Fix_mps = find_desired_velocity(
            targetPos_Fix_m[0], currentPos_Fix_m[0],
            targetPos_Fix_m[1], currentPos_Fix_m[1],
            10,
            10.0
        )

        # calculate desired heading angle
        headingDesired_Fix_rad = find_desired_heading(
            currentPos_Fix_m[0], currentPos_Fix_m[1],
            targetPos_Fix_m[0], targetPos_Fix_m[1],
        )
        if headingDesired_Fix_rad - currentHeading_Fix_rad > math.pi:
            headingDesired_Fix_rad -= 2 * math.pi
        elif headingDesired_Fix_rad - currentHeading_Fix_rad < -math.pi:
            headingDesired_Fix_rad += 2 * math.pi

        # calculate desired steering angle using desired and current headings
        angleDesired_Fix_rad = find_desired_steering_angle_dynamic(
            headingDesired_Fix_rad,
            currentHeading_Fix_rad
        )

        # create a drive message using the desired values calculated
        driveMsg = AckermannDriveStamped()
        driveMsg.drive.speed = velocityDesired_Fix_mps
        driveMsg.drive.steering_angle = angleDesired_Fix_rad
        self.get_logger().info(f'{currentPos_Fix_m} -> {targetPos_Fix_m} : '
                               f'{velocityDesired_Fix_mps}m/s')
        self.get_logger().info(f'{currentHeading_Fix_rad} -> {headingDesired_Fix_rad} : '
                               f'{angleDesired_Fix_rad}rad')

        # publish the new drive message
        self.drivePublisher.publish(driveMsg)

    def goal_update_callback(self, msg: PoseStamped):
        """
        Callback function for the 'goal_pose' topic.

        Extracts the goal position from the PoseStamped message and adds it to the target list.

        Parameters:
            msg (PoseStamped):
                Message containing the position of the goal.
        """
        self.target_update_callback(msg.pose.position)

    def target_update_callback(self, msg: Point):
        """
        Callback function for the 'target_pos' topic.

        Receives a manually specified goal position and appends it to the target list.

        Parameters:
            msg (Point):
                The received manual goal position.
                Contains x and y coordinates.
        """
        self.targetPos_Fix_m.append((msg.x, msg.y))


def main(args=None):
    rclpy.init(args=args)

    navigator = SDM_Navigator()

    rclpy.spin(navigator)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    navigator.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
