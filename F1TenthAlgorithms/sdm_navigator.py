import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from tf_transformations import euler_from_quaternion
from ackermann_msgs.msg import AckermannDriveStamped

from F1TenthAlgorithms.desired_heading import *
from F1TenthAlgorithms.proportional_steering_angle import *
from F1TenthAlgorithms.desired_steering_angle import *
from F1TenthAlgorithms.desired_velocity import *


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

        self.targetPos_Fix_m = [5, 5]  # TODO

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

    def odom_callback(self, odomMessage: Odometry) -> None:
        """
        Callback function for odometry message callback.

        Calculates new driving instructions and publishes them to `/drive`.

        Parameters:
            odomMessage:
                The car's odometry message received by the subscription.
        """

        # extract pose position and orientation from odometry message
        posePos_Fix_m = odomMessage.pose.pose.position
        orientation_Fix_rad = odomMessage.pose.pose.orientation

        # get target position
        targetPos_Fix_m = self.targetPos_Fix_m

        # convert pose position into a list for easier readability when used
        # alongside `targetPos_Fix_m`.
        currentPos_Fix_m = [posePos_Fix_m.x, posePos_Fix_m.y]

        # get heading (yaw) from orientation quaternion
        (_, currentHeading_Fix_rad, _) = euler_from_quaternion(
            orientation_Fix_rad
        )

        # calculate desired velocity
        velocityDesired_Fix_mps = find_desired_velocity(
            currentPos_Fix_m[0], currentPos_Fix_m[1],
            targetPos_Fix_m[0], targetPos_Fix_m[1],
            10,
            100
        )

        # calculate desired heading angle
        headingDesired_Fix_rad = find_desired_heading(
            currentPos_Fix_m[0], currentPos_Fix_m[1],
            targetPos_Fix_m[0], targetPos_Fix_m[1],
        )

        # calculate desired steering angle using desired and current headings
        angleDesired_Fix_rad = find_desired_steering_angle(
            headingDesired_Fix_rad,
            currentHeading_Fix_rad
        )

        # create a drive message using the desired values calculated
        driveMsg = AckermannDriveStamped()
        driveMsg.drive.speed = velocityDesired_Fix_mps
        driveMsg.drive.steering_angle = angleDesired_Fix_rad

        # publish the new drive message
        self.drivePublisher.publish(driveMsg)


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
