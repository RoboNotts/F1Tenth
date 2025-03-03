import numpy as np
from F1TenthAlgorithms.desired_acceleration import find_desired_acceleration
from F1TenthAlgorithms.desired_heading import find_desired_heading
from F1TenthAlgorithms.proportional_steering_angle import find_proportional_control_steering_angle
from F1TenthAlgorithms.desired_velocity import find_desired_velocity
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from tf_transformations import euler_from_quaternion


class Navigation(Node):
    """
    Navigation node for Simple Drive Mode
    #TODO: May instead have all of SDM as one node

    Subscribes to odometry,
    Calculates desired heading, steering angle, and velocity, given the target
    """

    def __init__(self):
        super().__init__('navigation')

        # Create subscriber
        self.odometrySubscriber = self.create_subscription(
            Odometry, "/egoracecar/odom",
            self.ego_odometry_callback, 10
        )
        # TODO: Integrate with other tickets (Take in target position) [F1T-16]
        self.targetPosition_fix_m = [10, 10]  # Placeholder
        self.currentPosition_fix_m = None

    def ego_odometry_callback(self, msg: Odometry):
        """
        Processes Odometry data:
            Calculates the desired heading, steering angle, and velocity based\
            on the race car's current position to reach the target position.

        Parameters:
            msg:
                Odometry message coming from /egoracecar/odom topic

        Returns:
            None:
        """
        # Update current position using the values in msg.
        self.currentPosition_fix_m = np.array([
            msg.pose.pose.position.x, msg.pose.pose.position.y
        ])

        # Update current velocities using the values in msg.
        self.currentLinearVelocity_fix_mps = msg.twist.twist.linear.x
        self.currentAngularVelocity_fix_radps = msg.twist.twist.angular.z

        # Update current orientation using the values in msg.
        # Convert from quaternion to euler angles [roll, pitch, yaw]
        self.currentOrientation_fix_rad = np.array(
            euler_from_quaternion(msg.pose.pose.orientation)
        )

        # Calculate the desired heading
        self.desiredHeading_fix = find_desired_heading(
            self.currentPosition_fix_m[0],
            self.currentPosition_fix_m[1],
            self.targetPosition_fix_m[0],
            self.targetPosition_fix_m[1]
        )

        # Calculate the desired steering angle
        self.desiredSteeringAngle_ego_rad = find_proportional_control_steering_angle(
            0.5,
            self.desiredHeading_fix,
            self.currentOrientation_fix_rad[2]  # [2] refers to yaw
        )

        # Calculate the desired velocity
        self.desiredVelocity_ego_mps = find_desired_velocity(
            self.targetPosition_fix_m[0],
            self.currentPosition_fix_m[0],
            self.targetPosition_fix_m[1],
            self.currentPosition_fix_m[1],
            1,
            2
        )

        # Calculate the desired acceleration
        self.desiredAcceleration = find_desired_acceleration(
            self.currentLinearVelocity_fix_mps,
            self.desiredVelocity_ego_mps
        )

        # TODO: Integrate with other tickets (publish to /AckermannDriveStamped) [F1T-16]


def main(args=None):
    pass


if __name__ == '__main__':
    main()
