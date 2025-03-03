#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from ackermann_msgs.msg import AckermannDriveStamped
from typing import Optional


class DriveNode(Node):
    def __init__(self):
        """
        A  Drive node that publishes Ackermann drive commands to the robot.
        """
        super().__init__('drive_node')

        # Create publisher for the /drive topic
        self.drivePublisher = self.create_publisher(
            AckermannDriveStamped,  # message type
            '/drive',  # topic name
            10  # queue size
        )

        self.get_logger().info('Drive Node has been initialized')

    def publishDriveCommand(
            self,
            desiredSteeringAngle_rad: float,
            desiredAcceleration_mps2: float,
            desiredSpeed_mps: float = 0.0,
            desiredSteeringAngleVelocity_radps: Optional[float] = None,
            desiredJerk_mps3: Optional[float] = None
    ) -> None:
        """
        Publish drive commands to the /drive topic.

        Parameters:
            desiredSteeringAngle_rad:
                Desired steering angle in radians. Positive values turn left,
                negative values turn right.
            desiredAcceleration_mps2:
                Desired acceleration in meters per second squared.
            desiredSpeed_mps:
                Desired speed in meters per second (default 0.0).
            desiredSteeringAngleVelocity_radps:
                Desired rate of change of the steering angle in radians
                per second (Optional. Default None, not used if None).
            desiredJerk_mps3:
                Desired rate of change of acceleration in meters per second cubed
                (Optional. Default None, not used if None).

        Returns:
            None
        """
        # Create AckermannDriveStamped message
        driveMsg = AckermannDriveStamped()

        # Set header with current time
        driveMsg.header.stamp = self.get_clock().now().to_msg()
        driveMsg.header.frame_id = ''

        # Set drive parameters
        driveMsg.drive.steering_angle = desiredSteeringAngle_rad
        driveMsg.drive.acceleration = desiredAcceleration_mps2
        driveMsg.drive.speed = desiredSpeed_mps

        # Set steering angle velocity if provided
        if desiredSteeringAngleVelocity_radps is not None:
            driveMsg.drive.steering_angle_velocity = desiredSteeringAngleVelocity_radps

        # Set Jerk if provided
        if desiredJerk_mps3 is not None:
            driveMsg.drive.jerk = desiredJerk_mps3

        # Publish the message
        self.drivePublisher.publish(driveMsg)
        # self.get_logger().info(
        #     f'Published: steering_angle={desiredSteeringAngle_rad:.2f} rad, '
        #     f'accel={desiredAcceleration_mps2:.2f} m/s^2, '
        #     f'speed={desiredSpeed_mps:.2f} m/s, '
        #     # f'steering_velocity={desiredSteeringAngleVelocity_radps:.2f} m/s, '
        #     # f'jerk={desiredJerk_mps3:.2f} m/s'
        # )