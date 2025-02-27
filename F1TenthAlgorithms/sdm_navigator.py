import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from tf_transformations import euler_from_quaternion
from ackermann_msgs.msg import AckermannDriveStamped

from F1TenthAlgorithms.desired_heading import *
from F1TenthAlgorithms.proportional_steering_angle import *
from F1TenthAlgorithms.desired_steering_angle import *


def desired_velocity(desired_x_position_m,
                     current_x_position_m,
                     desired_y_position_m,
                     current_y_position_m,
                     time_to_go_s,
                     maximum_velocity_ms):
    return  # temporary function while waiting for it to be implemented


class SDM_Navigator(Node):
    """
    Simple drive mode navigator.

    Topics subscribed to:
        /ego_racecar/odom

    Topics published to:
        /drive
    """

    def __init__(self):
        super().__init__('sdm_navigator')

        self.target_pos_fix = [5, 5]  # TODO

        self.odom_subscription = self.create_subscription(
            Odometry,
            '/ego_racecar/odom',
            self.odom_callback,
            10
        )

        self.drive_publisher = self.create_publisher(
            AckermannDriveStamped,
            '/drive',
            10
        )

    def odom_callback(self, msg: Odometry):
        pose_pos = msg.pose.pose.position
        current_pos = [pose_pos.x, pose_pos.y]
        target_pos = self.target_pos_fix

        orientation = msg.pose.pose.orientation
        (_, current_heading, _) = euler_from_quaternion(orientation)

        v_desired = desired_velocity(
            current_pos[0], current_pos[1],
            target_pos[0], target_pos[1],
            10, 100
        )

        heading_desired = find_desired_heading(
            current_pos[0], current_pos[1],
            target_pos[0], target_pos[1],
        )

        angle_desired = find_desired_steering_angle(
            heading_desired,
            current_heading
        )

        drive_msg_output = AckermannDriveStamped()
        drive_msg_output.drive.speed = v_desired
        drive_msg_output.drive.steering_angle = angle_desired

        self.drive_publisher.publish(drive_msg_output)


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
