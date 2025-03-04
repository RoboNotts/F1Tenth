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
        /ego_racecar/odom:
            nav_msgs/msg/Odometry

    Topics published to:
        /drive:
            ackermann_msgs/msg/AckermannDriveStamped
    """

    def __init__(self):
        super().__init__('sdm_navigator')

        self.targetPos_fix_m = [5, 5]  # TODO

        # subscription to receive the car's odometry data from the
        # `/ego_racecar/odom` topic
        # calls `odom_callback` function when a value is published
        self.odom_subscription = self.create_subscription(
            Odometry,
            '/ego_racecar/odom',
            self.odom_callback,
            10
        )

        # create publisher to send driving instructions to the car
        # via the `/drive` topic
        self.drive_publisher = self.create_publisher(
            AckermannDriveStamped,
            '/drive',
            10
        )

    def odom_callback(self, msg: Odometry):
        posePos_fix_m = msg.pose.pose.position
        currentPos_fix_m = [posePos_fix_m.x, posePos_fix_m.y]
        targetPos_fix_m = self.target_pos_fix

        orientation_fix_rad = msg.pose.pose.orientation
        (_, currentHeading_fix_rad, _) = euler_from_quaternion(
            orientation_fix_rad
        )

        vDesired_fix_mps = desired_velocity(
            currentPos_fix_m[0], currentPos_fix_m[1],
            targetPos_fix_m[0], targetPos_fix_m[1],
            10,
            100
        )

        headingDesired_fix_rad = find_desired_heading(
            currentPos_fix_m[0], currentPos_fix_m[1],
            targetPos_fix_m[0], targetPos_fix_m[1],
        )

        angleDesired_fix_rad = find_desired_steering_angle(
            headingDesired_fix_rad,
            currentHeading_fix_rad
        )

        drive_msg_output = AckermannDriveStamped()
        drive_msg_output.drive.speed = vDesired_fix_mps
        drive_msg_output.drive.steering_angle = angleDesired_fix_rad

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
