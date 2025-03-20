import math

from F1TenthAlgorithms.desired_steering_angle import find_desired_steering_angle
from F1TenthAlgorithms.proportional_steering_angle import find_proportional_control_steering_angle


def find_desired_steering_angle_dynamic(
        desiredHeading_Fix_rad: float,
        currentHeading_Fix_rad: float,
        k: int = 1) -> float:
    """
    Calculates the desired steering angle dynamically by using either proportional or fixed steering
    based on the heading error.

    Both input headings must be in the same reference frame. The output steering angle will be in
    the body frame (Bod).

    Parameters:
        k:
            The proportional gain (int). Default is 1.

        desiredHeading_Fix_rad:
            Desired heading angle measured in radians.

        currentHeading_Fix_rad:
            Current heading angle measured in radians.

    Returns:
        steeringAngle_Bod_rad:
            the steering angle alpha measured in radians.
    """
    error = desiredHeading_Fix_rad - currentHeading_Fix_rad

    if abs(error) <= math.pi/4:
        steeringAngle_Bod_rad = find_proportional_control_steering_angle(
            desiredHeading_Fix_rad, currentHeading_Fix_rad, k)
    else:
        steeringAngle_Bod_rad = find_desired_steering_angle(
            desiredHeading_Fix_rad, currentHeading_Fix_rad)

    return steeringAngle_Bod_rad
