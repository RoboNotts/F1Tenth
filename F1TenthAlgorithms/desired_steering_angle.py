import math


def find_desired_steering_angle(
        desiredHeading_fix_rad: float,
        currentHeading_fix_rad: float) -> float:
    """
    Calculates the desired steering angle based on the heading error.

    Parameters:
        desiredHeading_fix_rad:
            Desired heading angle,
            measured in radians.
         currentHeading_fix_rad:
            Current heading angle,
            measured in radians.

    Returns:
        desired steering angle:
            the steering angle alpha measured in radians.
    """
    return (math.pi / 4) * math.copysign(1, desiredHeading_fix_rad - currentHeading_fix_rad)
