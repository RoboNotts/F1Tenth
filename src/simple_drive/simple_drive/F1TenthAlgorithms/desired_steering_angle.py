import math


def find_desired_steering_angle(
        desiredHeading_Fix_rad: float,
        currentHeading_Fix_rad: float) -> float:
    """
    Calculates the desired steering angle based on the heading error.

    Parameters:
        desiredHeading_Fix_rad:
            Desired heading angle measured in radians.

         currentHeading_Fix_rad:
            Current heading angle measured in radians.

    Returns:
        desired steering angle:
            the steering angle alpha measured in radians.
    """
    return (math.pi / 4) * math.copysign(1, desiredHeading_Fix_rad -
                                         currentHeading_Fix_rad)
