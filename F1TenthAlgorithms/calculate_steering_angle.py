import math

def find_desired_steering_angle_dynamic(
        desiredHeading_fix_rad: float,
        currentHeading_fix_rad: float,
        k: int = 1) -> float:
    """
    Calculates the desired steering angle dynamically by using either proportional or fixed steering
    based on the heading error.

    Parameters:
        k:
            The proportional gain (int). Default is 1.

        desiredHeading_fix_rad:
            Desired heading angle measured in radians.

        currentHeading_fix_rad:
            Current heading angle measured in radians.

    Returns:
        desired steering angle:
            the steering angle alpha measured in radians.
    """
    error = desiredHeading_fix_rad - currentHeading_fix_rad

    if abs(error) <= math.pi/4:
        return proportional_steering_angle(k, desiredHeading_fix_rad, currentHeading_fix_rad)
    else:
        return fixed_steering_angle(desiredHeading_fix_rad, currentHeading_fix_rad)