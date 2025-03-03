import math

def find_desired_steering_angle_dynamic(
        desiredHeading_fix_rad: float,
        currentHeading_fix_rad: float,
        k: int = 1) -> float:
    """
    Calculates the desired steering angle dynamically by using either proportional or fixed steering
    based on the heading error.

    Both input headings must be in the same reference frame. The output steering angle will be in
    the body frame (bod).

    Parameters:
        k:
            The proportional gain (int). Default is 1.

        desiredHeading_fix_rad:
            Desired heading angle measured in radians.

        currentHeading_fix_rad:
            Current heading angle measured in radians.

    Returns:
        steeringAngle_bod_rad:
            the steering angle alpha measured in radians.
    """
    error = desiredHeading_fix_rad - currentHeading_fix_rad

    if abs(error) <= math.pi/4:
        steeringAngle_bod_rad = proportional_steering_angle(k, desiredHeading_fix_rad, currentHeading_fix_rad)
    else:
        steeringAngle_bod_rad = fixed_steering_angle(desiredHeading_fix_rad, currentHeading_fix_rad)

    return steeringAngle_bod_rad