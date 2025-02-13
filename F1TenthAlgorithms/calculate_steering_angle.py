import math

def proportional_steering_angle(*, k: int = 1, heading_des: float, heading_curr: float) -> float: ...

def fixed_steering_angle(*, heading_des: float, heading_curr: float) -> float: ...

def dynamic_steering_angle(*, k: int = 1, heading_des: float, heading_curr: float) -> float:
    """
    Calculate the steering angle dynamically by using either proportional or fixed steering
    based on the heading error.

    :param k: The proportional gain (int). Default is 1.
    :param heading_des: The desired heading in radians (float).
    :param heading_curr: The current heading in radians (float).
    :return: The steering angle in radians (float).
    """
    error = heading_des - heading_curr

    if abs(error) <= math.pi/4:
        return proportional_steering_angle(k=k, heading_des=heading_des, heading_curr=heading_curr)
    else:
        return fixed_steering_angle(heading_des=heading_des, heading_curr=heading_curr)