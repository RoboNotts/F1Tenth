def proportional_steering_angle(*, k:int=1, heading_des:float, heading_curr:float) -> float:
    """
    Calculate the steering angle based on the difference between the desired heading and the current heading.

    Parameters:
    k (float): The proportional gain.
    heading_des (float): The desired heading in radians.
    heading_curr (float): The current heading in radians.

    Returns:
    float: The steering angle in radians.
    """
    return k*(heading_des - heading_curr)