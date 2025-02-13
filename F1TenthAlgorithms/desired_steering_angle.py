import math

def desired_steering_angle(psi_des, psi):
    """
    Calculates the desired steering angle based on the given equation.

    Parameters:
    psi_des (float): Desired heading angle
    psi (float): Current heading angle

    Returns:
    float: Steering angle alpha
    """

    return (math.pi / 4) * math.copysign(1, psi_des - psi)