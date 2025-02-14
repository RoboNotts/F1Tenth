def proportional_steering_angle_control(
        *,
        k:int=1,
        desired_heading_rad:float,
        current_heading_rad:float
    ) -> float:
    """
    Calculate the steering angle based on the difference between the desired heading and the current heading, scaled by a proportional gain.

    Parameters:
        k:
            The proportional gain used to scale the heading error,
            this is a constant.
        desired_heading_rad:
            The desired heading the car should be facing,
            measured in radians.
        current_heading_rad:
            The current heading the car is facing,
            measured in radians.

    Returns:
        steering_angle_rad:
            The steering angle the car should use to correct its heading,
            measured in radians.
    """
    return k*(desired_heading_rad - current_heading_rad)