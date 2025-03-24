def find_proportional_control_steering_angle(
    desiredHeading_Fix_rad: float,
        currentHeading_Fix_rad: float,
        k: int = 1) -> float:
    """
    Calculate the steering angle based on the difference between the desired
    heading and the current heading, scaled by a proportional gain.

    Parameters:
        desiredHeading_Fix_rad:
            The desired heading the car should be facing,
            measured in radians.

        currentHeading_Fix_rad:
            The current heading the car is facing,
            measured in radians.

        k = 1:
            The proportional gain used to scale the heading error,
            this is a constant. If no value is passed then it is set to 1.

    Returns:
        steeringAngle_rad:
            The steering angle the car should use to correct its heading,
            measured in radians.
    """
    return k * (desiredHeading_Fix_rad - currentHeading_Fix_rad)
