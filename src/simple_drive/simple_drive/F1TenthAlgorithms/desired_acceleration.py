def find_desired_acceleration(
    currentVelocity_Fix_mps: float,
    desiredVelocity_Fix_mps: float,
    tauTimeConstant_s: float = 1.0
) -> float:
    """
    Calculate the acceleration needed to reach the desired velocity.

    Parameters:
        currentVelocity_Fix_mps:
            Current velocity of the car in its own frame of reference,
            measured in metres per second.

        desiredVelocity_Fix_mps:
            Desired velocity of the car in its own frame of reference,
            measured in metres per second.

        tauTimeConstant_s:
            The amount of seconds to reach the desired velocity
            `desiredVelocity_Fix_mps` (default 1.0).

    Returns:
        desired_acceleration:
            The acceleration in m/s^2 needed to reach velocity
            `desiredVelocity_Fix_mps` in `tauTimeConstant_s` seconds.
    """

    # Find the desiered change in velocity in velocity
    velocityDiff_Fix_mps = desiredVelocity_Fix_mps - currentVelocity_Fix_mps

    return velocityDiff_Fix_mps / tauTimeConstant_s
