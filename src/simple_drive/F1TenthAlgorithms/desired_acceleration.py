def find_desired_acceleration(
    currentSpeed_Fix_mps: float,
    desiredSpeed_Fix_mps: float,
    tauTimeConstant_s: float = 1.0
) -> float:
    """
    Calculate the acceleration needed to reach the desired speed.

    Parameters:
        currentSpeed_Fix_mps:
            Current speed of the car in its own frame of reference,
            measured in metres per second.

        desiredSpeed_Fix_mps:
            Desired speed of the car in its own frame of reference,
            measured in metres per second.

        tauTimeConstant_s:
            The amount of seconds to reach the desired speed
            `desiredSpeed_Fix_mps` (default 1.0).

    Returns:
        desired_acceleration:
            The acceleration in m/s^2 needed to reach speed
            `desiredSpeed_Fix_mps` in `tauTimeConstant_s` seconds.
    """

    # Find the desired change in speed
    speedDiff_Fix_mps = desiredSpeed_Fix_mps - currentSpeed_Fix_mps

    return speedDiff_Fix_mps / tauTimeConstant_s
