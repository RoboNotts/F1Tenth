from numpy.typing import ArrayLike


def find_desired_acceleration(
    currentVelocity_fix_mps: ArrayLike[float],
    desiredVelocity_fix_mps: ArrayLike[float],
    tauTimeConstant_s: float = 1.0
) -> float:
    """
    Calculate the acceleration needed to reach the desired velocity.

    Parameters:
        currentVelocity_fix_mps:
            Current velocity of the car in its own frame of reference,
            measured in metres per second.

        desiredVelocity_fix_mps:
            Desired velocity of the car in its own frame of reference,
            measured in metres per second.

        tauTimeConstant_s:
            The amount of seconds to reach the desired velocity
            `desiredVelocity_fix_mps` (default 1.0).

    Returns:
        desired_acceleration:
            The acceleration in m/s^2 needed to reach velocity
            `desiredVelocity_fix_mps` in `tauTimeConstant_s` seconds.
    """

    # Find the desiered change in velocity in velocity
    velocityDiff_fix_mps = desiredVelocity_fix_mps - currentVelocity_fix_mps

    return velocityDiff_fix_mps / tauTimeConstant_s
