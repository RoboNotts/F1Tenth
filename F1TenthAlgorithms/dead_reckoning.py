import numpy as np
from numpy.typing import ArrayLike


def dead_reckoning(
        position_fix_m: ArrayLike[float],
        velocity_fix_mps: ArrayLike[float],
        timeInterval_s: ArrayLike[float],
) -> ArrayLike[float]:
    """
    Performs simple dead reckoning using Euler's method of integration.

    Parameters:
        position_fix_m: Previous (x, y) location in the fixed coordinate frame (meters).
        velocity_fix_mps: Velocity vector (v_x, v_y) in the fixed coordinate frame (meters per second).
        timeInterval_s: Total time interval for integration (seconds).

    Returns:
        updatedPosition_fix_m:
            Updated location in the fixed coordinate frame (meters).
    """

    # checking input position_fix_m (previous location)
    position_fix_m = np.array(position_fix_m)

    # checking velocity_fix_mps
    velocity_fix_mps = np.array(velocity_fix_mps)

    # checking timeInterval_s (change in time)
    timeInterval_s = np.array(timeInterval_s)

    # Apply Euler's method: new_position = prev_position + velocity * time
    updatedPosition_fix_m = position_fix_m + timeInterval_s * velocity_fix_mps

    return updatedPosition_fix_m
