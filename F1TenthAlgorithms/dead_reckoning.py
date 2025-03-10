import numpy as np
from numpy.typing import ArrayLike


def dead_reckoning(
        position_Fix_m: ArrayLike[float],
        velocity_Fix_mps: ArrayLike[float],
        changeInTime_s: ArrayLike[float],
) -> ArrayLike[float]:
    """
    Performs simple dead reckoning using Euler's method of integration.

    Parameters:
        position_Fix_m:
            Previous (x, y) location in the fixed coordinate frame (meters).

        velocity_Fix_mps:
            Velocity vector (v_x, v_y) in the fixed coordinate frame (meters per second).

        changeInTime_s:
            Total time interval for integration (seconds).

    Returns:
        updatedPosition_Fix_m:
            Updated location in the fixed coordinate frame (meters).
    """

    # checking input position_fix_m (previous location)
    position_Fix_m = np.array(position_Fix_m)

    # checking velocity_fix_mps
    velocity_Fix_mps = np.array(velocity_Fix_mps)

    # checking changeInTime_s (change in time)
    changeInTime_s = np.array(changeInTime_s)

    # Apply Euler's method: new_position = prev_position + velocity * time
    updatedPosition_Fix_m = position_Fix_m + changeInTime_s * velocity_Fix_mps

    return updatedPosition_Fix_m
