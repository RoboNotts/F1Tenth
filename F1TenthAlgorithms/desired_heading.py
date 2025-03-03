from math import atan2


def find_desired_heading(
        currentPositionX_m: float,
        currentPositionY_m: float,
        desiredPositionX_m: float,
        desiredPositionY_m: float) -> float:
    """
    Calculate the desired heading angle based on the current and target 
    positions. This function works within any frame, as long as inputs are all 
    within the same frame.

    Parameters:
        currentPositionX_m:
            Current x-coordinate, measured in metres.

        currentPositionY_m:
            Current y-coordinate, measured in metres.

        desiredPositionX_m:
            Desired x-coordinate, measured in metres.

        desiredPositionY_m:
            Desired y-coordinate, measured in metres.

    Returns:
        desired_heading:
            The desired heading angle in radians, measured counterclockwise
            from the positive x-axis. This will be in the same frame as the
            inputs.
    """
    return atan2(desiredPositionX_m - currentPositionX_m,
                 desiredPositionY_m - currentPositionY_m)
