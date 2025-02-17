from math import atan2

def find_desired_heading(current_x_position_m: float, current_y_position_m: float, desired_x_position_m: float, desired_y_position_m: float) -> float:
    """
    Calculate the desired heading angle based on the current and target positions.
    This function works within any frame, as long as inputs are all within the same frame.

    Parameters:
        current_x_position_m:
            Current x-coordinate,
            measured in metres.
        current_y_position_m:
            Current y-coordinate,
            measured in metres.
        desired_x_position_m:
            Desired x-coordinate,
            measured in metres.
        desired_y_position_m:
            Desired y-coordinate,
            measured in metres.

    Returns:
        desired_heading:
            The desired heading angle in radians, measured counterclockwise
            from the positive x-axis. This will be in the same frame as the inputs.
    """
    return atan2(desired_x_position_m - current_x_position_m, desired_y_position_m - current_y_position_m)