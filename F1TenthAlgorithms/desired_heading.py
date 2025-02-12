from math import atan2

def find_desired_heading(x: float, y:float, x_des: float, y_des: float) -> float:
    """
    Calculate the desired heading angle based on the current and target positions.

    Parameters:
        x:
            Current x-coordinate.
        y:
            Current y-coordinate.
        x_des:
            Target x-coordinate.
        y_des:
            Target y-coordinate.

    Returns:
        desired_heading:
            The desired heading angle in radians, measured counterclockwise
            from the positive x-axis.
    """
    return atan2(x_des - x, y_des - y)