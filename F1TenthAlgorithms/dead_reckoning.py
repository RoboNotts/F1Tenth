import numpy as np


def dead_reckoning(prev_location: np.ndarray,
                   prev_velocity: np.ndarray,
                   heading: float, total_time: float,
                   body_frame: bool = False
                   ) -> np.ndarray:
    """
    Performs simple dead reckoning using Euler's method of integration.

    Parameters:
        prev_location (np.ndarray): Previous (x, y) location in global coordinates.
        prev_velocity (np.ndarray): Velocity vector (v_x, v_y) in the global frame,
                                    OR (v_forward, v_lateral) in the body frame.
        heading (float): Current heading angle (yaw) in radians.
        total_time (float): Total time interval for integration.
        body_frame (bool): If True, velocity is given in the body frame and must be rotated.

    Returns:
        np.ndarray: Estimated (x, y) position after total_time.
    """

    prev_location = np.array(prev_location)
    prev_velocity = np.array(prev_velocity)

    # Convert body-frame velocity to global frame if necessary
    if body_frame:
        rotation_matrix = np.array([
            [np.cos(heading), -np.sin(heading)],
            [np.sin(heading), np.cos(heading)]
        ])
        global_velocity = rotation_matrix @ prev_velocity  # Transform to global frame
    else:
        global_velocity = prev_velocity  # Already in global frame

    # Apply Euler's method: new_position = prev_position + velocity * time
    new_position = prev_location + total_time * global_velocity

    return new_position
