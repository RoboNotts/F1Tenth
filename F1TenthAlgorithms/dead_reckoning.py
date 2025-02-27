import numpy as np


def dead_reckoning(prev_location: np.ndarray, prev_velocity: np.ndarray, total_time: float) -> np.ndarray:
    """
       Calculates current position of the vehicle based on previous location, velocity, and total time
       using Euler's method with NumPy.

       Parameters:
           prev_location (np.ndarray): Previous (x, y) location of the vehicle as a state vector
           prev_velocity (np.ndarray): Previous (v_x, v_y) velocity of the vehicle as a state vector
           total_time (float): Total time spent moving

       Returns:
           np.ndarray: Estimated (x, y) position after total_time as a state vector
       """

    prev_location = np.array(prev_location)
    prev_velocity = np.array(prev_velocity)

    new_position = prev_location + total_time * prev_velocity

    return new_position
