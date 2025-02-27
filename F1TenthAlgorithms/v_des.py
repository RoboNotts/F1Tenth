import math

def find_desired_velocity(desired_x_position_m,current_x_position_m,desired_y_position_m,current_y_position_m,time_to_go_s,maximum_velocity_ms):

    """
    Calculate the desired velocity to go from the currect position to the desired position

    Parameters:
        desired_x_position_m:
            The x co ordinate of the desired point
        current_x_position_m:
            Current x coordinate
        desired_y_position_m:
            Y coodinate of the desired point
        current_y_position_m:
            Current y coordinate
        time_to_go_s:
            The time to do to travel to the deired point
        maximum_velocity_ms:
            The vehicles maximum speed
    
    Returns:
        desired_velocity_m/s:
            Velocity needed in m/s

    """

    return max((math.sqrt((desired_x_position_m - current_x_position_m)**2 + (desired_y_position_m - current_y_position_m)**2))/time_to_go_s , maximum_velocity_ms)
