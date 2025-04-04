import math


def find_desired_velocity(
    desiredPositionX_Fix_m: float,
    currentPositionX_Fix_m: float,
    desiredPositionY_Fix_m: float,
    currentPositionY_Fix_m: float,
    timeToGo_s: float,
    maximumVelocity_Fix_mps: float,
    minimumVelocity_Fix_mps: float
) -> float:
    """
    Calculate the desired velocity to go from the currect position to the
    desired position.

    Parameters:
        desiredPositionX_Fix_m:
            The x coordinate of the desired point,
            measured in metres
        currentPositionX_Fix_m:
            The current x coordinate,
            measured in metres
        desiredPositionY_Fix_m:
            The y coordinate of the desired point,
            measured in metres
        currentPositionY_Fix_m:
            The current y coordinate,
            measured in metres
        timeToGo_s:
            The time to travel to te desired point
            measured in seconds
        maximumVelocity_Fix_mps:
            The vehicles maximum velocity
            measured in meters per second
        minimumVelocity_Fix_mps:
            The vehicles minimum velocity
            measured in meters per second


    Returns:
        desiredVelocity_mps:
            Velocity needed in meters per second
    """

    return max(minimumVelocity_Fix_mps,
               min(
                   (math.sqrt(
                       (desiredPositionX_Fix_m - currentPositionX_Fix_m)**2 +
                       (desiredPositionY_Fix_m - currentPositionY_Fix_m)**2)
                    ) / timeToGo_s,
                   maximumVelocity_Fix_mps)
               )
