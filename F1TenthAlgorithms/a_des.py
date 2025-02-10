from numpy.typing import ArrayLike

# desired acceleration
def a_des(v_des: ArrayLike[float], v: ArrayLike[float], tau: float = 1) -> float:
    '''
    Calculate the acceleration needed to reach the desired velocity.

    Parameters:
        v_des : Desired velocity.
        v : Current velocity.
        tau : Time constant, i.e. the amount of time to reach velocity `v_des`.

    Returns:
        a_des : Desired acceleration.
    '''
    return (v_des - v) / tau
