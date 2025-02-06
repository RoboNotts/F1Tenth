from math import tan

def specified_heading(x_des, x, y_des, y):
    return -(tan**-1) * ((x_des - x)/(y_des - y))