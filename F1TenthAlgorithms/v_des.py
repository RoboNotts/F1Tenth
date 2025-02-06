import math

def v_des(x_des,x,y_des,y,delta_tao,v_max):
    return max((math.sqrt(((x_des - x)**2 + y_des - y)**2))/delta_tao , v_max)
