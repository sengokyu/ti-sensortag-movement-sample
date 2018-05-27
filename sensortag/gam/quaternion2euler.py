import math

def quaternion2euler(q):
    q_0 = q[0]
    q_1 = q[1]
    q_2 = q[2]
    q_3 = q[3]

    x = math.atan2(2*(q_0*q_1+q_2*q_3), (1-2*(q_1*q_1+q_2*q_2)))
    y = math.asin(2*(q_0*q_2-q_3*q_1))
    z = math.atan2(2*(q_0*q_3+q_1*q_2), (1-2*(q_2*q_2+q_3*q_3)))

    return [x, y, z]
    