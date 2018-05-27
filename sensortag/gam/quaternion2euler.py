import numpy as np


def quaternion2euler(q):
    qw = q[0]
    qx = q[1]
    qy = q[2]
    qz = q[3]

    qy2 = qy ** 2

    sinr = 2.0 * (qw * qx + qy * qz)
    cosr = 1.0 - 2.0 * (qx ** 2 + qy2)
    roll = np.arctan2(sinr, cosr)

    sinp = 2.0 * (qw * qy - qz * qx)
    sinp = 1.0 if sinp > 1.0 else sinp
    sinp = -1.0 if sinp < -1.0 else sinp
    pitch = np.arcsin(sinp)

    siny = 2.0 * (qw * qz + qx * qy)
    cosy = 1.0 - 2.0 * (qy2 + qz ** 2)
    yaw = np.arctan2(siny, cosy)

    return roll, pitch, yaw
