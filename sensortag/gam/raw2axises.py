import struct

GYRO_SCALE = 65536.0/500.0
ACC_SCALE = 32768.0/8.0 # ACC RANGE 8G

def raw2axises(data):
    gam = struct.unpack('<hhhhhhhhh', data)

    # GYRO
    gx = gam[0] / GYRO_SCALE
    gy = gam[1] / GYRO_SCALE
    gz = gam[2] / GYRO_SCALE

    # ACC
    ax = gam[3] / ACC_SCALE
    ay = gam[4] / ACC_SCALE
    az = gam[5] / ACC_SCALE

    # MAG
    mx = gam[6]
    my = gam[7]
    mz = gam[8]

    return [gx, gy, gz, ax, ay, az, mx, my, mz]
    