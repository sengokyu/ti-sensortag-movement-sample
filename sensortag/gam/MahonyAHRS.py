# coding:utf-8
# derived from http://x-io.co.uk/open-source-imu-and-ahrs-algorithms/
import math


class MahonyAHRS(object):
    # Gets or sets the sample period.
    samplePeriod = 0

    # Gets or sets the algorithm proportional gain.
    kp = 0.0

    # Gets or sets the algorithm integral gain.
    ki = 0.0

    # Gets or sets the Quaternion output.
    quaternion = []

    # Gets or sets the integral error.
    __eInt = 0

    """
    Initializes a new instance of the <see cref="MadgwickAHRS"/> class.
    
    Sample period.
    Algorithm proportional gain.
    Algorithm integral gain.
    """

    def __init__(self, samplePeriod, kp=1.0, ki=0.0):
        self.samplePeriod = samplePeriod
        self.kp = kp
        self.ki = ki
        self.quaternion = [1.0, 0.0, 0.0, 0.0]
        self.__eInt = [0.0, 0.0, 0.0]

    """
    Algorithm AHRS update method. Requires only gyroscope and accelerometer data.

    gx: Gyroscope x axis measurement in radians/s.
    gy: Gyroscope y axis measurement in radians/s.
    gz: Gyroscope z axis measurement in radians/s.
    ax: Accelerometer x axis measurement in any calibrated units.
    ay: Accelerometer y axis measurement in any calibrated units.
    az: Accelerometer z axis measurement in any calibrated units.
    Magnetometer x axis measurement in any calibrated units.
    Magnetometer y axis measurement in any calibrated units.
    Magnetometer z axis measurement in any calibrated units.
    
    Optimised for minimal arithmetic.
    """

    def update(self, gx, gy, gz, ax, ay, az, mx, my, mz):
        q1 = self.quaternion[0]
        q2 = self.quaternion[1]
        q3 = self.quaternion[2]
        q4 = self.quaternion[3]

        q1q1 = q1 * q1
        q1q2 = q1 * q2
        q1q3 = q1 * q3
        q1q4 = q1 * q4
        q2q2 = q2 * q2
        q2q3 = q2 * q3
        q2q4 = q2 * q4
        q3q3 = q3 * q3
        q3q4 = q3 * q4
        q4q4 = q4 * q4

        # Normalise accelerometer measurement
        norm = math.sqrt(ax * ax + ay * ay + az * az)

        if (norm == 0):
            return

        norm = 1 / norm  # use reciprocal for division
        ax *= norm
        ay *= norm
        az *= norm

        # Normalise magnetometer measurement
        norm = math.sqrt(mx * mx + my * my + mz * mz)
        if norm == 0:
            return

        norm = 1 / norm  # use reciprocal for division
        mx *= norm
        my *= norm
        mz *= norm

        # Reference direction of Earth's magnetic field
        hx = 2.0 * mx * (0.5 - q3q3 - q4q4) + 2.0 * my * \
            (q2q3 - q1q4) + 2.0 * mz * (q2q4 + q1q3)
        hy = 2.0 * mx * (q2q3 + q1q4) + 2.0 * my * \
            (0.5 - q2q2 - q4q4) + 2.0 * mz * (q3q4 - q1q2)
        bx = math.sqrt((hx * hx) + (hy * hy))
        bz = 2.0 * mx * (q2q4 - q1q3) + 2.0 * my * \
            (q3q4 + q1q2) + 2.0 * mz * (0.5 - q2q2 - q3q3)

        # Estimated direction of gravity and magnetic field
        vx = 2.0 * (q2q4 - q1q3)
        vy = 2.0 * (q1q2 + q3q4)
        vz = q1q1 - q2q2 - q3q3 + q4q4
        wx = 2.0 * bx * (0.5 - q3q3 - q4q4) + 2.0 * bz * (q2q4 - q1q3)
        wy = 2.0 * bx * (q2q3 - q1q4) + 2.0 * bz * (q1q2 + q3q4)
        wz = 2.0 * bx * (q1q3 + q2q4) + 2.0 * bz * (0.5 - q2q2 - q3q3)

        # Error is cross product between estimated direction and measured direction of gravity
        ex = (ay * vz - az * vy) + (my * wz - mz * wy)
        ey = (az * vx - ax * vz) + (mz * wx - mx * wz)
        ez = (ax * vy - ay * vx) + (mx * wy - my * wx)
        if (self.ki > 0):
            self.__eInt[0] += ex  # accumulate integral error
            self.__eInt[1] += ey
            self.__eInt[2] += ez
        else:
            self.__eInt[0] = 0.0  # prevent integral wind up
            self.__eInt[1] = 0.0
            self.__eInt[2] = 0.0

        # Apply feedback terms
        gx = gx + self.kp * ex + self.ki * self.__eInt[0]
        gy = gy + self.kp * ey + self.ki * self.__eInt[1]
        gz = gz + self.kp * ez + self.ki * self.__eInt[2]

        # Integrate rate of change of quaternion
        pa = q2
        pb = q3
        pc = q4
        q1 = q1 + (-q2 * gx - q3 * gy - q4 * gz) * (0.5 * self.samplePeriod)
        q2 = pa + (q1 * gx + pb * gz - pc * gy) * (0.5 * self.samplePeriod)
        q3 = pb + (q1 * gy - pa * gz + pc * gx) * (0.5 * self.samplePeriod)
        q4 = pc + (q1 * gz + pa * gy - pb * gx) * (0.5 * self.samplePeriod)

        # Normalise quaternion
        norm = math.sqrt(q1 * q1 + q2 * q2 + q3 * q3 + q4 * q4)
        norm = 1.0 / norm
        self.quaternion[0] = q1 * norm
        self.quaternion[1] = q2 * norm
        self.quaternion[2] = q3 * norm
        self.quaternion[3] = q4 * norm

