#!/usr/bin/env python

import math
import os
import struct
import sys
import time
import uuid
import Adafruit_BluefruitLE
from Adafruit_BluefruitLE.services import DeviceInformation
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))
import sensortag.gam as gam

SENSOR_NAME = 'CC2650 SensorTag'
SCAN_TIMEOUT = 30
CONNECT_TIMEOUT = 30
DISCOVER_TIMEOUT = 30

# The TI Base 128-bit UUID is: F0000000-0451-4000-B000-000000000000.

MOVEMENT_SENSOR_SRV = uuid.UUID('f000aa80-0451-4000-b000-000000000000')
# GyroX[0:7], GyroX[8:15], GyroY[0:7], GyroY[8:15], GyroZ[0:7], GyroZ[8:15],
# AccX[0:7], AccX[8:15], AccY[0:7], AccY[8:15], AccZ[0:7], AccZ[8:15],
# MagX[0:7], MagX[8:15], MagY[0:7], MagY[8:15], MagZ[0:7], MagZ[8:15]
MOVEMENT_SENSOR_CHR_DATA = uuid.UUID('f000aa81-0451-4000-b000-000000000000')
# One bit for each gyro and accelerometer axis (6), magnetometer (1),
# wake-on-motion enable (1), accelerometer range (2).
# Write any bit combination top enable the desired features.
# Writing 0x0000 powers the unit off.
MOVEMENT_SENSOR_CHR_CONFIG = uuid.UUID('f000aa82-0451-4000-b000-000000000000')
#  Resolution 10 ms. Range 100 ms (0x0A) to 2.55 sec (0xFF). Default 1 second (0x64).
MOVEMENT_SENSOR_CHR_PERIOD = uuid.UUID('f000aa83-0451-4000-b000-000000000000')
# Write 0x0001 to enable notifications, 0x0000 to disable.
# CLIENT_CHARACTERISTIC_CONFIG = uuid.UUID(
#    '00002902-0000-1000-8000-00805f9b34fb')


provider = Adafruit_BluefruitLE.get_provider()
ahrs = gam.MadgwickAHRS(0.1, 0.075574973)

def main():
    #    provider.clear_cached_data()

    adapter = provider.get_default_adapter()

    print "Waiting powered on..."
    adapter.power_on()
    while not adapter.is_powered:
        time.sleep(1)

    print 'Using adapter {0}'.format(adapter.name)

    device = None

    try:
        print 'Start scanning...'
        adapter.start_scan()
        device = provider.find_device([], SENSOR_NAME, SCAN_TIMEOUT)
    finally:
        adapter.stop_scan()

    if device is None:
        print 'SensorTag not found'
        return

    try:
        print 'Connecting...'
        device.connect(CONNECT_TIMEOUT)
    except RuntimeError as e:
        print e
        return

    try:
        print 'Discovering device information...'
        DeviceInformation.discover(device)

        di = DeviceInformation(device)
        print 'Manufacture: {0}'.format(di.manufacturer)
        print 'Model: {0}'.format(di.model)
        print 'Serial: {0}'.format(di.serial)
        print 'Hardware Revision: {0}'.format(di.hw_revision)
        print 'Software Revision: {0}'.format(di.sw_revision)
        print 'Firmware Revision: {0}'.format(di.fw_revision)
        print 'System ID: {0}'.format(di.system_id)

        print 'Discovering service...'
        try:
            device.discover([MOVEMENT_SENSOR_SRV], [
                            MOVEMENT_SENSOR_CHR_CONFIG,
                            MOVEMENT_SENSOR_CHR_DATA,
                            MOVEMENT_SENSOR_CHR_PERIOD
                            ], DISCOVER_TIMEOUT)
        except RuntimeError as e:
            print e
            return

        service = device.find_service(MOVEMENT_SENSOR_SRV)
        if service is None:
            print 'Service not found'
            return

        print 'Setting period...'
        c_period = service.find_characteristic(MOVEMENT_SENSOR_CHR_PERIOD)
        if c_period is None:
            print 'Period Characteristics not found'
            return
        c_period.write_value('\x0a') # 100 msec

        #
        # Notification callback
        #
        def on_notified(data):
            [gx, gy, gz, ax, ay, az, mx, my, mz] = gam.raw2nineaxis(data)
            ahrs.update(gam.deg2rad(gx), gam.deg2rad(gy),
                        gam.deg2rad(gz), ax, ay, az, mx, my, mz)
            roll, pitch, yaw = gam.quaternion2euler(ahrs.quaternion)
            print '%10.5f %10.5f %10.5f' % (
                math.degrees(roll), math.degrees(pitch), math.degrees(yaw))

        print 'Subsribing notification...'
        c_data = service.find_characteristic(MOVEMENT_SENSOR_CHR_DATA)
        if c_data is None:
            print 'Data Characteristics not found'
            return
        c_data.start_notify(on_notified)

        print 'Enabling notification...'
        c_data._device._peripheral.setNotifyValue_forCharacteristic_(
            True, c_data._characteristic)

        print 'Enabling sensor...'
        c_config = service.find_characteristic(MOVEMENT_SENSOR_CHR_CONFIG)
        if c_config is None:
            print 'Config Characteristics not found'
            return
        c_config.write_value('\x7f\x02') # ACC RANGE 8G

        print 'Now, sleeping 60 seconds...'
        time.sleep(60)

    finally:
        print 'Disconnecting device...'
        device.disconnect()


provider.initialize()
provider.run_mainloop_with(main)
