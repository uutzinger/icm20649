# SPDX-FileCopyrightText: 2021 ladyada for Adafruit Industries
# SPDX-License-Identifier: MIT

import time
import board
from copy import copy
from adafruit_icm20x import ICM20649, AccelRange, GyroRange


def printNewMax(value, current_max, axis):
    if value > current_max:
        current_max = value
        print(axis, "Max:", current_max)
    return current_max


i2c = board.I2C()  # uses board.SCL and board.SDA

ism = ICM20649(i2c, address=0x69)

ism.accelerometer_range = AccelRange.RANGE_30G
print("Accelerometer range set to: %d g" % AccelRange.string[ism.accelerometer_range])

ism.gyro_range = GyroRange.RANGE_500_DPS
print("Gyro range set to: %d DPS" % GyroRange.string[ism.gyro_range])

ax_max = ay_max = az_max = 0
gx_max = gy_max = gz_max = 0

ism.gyro_data_rate = 1100          # 1100
ism.accelerometer_data_rate = 1125 # 1125
print('Gyro rate: {:f}'.format(ism.gyro_data_rate))
print('Accel rate: {:f}'.format(ism.accelerometer_data_rate))
previousTime = time.perf_counter()
while True:
    currentTime = time.perf_counter()
    print('\033[2J')
    print(
        "Accel X:%5.2f Y:%5.2f Z:%5.2f ms^2 Gyro X:%8.3f Y:%8.3f Z:%8.3f degrees/s"
        % (ism.acceleration + ism.gyro)
    )
    print('SPS: {:f}'.format(1.0/(currentTime-previousTime)))
    previousTime = copy(currentTime)
