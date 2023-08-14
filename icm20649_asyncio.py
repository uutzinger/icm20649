#!/usr/bin/python3

################################################################
# ICM 20649 IMU
################################################################

# IMPORTS
#################################################################
import asyncio
import logging
import argparse
import signal
import math
import time
import numpy as np
import os
from copy import copy

from pyIMU.quaternion import Vector3D

import board
from adafruit_icm20x import ICM20649, AccelRange, GyroRange, AccelDLPFFreq, GyroDLPFFreq

if os.name != 'nt':
    import uvloop
    asyncio.set_event_loop_policy(uvloop.EventLoopPolicy())
    import subprocess

RAD2DEG = 180.0 / math.pi
DEG2RAD = math.pi / 180.0
TWOPI   = 2.0*math.pi

# Program Timing:
################################################################
REPORTINTERVAL                   = 1./10. # 10 Hz

FUZZY_ACCEL_ZERO_MAX    = 10.0  # threshold for acceleration activity
FUZZY_ACCEL_ZERO_MIN    = 9.8   # threshold for acceleration activity
FUZZY_DELTA_ACCEL_ZERO  = 0.09  # threshold for acceleration change
FUZZY_GYRO_ZERO         = 0.04  # threshold for gyroscope activity
FUZZY_DELTA_GYRO_ZERO   = 0.015 # threshold for gyroscope change

def detectMotion(acc: float, gyr: float, acc_avg: float, gyr_avg:float) -> bool:
        # Three Stage Motion Detection
        # Original Code is from FreeIMU Processing example
        # Some modifications and tuning
        #
        # 0. Acceleration Activity
        # 1. Change in Acceleration
        # 2. Gyration Activity
        # 2. Change in Gyration

        # ACCELEROMETER
        # Absolute value
        acc_test       = abs(acc) > FUZZY_ACCEL_ZERO_MAX and abs(acc) < FUZZY_ACCEL_ZERO_MIN
        # Sudden changes
        acc_delta_test = abs(acc_avg - acc) > FUZZY_DELTA_ACCEL_ZERO

        # GYROSCOPE
        # Absolute value
        gyr_test       = abs(gyr)           > FUZZY_GYRO_ZERO
        # Sudden changes
        gyr_delta_test = abs(gyr_avg - gyr) > FUZZY_DELTA_GYRO_ZERO

        # DEBUG for fine tuning
        # print(abs(acc), abs(acc_avg-acc), abs(gyr), abs(gyr_avg-gyr), acc_test, acc_delta_test, gyr_test, gyr_delta_test)

        # Combine acceleration test, acceleration deviation test and gyro test
        return (acc_test or acc_delta_test or gyr_test or gyr_delta_test)

#########################################################################################################
# ICM 20x
#########################################################################################################

class icm20x:

    def __init__(self, logger=None, args=None) -> None:

        self.args                       = args

        # Signals
        self.dataAvailable          = asyncio.Event()
        self.terminate              = asyncio.Event()
        # These Signals are easier to deal with without Event
        self.finish_up              = False

        if logger is not None: self.logger = logger
        else:                  self.logger = logging.getLogger('icm20x')

        # device sensors
        self.sensorTime             = 0.    # Sensor Time
        self.previous_sensorTime    = 0.    #

        # IMU
        self.accX                   = 0.    # IMU Accelerometer
        self.accY                   = 0.    #
        self.accZ                   = 0.    #
        self.gyrX                   = 0.    # IMY Gyroscope
        self.gyrY                   = 0.    #
        self.gyrZ                   = 0.    #
        self.magX                   = 0.    # IMU Magnetometer
        self.magY                   = 0.    #
        self.magZ                   = 0.    #

        # Timing
        ###################
        self.startTime              = 0.
        self.runTime                = 0.
        self.delta_sensorTime       = 0

        self.data_processTime       = 0.
        self.data_rate              = 0
        self.data_updateCounts      = 0
        self.data_lastTime          = time.perf_counter()
        self.data_lastTimeRate      = time.perf_counter()

        self.report_deltaTime       = 0.
        self.report_rate            = 0
        self.report_updateInterval  = REPORTINTERVAL
        self.report_updateCounts    = 0
        self.report_lastTimeRate    = time.perf_counter()

        self.i2cHits                = 0
        self.i2cCounts              = 0

        self.acc                    = Vector3D(0.,0.,0.)
        self.gyr                    = Vector3D(0.,0.,0.)
        self.mag                    = Vector3D(0.,0.,0.)
        self.acc_average            = Vector3D(0.,0.,0.)
        self.gyr_average            = Vector3D(0.,0.,0.)
        self.acc_offset             = Vector3D(0.,0.,0.)
        self.gyr_offset             = Vector3D(0.,0.,0.)
        self.gyr_offset_updated     = False

        self.moving                 = True

        # First Time Getting Data
        self.firstTimeData          = True  # want to initialize average acc,mag,gyr with current reading first time
        self.sensorIsBooting        = True  # need to read sensor a few times until we get reasonable data
        self.sensorRunInCounts      = 0     # for boot up

        self.i2c                    = board.I2C()  # uses board.SCL and board.SDA
        self.icm                    = ICM20649(self.i2c, address=0x69)
        self.mag_available          = False
        self.magok                  = False

        # Sensor range
        self.icm.accelerometer_range     = AccelRange.RANGE_4G     # 4G, 8G, 16G, 30G
        self.icm.gyro_range              = GyroRange.RANGE_500_DPS # 500, 1000, 2000, 4000
        # This is output rate, internally runs at max rate
        self.icm.accelerometer_data_rate = 500                     # 0.27 ..1125 HZ
        self.icm.gyro_data_rate          = 500                     # 4.3 .. 1100 Hz
        # Low pass filters
        self.icm.accel_dlpf_cutoff       = AccelDLPFFreq.FREQ_111_4HZ_3DB # 246, 111.4, 50.4, 23.9, 11.5, 5.7, 473
        self.icm.gyro_dlpf_cutoff        = GyroDLPFFreq.FREQ_151_8HZ_3DB  # 196.6, 151.8, 119.5, 51.2, 23.9, 11.6, 5.7, 361.4, 

        self.logger.log(logging.INFO, "Accelerometer range set to: {:d} g".format(AccelRange.string[self.icm.accelerometer_range]))
        self.logger.log(logging.INFO, "Gyro range set to:          {:d} dps".format(GyroRange.string[self.icm.gyro_range]))
        self.logger.log(logging.INFO, "Acc  data rate set to:      {:f} Hz".format(self.icm.accelerometer_data_rate))
        self.logger.log(logging.INFO, "Gyro data rate set to:      {:f} Hz".format(self.icm.gyro_data_rate))
        self.logger.log(logging.INFO, "Acc  low pass 3dB point at: {:f} Hz".format(AccelDLPFFreq.string[self.icm.accel_dlpf_cutoff]))
        self.logger.log(logging.INFO, "Gyro low pass 3dB point at: {:f} Hz".format(GyroDLPFFreq.string[self.icm.gyro_dlpf_cutoff]))

    ##############################################################################################
    # Sensor Loop
    ##############################################################################################

    async def update_data(self):
        '''
        Check if data available
        Obtain data from sensor
        '''

        while not self.finish_up:

            startTime = time.perf_counter()

            # Disregrad the first 50 measurements
            if self.sensorIsBooting:
                self.sensorRunInCounts += 1
                if self.sensorRunInCounts > 50:
                    self.sensorIsBooting = False
                    self.sensorRunInCounts = 0

            # Update data rate
            self.data_updateCounts += 1
            if startTime - self.data_lastTimeRate >= 1:
                self.data_rate = copy(self.data_updateCounts)
                self.data_lastTimeRate = copy(startTime)
                self.data_updateCounts = 0

            # Process the Data
            ###############################################################
            if not self.sensorIsBooting:

                # Obtain Data
                ###############################################################
                # if True:
                if self.icm.dataReady: # is there new data

                    self.i2cCounts = 0

                    # ICM does not provide internal time stamp
                    self.sensorTime = time.perf_counter()
                    self.delta_sensorTime = self.sensorTime - self.previous_sensorTime
                    # for data fusion we need time interval between readings
                    self.previous_sensorTime = copy(self.sensorTime) 

                    # Obtain new data
                    self.acc=Vector3D(*self.icm.acceleration)
                    self.gyr=Vector3D(*self.icm.gyro)
                    if self.mag_available:
                        self.mag=Vector3D(*self.icm.magnetic)
                    else:
                        self.mage=Vector3D(0.,0.,0.)

                    # Low pass filtered data for motion detection
                    if self.firstTimeData:
                        self.firstTimeData = False
                        self.gyr_average = self.gyr # Initialize the low pass filter
                        self.acc_average = self.acc
                    else:
                        self.gyr_average = 0.99*self.gyr_average + 0.01*self.gyr # Poor man's lowpass
                        self.acc_average = 0.99*self.acc_average + 0.01*self.acc

                    # Detect motion
                    self.moving = detectMotion(self.acc.norm, self.gyr.norm, self.acc_average.norm, self.gyr_average.norm)
                    # Update gyroscope offset
                    if not self.moving:
                        self.gyr_offset = 0.99*self.gyr_offset + 0.01*self.gyr
                        self.gyr_offset_updated = True

                    self.data_processTime = time.perf_counter() - startTime

                else:
                    # Keep track if we had to wait for data
                    # If we never need to wait for data we should not check if new data is available
                    self.i2cCounts += 1
                    if self.i2cCounts > self.i2cHits:
                        self.i2cHits = copy(self.i2cCounts)

            await asyncio.sleep(0) # allow other tasks to run

    ##############################################################################################
    # gearVRC Tasks
    ##############################################################################################

    async def update_report(self):
        '''
        Report latest data
        '''

        self.logger.log(logging.INFO, 'Starting Reporting Task...')

        self.report_lastTimeRate    = time.perf_counter()
        self.report_updateCounts    = 0

        while not self.finish_up:

            startTime = time.perf_counter()

            self.report_updateCounts += 1
            if (startTime - self.report_lastTimeRate)>= 1.:
                self.report_rate = copy(self.report_updateCounts)
                self.report_lastTimeRate = time.perf_counter()
                self.report_updateCounts = 0

            # Display the Data
            # We create text buffer first
            # Then print it to terminal

            msg_out = '\033[2J\n'
            msg_out+= '-------------------------------------------------\n'
            if self.args.report > 0:
                msg_out+= 'icm 20x: Moving:{}, Mag:{}\n'.format(
                                                    'Y' if self.moving else 'N',
                                                    'Y' if self.magok  else 'N')

            msg_out+= '-------------------------------------------------\n'

            if self.args.report > 0:
                msg_out+= 'Data    {:>10.6f}us, {:>3d}/s\n'.format(self.data_processTime*1000., self.data_rate)
                msg_out+= 'Report  {:>10.6f}us, {:>3d}/s\n'.format(self.report_deltaTime*1000., self.report_rate)
                msg_out+= 'Max i2c polls until data {:d}\n'.format(self.i2cHits)
                msg_out+= '-------------------------------------------------\n'

            if self.args.report > 1:
                msg_out+= 'Time  {:>10.6f}, dt {:>10.6f}\n'.format(self.sensorTime, self.delta_sensorTime)
                msg_out+= 'Accel     {:>8.3f} {:>8.3f} {:>8.3f} N:  {:>8.3f}\n'.format(self.acc.x,self.acc.y,self.acc.z,self.acc.norm)
                msg_out+= 'Gyro      {:>8.3f} {:>8.3f} {:>8.3f} N:  {:>8.3f}\n'.format(self.gyr.x,self.gyr.y,self.gyr.z,self.gyr.norm)
                if self.magok: msg_out+= 'Magno     {:>8.3f} {:>8.3f} {:>8.3f} N:  {:>8.3f}\n'.format(self.mag.x,self.mag.y,self.mag.z,self.mag.norm)
                msg_out+= 'Accel avg {:>8.3f} {:>8.3f} {:>8.3f} N:  {:>8.3f}\n'.format(self.acc_average.x, self.acc_average.y, self.acc_average.z, self.acc_average.norm)
                msg_out+= 'Gyro  avg {:>8.3f} {:>8.3f} {:>8.3f} RPM:{:>8.3f}\n'.format(self.gyr_average.x, self.gyr_average.y, self.gyr_average.z, self.gyr_average.norm*60./TWOPI)
                msg_out+= 'Gyro bias {:>8.3f} {:>8.3f} {:>8.3f} RPM:{:>8.3f}\n'.format(self.gyr_offset.x, self.gyr_offset.y, self.gyr_offset.z, self.gyr_offset.norm*60./TWOPI)

                msg_out+= '-------------------------------------------------\n'


            print(msg_out, flush=True)

            self.report_deltaTime = time.perf_counter() - startTime

            # Wait to next interval time
            sleepTime = self.report_updateInterval - (time.perf_counter() - startTime)
            await asyncio.sleep(max(0.,sleepTime))
            timingError = time.perf_counter() - startTime - self.report_updateInterval
            self.report_updateInterval = max(0., REPORTINTERVAL - timingError)

        self.logger.log(logging.INFO, 'Reporting stopped')

    async def handle_termination(self, tasks:None):
        '''
        Cancel tasks based on provided list
        '''
        self.logger.log(logging.INFO, 'Control-C or Kill signal detected')
        self.finish_up = True
        if tasks is not None:
            self.logger.log(logging.INFO, 'Cancelling all Tasks...')
            await asyncio.sleep(1) # give some time for tasks to finish up
            for task in tasks:
                if task is not None:
                    task.cancel()

##############################################################################################
# MAIN
##############################################################################################

async def main(args: argparse.Namespace):

    # Setup logging
    logger = logging.getLogger(__name__)
    logger.log(logging.INFO, 'Starting ICM 20649 IMU...')

    # ICM IMU
    imu = icm20x(logger=logger, args=args)

    # Create all the async tasks
    # They will run until stop signal is created, stop signal is indicated with event
    imu_task = asyncio.create_task(imu.update_data())

    tasks = [imu_task] # task list

    if args.report > 0:
        reporting_task  = asyncio.create_task(imu.update_report())   # report new data, will not terminate
        tasks.append(reporting_task)

    # Set up a Control-C handler to gracefully stop the program
    # This mechanism is only available in Unix
    if os.name == 'posix':
        # Get the main event loop
        loop = asyncio.get_running_loop()
        loop.add_signal_handler(signal.SIGINT,  lambda: asyncio.create_task(imu.handle_termination(tasks=tasks)) ) # control-c
        loop.add_signal_handler(signal.SIGTERM, lambda: asyncio.create_task(imu.handle_termination(tasks=tasks)) ) # kill

    # Wait until all tasks are completed, which is when user wants to terminate the program
    await asyncio.wait(tasks, timeout=float('inf'))

    logger.log(logging.INFO,'ICM 20649 exit')

if __name__ == '__main__':

    parser = argparse.ArgumentParser(description="ICM 20649")

    parser.add_argument(
        '-d',
        '--debug',
        action='store_true',
        help='sets the log level from info to debug',
        default = False
    )

    parser.add_argument(
        '-r',
        '--report',
        dest = 'report',
        type = int,
        metavar='<report>',
        help='report level: 0(None), 1(Rate only), 2(Regular)',
        default = 0
    )

    args = parser.parse_args()

    log_level = logging.DEBUG if args.debug else logging.INFO
    logging.basicConfig(
        level=log_level,
        # format='%(asctime)-15s %(name)-8s %(levelname)s: %(message)s'
        format='%(asctime)-15s %(levelname)s: %(message)s'
    )

    try:
        asyncio.run(main(args))
    except KeyboardInterrupt:
        pass
