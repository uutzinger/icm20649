#!/usr/bin/python3

################################################################
# ICM 20649 IMU
################################################################

# IMPORTS
#########################################u#######################
import asyncio
import logging
import argparse
import signal
import math
import time
import numpy as np
import os
from   copy import copy
import msgpack
import pathlib
import json

import board
from adafruit_icm20x import ICM20649, AccelRange, GyroRange, AccelDLPFFreq, GyroDLPFFreq

from pyIMU.madgwick import Madgwick
from pyIMU.quaternion import Vector3D, Quaternion
from pyIMU.utilities import q2rpy, rpymag2h
from pyIMU.motion import Motion

import zmq
import zmq.asyncio

if os.name != 'nt':
    import uvloop
    asyncio.set_event_loop_policy(uvloop.EventLoopPolicy())
    import subprocess
    
RAD2DEG = 180.0 / math.pi
DEG2RAD = math.pi / 180.0
TWOPI   = 2.0*math.pi

ZMQPORT  = 5556

# LOCATION
################################################################
DECLINATION  = 8.124973426113137 * DEG2RAD      # Declination at your location
LATITUDE     = 32.253460           # [degrees] Tucson
LONGITUDE    = -110.911789         # [degrees] Tucson
ALTITUDE     = 730                 # [m], Tucson
MAGFIELD     = 33078.93064435485   # [nano Tesla] Tucson
MAGFIELD_MAX = 1.2*MAGFIELD/1000.  # micro Tesla
MAGFIELD_MIN = 0.8*MAGFIELD/1000.  # micro Tesla

# Program Timing:
################################################################
REPORTINTERVAL                   = 1./10. # 10 Hz

FUZZY_ACCEL_ZERO_MAX    = 10.0  # threshold for acceleration activity
FUZZY_ACCEL_ZERO_MIN    = 9.8   # threshold for acceleration activity
FUZZY_DELTA_ACCEL_ZERO  = 0.09  # threshold for acceleration change
FUZZY_GYRO_ZERO         = 0.04  # threshold for gyroscope activity
FUZZY_DELTA_GYRO_ZERO   = 0.015 # threshold for gyroscope change

################################################################
# Support Functions
################################################################

def calibrate(data:Vector3D, offset:Vector3D, crosscorr=None, diagonal=False):
    '''
    IMU calibration
    bias is offset so that 0 is in the middle of the range
    scale is the gain so that 1 is the maximum value
    cross correlation is cross axis sensitivity
    the diagonal elements of the cross correlation matrix are the scale

    Expects data and bias to be a Vector3D
    Expects crosscorr to be 3x3 numpy array
    '''

    # Bias
    d = copy(data)   # we do not want input parameter to change globally
    d = d-offset
    if diagonal:
        if crosscorr is not None:
            d=Vector3D(d.x*crosscorr[0,0], d.y*crosscorr[1,1], d.z*crosscorr[2,2])
    else:
        # Cross Correlation
        if crosscorr is not None:
            d = d.rotate(crosscorr)

    return d

def loadCalibration(filename):
    #
    with open(filename, 'r') as file:
        d = json.load(file)

    center = np.array([d['offset_x'],d['offset_y'],d['offset_z']])
    correctionMat = np.empty([3,3])

    correctionMat[0,0] = d['cMat_00']
    correctionMat[0,1] = d['cMat_01']
    correctionMat[0,2] = d['cMat_02']
    correctionMat[1,0] = d['cMat_10']
    correctionMat[1,1] = d['cMat_11']
    correctionMat[1,2] = d['cMat_12']
    correctionMat[2,0] = d['cMat_20']
    correctionMat[2,1] = d['cMat_21']
    correctionMat[2,2] = d['cMat_22']

    return center, correctionMat

def saveCalibration(filename, center, correctionMat):

    d = {
    "offset_x": center[0],
    "offset_y": center[1],
    "offset_z": center[2],
    "cMat_00":  correctionMat[0,0],
    "cMat_01":  correctionMat[0,1],
    "cMat_02":  correctionMat[0,2],
    "cMat_10":  correctionMat[1,0],
    "cMat_11":  correctionMat[1,1],
    "cMat_12":  correctionMat[1,2],
    "cMat_20":  correctionMat[2,0],
    "cMat_21":  correctionMat[2,1],
    "cMat_22":  correctionMat[2,2]
    }

    with open(filename, 'w') as file:
        json.dump(d, file)

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


def obj2dict(obj):
    '''
    encoding object variables to nested dict
    '''
    if isinstance(obj, dict):
        return {k: obj2dict(v) for k, v in obj.items()}
    elif hasattr(obj, '__dict__'):
        return obj2dict(vars(obj))
    elif isinstance(obj, list):
        return [obj2dict(item) for item in obj]
    else:
        return obj

class dict2obj:
    '''
    decoding nested dictionary to object
    '''
    def __init__(self, data):
        for key, value in data.items():
            if isinstance(value, dict):
                setattr(self, key, dict2obj(value))
            else:
                setattr(self, key, value)

###################################################################
# Data Classes
###################################################################

class iCMSystemData(object):
    '''System relevant performance data'''
    def __init__(self,  
                 data_rate: int = 0, fusion_rate:    int = 0, 
                 zmq_rate:  int = 0, reporting_rate: int = 0) -> None:
        self.data_rate       = data_rate
        self.fusion_rate     = fusion_rate
        self.zmq_rate        = zmq_rate
        self.reporting_rate  = reporting_rate

class icIMUData(object):
    '''IMU Data from the sensor'''
    def __init__(self, 
                 time: float=0.0,
                 acc: Vector3D = Vector3D(0.,0.,0.),
                 gyr: Vector3D = Vector3D(0.,0.,0.),
                 mag: Vector3D = Vector3D(0.,0.,0.),
                 moving: bool = True,
                 magok: bool  = False) -> None:
        self.time = time
        self.acc  = acc
        self.mag  = mag
        self.gyr  = gyr
        self.moving = moving
        self.magok = magok

class icmFusionData(object):
    '''AHRS fusion data'''
    def __init__(self, 
                 time: float   = 0.,
                 acc: Vector3D = Vector3D(0.,0.,0.),
                 mag: Vector3D = Vector3D(0.,0.,0.),
                 gyr: Vector3D = Vector3D(0.,0.,0.),
                 rpy: Vector3D = Vector3D(0.,0.,0.),
                 heading: float = 0.0,
                 q: Quaternion = Quaternion(1.,0.,0.,0.)) -> None:
        self.time           = time
        self.acc            = acc
        self.mag            = mag
        self.gyr            = gyr
        self.rpy            = rpy
        self.heading        = heading
        self.q              = q

class icmMotionData(object):
    '''Motion data'''
    def __init__(self,
                 time: float   = 0.,
                 residuals:    Vector3D = Vector3D(0.,0.,0.),
                 velocity:     Vector3D = Vector3D(0.,0.,0.),
                 position:     Vector3D = Vector3D(0.,0.,0.),
                 accBias:      Vector3D = Vector3D(0.,0.,0.),
                 velocityBias: Vector3D = Vector3D(0.,0.,0.),
                 dtmotion:     float = 0.0) -> None:
            self.time         = time
            self.residuals    = residuals
            self.velocity     = velocity
            self.position     = position
            self.dtmotion     = dtmotion
            self.accBias      = accBias
            self.velocityBias = velocityBias

#########################################################################################################
# ZMQ Data Receiver for ICM 20x
# Primarily useful for 3rd party client
#########################################################################################################

class zmqWorkerICM():

    def __init__(self, logger, zmqPort='tcp://localhost:5556', parent=None):
        super(zmqWorkerICM, self).__init__(parent)

        self.dataReady =  asyncio.Event()
        self.finished  =  asyncio.Event()

        self.logger     = logger
        self.finish_up  = False
        self.paused     = False
        self.zmqPort    = zmqPort

        self.new_system = False
        self.new_imu    = False
        self.new_fusion = False
        self.new_motion = False
        self.timeout    = False

        self.zmqTimeout = ZMQTIMEOUT

        self.logger.log(logging.INFO, 'IC20x zmqWorker initialized')

    async def start(self, stop_event: asyncio.Event):

        self.new_system = False
        self.new_imu    = False
        self.new_fusion = False
        self.new_motion = False

        context = zmq.Context()
        poller  = zmq.Poller()
        
        self.data_system = None
        self.data_imu    = None
        self.data_motion = None
        self.data_fusion = None

        socket = context.socket(zmq.SUB)
        socket.setsockopt(zmq.SUBSCRIBE, b"system")
        socket.setsockopt(zmq.SUBSCRIBE, b"imu")
        socket.setsockopt(zmq.SUBSCRIBE, b"fusion")
        socket.setsockopt(zmq.SUBSCRIBE, b"motion")
        socket.connect(self.zmqPort)
        poller.register(socket, zmq.POLLIN)

        self.logger.log(logging.INFO, 'IC20x  zmqWorker started on {}'.format(self.zmqPort))

        while not stop_event.is_set():
            try:
                events = dict(poller.poll(timeout=self.zmqTimeout))
                if socket in events and events[socket] == zmq.POLLIN:
                    response = socket.recv_multipart()
                    if len(response) == 2:
                        [topic, msg_packed] = response
                        if topic == b"system":
                            msg_dict = msgpack.unpackb(msg_packed)
                            self.data_system = dict2obj(msg_dict)
                            self.new_system = True
                        elif topic == b"imu":
                            msg_dict = msgpack.unpackb(msg_packed)
                            self.data_imu = dict2obj(msg_dict)
                            self.new_imu = True                            
                        if topic == b"fusion":
                            msg_dict = msgpack.unpackb(msg_packed)
                            self.data_fusion = dict2obj(msg_dict)
                            self.new_fusion = True
                        elif topic == b"motion":
                            msg_dict = msgpack.unpackb(msg_packed)
                            self.data_motion = dict2obj(msg_dict)
                            self.new_motion = True
                        else:
                            pass  # not a topic we need
                    else:
                        self.logger.log(
                            logging.ERROR, 'IC20x zmqWorker malformed message')
                else:  # ZMQ TIMEOUT
                    self.logger.log(logging.ERROR, 'IC20x zmqWorker timed out')
                    poller.unregister(socket)
                    socket.close()
                    socket = context.socket(zmq.SUB)
                    socket.setsockopt(zmq.SUBSCRIBE, b"system")
                    socket.setsockopt(zmq.SUBSCRIBE, b"imu")
                    socket.setsockopt(zmq.SUBSCRIBE, b"fusion")
                    socket.setsockopt(zmq.SUBSCRIBE, b"motion")
                    socket.connect(self.zmqPort)
                    poller.register(socket, zmq.POLLIN)
                    self.new_system = \
                    self.new_imu    = \
                    self.new_fusion = \
                    self.new_motion = False

                if (self.new_imu or self.new_fusion or self.new_motion):
                    if not self.paused:
                        self.dataReady.set()
                        self.new_system  = \
                        self.new_imu     = \
                        self.new_fusion  = \
                        self.new_motion  = False
                else:
                    if not self.paused:
                        pass

            except:
                self.logger.log(logging.ERROR, 'IC20x zmqWorker error')
                poller.unregister(socket)
                socket.close()
                socket = context.socket(zmq.SUB)
                socket.setsockopt(zmq.SUBSCRIBE, b"system")
                socket.setsockopt(zmq.SUBSCRIBE, b"imu")
                socket.setsockopt(zmq.SUBSCRIBE, b"fusion")
                socket.setsockopt(zmq.SUBSCRIBE, b"motion")
                socket.connect(self.zmqPort)
                poller.register(socket, zmq.POLLIN)
                self.new_system = \
                self.new_imu    = \
                self.new_fusion = \
                self.new_motion = False

        self.logger.log(logging.DEBUG, 'IC20x zmqWorker finished')
        socket.close()
        context.term()
        self.finished.set()

    def set_zmqPort(self, port):
        self.zmqPort = port

    def pause(self):
        self.paused = not self.paused

#########################################################################################################
# ICM 20649
#########################################################################################################

class icm20x:

    def __init__(self, logger=None,
                  args=None) -> None:

        # super(icm20x, self).__init__()

        self.args                               = args

        # Signals
        self.processedDataAvailable     = asyncio.Event()
        self.terminate                  = asyncio.Event()
        # These Signals are easier to deal with without Event
        self.connected                  = False
        self.sensorStarted              = False
        self.finish_up                  = False

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

        self.data_deltaTime         = 0.
        self.data_rate              = 0
        self.data_updateCounts      = 0
        self.data_lastTime          = time.perf_counter()
        self.data_lastTimeRate      = time.perf_counter()

        self.fusion_deltaTime       = 0.
        self.fusion_rate            = 0
        self.fusion_updateCounts    = 0
        self.fusion_lastTimeRate    = time.perf_counter()
        self.previous_fusionTime    = time.perf_counter()

        self.motion_deltaTime       = 0.
        self.motion_rate            = 0
        self.motion_updateCounts    = 0
        self.motion_lastTimeRate    = time.perf_counter()
        self.previous_motionTime    = time.perf_counter()
        self.firstTimeMotion        = time.perf_counter()

        self.report_deltaTime       = 0.
        self.report_rate            = 0
        self.report_updateInterval  = REPORTINTERVAL
        self.report_updateCounts    = 0
        self.report_lastTimeRate    = time.perf_counter()

        self.zmq_rate               = 0
        self.zmq_deltaTime          = 0.
        self.zmq_updateCounts       = 0
        self.zmq_lastTimeRate       = time.perf_counter()

        self.i2cHits                = 0
        self.i2cCounts              = 0

        self.current_directory = str(pathlib.Path(__file__).parent.absolute())

        my_file = pathlib.Path(self.current_directory + '/Gyr.json')
        if my_file.is_file():
            self.logger.log(logging.INFO,'Loading Gyroscope Calibration from File...')
            gyr_offset, gyr_crosscorr = loadCalibration(my_file)
        else:
            self.logger.log(logging.INFO,'Loading default Gyroscope Calibration...')
            gyr_crosscorr  = np.array(([1.,0.,0.], [0.,1.,0.], [0.,0.,1.]))
            gyr_offset     = np.array(([-0.01335,-0.01048,0.03801]))

        my_file = pathlib.Path(self.current_directory + '/Acc.json')
        if my_file.is_file():
            self.logger.log(logging.INFO,'Loading Accelerometer Calibration from File...')
            acc_offset, acc_crosscorr = loadCalibration(my_file)
        else:
            self.logger.log(logging.INFO,'Loading default Accelerometer Calibration...')
            acc_crosscorr  = np.array(([1.,0.,0.], [0.,1.,0.], [0.,0.,1.]))
            acc_offset     = np.array(([0.,0.,0.]))

        my_file = pathlib.Path(self.current_directory + '/Mag.json')
        if my_file.is_file():
            self.logger.log(logging.INFO,'Loading Magnetometer Calibration from File...')
            mag_offset, mag_crosscorr = loadCalibration(my_file)
        else:
            self.logger.log(logging.INFO,'Loading default Magnetometer Calibration...')
            mag_crosscorr  = np.array(([1.,0.,0.], [0.,1.,0.], [0.,0.,1.]))
            mag_offset     = np.array(([-48.492,-222.802,-35.28]))

        self.acc_offset    = Vector3D(acc_offset)
        self.gyr_offset    = Vector3D(gyr_offset)
        self.mag_offset    = Vector3D(mag_offset)
        self.acc_crosscorr = acc_crosscorr
        self.gyr_crosscorr = gyr_crosscorr
        self.mag_crosscorr = mag_crosscorr

        self.gyr_offset_updated = False

        self.acc           = Vector3D(0.,0.,0.)
        self.gyr           = Vector3D(0.,0.,0.)
        self.mag           = Vector3D(0.,0.,0.)
        self.gyr_average   = Vector3D(0.,0.,0.)
        self.acc_average   = Vector3D(0.,0.,0.)

        # Attitude fusion
        self.AHRS          = Madgwick()
        self.q             = Quaternion(1.,0.,0.,0.)
        self.heading       = 0.
        self.rpy           = Vector3D(0.,0.,0.)

        self.acc_cal       = Vector3D(0.,0.,0.)
        self.gyr_cal       = Vector3D(0.,0.,0.)
        self.mag_cal       = Vector3D(0.,0.,0.)

        self.moving        = True
        self.magok         = False
        self.mag_available = False

        # Motion
        self.Position      = Motion()
        self.residuals     = Vector3D(0.,0.,0.)
        self.velocity      = Vector3D(0.,0.,0.)
        self.position      = Vector3D(0.,0.,0.)
        self.dtmotion      = 0.
        self.dt            = 0.
        self.accBias       = Vector3D(0.,0.,0.)
        self.velocityBias  = Vector3D(0.,0.,0.)

        # First Time Getting Data
        self.firstTimeData     = True  # want to initialize average acc,mag,gyr with current reading first time
        self.sensorIsBooting   = True  # need to read sensor a few times until we get reasonable data
        self.sensorRunInCounts = 0     # for boot up

        self.i2c = board.I2C()  # uses board.SCL and board.SDA
        self.icm = ICM20649(self.i2c, address=0x69)
        self.mag_available = False

        self.icm.accelerometer_range     = AccelRange.RANGE_4G     # 4G, 8G, 16G, 30G
        self.icm.gyro_range              = GyroRange.RANGE_500_DPS # 500, 1000, 2000, 4000
        self.logger.log(logging.INFO, "Accelerometer range set to: {:d} g".format(AccelRange.string[self.icm.accelerometer_range]))
        self.logger.log(logging.INFO, "Gyro range set to: {:d} DPS".format(GyroRange.string[self.icm.gyro_range]))

        # This is output rate, internally runs at max rate
        self.icm.gyro_data_rate          = 500                     # 4.3 .. 1100 Hz
        self.icm.accelerometer_data_rate = 500                     # 0.27 ..1125 HZ
        self.logger.log(logging.INFO, "Gyro data rate set to: {:f} Hz".format(self.icm.gyro_data_rate))
        self.logger.log(logging.INFO, "Acc  data rate set to: {:f} Hz".format(self.icm.accelerometer_data_rate))

        # Low pass filters
        self.icm.accel_dlpf_cutoff       = AccelDLPFFreq.FREQ_111_4HZ_3DB # 246, 111.4, 50.4, 23.9, 11.5, 5.7, 473
        self.icm.gyro_dlpf_cutoff        = GyroDLPFFreq.FREQ_151_8HZ_3DB  # 196.6, 151.8, 119.5, 51.2, 23.9, 11.6, 5.7, 361.4, 
        self.logger.log(logging.INFO, "Gyro low pass 3dB point at: {:f} Hz".format(GyroDLPFFreq.string[self.icm.gyro_dlpf_cutoff]))
        self.logger.log(logging.INFO, "Acc  low pass 3dB point at: {:f} Hz".format(AccelDLPFFreq.string[self.icm.accel_dlpf_cutoff]))

    def update_times(self):
        self.data_lastTime          = \
        self.data_lastTimeRate      = \
        self.fusion_lastTimeRate    = \
        self.previous_fusionTime    = \
        self.report_lastTimeRate    = \
        self.zmq_lastTimeRate       = \
        self.motion_lastTimeRate    = \
        self.previous_motionTime    = \
        self.firstTimeMotion        = time.perf_counter()

    def compute_fusion(self):
        '''
        Update AHRS and compute heading and roll/pitch/yaw
        Currently uses pyIMU MadgwickAHRS, in future might be upgraded to imufusion & https://pypi.org/project/imufusion/
        '''

        dt = self.sensorTime - self.previous_fusionTime # time interval between sensor data
        self.previous_fusionTime = copy(self.sensorTime) # keep track of last sensor time

        # Calibrate IMU Data
        self.acc_cal = calibrate(data=self.acc, offset=self.acc_offset, crosscorr=self.acc_crosscorr, diagonal=True)
        self.gyr_cal = calibrate(data=self.gyr, offset=self.gyr_offset, crosscorr=self.gyr_crosscorr, diagonal=True)
        if self.mag_available:
            self.mag_cal = calibrate(data=self.mag, offset=self.mag_offset, crosscorr=self.mag_crosscorr, diagonal=True)
        else:
            self.mag_cal = Vector3D(0.,0.,0.)

        if dt > 1.0:
            # First run or reconnection, need AHRS algorithm to initialize
            if self.mag_available:
                if (self.mag_cal.norm >  MAGFIELD_MAX) or (self.mag_cal.norm < MAGFIELD_MIN):
                    # Mag is not in acceptable range
                    self.q = self.AHRS.update(acc=self.acc_cal,gyr=self.gyr_cal,mag=None,dt=-1)
                    self.magok = False
                else:
                    self.q = self.AHRS.update(acc=self.acc_cal,gyr=self.gyr_cal,mag=self.mag_cal,dt=-1)
                    self.magok = True
            else:
                self.q = self.AHRS.update(acc=self.acc_cal,gyr=self.gyr_cal,mag=None,dt=-1)
                self.magok = False

        else:
            if self.mag_available:
                if (self.mag_cal.norm >  MAGFIELD_MAX) or (self.mag_cal.norm < MAGFIELD_MIN):
                    # Mag not in acceptable range
                    self.q = self.AHRS.update(acc=self.acc_cal,gyr=self.gyr_cal,mag=None,dt=dt)
                    self.magok = False
                else:
                    self.q = self.AHRS.update(acc=self.acc_cal,gyr=self.gyr_cal,mag=self.mag_cal,dt=dt)
                    self.magok = True
            else:
                self.q = self.AHRS.update(acc=self.acc_cal,gyr=self.gyr_cal,mag=None,dt=dt)
                self.magok = False

        self.rpy=q2rpy(q=self.q)
        if self.mag_available:
            self.heading = rpymag2h(rpy=self.rpy, mag=self.mag_cal, declination=DECLINATION)
        else:
            self.heading = 0.0

    def compute_motion(self):
        self.Position.update(q=self.q, acc=self.acc_cal, moving=self.moving, timestamp=self.sensorTime)
        self.residuals    = self.Position.worldResiduals
        self.velocity     = self.Position.worldVelocity
        self.position     = self.Position.worldPosition
        self.dtmotion     = self.Position.dtmotion
        self.dt           = self.Position.dt
        self.accBias      = self.Position.residuals_bias
        self.velocityBias = self.Position.worldVelocity_drift

    ##############################################################################################
    # Sensor Loop
    ##############################################################################################

    async def update_data(self):
        '''
        Check if Data available
        Obtain data from sensor
        Fuse Data
        Update Motion
        '''

        while not self.finish_up:

            startTime = time.perf_counter()

            if self.sensorIsBooting:
                self.sensorRunInCounts += 1
                if self.sensorRunInCounts > 50:
                    self.sensorIsBooting = False
                    self.sensorRunInCounts = 0

            # Update rate
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
                if self.icm.dataReady:

                    self.i2cHits = copy(self.i2cCounts)
                    self.i2cCounts = 0

                    self.sensorTime = time.perf_counter()
                    self.delta_sensorTime = self.sensorTime - self.previous_sensorTime
                    self.previous_sensorTime = copy(self.sensorTime)

                    self.acc=Vector3D(*self.icm.acceleration)
                    self.gyr=Vector3D(*self.icm.gyro)
                    if self.mag_available:
                        self.mag=Vector3D(*self.icm.magnetic)
                    else:
                        self.mage=Vector3D(0.,0.,0.)

                    if self.firstTimeData:
                        self.firstTimeData = False
                        self.gyr_average = self.gyr
                        self.acc_average = self.acc
                    else:
                        self.gyr_average = 0.99*self.gyr_average + 0.01*self.gyr
                        self.acc_average = 0.99*self.acc_average + 0.01*self.acc

                    self.moving = detectMotion(self.acc.norm, self.gyr.norm, self.acc_average.norm, self.gyr_average.norm)
                    if not self.moving:
                        self.gyr_offset = 0.99*self.gyr_offset + 0.01*self.gyr
                        self.gyr_offset_updated = True

                    # Fusion
                    ###############################################################
                    if self.args.fusion:
                        # update interval
                        start_fusionUpdate = time.perf_counter()
                        # fps
                        self.fusion_updateCounts += 1
                        if (startTime - self.fusion_lastTimeRate)>= 1.:
                            self.fusion_rate = copy(self.fusion_updateCounts)
                            self.fusion_lastTimeRate = copy(startTime)
                            self.fusion_updateCounts = 0
                        #
                        self.compute_fusion()
                        self.fusion_deltaTime = time.perf_counter() - start_fusionUpdate

                    # Motion
                    ###############################################################
                    if self.args.motion:
                        # update interval
                        start_motionUpdate = time.perf_counter()
                        # fps
                        self.motion_updateCounts += 1
                        if (startTime - self.motion_lastTimeRate)>= 1.:
                            self.motion_rate = copy(self.motion_updateCounts)
                            self.motion_lastTimeRate = copy(startTime)
                            self.motion_updateCounts = 0

                        # we need some time to compute averages, once system is stable start compute motion.
                        if start_motionUpdate - self.firstTimeMotion > 5.0:
                            self.compute_motion()

                            self.motion_deltaTime = time.perf_counter() - start_motionUpdate

                    self.processedDataAvailable.set()

                    self.data_deltaTime = time.perf_counter() - startTime
                else:
                    self.i2cCounts += 1

            await asyncio.sleep(0) # allow other tasks to run

    ##############################################################################################
    # gearVRC Tasks
    ##############################################################################################

    async def update_gyrOffset(self):
        '''
        Save new Gyroscope Offset in Calibration File every minute if it has changed
        '''

        self.logger.log(logging.INFO, 'Starting Gyroscope Bias Saving Task...')

        while not self.finish_up:

            if self.gyr_offset_updated:
                my_file = pathlib.Path(self.current_directory + '/Gyr.json')
                saveCalibration(my_file, self.gyr_offset, self.gyr_crosscorr)
                self.gyr_offset_updated = False

            await asyncio.sleep(60.0)

        self.logger.log(logging.INFO, 'Gyroscope Bias Saving Task stopped')


    async def update_report(self):
        '''
        Report latest data
        Report fused data
        report motion data
        '''

        self.logger.log(logging.INFO, 'Starting Reporting Task...')

        self.report_lastTimeRate    = time.perf_counter()
        self.report_updateCounts    = 0

        await self.processedDataAvailable.wait()
        # no clear needed as we just wait for system to start

        while not self.finish_up:

            startTime = time.perf_counter()

            self.report_updateCounts += 1
            if (startTime - self.report_lastTimeRate)>= 1.:
                self.report_rate = copy(self.report_updateCounts)
                self.report_lastTimeRate = time.perf_counter()
                self.report_updateCounts = 0

            # Display the Data
            msg_out = '\033[2J\n'
            msg_out+= '-------------------------------------------------\n'
            if self.args.report > 0:
                msg_out+= 'icm 20x: Moving:{}, Mag:{}\n'.format(
                                                    'Y' if self.moving else 'N',
                                                    'Y' if self.magok  else 'N')

            msg_out+= '-------------------------------------------------\n'

            if self.args.report > 0:
                msg_out+= 'Data    {:>10.6f}, {:>3d}/s\n'.format(self.data_deltaTime*1000.,        self.data_rate)
                msg_out+= 'i2c polls until data {:d}\n'.format(self.i2cHits)
                msg_out+= 'Report  {:>10.6f}, {:>3d}/s\n'.format(self.report_deltaTime*1000.,      self.report_rate)
                if self.args.fusion:
                    msg_out+= 'Fusion  {:>10.6f}, {:>3d}/s\n'.format(self.fusion_deltaTime*1000.,  self.fusion_rate)
                if self.args.zmqport is not None:
                    msg_out+= 'ZMQ     {:>10.6f}, {:>3d}/s\n'.format(self.zmq_deltaTime*1000.,     self.zmq_rate)

                msg_out+= '-------------------------------------------------\n'

            if self.args.report > 1:
                msg_out+= 'Time  {:>10.6f}, dt {:>10.6f}\n'.format(self.sensorTime, self.delta_sensorTime)

                msg_out+= 'Accel     {:>8.3f} {:>8.3f} {:>8.3f} N:  {:>8.3f}\n'.format(self.acc.x,self.acc.y,self.acc.z,self.acc.norm)
                msg_out+= 'Gyro      {:>8.3f} {:>8.3f} {:>8.3f} N:  {:>8.3f}\n'.format(self.gyr.x,self.gyr.y,self.gyr.z,self.gyr.norm)
                msg_out+= 'Magno     {:>8.3f} {:>8.3f} {:>8.3f} N:  {:>8.3f}\n'.format(self.mag.x,self.mag.y,self.mag.z,self.mag.norm)
                msg_out+= 'Accel avg {:>8.3f} {:>8.3f} {:>8.3f} N:  {:>8.3f}\n'.format(self.acc_average.x, self.acc_average.y, self.acc_average.z, self.acc_average.norm)
                msg_out+= 'Gyro  avg {:>8.3f} {:>8.3f} {:>8.3f} RPM:{:>8.3f}\n'.format(self.gyr_average.x, self.gyr_average.y, self.gyr_average.z, self.gyr_average.norm*60./TWOPI)
                msg_out+= 'Gyro bias {:>8.3f} {:>8.3f} {:>8.3f} RPM:{:>8.3f}\n'.format(self.gyr_offset.x, self.gyr_offset.y, self.gyr_offset.z, self.gyr_offset.norm*60./TWOPI)

                msg_out+= '-------------------------------------------------\n'

                if self.args.fusion:

                    msg_out+= 'Acc     {:>8.3f} {:>8.3f} {:>8.3f} N:  {:>8.3f}\n'.format(self.acc_cal.x,self.acc_cal.y,self.acc_cal.z,self.acc_cal.norm)
                    msg_out+= 'Gyr     {:>8.3f} {:>8.3f} {:>8.3f} RPM:{:>8.3f}\n'.format(self.gyr_cal.x,self.gyr_cal.y,self.gyr_cal.z,self.gyr_cal.norm*60./TWOPI)
                    msg_out+= 'Mag     {:>8.3f} {:>8.3f} {:>8.3f} N:  {:>8.3f}\n'.format(self.mag_cal.x,self.mag_cal.y,self.mag_cal.z,self.mag_cal.norm)
                    msg_out+= 'Euler: R{:>6.1f} P{:>6.1f} Y{:>6.1f}, Heading {:>4.0f}\n'.format(
                                                    self.rpy.x*RAD2DEG, self.rpy.y*RAD2DEG, self.rpy.z*RAD2DEG, 
                                                    self.heading*RAD2DEG)
                    msg_out+= 'Q:     W{:>6.3f} X{:>6.3f} Y{:>6.3f} Z{:>6.3f}\n'.format(
                                                    self.q.w, self.q.x, self.q.y, self.q.z)

                if self.args.motion:
                    msg_out+= '-------------------------------------------------\n'

                    msg_out+= 'Residual {:>8.3f} {:>8.3f} {:>8.3f} N:  {:>8.3f}\n'.format(self.residuals.x,self.residuals.y,self.residuals.z,self.residuals.norm)
                    msg_out+= 'Vel      {:>8.3f} {:>8.3f} {:>8.3f} N:  {:>8.3f}\n'.format(self.velocity.x,self.velocity.y,self.velocity.z,self.velocity.norm)
                    msg_out+= 'Pos      {:>8.3f} {:>8.3f} {:>8.3f} N:  {:>8.3f}\n'.format(self.position.x,self.position.y,self.position.z,self.position.norm)
                    msg_out+= 'Vel Bias {:>8.3f} {:>8.3f} {:>8.3f} N:  {:>8.3f}\n'.format(self.velocityBias.x,self.velocityBias.y,self.velocityBias.z,self.velocityBias.norm)
                    msg_out+= 'Acc Bias {:>8.3f} {:>8.3f} {:>8.3f} N:  {:>8.3f}\n'.format(self.accBias.x,self.accBias.y,self.accBias.z,self.accBias.norm)
                    msg_out+= 'dt       {:>10.6f} s {:>10.6f} ms\n'.format(self.dtmotion, self.dt*1000.)

            print(msg_out, flush=True)

            self.report_deltaTime = time.perf_counter() - startTime

            # Wait to next interval time
            sleepTime = self.report_updateInterval - (time.perf_counter() - startTime)
            await asyncio.sleep(max(0.,sleepTime))
            timingError = time.perf_counter() - startTime - self.report_updateInterval
            self.report_updateInterval = max(0., REPORTINTERVAL - timingError)

        self.logger.log(logging.INFO, 'Reporting stopped')

    async def update_zmq(self):
        '''
        Report data on ZMQ socket
        There are 4 data packets presented:
         - system 
         - imu
         - fusion if enabled
         - motion if enabled
        '''

        self.logger.log(logging.INFO, 'Creating ZMQ Publisher at \'tcp://*:{}\' ...'.format(self.args.zmqport))

        context = zmq.asyncio.Context()
        socket = context.socket(zmq.PUB)
        socket.bind("tcp://*:{}".format(self.args.zmqport))

        data_system  = icmSystemData()
        data_imu     = icmIMUData()
        data_fusion  = icmFusionData()
        data_motion  = icmMotionData()

        self.zmq_lastTimeRate   = time.perf_counter()
        self.zmq_updateCounts   = 0

        while not self.finish_up:

            startTime = time.perf_counter()

            await self.processedDataAvailable.wait()
            self.processedDataAvailable.clear()

            # fps
            self.zmq_updateCounts += 1
            if (startTime - self.zmq_lastTimeRate)>= 1.:
                self.zmq_rate = copy(self.zmq_updateCounts)
                self.zmq_lastTimeRate = copy(startTime)
                self.zmq_updateCounts = 0

            # format the imu data
            data_imu.time   = self.sensorTime
            data_imu.acc    = self.acc
            data_imu.gyr    = self.gyr
            data_imu.mag    = self.mag
            data_imu.moving = self.moving
            data_imu.magok  = self.magok
            imu_msgpack     = msgpack.packb(obj2dict(vars(data_imu)))
            socket.send_multipart([b"imu", imu_msgpack])

            # format the system data
            data_system.data_rate      = self.data_rate
            data_system.fusion_rate    = self.fusion_rate
            data_system.zmq_rate       = self.zmq_rate
            data_system.reporting_rate = self.report_rate
            system_msgpack = msgpack.packb(obj2dict(vars(data_system)))
            socket.send_multipart([b"system", system_msgpack])

            if self.args.fusion:

                # report fusion data
                data_fusion.time = self.sensorTime
                data_fusion.acc  = self.acc_cal
                data_fusion.gyr  = self.gyr_cal
                data_fusion.mag  = self.mag_cal
                data_fusion.rpy  = self.rpy
                data_fusion.heading = self.heading
                data_fusion.q    = self.q
                fusion_msgpack   = msgpack.packb(obj2dict(vars(data_fusion)))
                socket.send_multipart([b"fusion", fusion_msgpack])

            if self.args.motion:
                data_motion.time      = self.sensorTime
                data_motion.accBias   = self.accBias
                data_motion.velocityBias = self.velocityBias
                data_motion.position  = self.position
                data_motion.velocity  = self.velocity
                data_motion.residuals = self.residuals
                data_motion.dtmotion  = self.dtmotion
                motion_msgpack = msgpack.packb(obj2dict(vars(data_motion)))
                socket.send_multipart([b"motion", motion_msgpack])

            # update interval
            self.zmq_deltaTime = time.perf_counter() - startTime

            await asyncio.sleep(0)

        self.logger.log(logging.INFO, 'ZMQ stopped')

    async def handle_termination(self, tasks:None):
        '''
        Cancel slow tasks based on provided list (speed up closing for program)
        '''
        self.logger.log(logging.INFO, 'Control-C or Kill signal detected')
        self.finish_up = True
        if tasks is not None: # This will terminate tasks faster
            self.logger.log(logging.INFO, 'Cancelling all Tasks...')
            await asyncio.sleep(1) # give some time for tasks to finish up
            for task in tasks:
                if task is not None:
                    task.cancel()

    async def update_terminator(self, tasks):
        '''
        Wrapper for Task Termination
        Waits for termination signal and then executes the termination sequence
        '''
        self.logger.log(logging.INFO, 'Starting Terminator...')

        while not self.finish_up:

            await self.terminate.wait()
            self.terminate.clear()
            await self.handle_termination(tasks=tasks)

        self.logger.log(logging.INFO, 'Terminator completed')

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

    tasks = [imu_task]    # frequently used tasks
    terminator_tasks = [] # slow tasks, long wait times

    if args.fusion:
        gyroffset_task  = asyncio.create_task(imu.update_gyrOffset())
        tasks.append(gyroffset_task)
        terminator_tasks.append(gyroffset_task)

    if args.report > 0:
        reporting_task  = asyncio.create_task(imu.update_report())   # report new data, will not terminate
        tasks.append(reporting_task)

    if args.zmqport is not None:
        zmq_task     = asyncio.create_task(imu.update_zmq())         # update zmq, will not terminate
        tasks.append(zmq_task)

    terminator_task = asyncio.create_task(imu.update_terminator(terminator_tasks)) # make sure we shutdown in timely fashion
    tasks.append(terminator_task)

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

    parser.add_argument(
        '-f',
        '--fusion',
        dest = 'fusion',
        action='store_true',
        help='turns on IMU data fusion',
        default = False
    )

    parser.add_argument(
        '-m',
        '--motion',
        dest = 'motion',
        action='store_true',
        help='turns on velocity and position computation',
        default = False
    )

    parser.add_argument(
        '-z',
        '--zmq',
        dest = 'zmqport',
        type = int,
        metavar='<zmqport>',
        help='port used by ZMQ, e.g. 5556',
        default =  None
    )

    args = parser.parse_args()

    if args.motion: args.fusion=True

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