#!/usr/bin/python3

################################################################
# ICM 20649 IMU Display Test Program
#
# Urs Utzinger, Summer 2023
################################################################

# IMPORTS
################################################################
import math
import logging
import argparse
import time
import asyncio
import os
import zmq
import zmq.asyncio
import msgpack

from pyIMU.quaternion import Vector3D, Quaternion

# Activate uvloop to speed up asyncio
if os.name == 'nt':
    from asyncio.windows_events import WindowsSelectorEventLoopPolicy
    asyncio.set_event_loop_policy(WindowsSelectorEventLoopPolicy())
else:
    import uvloop
    asyncio.set_event_loop_policy(uvloop.EventLoopPolicy())

# CONSTANTS
RAD2DEG = 180.0 / math.pi
DEG2RAD = math.pi / 180.0
TWOPI   = 2.0*math.pi

report_updateInterval = 0.1

ZMQTIMEOUT = 10000  # milli second, 10 secs

async def handle_termination(zmq, logger, tasks):
    '''
    Cancel slow tasks based on provided list (speed up closing for program)
    Not applicable on Windows
    '''
    logger.log(logging.INFO, 'Control-C or kill signal detected')
    if tasks is not None:
        logger.log(logging.INFO, 'Cancelling all tasks ...')
        zmq.finish_up = True
        await asyncio.sleep(1) # give some time for tasks to finish up
        for task in tasks:  # force task cancel if it still running
            if task is not None:
                task.cancel()

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

class icmSystemData(object):
    '''System relevant performance data'''
    def __init__(self,  
                 data_rate:      int = 0, 
                 fusion_rate:    int = 0, 
                 zmq_rate:       int = 0, 
                 reporting_rate: int = 0) -> None:
        self.data_rate       = data_rate
        self.fusion_rate     = fusion_rate
        self.zmq_rate        = zmq_rate
        self.reporting_rate  = reporting_rate

class icmIMUData(object):
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

##############################################################################################
# ZMQ Worker 
# listens to PUB from server
##############################################################################################

class zmqWorkerICM():

    def __init__(self, logger, zmqPortPUB='tcp://localhost:5556'):

        self.dataReady =  asyncio.Event()
        self.finished  =  asyncio.Event()
        self.dataReady.clear()
        self.finished.clear()

        self.logger     = logger
        self.zmqPortPUB = zmqPortPUB

        self.new_system = False
        self.new_imu    = False
        self.new_fusion = False
        self.new_motion = False
        self.timeout    = False

        self.finish_up  = False
        self.paused     = False

        self.zmqTimeout = ZMQTIMEOUT

        self.logger.log(logging.INFO, 'IC20x zmqWorker initialized')

    async def start(self):

        self.new_system = False
        self.new_imu    = False
        self.new_fusion = False
        self.new_motion = False

        context = zmq.asyncio.Context()
        poller  = zmq.asyncio.Poller()
        
        self.data_system = icmSystemData()
        self.data_imu    = icmIMUData()
        self.data_motion = icmMotionData()
        self.data_fusion = icmFusionData()

        socket = context.socket(zmq.SUB)
        socket.setsockopt(zmq.SUBSCRIBE, b"system")
        socket.setsockopt(zmq.SUBSCRIBE, b"imu")
        socket.setsockopt(zmq.SUBSCRIBE, b"fusion")
        socket.setsockopt(zmq.SUBSCRIBE, b"motion")
        socket.connect(self.zmqPortPUB)
        poller.register(socket, zmq.POLLIN)

        self.logger.log(logging.INFO, 'IC20x  zmqWorker started on {}'.format(self.zmqPortPUB))

        while not self.finish_up:
            try:
                events = dict(await poller.poll(timeout=self.zmqTimeout))
                if socket in events and events[socket] == zmq.POLLIN:
                    response = await socket.recv_multipart()
                    if len(response) == 2:
                        [topic, msg_packed] = response
                        if topic == b"system":
                            msg_dict = msgpack.unpackb(msg_packed)
                            self.data_system = dict2obj(msg_dict)
                            self.new_system = True
                            # self.logger.log(logging.INFO, 'IC20x zmqWorker received system data')
                        elif topic == b"imu":
                            msg_dict = msgpack.unpackb(msg_packed)
                            self.data_imu = dict2obj(msg_dict)
                            self.new_imu = True                            
                            # self.logger.log(logging.INFO, 'IC20x zmqWorker received imu data')
                        elif topic == b"fusion":
                            msg_dict = msgpack.unpackb(msg_packed)
                            self.data_fusion = dict2obj(msg_dict)
                            self.new_fusion = True
                            # self.logger.log(logging.INFO, 'IC20x zmqWorker received fusion data')
                        elif topic == b"motion":
                            msg_dict = msgpack.unpackb(msg_packed)
                            self.data_motion = dict2obj(msg_dict)
                            self.new_motion = True
                            # self.logger.log(logging.INFO, 'IC20x zmqWorker received motion data')
                        else:
                            pass
                            # self.logger.log(logging.INFO, 'IC20x zmqWorker topic {} not of interest. Dict: {}'.format(topic, msg_dict))
                    else:
                        self.logger.log(logging.ERROR, 'IC20x zmqWorker malformed message')
                else:  # ZMQ TIMEOUT
                    self.logger.log(logging.ERROR, 'IC20x zmqWorker timed out')
                    poller.unregister(socket)
                    socket.close()
                    socket = context.socket(zmq.SUB)
                    socket.setsockopt(zmq.SUBSCRIBE, b"system")
                    socket.setsockopt(zmq.SUBSCRIBE, b"imu")
                    socket.setsockopt(zmq.SUBSCRIBE, b"fusion")
                    socket.setsockopt(zmq.SUBSCRIBE, b"motion")
                    socket.connect(self.zmqPortPUB)
                    poller.register(socket, zmq.POLLIN)
                    self.new_system = \
                    self.new_imu    = \
                    self.new_fusion = \
                    self.new_motion = False


                if (self.new_imu and self.new_system):
                    if self.data_system.fusion:
                        if self.new_fusion:
                            if self.data_system.motion:
                                if self.new_motion:
                                    if not self.paused: self.dataReady.set()
                                    # self.logger.log(logging.INFO, 'IC20x zmqWorker imu, system, fusion and motion ready')
                                    self.new_system  = \
                                    self.new_imu     = \
                                    self.new_fusion  = \
                                    self.new_motion  = False
                                else:
                                    pass # just wait until we have motion data
                            else:
                                if not self.paused: self.dataReady.set()
                                # self.logger.log(logging.INFO, 'IC20x zmqWorker imu, system and fusion ready')
                                self.new_system  = \
                                self.new_imu     = \
                                self.new_fusion  = False
                        else:
                            pass # just wait until we have fusion data
                    else: 
                        if not self.paused: self.dataReady.set()
                        self.new_system  = \
                        self.new_imu     = False
                        # self.logger.log(logging.INFO, 'IC20x zmqWorker imu and system ready')
                else:
                    pass # we need imu and system data, lets wiat for both
                    # self.logger.log(logging.ERROR, 'IC20x zmqWorker no new data')

            except:
                self.logger.log(logging.ERROR, 'IC20x zmqWorker error')
                poller.unregister(socket)
                socket.close()
                socket = context.socket(zmq.SUB)
                socket.setsockopt(zmq.SUBSCRIBE, b"system")
                socket.setsockopt(zmq.SUBSCRIBE, b"imu")
                socket.setsockopt(zmq.SUBSCRIBE, b"fusion")
                socket.setsockopt(zmq.SUBSCRIBE, b"motion")
                socket.connect(self.zmqPortPUB)
                poller.register(socket, zmq.POLLIN)
                self.new_system = \
                self.new_imu    = \
                self.new_fusion = \
                self.new_motion = False

            await asyncio.sleep(0) # allow other asyncio tasks to run

        self.logger.log(logging.DEBUG, 'IC20x zmqWorker finished')
        socket.close()
        context.term()
        self.finished.set()

    def set_zmqPortPUB(self, port):
        self.zmqPortPUB = port

##############################################################################################
# MAIN
##############################################################################################
def norm(v: Vector3D) -> float:
    return math.sqrt(v.x*v.x+v.y*v.y+v.z*v.z)
    
async def main(args: argparse.Namespace):

    # Setup logging
    logger = logging.getLogger(__name__)

    logger.log(logging.INFO, 'Testing ICM sensor')

    #########################################################
    # Change the settings of the data publisher
    #########################################################

    logger.log(logging.INFO, 'Connecting to port {}'.format(args.zmqportREP))

    context = zmq.Context()
    socket = context.socket(zmq.REQ)
    socket.connect(args.zmqportREP)
    socket.setsockopt(zmq.RCVTIMEO, 10000)

    logger.log(logging.INFO, 'Turning on Fusion, Displaying Data')

    communicationError = False
    # Turn on fusion
    try:
        socket.send_multipart([b"fusion", b"\x01"])
        response = socket.recv_string()
        logger.log(logging.INFO, 'Fusion on response: ' + response)
    except:
        logger.log(logging.ERROR, 'Fusion no response')
        communicationError = True

    # Turn on motion
    try:
        socket.send_multipart([b"motion", b"\x01"])
        response = socket.recv_string()
        logger.log(logging.INFO, 'Motion on response: ' + response)
    except:
        logger.log(logging.ERROR, 'Motion no response')
        communicationError = True

    # Turn off reporting
    try:
        socket.send_multipart([b"report", b"\x00"])
        response = socket.recv_string()
        logger.log(logging.INFO, 'Reporting level 0 response: ' + response)
    except:
        logger.log(logging.ERROR, 'Reporting no response')
        communicationError = True

    #########################################################
    # Listen to data publisher and display data
    #########################################################

    if not communicationError:

        # Lisen to sensors data server/publisher
        zmqWorker =  zmqWorkerICM(logger=logger, zmqPortPUB=args.zmqPortPUB)
        imu_task = asyncio.create_task(zmqWorker.start())
        tasks = [imu_task]

        # Set up a Control-C handler to gracefully stop the program
        if os.name == 'posix':
            # Get the main event loop
            loop = asyncio.get_running_loop()
            loop.add_signal_handler(signal.SIGINT,  lambda: asyncio.create_task(imu.handle_termination(zmq=zmqWorker, logger=logger, tasks=tasks)) ) # control-c
            loop.add_signal_handler(signal.SIGTERM, lambda: asyncio.create_task(imu.handle_termination(zmq=zmqWorker, logger=logger, tasks=tasks)) ) # kill

        while True:
            reportTime = time.perf_counter()

            await zmqWorker.dataReady.wait()
            zmqWorker.dataReady.clear()
            
            # Display the Data
            msg_out = '\033[2J\n'
            msg_out+= '-------------------------------------------------\n'
            msg_out+= 'icm 20x: Moving:{}, Mag:{}\n'.format(
                                                'Y' if zmqWorker.data_imu.moving else 'N',
                                                'Y' if zmqWorker.data_imu.magok  else 'N')
            msg_out+= '-------------------------------------------------\n'
            msg_out+= 'Data    {:>3d}/s\n'.format(zmqWorker.data_system.data_rate)
            msg_out+= 'Report  {:>3d}/s\n'.format(zmqWorker.data_system.reporting_rate)
            msg_out+= 'Fusion  {:>3d}/s\n'.format(zmqWorker.data_system.fusion_rate)
            msg_out+= 'ZMQ     {:>3d}/s\n'.format(zmqWorker.data_system.zmq_rate)
            msg_out+= '-------------------------------------------------\n'
            msg_out+= 'Time  {:>10.6f}\n'.format(zmqWorker.data_imu.time)
            msg_out+= 'Accel   {:>8.3f} {:>8.3f} {:>8.3f} N:  {:>8.3f}\n'.format(zmqWorker.data_imu.acc.x,zmqWorker.data_imu.acc.y,zmqWorker.data_imu.acc.z,norm(zmqWorker.data_imu.acc))
            msg_out+= 'Gyro    {:>8.3f} {:>8.3f} {:>8.3f} N:  {:>8.3f}\n'.format(zmqWorker.data_imu.gyr.x,zmqWorker.data_imu.gyr.y,zmqWorker.data_imu.gyr.z,norm(zmqWorker.data_imu.gyr))
            msg_out+= 'Mag     {:>8.3f} {:>8.3f} {:>8.3f} N:  {:>8.3f}\n'.format(zmqWorker.data_imu.mag.x,zmqWorker.data_imu.mag.y,zmqWorker.data_imu.mag.z,norm(zmqWorker.data_imu.mag))
            msg_out+= '-------------------------------------------------\n'
            msg_out+= 'Acc     {:>8.3f} {:>8.3f} {:>8.3f} N:  {:>8.3f}\n'.format(zmqWorker.data_fusion.acc.x,zmqWorker.data_fusion.acc.y,zmqWorker.data_fusion.acc.z,norm(zmqWorker.data_fusion.acc))
            msg_out+= 'Gyr     {:>8.3f} {:>8.3f} {:>8.3f} RPM:{:>8.3f}\n'.format(zmqWorker.data_fusion.gyr.x,zmqWorker.data_fusion.gyr.y,zmqWorker.data_fusion.gyr.z,norm(zmqWorker.data_fusion.gyr)*60./TWOPI)
            msg_out+= 'Mag     {:>8.3f} {:>8.3f} {:>8.3f} N:  {:>8.3f}\n'.format(zmqWorker.data_fusion.mag.x,zmqWorker.data_fusion.mag.y,zmqWorker.data_fusion.mag.z,norm(zmqWorker.data_fusion.mag))
            msg_out+= 'Euler: R{:>6.1f} P{:>6.1f} Y{:>6.1f}, Heading {:>4.0f}\n'.format(
                                            zmqWorker.data_fusion.rpy.x*RAD2DEG, zmqWorker.data_fusion.rpy.y*RAD2DEG, zmqWorker.data_fusion.rpy.z*RAD2DEG, 
                                            zmqWorker.data_fusion.heading*RAD2DEG)
            msg_out+= 'Q:     W{:>6.3f} X{:>6.3f} Y{:>6.3f} Z{:>6.3f}\n'.format(
                                            zmqWorker.data_fusion.q.w, zmqWorker.data_fusion.q.x, zmqWorker.data_fusion.q.y, zmqWorker.data_fusion.q.z)
            msg_out+= '-------------------------------------------------\n'

            msg_out+= 'Residual {:>8.3f} {:>8.3f} {:>8.3f} N:  {:>8.3f}\n'.format(zmqWorker.data_motion.residuals.x,zmqWorker.data_motion.residuals.y,zmqWorker.data_motion.residuals.z,norm(zmqWorker.data_motion.residuals))
            msg_out+= 'Vel      {:>8.3f} {:>8.3f} {:>8.3f} N:  {:>8.3f}\n'.format(zmqWorker.data_motion.velocity.x,zmqWorker.data_motion.velocity.y,zmqWorker.data_motion.velocity.z,norm(zmqWorker.data_motion.velocity))
            msg_out+= 'Pos      {:>8.3f} {:>8.3f} {:>8.3f} N:  {:>8.3f}\n'.format(zmqWorker.data_motion.position.x,zmqWorker.data_motion.position.y,zmqWorker.data_motion.position.z,norm(zmqWorker.data_motion.position))
            msg_out+= 'Vel Bias {:>8.3f} {:>8.3f} {:>8.3f} N:  {:>8.3f}\n'.format(zmqWorker.data_motion.velocityBias.x,zmqWorker.data_motion.velocityBias.y,zmqWorker.data_motion.velocityBias.z,norm(zmqWorker.data_motion.velocityBias))
            msg_out+= 'Acc Bias {:>8.3f} {:>8.3f} {:>8.3f} N:  {:>8.3f}\n'.format(zmqWorker.data_motion.accBias.x,zmqWorker.data_motion.accBias.y,zmqWorker.data_motion.accBias.z,norm(zmqWorker.data_motion.accBias))
            msg_out+= 'dt       {:>10.6f} s\n'.format(zmqWorker.data_motion.dtmotion)

            print(msg_out, flush=True)

            sleepTime = report_updateInterval - (time.perf_counter() - reportTime)
            time.sleep(max(0.,sleepTime))


    logger.log(logging.INFO, 'Reporting stopped')


if __name__ == '__main__':

    parser = argparse.ArgumentParser()

    parser.add_argument(
        '-d',
        '--debug',
        action='store_true',
        help='sets the log level from info to debug',
        default = False
    )

    parser.add_argument(
        '-zp',
        '--zmq_pub',
        dest = 'zmqPortPUB',
        type = str,
        metavar='<zmqPortPUB>',
        help='port used by ZMQ, e.g. \'tcp://10.0.0.2:5556\'',
        default = 'tcp://localhost:5556'
    )

    parser.add_argument(
        '-zr',
        '--zmq_rep',
        dest = 'zmqportREP',
        type = str,
        metavar='<zmqportREP>',
        help='port used by ZMQ, e.g. \'tcp://10.0.0.2:5555\'',
        default = 'tcp://localhost:5555'
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
