#!/usr/bin/python3

################################################################
# ICM 20649 IMU
#
# Urs Utzinger, Summer 2023
################################################################

# IMPORTS
#########################################u#######################
import math
import logging
import argparse
import time
import asyncio
import os
import zmq
import zmq.asyncio

from icm20649 import zmqWorkerICM

from pyIMU.quaternion import Vector3D, Quaternion

# Activate uvloop to speed up asyncio
if os.name != 'nt':
    import uvloop
    asyncio.set_event_loop_policy(uvloop.EventLoopPolicy())

# CONSTANTS
RAD2DEG = 180.0 / math.pi
DEG2RAD = math.pi / 180.0
TWOPI   = 2.0*math.pi

report_updateInterval = 0.1

# MAIN
##############################################################################################

if __name__ == '__main__':

    # Setup logging
    logger = logging.getLogger(__name__)

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
        dest = 'zmqportPUB',
        type = str,
        metavar='<zmqportPUB>',
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

    logger.log(logging.INFO, 'Turning on Fusion, Displaying Data')
    logger.log(logging.INFO, 'Connecting to port {}'.format(args.zmqportREP))

    context = zmq.Context()
    socket = context.socket(zmq.REQ)
    socket.connect(args.zmqportREP)

    # socket.setsockopt(zmq.RCVTIMEO, 500)

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

    if not communicationError:
        zmqWorker =  zmqWorkerICM(logger=logger, zmqportPUB=args.zmqportPUB, parent=None)

        imu_task = asyncio.create_task(zmqWorker.start())

        while True:
            reportTime = time.perf_counter()

            zmqWorker.dataReady.wait()
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
            msg_out+= 'Time  {:>10.6f}}\n'.format(zmqWorker.data_imu.time)
            msg_out+= 'Accel   {:>8.3f} {:>8.3f} {:>8.3f} N:  {:>8.3f}\n'.format(zmqWorker.data_imu.acc.x,zmqWorker.data_imu.acc.y,zmqWorker.data_imu.acc.z,zmqWorker.data_imu.acc.norm)
            msg_out+= 'Gyro    {:>8.3f} {:>8.3f} {:>8.3f} N:  {:>8.3f}\n'.format(zmqWorker.data_imu.gyr.x,zmqWorker.data_imu.gyr.y,zmqWorker.data_imu.gyr.z,zmqWorker.data_imu.gyr.norm)
            msg_out+= 'Mag     {:>8.3f} {:>8.3f} {:>8.3f} N:  {:>8.3f}\n'.format(zmqWorker.data_imu.mag.x,zmqWorker.data_imu.mag.y,zmqWorker.data_imu.mag.z,zmqWorker.data_imu.mag.norm)
            msg_out+= '-------------------------------------------------\n'

            msg_out+= 'Acc     {:>8.3f} {:>8.3f} {:>8.3f} N:  {:>8.3f}\n'.format(zmqWorker.data_fusion.acc.x,zmqWorker.data_fusion.acc.y,zmqWorker.data_fusion.acc.z,zmqWorker.data_fusion.acc.norm)
            msg_out+= 'Gyr     {:>8.3f} {:>8.3f} {:>8.3f} RPM:{:>8.3f}\n'.format(zmqWorker.data_fusion.gyr.x,zmqWorker.data_fusion.gyr.y,zmqWorker.data_fusion.gyr.z,zmqWorker.data_fusion.gyr.norm*60./TWOPI)
            msg_out+= 'Mag     {:>8.3f} {:>8.3f} {:>8.3f} N:  {:>8.3f}\n'.format(zmqWorker.data_fusion.mag.x,zmqWorker.data_fusion.mag.y,zmqWorker.data_fusion.mag.z,zmqWorker.data_fusion.mag.norm)
            msg_out+= 'Euler: R{:>6.1f} P{:>6.1f} Y{:>6.1f}, Heading {:>4.0f}\n'.format(
                                            zmqWorker.data_fusion.rpy.x*RAD2DEG, zmqWorker.data_fusion.rpy.y*RAD2DEG, zmqWorker.data_fusion.rpy.z*RAD2DEG, 
                                            zmqWorker.data_fusion.heading*RAD2DEG)
            msg_out+= 'Q:     W{:>6.3f} X{:>6.3f} Y{:>6.3f} Z{:>6.3f}\n'.format(
                                            zmqWorker.data_fusion.q.w, zmqWorker.data_fusion.q.x, zmqWorker.data_fusion.q.y, zmqWorker.data_fusion.q.z)
            msg_out+= '-------------------------------------------------\n'
            msg_out+= 'Residual {:>8.3f} {:>8.3f} {:>8.3f} N:  {:>8.3f}\n'.format(zmqWorker.data_motion.residuals.x,zmqWorker.data_motion.residuals.y,zmqWorker.data_motion.residuals.z,zmqWorker.data_motion.residuals.norm)
            msg_out+= 'Vel      {:>8.3f} {:>8.3f} {:>8.3f} N:  {:>8.3f}\n'.format(zmqWorker.data_motion.velocity.x,zmqWorker.data_motion.velocity.y,zmqWorker.data_motion.velocity.z,zmqWorker.data_motion.velocity.norm)
            msg_out+= 'Pos      {:>8.3f} {:>8.3f} {:>8.3f} N:  {:>8.3f}\n'.format(zmqWorker.data_motion.position.x,zmqWorker.data_motion.position.y,zmqWorker.data_motion.position.z,zmqWorker.data_motion.position.norm)
            msg_out+= 'Vel Bias {:>8.3f} {:>8.3f} {:>8.3f} N:  {:>8.3f}\n'.format(zmqWorker.data_motion.velocityBias.x,zmqWorker.data_motion.velocityBias.y,zmqWorker.data_motion.velocityBias.z,zmqWorker.data_motion.velocityBias.norm)
            msg_out+= 'Acc Bias {:>8.3f} {:>8.3f} {:>8.3f} N:  {:>8.3f}\n'.format(zmqWorker.data_motion.accBias.x,zmqWorker.data_motion.accBias.y,zmqWorker.data_motion.accBias.z,zmqWorker.data_motion.accBias.norm)
            msg_out+= 'dt       {:>10.6f} s\n'.format(zmqWorker.data_motion.dtmotion)

            print(msg_out, flush=True)


            sleepTime = report_updateInterval - (time.perf_counter()-reportTime)
            time.sleep(max(0.,sleepTime))


    logger.log(logging.INFO, 'Reporting stopped')
