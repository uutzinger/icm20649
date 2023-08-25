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
import zmq

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
        '-z',
        '--zmq',
        dest = 'zmqport',
        type = str,
        metavar='<zmqport>',
        help='port used by ZMQ, e.g. \'tcp://10.0.0.2:5555\'',
        default = 'tcp://localhost:5551'
    )

    args = parser.parse_args()

    log_level = logging.DEBUG if args.debug else logging.INFO
    logging.basicConfig(
        level=log_level,
        # format='%(asctime)-15s %(name)-8s %(levelname)s: %(message)s'
        format='%(asctime)-15s %(levelname)s: %(message)s'
    )

    logger.log(logging.INFO, 'Turning on Fusion, Displaying Data')
    logger.log(logging.INFO, 'Connecting to port {}'.format(args.zmqport))

    context = zmq.Context()
    socket = context.socket(zmq.REQ)
    socket.connect(args.zmqport)

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

    logger.log(logging.INFO, 'Reporting stopped')
