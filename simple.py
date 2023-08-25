#!/usr/bin/python3

################################################################
# ICM 20649 IMU
#
# Urs Utzinger, Summer 2023
################################################################

# IMPORTS
################################################################
import asyncio
import logging
import zmq
import zmq.asyncio
import argparse
import os
import time

# Activate uvloop to speed up asyncio
if os.name != 'nt':
    import uvloop
    asyncio.set_event_loop_policy(uvloop.EventLoopPolicy())

#########################################################################################################
# Simple
#########################################################################################################

class zmqWorker:

    def __init__(self, logger=None, args=None) -> None:

        self.args                       = args
        self.logger                     = logger

        self.zmqport                 = args.zmqport

        self.finish_up                  = False

        # Timing
        ###################
        self.startTime                  = 0.

        self.fusion = False
        self.motion = False
        self.report = 0

        self.terminate = asyncio.Event()


    async def start(self):
        '''
        Handle program control
        - fusion
        - motion
        - report
        - stop
        '''

        self.logger.log(logging.INFO, 'Creating ZMQ Reply at \'tcp://*:{}\' ...'.format(self.zmqport))

        context = zmq.asyncio.Context()
        socket  = context.socket(zmq.REQ)
        socket.bind("tcp://*:{}".format(self.zmqport))

        poller = zmq.asyncio.Poller()
        poller.register(socket, zmq.POLLIN)

        while not self.finish_up:

            startTime = time.perf_counter

            try:
                events = dict(await poller.poll(timeout=-1))
                if socket in events and events[socket] == zmq.POLLIN:
                    response = await socket.recv_multipart()
                    if len(response) == 2:
                        [topic, value] = response
                        if topic == b"motion":
                            if value == b"\x01":
                                self.motion = True
                                self.fusion = True
                            else:
                                self.motion = False
                                self.fusion = False
                            socket.send_string("OK")
                            self.logger.log(logging.INFO, 'ZMQ motion received: {}'.format(value))
                        elif topic == b"fusion":
                            if value == b"\x01": self.fusion = True
                            else:                self.fusion = False
                            socket.send_string("OK")
                            self.logger.log(logging.INFO, 'ZMQ fusion received: {}'.format(value))
                        elif topic == b"report":
                            self.report = int.from_bytes(value)
                            socket.send_string("OK")
                            self.logger.log(logging.INFO, 'ZMQ report received: {}'.format(value))
                        elif topic == b"stop":
                            if value == b"\x01": 
                                self.terminate.set()
                                self.finish_up = True
                            else:          
                                self.terminate.clear()
                                self.finish_up = False
                            socket.send_string("OK")
                            self.logger.log(logging.INFO, 'ZMQ stop received: {}'.format(value))
                        else:
                            socket.send_string("UNKNOWN")
                            self.logger.log(logging.INFO, 'ZMQ received UNKNOWN')
                    else:
                        self.logger.log(logging.ERROR, 'ICM zmqWorker rep malformed message')
                        socket.send_string("ERROR")

            except:
                self.logger.log(logging.ERROR, 'ICM zmqWorker REQ/REP error')
                poller.unregister(socket)
                socket.close()
                socket = context.socket(zmq.REP)
                socket.bind("tcp://*:{}".format(self.zmqport))
                poller.register(socket, zmq.POLLIN)

            # update interval
            self.zmqREP_deltaTime = time.perf_counter() - startTime

            await asyncio.sleep(0)

        self.logger.log(logging.INFO, 'ZMQ REP stopped')
        socket.close()
        context.term()


##############################################################################################
# MAIN
##############################################################################################

async def main(args: argparse.Namespace):

    # Setup logging
    logger = logging.getLogger(__name__)
    logger.log(logging.INFO, 'Starting Simple ...')

    zmq = zmqWorker(logger=logger, args=args)

    tasks = []

    if args.zmqport is not None:
        # listen to external commands
        zmq_task = asyncio.create_task(zmq.start())
        tasks.append(zmq_task)

    # Wait until all tasks are completed, which is when user wants to terminate the program
    await asyncio.wait(tasks, timeout=float('inf'))

    logger.log(logging.INFO,'Simple exit')

if __name__ == '__main__':

    parser = argparse.ArgumentParser(description="Simple")

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
        type = int,
        metavar='<zmqportREP>',
        help='port used by ZMQ, e.g. 5551',
        default = 5551
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
