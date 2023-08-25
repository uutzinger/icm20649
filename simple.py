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
import signal
import msgpack
import time

# Activate uvloop to speed up asyncio
if os.name == 'nt':
    from asyncio.windows_events import WindowsSelectorEventLoopPolicy
    asyncio.set_event_loop_policy(WindowsSelectorEventLoopPolicy())
else:
    import uvloop
    asyncio.set_event_loop_policy(uvloop.EventLoopPolicy())

#########################################################################################################
# Simple
#########################################################################################################

class zmqWorkerSimple:

    def __init__(self, logger, zmqPort: int = 5551):

        self.logger                     = logger
        self.zmqPort                    = zmqPort

        self.finish_up                  = False
        self.fusion = False
        self.motion = False
        self.report = 0

        self.startTime                  = 0.

        self.terminate = asyncio.Event()
        self.logger.log(logging.INFO, 'Simple zmqWorker initialized')

    async def start(self):
        '''
        Simulate program control
        - fusion
        - motion
        - report
        - stop
        '''

        context = zmq.asyncio.Context()
        socket = context.socket(zmq.REP)
        socket.bind("tcp://*:{}".format(self.zmqPort))

        poller = zmq.asyncio.Poller()
        poller.register(socket, zmq.POLLIN)

        self.logger.log(logging.INFO, 'Creating ZMQ Reply at \'tcp://*:{}\' ...'.format(self.zmqPort))

        while not self.finish_up:

            startTime = time.perf_counter()

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
                            self.report = int.from_bytes(value, byteorder='big', signed=True)
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
                        self.logger.log(logging.ERROR, 'zmqWorker received malformed REQ message')
                        socket.send_string("ERROR")

            except:
                self.logger.log(logging.ERROR, 'zmqWorker REQ/REP error')
                poller.unregister(socket)
                socket.close()
                socket = context.socket(zmq.REP)
                socket.bind("tcp://*:{}".format(self.zmqPort))
                poller.register(socket, zmq.POLLIN)

            # update interval
            self.zmqREP_deltaTime = time.perf_counter() - startTime

            await asyncio.sleep(0)

        self.logger.log(logging.INFO, 'Simple ZMQ worker finished')
        socket.close()
        context.term()

async def handle_termination(zmq, logger, tasks):
    '''
    Cancel slow tasks based on provided list (speed up closing of program)
    '''
    logger.log(logging.INFO, 'Controller ESC, Control-C or Kill signal detected')
    if tasks is not None: # This will terminate tasks faster
        logger.log(logging.INFO, 'Cancelling all Tasks...')
        zmq.finish_up = True
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
    logger.log(logging.INFO, 'Starting Simple ...')

    zmq = zmqWorkerSimple(logger=logger, zmqPort=args.zmqport)

    zmq_task = asyncio.create_task(zmq.start())
    tasks = [zmq_task]

    # Set up a Control-C handler to gracefully stop the program
    # This mechanism is only available in Unix
    if os.name == 'posix':
        # Get the main event loop
        loop = asyncio.get_running_loop()
        loop.add_signal_handler(signal.SIGINT,  lambda: asyncio.create_task(handle_termination(zmq=zmq, logger=logger, tasks=tasks)) ) # control-c
        loop.add_signal_handler(signal.SIGTERM, lambda: asyncio.create_task(handle_termination(zmq=zmq, logger=logger, tasks=tasks)) ) # kill

    # Wait until all tasks are completed, which is when user wants to terminate the program
    await asyncio.wait(tasks, timeout=float('inf'))

    logger.log(logging.INFO,'Simple ZMQ exit')

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
        '-z',
        '--zmq',
        dest = 'zmqport',
        type = int,
        metavar='<zmqport>',
        help='port used by ZMQ, e.g. 5554 for \'tcp://*:5554\'',
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
