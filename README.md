# icm20649
async client with fusion and motion processing for ICM 20649. This client can be run as system service.
test program connecting to client via ZMQ, enabling fusion, motion and displaying data.

## Requirements
- adafruit_icm20x with modification to provide data ready status and unnecessary sleep statement removed: https://github.com/uutzinger/Adafruit_CircuitPython_ICM20X
- asyncio, zmq, numpy, json, msgpack

Urs Utzinger, Summer 2023