# icm20649

## icm20649.py
async client with fusion and motion processing for ICM 20649. This client can be run as system service.
```
icm20649.py -f -m -r0
```

## test_icm20649.py
test program connecting to client via ZMQ, enabling fusion, motion and displaying data.
- zp is port where data is published
- zr is port where server listens to external commands

```
test_icm20649.py -zp 'tcp://localhost:5556' -zr 'tcp://localhost:5555'
```

## Requirements
- adafruit_icm20x with modification to provide data ready status and unnecessary sleep statement removed: https://github.com/uutzinger/Adafruit_CircuitPython_ICM20X
- asyncio, zmq, numpy, json, msgpack

Urs Utzinger, Summer 2023