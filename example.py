#!/usr/bin/python3
from cemu_motionserver import MotionServer, Controller
import time

server = MotionServer(26760, True)

server.start()

time.sleep(10)
with server as controllerList:
    print("add controllers")
    controllerList.append(Controller(1, 0, 0, [0x34,0x56,0x78,0x9a,0xbc,0xdf], 250))
    controllerList.append(Controller(2, 0, 0, [0x34,0x56,0x78,0x9a,0xbc,0xde], 250))
    controllerList[0].gyroP = 120.4
    controllerList[0].gyroY = 220.4
    controllerList[0].gyroR = 90.0
    controllerList[0].accelY = -14.4
    print("controllers added")

time.sleep(10)
server.stop()
