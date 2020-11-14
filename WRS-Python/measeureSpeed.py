#!/usr/bin/env python
import math
import board
import piplates.MOTORplate as MOTOR  
import traceback
import configparser
import numpy as np

from qencoder import *
from motorcontroller import *
from osensor import *
from odometry import *
from globaldata import *

config = configparser.ConfigParser()
config.read("config.ini")
 
MOTOR.dcCONFIG(0,1,'ccw',0,0)
MOTOR.dcCONFIG(0,2,'ccw',0,0)

leftEncoder = QuadratureEncoder(26,19)
rightEncoder = QuadratureEncoder(20,21)

leftController = MotorController(95/150, 0.0005, 0.05, 0, 0)
rightController = MotorController(95/150, 0.0005, 0.05, 0, 0)

orientationSensor = OSensor(board.SCL, board.SDA)

odometry = Odometry(orientationSensor, leftEncoder, rightEncoder)

MOTOR.dcSTART(0,1) #right
MOTOR.dcSTART(0,2) #left

targetVel = 100

timeStart = time.time()
loopExecuted = 0

leftSpeed = 0
rightSpeed = 0


speedFile = open(config["FILES"]["SPEED"], "w+")

try:
    while True:
        loopExecuted+=1

        
        globalPosition = odometry.getPosition()

        lRpm = leftEncoder.Rpm(odometry.leftTicks, odometry.lastLeftEncoder)
        rRpm = rightEncoder.Rpm(odometry.rightTicks, odometry.lastRightEncoder)
        
        actualtargetLeft = targetVel
        actualtargetRight = targetVel
            
        
        leftSpeed = leftController.calculate(lRpm, actualtargetLeft)
        rightSpeed = rightController.calculate(rRpm, actualtargetRight)
        
        MOTOR.dcSPEED(0,1,rightSpeed)
        MOTOR.dcSPEED(0,2,leftSpeed)

        
        print('X: {0:.2f} Y: {1:.2f} Yaw: {2:.2f}'.format(globalPosition[0], globalPosition[1], globalPosition[2]))
        speedFile.write("{0:.3f},{1:.2f},{2:.2f}\n".format(time.time()-timeStart, lRpm, rRpm))
        time.sleep(0.03)
except:
    traceback.print_exc()
    print('Executions/sec: {0:.2f}'.format(loopExecuted/(time.time()-timeStart)))
    time.sleep(0.05)
    MOTOR.dcSTOP(0,1)
    MOTOR.dcSTOP(0,2)