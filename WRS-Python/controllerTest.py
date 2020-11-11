#!/usr/bin/env python
import time
import board
import busio
import math
import adafruit_bno055
import piplates.MOTORplate as MOTOR  
from gpiozero import DigitalInputDevice
import configparser
import numpy as np


i2c = busio.I2C(board.SCL, board.SDA)
orientationSensor = adafruit_bno055.BNO055_I2C(i2c)

MOTOR.dcCONFIG(0,1,'ccw',0,0)
MOTOR.dcCONFIG(0,2,'ccw',0,0)

WHEEL_BASE = 25.6


class Encoder:
    ticks = 0
    
    result = [0, 1, -1, 0, -1, 0, 0, 1, 1, 0, 0, -1, 0, -1, 1, 0]
    pAB = 0b00
    
    stateA = 0
    stateB = 0
    
    ticksNow = 0
    ticksLast = 0
    
    timeLast = 0
    timeNow = 0
    
    
    def __init__(self, pinA, pinB):

        self.pinA = "GPIO" + str(pinA)
        self.pinB = "GPIO" + str(pinB)
        
        self.encoderA = DigitalInputDevice(pinA)
        self.encoderB = DigitalInputDevice(pinB)
        
        self.encoderA.when_activated = lambda:self.changed(self.encoderA)
        self.encoderA.when_deactivated = lambda:self.changed(self.encoderA)
        
        self.encoderB.when_activated = lambda:self.changed(self.encoderB)
        self.encoderB.when_deactivated = lambda:self.changed(self.encoderB)
        
        
    def changed(self, encoder):
        if(str(encoder.pin) == self.pinA):
            self.stateA = int(encoder.is_active)
        elif(str(encoder.pin) == self.pinB):
            self.stateB = int(encoder.is_active)
        
        cAB = (self.stateA << 1) | self.stateB
        pos = (self.pAB << 2) | cAB
        self.ticks += self.result[pos]
        self.pAB=cAB
        
    def reset():
        ticks = 0
        
    def Rpm(self):
        self.ticksNow = self.ticks - self.ticksLast
        self.timeNow = time.time() - self.timeLast
        
        rpm = (self.ticksNow / 1137.0) / ((self.timeNow) / 60.0)
        
        self.timeLast = time.time()
        self.ticksLast = self.ticks
        
        return abs(rpm)
        


leftEncoder = Encoder(26,19)
rightEncoder = Encoder(20,21)

FF = 0
FBr = 0
FBl = 0

Kv = 90.0/150.0
Ka = 0
Kp = 0.05

lastLeftEncoder = 0
lastRightEncoder = 0

globalX = 0
globalY = 0
wheelTheta = 0
theta_global = 0


MOTOR.dcSTART(0,1) #right
MOTOR.dcSTART(0,2) #left

targetVel = 30

try:
    while True:
        leftChange = leftEncoder.ticks - lastLeftEncoder
        rightChange = rightEncoder.ticks - lastRightEncoder
        
        leftChangeCm = leftChange / 40.21315
        rightChangeCm = rightChange / 40.21315
        
        distance = (leftChangeCm + rightChangeCm) / 2
        #wheelTheta += (leftChangeCm - rightChangeCm) / WHEEL_BASE;
        #theta_global = orientationSensor.euler[0]
        #if(type(test) == type(1.0)):
        #   theta_global = math.radians(test)
        
        globalX += distance * math.sin(theta_global)
        globalY += distance * math.cos(theta_global)
        
        lastLeftEncoder = leftEncoder.ticks;
        lastRightEncoder = rightEncoder.ticks;
        
        #print("left: " + str(leftEncoder.ticks) + " right: " + str(rightEncoder.ticks))
        #print("X: " + str(globalX) + " Y: " + str(globalY) + " Theta: " + str(math.degrees(theta_global)))
        print(orientationSensor.euler[0])
        time.sleep(0.02)
except KeyboardInterrupt:
    MOTOR.dcSTOP(0,1)
    MOTOR.dcSTOP(0,2)