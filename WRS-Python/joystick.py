#!/usr/bin/env python
import pygame
import time
import board
import busio
import math
import adafruit_bno055
import piplates.MOTORplate as MOTOR  
from gpiozero import DigitalInputDevice
import configparser
import numpy as np


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
        #print("A:{} B:{}".format(self.stateA, self.stateB))
        #print(self.ticks)
        
    def reset():
        ticks = 0
        
    def Rpm(self):
        self.ticksNow = self.ticks - self.ticksLast
        self.timeNow = time.time() - self.timeLast
        
        rpm = (self.ticksNow / 1137.0) / ((self.timeNow) / 60.0)
        
        self.timeLast = time.time()
        self.ticksLast = self.ticks
        
        return abs(rpm)
        
MOTOR.dcCONFIG(0,1,'ccw',0,0)
MOTOR.dcCONFIG(0,2,'ccw',0,0)

leftEncoder = Encoder(26,19)
rightEncoder = Encoder(21,20)


i2c = busio.I2C(board.SCL, board.SDA)
orientationSensor = adafruit_bno055.BNO055_I2C(i2c)


lastLeftEncoder = 0
lastRightEncoder = 0
last_theta = 0
wheelTheta = 0

globalX = 0
globalY = 0
theta_global = 0

#JOYSTICK
pygame.init()

#screen = pygame.display.set_mode((500, 700))

#pygame.display.set_caption("My Game")


MOTOR.dcSTART(0,1) #right
MOTOR.dcSTART(0,2) #left

targetVel = 30
WHEEL_BASE = 16.480523


try:
    while True:
    
        joystick = pygame.joystick.Joystick(0)
        joystick.init()
        
        movementAxis = -joystick.get_axis(4)

        buttonleft = joystick.get_button(6)
        buttonRight = joystick.get_button(7)
        for event in pygame.event.get(): # User did something.
            if event.type == pygame.QUIT: # If user clicked close.
                done = True # Flag that we are done so we exit this loop.
            #elif event.type == pygame.JOYBUTTONDOWN:
                #print("Joystick button pressed.")
            #elif event.type == pygame.JOYBUTTONUP:
                #print("Joystick button released.")
        leftChange = leftEncoder.ticks - lastLeftEncoder
        rightChange = rightEncoder.ticks - lastRightEncoder
        
        leftChangeCm = leftChange / 40.21315
        rightChangeCm = rightChange / 40.21315
        
        distance = (leftChangeCm + rightChangeCm) / 2
        wheelTheta += (leftChangeCm - rightChangeCm) / WHEEL_BASE;
        
        test = orientationSensor.euler[0]
        if(type(test) == type(1.0)):
            theta_global = math.radians(test)
            imu_theta = theta_global - last_theta
            last_theta = theta_global
        
        
        globalX += distance * math.sin(theta_global)
        globalY += distance * math.cos(theta_global)
        
        MOTOR.dcSPEED(0,1,movementAxis * targetVel + buttonRight * 20)
        MOTOR.dcSPEED(0,2,movementAxis * targetVel + buttonleft * 20)
        
        #print("X: " + str(globalX) + " Y: " + str(globalY) + " Theta: " + str(math.degrees(theta_global)))
        print(" Wheel: " + str(math.degrees(wheelTheta)) + " IMU: " + str(math.degrees(theta_global)))
        
        lastLeftEncoder = leftEncoder.ticks;
        lastRightEncoder = rightEncoder.ticks;
            
        time.sleep(0.01)
        
except KeyboardInterrupt:
    MOTOR.dcSTOP(0,1)
    MOTOR.dcSTOP(0,2)