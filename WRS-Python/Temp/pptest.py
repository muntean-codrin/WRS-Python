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

WHEEL_BASE = 25.6

def closest():
    global path, pos

    mindist = (0, math.sqrt((path[0][0] - pos[0]) ** 2 + (path[0][1] - pos[1]) ** 2))
    for i, p in enumerate(path):
        dist = math.sqrt((p[0]-pos[0])**2 + (p[1]-pos[1])**2)
        if dist < mindist[1]:
            mindist = (i, dist)

    return mindist[0]
def lookahead():
    global path, t, t_i, pos

    for i, p in enumerate(reversed(path[:-1])):
        i_ = len(path)-2 - i
        d = (path[i_+1][0]-p[0], path[i_+1][1]-p[1])
        f = (p[0]-pos[0], p[1]-pos[1])

        a = sum(j**2 for j in d)
        b = 2*sum(j*k for j,k in zip(d,f))
        c = sum(j**2 for j in f) - float(config["PATH"]["LOOKAHEAD"])**2
        disc = b**2 - 4*a*c
        if disc >= 0:
            disc = math.sqrt(disc)
            t1 = (-b + disc)/(2*a)
            t2 = (-b - disc)/(2*a)
            # print("t1=" + str(t1) + ", t2=" + str(t2))
            if 0<=t1<=1:
                # if (t1 >= t and i == t_i) or i > t_i:
                    t = t1
                    t_i = i_
                    # print("hit")
                    return p[0]+t*d[0], p[1]+t*d[1]
            if 0<=t2<=1:
                # if (t2 >= t and i == t_i) or i > t_i:
                    t = t2
                    t_i = i_
                    # print("hit")
                    return p[0]+t*d[0], p[1]+t*d[1]
    t = 0
    t_i = 0
    return path[closest()][0:2]
def curvature(lookahead):
    global path, pos, angle
    side = np.sign(math.sin(3.1415/2 - angle)*(lookahead[0]-pos[0]) - math.cos(3.1415/2 - angle)*(lookahead[1]-pos[1]))
    a = -math.tan(3.1415/2 - angle)
    c = math.tan(3.1415/2 - angle)*pos[0] - pos[1]
    # x = abs(-math.tan(3.1415/2 - angle) * lookahead[0] + lookahead[1] + math.tan(3.1415/2 - angle)*pos[0] - pos[1]) / math.sqrt((math.tan(3.1415/2 - angle))**2 + 1)
    x = abs(a*lookahead[0] + lookahead[1] + c) / math.sqrt(a**2 + 1)
    return side * (2*x/(float(config["PATH"]["LOOKAHEAD"])**2))
def turn(curv, vel, trackwidth):
    return  [vel*(2+curv*trackwidth)/2, vel*(2-curv*trackwidth)/2]

class Encoder:
    ticks = 0
    
    result = [0, 1, -1, 0, -1, 0, 0, 1, 1, 0, 0, -1, 0, -1, 1, 0]
    pAB = 0b00
    
    stateA = 0
    stateB = 0
    
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
        
    def Rpm(self, ticksNow):
        self.timeNow = time.time() - self.timeLast
        
        rpm = (ticksNow / 1120.0) / ((self.timeNow) / 60.0)
        
        self.timeLast = time.time()
        
        return abs(rpm)
        
MOTOR.dcCONFIG(0,1,'ccw',0,0)
MOTOR.dcCONFIG(0,2,'ccw',0,0)

leftEncoder = Encoder(26,19)
rightEncoder = Encoder(20,21)

counts = 5

lastLeftEncoder = 0
lastRightEncoder = 0
last_theta = 0
wheelTheta = 0

globalX = 0
globalY = 0


FF = 0
FBr = 0
FBl = 0

Kv = 90.0/150.0
Ka = 0
Kp = 0.05

MOTOR.dcSTART(0,1) #right
MOTOR.dcSTART(0,2) #left

targetVel = 30

#PURE PURSUIT
config = configparser.ConfigParser()
config.read("config.ini")

with open(config["PATH"]["FILE_LOCATION"]) as file:
    path = [([float(x) for x in line.split(",")]) for line in file.readlines()]

start_pos = (0, 0)
with open(config["PATH"]["FILE_STARTPOINT"]) as file:
    start_pos = [int(x) for x in file.readlines()[0].split(",")]

scaler = float(config["FIELD_IMAGE"]["PIXELS_PER_UNIT"])
width = float(config["ROBOT"]["TRACKWIDTH"])
length = float(config["ROBOT"]["LENGTH"])

pos = (0,0)
angle = 0
t = 0
t_i = 0
wheels = [0,0]

result = []
indice = 0
with open(config["PATH"]["FILE_RESULT"], "w+") as file:
    try:
        initialTime = time.perf_counter()
        while closest() != len(path)-1:

            look = lookahead()
            close = closest()
            curv = curvature(look) if t_i>close else 0.00001
            vel = path[close][2]
            last_wheels = wheels
            wheels = turn(curv, vel, width)
            
            counts = counts - 1
            
            lTicks = leftEncoder.ticks
            rTicks = rightEncoder.ticks
            leftChange = lTicks - lastLeftEncoder
            rightChange = rTicks - lastRightEncoder
            
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
            pos = (globalX, globalY)
            angle = theta_global
            
            
            lastLeftEncoder = lTicks;
            lastRightEncoder = rTicks;
            
            leftVelocity = leftEncoder.Rpm(leftChange)
            rightVelocity = rightEncoder.Rpm(rightChange)
            
            FFl = wheels[0] * Kv + 0 * Ka
            FFr = wheels[1] * Kv + 0 * Ka
            FBl = FBl + Kp * (wheels[0] - leftVelocity)
            FBr = FBr + Kp * (wheels[1] - rightVelocity)
            
            leftSpeed = max(min(FFl + FBl, 100), 0)
            rightSpeed = max(min(FFr + FBr, 100), 0)
            
            MOTOR.dcSPEED(0,1,rightSpeed)
            MOTOR.dcSPEED(0,2,leftSpeed)
            
            file.write(str(pos[0]) + "," + str(pos[1])+ "," + str((leftVelocity + rightVelocity) / 2) + "," + str(time.time() - initialTime) + " \n")

            
            #print("left: " + str(leftVelocity) + " right: " + str(rightVelocity))
            #print("left: " + str(wheels[0]) + " right: " + str(wheels[1]))
            
            print("X: " + str(globalX) + " Y: " + str(globalY) + " Theta: " + str(math.degrees(theta_global)))
            
           
            time.sleep(0.02)
        MOTOR.dcSTOP(0,1)
        MOTOR.dcSTOP(0,2)

    except KeyboardInterrupt:
        MOTOR.dcSTOP(0,1)
        MOTOR.dcSTOP(0,2)

