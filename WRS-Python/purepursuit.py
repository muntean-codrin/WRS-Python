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

MOTOR.dcCONFIG(0,1,'ccw',0,0)
MOTOR.dcCONFIG(0,2,'ccw',0,0)

leftEncoder = QuadratureEncoder(26,19)
rightEncoder = QuadratureEncoder(20,21)

leftController = MotorController(90.0/150.0, 0, 0.05, 0, 0)
rightController = MotorController(90.0/150.0, 0, 0.05, 0, 0)

orientationSensor = OSensor(board.SCL, board.SDA)

odometry = Odometry(orientationSensor, leftEncoder, rightEncoder)

MOTOR.dcSTART(0,1) #right
MOTOR.dcSTART(0,2) #left

config = configparser.ConfigParser()
config.read("config.ini")

with open(config["PATH"]["FILE_LOCATION"]) as file:
    path = [([float(x) for x in line.split(",")]) for line in file.readlines()]
width = float(config["ROBOT"]["TRACKWIDTH"])
pos = (0,0)
angle = 0
t = 0
t_i = 0
wheels = [0,0]


timeStart = time.time()
loopExecuted = 0
result = open(config["PATH"]["FILE_RESULT"], "w+")

try:
    while closest() != len(path)-1:
        loopExecuted+=1
        
        look = lookahead()
        close = closest()
        curv = curvature(look) if t_i>close else 0.00001
        vel = path[close][2]
        wheels = turn(curv, vel, width)
        
        globalPosition = odometry.getPosition()
        pos = (globalPosition[0], globalPosition[1])
        angle = math.radians(globalPosition[2])

        lRpm = leftEncoder.Rpm(odometry.leftTicks, odometry.lastLeftEncoder)
        rRpm = rightEncoder.Rpm(odometry.rightTicks, odometry.lastRightEncoder)
        
        leftSpeed = leftController.calculate(lRpm, wheels[0])
        rightSpeed = rightController.calculate(rRpm, wheels[1])
        
        MOTOR.dcSPEED(0,1,rightSpeed)
        MOTOR.dcSPEED(0,2,leftSpeed)

        result.write(str(pos[0]) + "," + str(pos[1])+ "," + str((lRpm + rRpm) / 2) + "," + str(time.time() - timeStart) +" \n")

        
        print('X: {0:.2f} Y: {1:.2f} Yaw: {2:.2f}'.format(globalPosition[0], globalPosition[1], globalPosition[2]))

        time.sleep(0.03)
    MOTOR.dcSTOP(0,1)
    MOTOR.dcSTOP(0,2)
except:
    traceback.print_exc()
    print('Executions/sec: {0:.2f}'.format(loopExecuted/(time.time()-timeStart)))
    time.sleep(0.05)
    MOTOR.dcSTOP(0,1)
    MOTOR.dcSTOP(0,2)