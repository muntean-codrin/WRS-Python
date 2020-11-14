from globaldata import *
from osensor import *
from qencoder import *

class Odometry:
    leftTicks = 0
    rightTicks = 0
    lastLeftEncoder = 0
    lastRightEncoder = 0
    
    globalX = 0
    globalY = 0
    
    def __init__(self, orientationSensor, leftEncoder, rightEncoder):
        self.orientationSensor = orientationSensor
        self.leftEncoder = leftEncoder
        self.rightEncoder = rightEncoder
    
    def getPosition(self):
        self.lastLeftEncoder = self.leftTicks
        self.lastRightEncoder = self.rightTicks
        
        self.leftTicks = self.leftEncoder.ticks
        self.rightTicks = self.rightEncoder.ticks
        
        leftChange = self.leftTicks - self.lastLeftEncoder
        rightChange = self.rightTicks - self.lastRightEncoder
        
        leftChangeCm = leftChange * CM_PER_TICK
        rightChangeCm = rightChange * CM_PER_TICK
        
        distance = (leftChangeCm + rightChangeCm) / 2
        
        self.yaw = self.orientationSensor.getYaw2()
            
        yawRadians = math.radians(self.yaw)
        self.globalX += distance * math.sin(yawRadians)
        self.globalY += distance * math.cos(yawRadians)
        
        return (self.globalX, self.globalY, self.yaw)