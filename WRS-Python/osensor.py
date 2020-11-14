import busio
import board
import adafruit_bno055
import numpy as np
import math

class OSensor:
    
    lastYaw = 0
    yaw = 0
    def __init__(self, SCL, SDA):
        self.i2c = busio.I2C(SCL, SDA)
        self.sensor = adafruit_bno055.BNO055_I2C(self.i2c)
        
    def getYaw(self):
        try:
            q = self.sensor.quaternion
            if(type(q[0]) == type(1.0)):
                self.yaw = self.quaternionToYaw(q[0], q[1], q[2], q[3])
                if(self.yaw < 0):
                    self.yaw+=360
                diff = abs(self.lastYaw - self.yaw)
                if(diff > 20 and diff < 340):
                    self.yaw = self.lastYaw
            
            self.lastYaw = self.yaw
            return self.yaw
        except KeyboardInterrupt:
            raise KeyboardInterrupt()
        except:
            print("Skipped imu")
            return self.lastYaw
            
    def getYaw2(self):
        test = self.sensor.euler[0]
        if(type(test) == type(1.0)):
            self.yaw = test
            self.last_theta = self.yaw
        return self.yaw
            
    def quaternionToYaw(self,w,x,y,z):
        t3 = +2.0 * (w * z + x * y)
        t4 = +1.0 - 2.0 * (y * y + z * z)
        Z = np.degrees(np.arctan2(t3, t4))
        
        return Z