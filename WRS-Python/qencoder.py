from gpiozero import DigitalInputDevice
import time

MOTOR_CPR = 1120.0

class QuadratureEncoder:
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
        
        rpm = (self.ticksNow / MOTOR_CPR) / ((self.timeNow) / 60.0)
        
        self.timeLast = time.time()
        self.ticksLast = self.ticks
        
        return abs(rpm)