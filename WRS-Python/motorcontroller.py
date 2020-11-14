import time

class MotorController:
    prevTargetValue = 0
    prevTime = 0
    ffSpeed = 0
    ffAcceleration = 0
    
    FB = 0
    prev_error = 0
    sum_error = 0
    
    def __init__(self, Kv, Ka, KP, KD, KI):
        self.Kv = Kv
        self.Ka = Ka
        self.KP = KP
        self.KD = KD
        self.KI = KI
        
        prevTime = time.time()
        
        
    def calculate(self, feedbackValue, targetValue):
        timeNow = time.time()
        
        #feedforward
        self.ffSpeed = targetValue * self.Kv
        self.ffAcceleration = (targetValue - self.prevTargetValue) / (timeNow - self.prevTime) * self.Ka
        FF = self.ffSpeed + self.ffAcceleration
        
        #feedbackward
        error = targetValue - feedbackValue
        self.FB += (error * self.KP) + (self.prev_error * self.KD) + (self.sum_error * self.KI)
        
        returnValue = max(min(100, FF + self.FB), 0)
        
        self.prev_error = error
        self.sum_error += error
        self.prevTime = timeNow
        
        return returnValue