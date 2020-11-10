import cv2
import configparser
import math
import sys
import time


path = []
result = []
start_pos = (0,0)

# READ CONFIG FILE
config = configparser.ConfigParser()
config.read("config.ini")
scaler = float(config["FIELD_IMAGE"]["PIXELS_PER_UNIT"])

# OPEN CV
img = cv2.imread(config["FIELD_IMAGE"]["FILE_LOCATION"])
cv2.imshow("Field", img)
with open(config["PATH"]["FILE_RESULT"]) as file:
    liness = file.readlines()
liness.pop()

# READ 
with open(config["PATH"]["FILE_LOCATION"]) as file:
    path = [([float(x) for x in line.split(",")]) for line in file.readlines()]

result = [([float(x) for x in line.split(",")]) for line in liness]
with open(config["PATH"]["FILE_STARTPOINT"]) as file:
    start_pos = [int(x) for x in file.readlines()[0].split(",")]
print(start_pos)

#width of application / highest time value
timeAxisScaler = (img.shape[1] / result[-1][3])

#height of application / max velocity
speedAxisScaler = img.shape[0] / int(config["VELOCITY"]["MAX_VEL"])

#spread the speed values on the whole application width
inputSpeedScaler = img.shape[1] / len(path)

#input speed
for i in range(len(path)):
    x = int(i * inputSpeedScaler * (8/10) + (1/10) * img.shape[1])
    y = img.shape[0] - int(path[i][2] * speedAxisScaler)
    #print (x , y)
    cv2.circle(img, (x, y), 5, (0, 0, 255), -1)

cv2.imshow("Field", img) 
cv2.waitKey()

    
#output speed
for i in range(len(result)):
    x = int(result[i][3] * timeAxisScaler * (8/10) + (1/10) * img.shape[1])  #time
    y = img.shape[0] - int(result[i][2] * speedAxisScaler) #speed
    #print (x , y)
    cv2.circle(img, (x, y), 3, (255, 0, 0), -1)

cv2.imshow("Field", img)
cv2.waitKey()



