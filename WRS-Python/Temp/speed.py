import cv2
import configparser
import math
import sys
import time
import matplotlib.pyplot as plt
import numpy as np

config = configparser.ConfigParser()
config.read("config.ini")

with open(config["PATH"]["FILE_RESULT"]) as file:
    liness = file.readlines()
liness.pop()
result = [([float(x) for x in line.split(",")]) for line in liness]

xpoints = []
ypoints = []

for i in range(len(result)):
    xpoints.append(result[i][3])
    ypoints.append(result[i][2])


plt.plot(xpoints, ypoints)
plt.show()