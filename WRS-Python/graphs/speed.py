import cv2
import configparser
import math
import sys
import time
import matplotlib.pyplot as plt
import numpy as np

config = configparser.ConfigParser()
config.read("config.ini")

with open(config["FILES"]["SPEED"]) as file:
    liness = file.readlines()
liness.pop()
result = [([float(x) for x in line.split(",")]) for line in liness]

xpoints = []
ypoints = []

for i in range(len(result)):
    xpoints.append(result[i][0])
    ypoints.append(result[i][1])

plt.plot(xpoints, ypoints, ".-b")

xpoints = []
ypoints = []

for i in range(len(result)):
    xpoints.append(result[i][0])
    ypoints.append(result[i][2])

plt.plot(xpoints, ypoints, ".-g")

xpoints = np.array([result[1][0], result[-1][0]])
ypoints = np.array([50, 50])


plt.plot(xpoints, ypoints, ".-r")


plt.show()