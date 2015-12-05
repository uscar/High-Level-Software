import numpy as np
import cv2
import cv2.cv as cv
import scipy as sp
from matplotlib import pyplot as plt
import time
import math
from operator import itemgetter, attrgetter, methodcaller
from numpy.linalg import inv

#sensor data
height = 1
fieldOfView = 5*math.pi/18

#creating hsv mask for original image
lower_white = np.array([0,0,50])
lower_white2 = np.array([0,0,100])
higher_white = np.array([179,50,255])

#loading the image
cap = cv2.VideoCapture(0)
lower_green = (55,245,245)
upper_green = (65,255,255)

index = 0
while(True):
    # Capture frame-by-frame
    ret, frame = cap.read()

    

