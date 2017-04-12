#!/usr/bin/env python

import cv2
import numpy as np

from os.path import expanduser
import os
import shlex, subprocess

#                 |y=-0.1
#        D        |      A
#x=-0.1           |             x=0.1
#------------------------------------
#                 |
#       C         |      B
#                 |y=0.1



# HSV color thresholds for YELLOW
THRESHOLD_LOW = (15, 210, 20);
THRESHOLD_HIGH = (35, 255, 255);

# Webcam parameters (your desired resolution)
CAMERA_WIDTH = 320
CAMERA_HEIGHT = 240

# Minimum required radius of enclosing circle of contour
MIN_RADIUS = 2

# Initialize camera and get actual resolution
cam = cv2.VideoCapture(0)
cam.set(cv2.CAP_PROP_FRAME_WIDTH, CAMERA_WIDTH)
cam.set(cv2.CAP_PROP_FRAME_HEIGHT, CAMERA_HEIGHT)
camWidth = cam.get(cv2.CAP_PROP_FRAME_WIDTH)
camHeight = cam.get(cv2.CAP_PROP_FRAME_HEIGHT)
print str(camWidth)
print  str(camHeight)

# below line expands to user's home directory, should work on most systems
resource=expanduser("~") + "/Downloads/capture1.mp4"
print "Resource path: ", resource
#resource="/home/user/Downloads/IMG_8074.MOV"

cap = cv2.VideoCapture(resource)
if not cap.isOpened():
    print "Error opening resource: " + str(resource)
    print "Maybe opencv VideoCapture can't open it"
    exit(0)

last_moment = 0        
moment_status = 0  
last_moment_status = 0
loop = 0

# Define the codec and create VideoWriter object
fourcc = cv2.VideoWriter_fourcc(*'mp4v') # Be sure to use the lower case
out = cv2.VideoWriter('output.mp4', fourcc, 20.0, (568, 516))

#out = cv2.VideoWriter('./video.avi', -1, 20.0, (568,320))
# Main loop
while True:

    # Get image from camera
    #ret_val, img = cam.read()
    
        
    rval, img = cap.read()
    
    
    VIDEO_WIDTH = cap.get(cv2.CAP_PROP_FRAME_WIDTH)
    VIDEO_HEIGHT = cap.get(cv2.CAP_PROP_FRAME_HEIGHT)

    print VIDEO_WIDTH
    print VIDEO_HEIGHT
    # Blur image to remove noise
    img_filter = cv2.GaussianBlur(img.copy(), (3, 3), 0)

    # Convert image from BGR to HSV
    img_filter = cv2.cvtColor(img_filter, cv2.COLOR_BGR2HSV)
    
    # define range of blue color in HSV
    THRESHOLD_LOW = np.array([110,50,50], dtype=np.uint8)
    THRESHOLD_HIGH = np.array([130,255,255], dtype=np.uint8)
    
    # Set pixels to white if in color range, others to black (binary bitmap)
    img_binary = cv2.inRange(img_filter.copy(), THRESHOLD_LOW, THRESHOLD_HIGH)

    # Dilate image to make white blobs larger
    img_binary = cv2.dilate(img_binary, None, iterations = 1)

    # Find center of object using contours instead of blob detection. From:
    # http://www.pyimagesearch.com/2015/09/14/ball-tracking-with-opencv/
    img_contours = img_binary.copy()
    contours = cv2.findContours(img_contours, cv2.RETR_EXTERNAL, \
        cv2.CHAIN_APPROX_SIMPLE)[-2]

    # Find the largest contour and use it to compute the min enclosing circle
    center = None
    radius = 0
    if len(contours) > 0:
        c = max(contours, key=cv2.contourArea)
        ((x, y), radius) = cv2.minEnclosingCircle(c)
        M = cv2.moments(c)
        if M["m00"] > 0:
            center = (int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"]))
            if radius < MIN_RADIUS:
                center = None

    if center != None:
        moment = int(center[0]/(VIDEO_WIDTH/32))
        #print moment
        if ( moment < last_moment):
            last_moment = moment
            moment_status = -1
        elif ( moment > last_moment):
            last_moment = moment
            moment_status = 1
            
        if ( moment_status != last_moment_status):
            #last_moment_status = moment_status
            print "Ouyahh :" + str(loop)
            loop += 1
            #os.system('rosrun sound_play say.py "yes"')
            #subprocess.call(["rosrun", "sound_play say.py \"yes\""])
            
   
    last_moment_status = moment_status    
    
    
    # Print out the location and size (radius) of the largest detected contour
    #if center != None:
        #print str(center) + " Center - Radius:" + str(radius)   

    # Draw a green circle around the largest enclosed contour
    if center != None:
        cv2.circle(img, center, int(round(radius)),  np.array([0,255,0]))

    font = cv2.FONT_HERSHEY_SIMPLEX
    cv2.putText(img,str(loop),(10,50), font, 1,(0,0,255),2)
    
    # Show image windows
    cv2.imshow('webcam', img)
    out.write(img)
    cv2.imshow('binary', img_binary)
    
    cv2.imshow('contours', img_contours)
    cv2.waitKey(40)
    
out.release()
