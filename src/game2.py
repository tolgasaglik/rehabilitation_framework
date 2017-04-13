#!/usr/bin/env python

import cv2
import numpy as np
import sys
import argparse

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

# argument parsing
parser = argparse.ArgumentParser(description='Reads video footage from a camera or a video file, and performs the Rehazenter exercise on it.')
parser.add_argument('--width', type=int, help='width of the camera window')
parser.add_argument('--height', type=int, help='height of the camera window')
parser.add_argument('--path-to-video', '-p', type=str, help='file path to the video file (default: webcam)')
parser.add_argument('--path-to-object-folder', '-f', type=str, help='path to the folder where the object\'s image samples are stored')
parser.add_argument('--path-to-output-video', '-o', type=str, help='path to the output video that contains the modified input video with the detected object contours')
args = parser.parse_args()


# HSV color thresholds for YELLOW
#THRESHOLD_LOW = (15, 210, 20);
#THRESHOLD_HIGH = (35, 255, 255);

# Webcam parameters (your desired resolution)
CAMERA_WIDTH = 320
CAMERA_HEIGHT = 240
if args.width != None:
	CAMERA_WIDTH = args.width
if args.height != None:
	CAMERA_HEIGHT = args.height

# Minimum required radius of enclosing circle of contour
MIN_RADIUS = 2

# define X position thresholds for hand movement
HAND_MOV_THRESHOLD_LEFT = CAMERA_WIDTH / 4
HAND_MOV_THRESHOLD_RIGHT = HAND_MOV_THRESHOLD_LEFT * 3

# Initialize camera and get actual resolution
if args.path_to_video == None:
	cap = cv2.VideoCapture(0)
else:
	cap = cv2.VideoCapture(args.path_to_video)

# check if camera/video file was opened successfully
if cap.isOpened() == False :
        print "Failed to open video capture stream! Aborting..."
        sys.exit()

cap.set(cv2.CAP_PROP_FRAME_WIDTH, CAMERA_WIDTH)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, CAMERA_HEIGHT)
camWidth = cap.get(cv2.CAP_PROP_FRAME_WIDTH)
camHeight = cap.get(cv2.CAP_PROP_FRAME_HEIGHT)
#print str(camWidth)
#print str(camHeight)

# define some trivial variables for our main loop
last_moment = 0        
moment_status = 0  
last_moment_status = 0
loop = 0

# variable that tracks direction in which patient should currently move his hand
hand_is_moving_right = True

# stores number of exercise repetitions done so far
repetitions = 0

# Define the codec and create VideoWriter object
fourcc = cv2.VideoWriter_fourcc(*'mp4v') # Be sure to use the lower case
out = cv2.VideoWriter('output.mp4', fourcc, 20.0, (568, 516))

print "Camera dimensions: " +  str(VIDEO_WIDTH) + " x " + str(VIDEO_HEIGHT)

#out = cv2.VideoWriter('./video.avi', -1, 20.0, (568,320))
# Main loop
while True:

    # Get image from camera
    #ret_val, img = cam.read()
    
        
    rval, img = cap.read()
    
    
    VIDEO_WIDTH = cap.get(cv2.CAP_PROP_FRAME_WIDTH)
    VIDEO_HEIGHT = cap.get(cv2.CAP_PROP_FRAME_HEIGHT)

    # Blur image to remove noise
    img_filter = cv2.GaussianBlur(img.copy(), (3, 3), 0)

    # Convert image from BGR to HSV
    img_filter = cv2.cvtColor(img_filter, cv2.COLOR_BGR2HSV)
    
    # define range of blue color in HSV
    #THRESHOLD_LOW = np.array([110,50,50], dtype=np.uint8)
    #THRESHOLD_HIGH = np.array([130,255,255], dtype=np.uint8)

    # range of black/grayish color in HSV (not recommended for usage due to huge amounts of noise)
    # (TODO: try to come up with a way to get the object and its color dynamically from some images in a folder f.ex., given as a parameter)
    THRESHOLD_LOW = np.array([60,15,0], dtype=np.uint8)
    THRESHOLD_HIGH = np.array([105,170,110], dtype=np.uint8)
    
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
            #print "Ouyahh :" + str(loop)
            loop += 1
            #os.system('rosrun sound_play say.py "yes"')
            #subprocess.call(["rosrun", "sound_play say.py \"yes\""])
            
   
    last_moment_status = moment_status    
    
    # check if hand movement thresholds have been reached and count repetitions accordingly 
    if center != None :
	if hand_is_moving_right == True and center[0] > HAND_MOV_THRESHOLD_RIGHT :
		hand_is_moving_right = False
		repetitions += 1
		print "Current number of repetitions done: " + str(repetitions)
    	elif hand_is_moving_right == False and center[0] < HAND_MOV_THRESHOLD_LEFT :
		hand_is_moving_right = True

    #print "Current center position of glove: ", center
    
    # Print out the location and size (radius) of the largest detected contour
    #if center != None:
        #print str(center) + " Center - Radius:" + str(radius)   

    # Draw a green circle around the largest enclosed contour
    if center != None:
        cv2.circle(img, center, int(round(radius)),  np.array([0,255,0]))

    font = cv2.FONT_HERSHEY_SIMPLEX
    cv2.putText(img,str(loop),(10,50), font, 1,(0,0,255),2)
    cv2.putText(img,"Repetitions: " + str(repetitions),(10,80), font, 1,(0,0,255),2)
    
    # Show image windows
    cv2.imshow('webcam', img)
    out.write(img)
    cv2.imshow('binary', img_binary)
    
    cv2.imshow('contours', img_contours)
    cv2.waitKey(40)
    
out.release()
