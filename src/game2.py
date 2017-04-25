#!/usr/bin/env python

import cv2
import numpy as np
import sys
import argparse
import roslib; roslib.load_manifest('reha_game')
import rospy
import rospkg
from std_msgs.msg import String
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

class EncouragerUnit(object):
	repetitions = 0
	repetitions_limit = 0
	pub = None
	sentences = []
	ENC_SENT_FILELOC = "/config/encouragement_sentences.txt"

	def __init__(self, repetitions_limit):
		self.repetitions_limit = repetitions_limit
		rospy.init_node('reha_game')
		self.pub = rospy.Publisher('/robot/voice', String)
		self.load_sentences()

	def load_sentences(self):
		# retrieve path to reha_game ROS package and load sentences file
		rospack = rospkg.RosPack()
		reha_game_pkg_path = rospack.get_path('reha_game')
		sentences_file = open(reha_game_pkg_path + self.ENC_SENT_FILELOC, "r")
		self.sentences = sentences_file.readlines()
		sentences_file.close()

	def incRepCounter(self):
		self.repetitions += 1
		self.pub.publish(str(self.repetitions))
		if self.repetitions == self.repetitions_limit/2:
			self.pub.publish(self.sentences[0])
		elif self.repetitions == self.repetitions_limit-1:
			self.pub.publish(self.sentences[1])
		elif self.repetitions == self.repetitions_limit:
			self.pub.publish(self.sentences[2])







# ************************************************************************************
# *********************************** MAIN PROGRAM ***********************************
# ************************************************************************************
if __name__ == '__main__':
	# set trivial variables and parse arguments
	CAMERA_WIDTH = 640
	CAMERA_HEIGHT = 480
	PATH_TO_VIDEO = ""
	#PATH_TO_SAMPLES_FOLDER = ""
	PATH_TO_OUTPUT_VIDEO = ""
	COLOR = "yellow"
	print rospy.myargv()[1:]
	print sys.argv

	# argument parsing
	parser = argparse.ArgumentParser(description='Reads video footage from a camera or a video file, and performs the Rehazenter exercise on it.')
	parser.add_argument('--width', type=int, dest="width", help='width of the camera window')
	parser.add_argument('--height', type=int, dest="height", help='height of the camera window')
	parser.add_argument('--path-to-video', '-p', type=str, dest="path_to_video", help='file path to the video file (default: webcam)')
	#parser.add_argument('--path-to-samples-folder', '-f', type=str, dest="arg_path_to_samples_folder", help='path to the folder where the object\'s image samples are stored')
	parser.add_argument('--path-to-output-video', '-o', type=str, dest="path_to_output_video", help='path to the output video that contains the modified input video with the detected object contours')
	parser.add_argument('--color', '-c', type=str, dest="color", help='the color of the object that should be tracked (default=yellow)')
	# use the "rospy.myargv" argument vector instead of the built-in "sys.argv" to avoid problems with ROS argument re-mapping
	# (reason: https://groups.google.com/a/rethinkrobotics.com/forum/#!topic/brr-users/ErXVWhRmtNA)
	args = parser.parse_args(rospy.myargv()[1:])

	# checking color argument
	if args.color != "yellow" or args.color != "blue" or args.color != "black" or args.color:
		print "Unrecognized color! Setting color back to default (yellow)."
	else:
		COLOR = args.color

	# checking camera dimensions (TODO: check if correct!)
	CAMERA_WIDTH = args.width
	CAMERA_WIDTH = args.height
	
	# TODO: add remaining checks for path params

	# default thresholds (yellow)
	# **************************************************
	# argument parsing (the ROS way)
	#if rospy.has_param(self._Cascade_Classifier):
	#      cascPath = rospy.get_param(self._Cascade_Classifier)
	#    else:
	#      rospy.logwarn("CAscade parameters need to be set to start recognizer.")
	#      return


	# set HSV color thresholds
	if COLOR == "blue":
		# HSV color thresholds for BLUE
		THRESHOLD_LOW = np.array([110,50,50], dtype=np.uint8)
		THRESHOLD_HIGH = np.array([130,255,255], dtype=np.uint8)
	elif COLOR == "black":
		# HSV color thresholds for BLACK (not recommended for usage due to huge amounts of noise)
		THRESHOLD_LOW = np.array([60,15,0], dtype=np.uint8)
		THRESHOLD_HIGH = np.array([105,170,110], dtype=np.uint8)
	else:
		# HSV color thresholds for YELLOW (default)
		THRESHOLD_LOW = np.array([15,210,20], dtype=np.uint8)
		THRESHOLD_HIGH = np.array([35,255,255], dtype=np.uint8)
	print "Using color " + COLOR + "."

	# TODO: get color automatically from images, if argument given
	#if args.path_to_samples_folder != None:
	#	if os.path.exists(args.path_to_samples_folder) == False :
	#		print "The specified path to the samples folder does not exist! Aborting..."
	#		sys.exit()
	#	# loop through image files
	#	for img_fn in os.listdir(args.path_to_samples_folder):
	#		if os.path.isfile(img_fn) and img_fn.lower().endswith(('.png', '.jpg', '.jpeg')):
	#			# TODO: go through pixels in image and store max/min thresholds of biggest blob
	#			img = cv2.imread(img_fn, cv2.IMREAD_COLOR)
	#	     		print (fn)


	# Minimum required radius of enclosing circle of contour
	MIN_RADIUS = 2

	# define X position thresholds for hand movement
	HAND_MOV_X_THRESHOLD_LEFT = (CAMERA_WIDTH / 4) * 3
	HAND_MOV_X_THRESHOLD_RIGHT = (CAMERA_WIDTH / 4) * 1.55	# multiply with some tolerance value for more impaired people
	# define Y position thresholds for hand movement
	HAND_MOV_Y_THRESHOLD_LEFT = ((CAMERA_HEIGHT / 4) * 3) * 0.75	# multiply with some tolerance value, same as above
	HAND_MOV_Y_THRESHOLD_RIGHT = CAMERA_HEIGHT / 4

	# Initialize camera and get actual resolution
	if PATH_TO_VIDEO == "":
		cap = cv2.VideoCapture(0)
	else:
		cap = cv2.VideoCapture(PATH_TO_VIDEO)

	# check if camera/video file was opened successfully
	if cap.isOpened() == False :
        	print "Failed to open video capture stream! Aborting..."
        	sys.exit()

	cap.set(cv2.CAP_PROP_FRAME_WIDTH, CAMERA_WIDTH)
	cap.set(cv2.CAP_PROP_FRAME_HEIGHT, CAMERA_HEIGHT)

	# define some variables needed for our main loop
	last_moment = 0        
	moment_status = 0  
	last_moment_status = 0
	loop = 0
	has_motivated = False

	# variable that tracks direction in which patient should currently move his hand
	hand_is_moving_right = True

	# Define the codec and create VideoWriter object
	fourcc = cv2.VideoWriter_fourcc(*'mp4v') # Be sure to use the lower case
	if PATH_TO_OUTPUT_VIDEO == "":
		out = cv2.VideoWriter('output.mp4v', fourcc, 20.0, (568, 516))
	else:
		out = cv2.VideoWriter(PATH_TO_OUTPUT_VIDEO, fourcc, 20.0, (568, 516))

	# launch ROS communication handler
	encourager = EncouragerUnit(10)
	print "Camera dimensions: " +  str(cv2.CAP_PROP_FRAME_WIDTH) + " x " + str(cv2.CAP_PROP_FRAME_HEIGHT)
	print "Press the \"q\" key to quit."

	# Main loop
	while True:
		# Get image from camera
		rval, img = cap.read()

		VIDEO_WIDTH = cap.get(cv2.CAP_PROP_FRAME_WIDTH)
		VIDEO_HEIGHT = cap.get(cv2.CAP_PROP_FRAME_HEIGHT)

		# Blur image to remove noise
		img_filter = cv2.GaussianBlur(img.copy(), (3, 3), 0)

		# Convert image from BGR to HSV
		img_filter = cv2.cvtColor(img_filter, cv2.COLOR_BGR2HSV)

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
	   
		last_moment_status = moment_status

		# check if hand movement thresholds have been reached and count repetitions accordingly 
		if center != None :
			if hand_is_moving_right == True and center[0] < HAND_MOV_X_THRESHOLD_RIGHT and center[1] < HAND_MOV_Y_THRESHOLD_RIGHT :
				hand_is_moving_right = False
				encourager.incRepCounter()
				print "Current number of repetitions done: " + str(encourager.repetitions)
			elif hand_is_moving_right == False and center[0] > HAND_MOV_X_THRESHOLD_LEFT and center[1] > HAND_MOV_Y_THRESHOLD_LEFT :
				hand_is_moving_right = True

		#print "Current center position of glove: ", center
	    
		# Print out the location and size (radius) of the largest detected contour
		#if center != None:
		#print str(center) + " Center - Radius:" + str(radius)   

		# Draw a green circle around the largest enclosed contour
		if center != None:
			cv2.circle(img, center, int(round(radius)), np.array([0,255,0]))

		font = cv2.FONT_HERSHEY_SIMPLEX
		cv2.putText(img,str(loop),(10,50), font, 1,(0,0,255),2)
		cv2.putText(img,"Repetitions: " + str(encourager.repetitions),(10,80), font, 1,(0,0,255),2)

		# Show image windows
		cv2.imshow('webcam', img)
		out.write(img)
		cv2.imshow('binary', img_binary)
		cv2.imshow('contours', img_contours)
		if cv2.waitKey(1) & 0xFF == ord('q'):
			break
	out.release()
