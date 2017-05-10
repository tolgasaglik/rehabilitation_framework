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
import shlex
from subprocess import Popen,PIPE,STDOUT
from threading import Thread
from time import sleep

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
	sentences = []
	ENC_SENT_FILELOC = "/config/encouragement_sentences.txt"

	def __init__(self, repetitions_limit=10, spawnRosNodes=False):
		if spawnRosNodes==True:
			# launch roscore, wm_voice_manager and soundplay
			self.roscore_node = Popen(['roscore'], stdout=None, stderr=None)
			sleep(0.2)
			self.soundplay_node = Popen(['rosrun', 'sound_play', 'soundplay_node.py'], stdout=None, stderr=None)
			self.wm_voice_generator_node = Popen(['rosrun', 'wm_voice_generator', 'wm_voice_component_short.py'], stdout=None, stderr=None)
		rospy.init_node('reha_game')
		self.pub = rospy.Publisher('/robot/voice', String)
		self.load_sentences()
		self.repetitions_limit = repetitions_limit

	def load_sentences(self):
		# retrieve path to reha_game ROS package and load sentences file
		rospack = rospkg.RosPack()
		reha_game_pkg_path = rospack.get_path('reha_game')
		sentences_file = open(reha_game_pkg_path + self.ENC_SENT_FILELOC, "r")
		self.sentences = sentences_file.readlines()
		sentences_file.close()

	def initExercise(self, width=640, height=480):
		self.exercise_node = Popen(['rosrun', 'reha_game', 'Exercise1.py', '--width', str(width), '--height', str(height)], bufsize=1, stdout=PIPE, stderr=STDOUT)

	def stopExercise(self):
		self.exercise_node.terminate()
		self.exercise_node.wait()

	def incRepCounter(self):
		self.repetitions += 1
		self.pub.publish(str(self.repetitions))
		rospy.loginfo("Current number of repetitions done: " +str(self.repetitions))
		if self.repetitions == self.repetitions_limit/2:
			self.pub.publish(self.sentences[0])
			rospy.loginfo(self.sentences[0])
		elif self.repetitions == self.repetitions_limit-1:
			self.pub.publish(self.sentences[1])
			rospy.loginfo(self.sentences[1])
		elif self.repetitions == self.repetitions_limit:
			self.pub.publish(self.sentences[2])
			rospy.loginfo(self.sentences[2])

	def sayCalibLeft(self):
		sentence = "Now, move your arm to the left, and hold the position for 5 seconds."
		self.pub.publish(sentence)
		rospy.loginfo(sentence)
	def sayCalibRight(self):
		sentence = "Please lay your arm strechted out on the table in front of you, and hold the position for 5 seconds."
		self.pub.publish(sentence)
		rospy.loginfo(sentence)
	def sayCalibSuccess(self):
		sentence = "Callibration completed! You may begin your exercise now."
		self.pub.publish(sentence)
		rospy.loginfo(sentence)
		


# implementation of a custom timer thread that simply counts seconds up to some defined limit
class Timer(Thread):
	seconds = 0
	def __init__(self, seconds):
		self.seconds = seconds
		Thread.__init__(self)
	def run(self):
		sleep(self.seconds)




class Exercise1Game(object):
	# set trivial variables and parse arguments
	CALIBRATION_DURATION_SECS = 10
	CAMERA_WIDTH = 640
	CAMERA_HEIGHT = 480
	PATH_TO_VIDEO = ""
	#PATH_TO_SAMPLES_FOLDER = ""
	PATH_TO_OUTPUT_VIDEO = ""
	COLOR = "yellow"
	HAS_CALIBRATED = True
	encourager = None

	def __init__(self, argv):
		#print rospy.myargv()[1:]
		#print sys.argv

		# argument parsing
		parser = argparse.ArgumentParser(description='Reads video footage from a camera or a video file, and performs the Rehazenter exercise on it.')
		parser.add_argument('--width', type=int, dest="width", help='width of the camera window')
		parser.add_argument('--height', type=int, dest="height", help='height of the camera window')
		parser.add_argument('--path-to-video', type=str, dest="path_to_video", help='file path to the video file (default: webcam)')
		parser.add_argument('--calibrate', action="store_true", dest="calibrate", help='pass this flag to enable manual calibration of hand position thresholds')
		#parser.add_argument('--path-to-samples-folder', '-f', type=str, dest="arg_path_to_samples_folder", help='path to the folder where the object\'s image samples are stored')
		parser.add_argument('--path-to-output-video', type=str, dest="path_to_output_video", help='path to the output video that contains the modified input video with the detected object contours')
		parser.add_argument('--color', type=str, dest="color", help='the color of the object that should be tracked (default=yellow)')
		args = parser.parse_args(argv)

		# checking color argument
		if args.color != "yellow" and args.color != "blue" and args.color != "black" and args.color:
			print "Unrecognized color! Setting color back to default (yellow)."
		elif args.color != None:
			self.COLOR = args.color

		# checking camera dimensions (TODO: check if correct!)
		self.CAMERA_WIDTH = args.width
		self.CAMERA_WIDTH = args.height
		
		self.HAS_CALIBRATED = not args.calibrate

		# TODO: add remaining checks for path params

		# default thresholds (yellow)
		# **************************************************
		# argument parsing (the ROS way)
		#if rospy.has_param(self._Cascade_Classifier):
		#      cascPath = rospy.get_param(self._Cascade_Classifier)
		#    else:
		#      rospy.logwarn("CAscade parameters need to be set to start recognizer.")
		#      return

	def startGame(self, spawnEncUnit=True):
		# set HSV color thresholds
		if self.COLOR == "blue":
			# HSV color thresholds for BLUE
			THRESHOLD_LOW = np.array([110,50,50], dtype=np.uint8)
			THRESHOLD_HIGH = np.array([130,255,255], dtype=np.uint8)
		elif self.COLOR == "black":
			# HSV color thresholds for BLACK (not recommended for usage due to huge amounts of noise)
			THRESHOLD_LOW = np.array([60,15,0], dtype=np.uint8)
			THRESHOLD_HIGH = np.array([105,170,110], dtype=np.uint8)
		else:
			# HSV color thresholds for YELLOW (default)
			THRESHOLD_LOW = np.array([15,210,20], dtype=np.uint8)
			THRESHOLD_HIGH = np.array([35,255,255], dtype=np.uint8)
		print "Using color " + self.COLOR + "."

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
		HAND_MOV_X_THRESHOLD_LEFT = (self.CAMERA_WIDTH / 4) * 3
		HAND_MOV_X_THRESHOLD_RIGHT = (self.CAMERA_WIDTH / 4) * 1.55	# multiply with some tolerance value for more impaired people
		# define Y position thresholds for hand movement
		HAND_MOV_Y_THRESHOLD_LEFT = ((self.CAMERA_HEIGHT / 4) * 3) * 0.75	# multiply with some tolerance value, same as above
		HAND_MOV_Y_THRESHOLD_RIGHT = self.CAMERA_HEIGHT / 4

		# define tolerance values for both axis, in case we're calibrating manually
		TOLERANCE_X = self.CAMERA_WIDTH / 8
		TOLERANCE_Y = self.CAMERA_HEIGHT / 8
		if self.CAMERA_WIDTH > 2*self.CAMERA_HEIGHT:
			self.CAMERA_WIDTH = self.CAMERA_WIDTH / 6
		elif 2*self.CAMERA_WIDTH < self.CAMERA_HEIGHT:
			self.CAMERA_HEIGHT = self.CAMERA_HEIGHT / 6

		# Initialize camera and get actual resolution
		if self.PATH_TO_VIDEO == "":
			cap = cv2.VideoCapture(0)
		else:
			cap = cv2.VideoCapture(self.PATH_TO_VIDEO)

		# check if camera/video file was opened successfully
		if cap.isOpened() == False :
			print "Failed to open video capture stream! Aborting..."
			sys.exit()

		cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.CAMERA_WIDTH)
		cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.CAMERA_HEIGHT)

		# define some variables needed for our main loop
		last_moment = 0        
		moment_status = 0  
		last_moment_status = 0
		loop = 0
		has_motivated = False
		#print HAS_CALIBRATED
		timer = None

		# variable that tracks direction in which patient should currently move his hand
		hand_is_moving_right = True

		# Define the codec and create VideoWriter object
		fourcc = cv2.VideoWriter_fourcc(*'mp4v') # Be sure to use the lower case
		if self.PATH_TO_OUTPUT_VIDEO == "":
			out = cv2.VideoWriter('output.mp4v', fourcc, 20.0, (568, 516))
		else:
			out = cv2.VideoWriter(self.PATH_TO_OUTPUT_VIDEO, fourcc, 20.0, (568, 516))

		# launch ROS communication handler (if not started already through GUI)
		if spawnEncUnit == True:
			self.encourager = EncouragerUnit(10)
		elif self.encourager == None:
			print "Encouragement unit not started! Aborting..."
			sys.exit()
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
				if self.HAS_CALIBRATED == True:
					if hand_is_moving_right == True and abs(center[0]-HAND_MOV_X_THRESHOLD_RIGHT) < TOLERANCE_X and abs(center[1]-HAND_MOV_Y_THRESHOLD_RIGHT) < TOLERANCE_Y :
						hand_is_moving_right = False
						self.encourager.incRepCounter()
					elif hand_is_moving_right == False and abs(center[0]-HAND_MOV_X_THRESHOLD_LEFT) < TOLERANCE_X and abs(center[1]-HAND_MOV_Y_THRESHOLD_LEFT) < TOLERANCE_Y :
						hand_is_moving_right = True
				else:
					if timer == None:
						timer = Timer(self.CALIBRATION_DURATION_SECS)
						timer.start()
						if hand_is_moving_right == True:
							self.encourager.sayCalibRight()
					elif timer != None and timer.is_alive() == False:
						if hand_is_moving_right == True:
							HAND_MOV_X_THRESHOLD_RIGHT = center[0]
							HAND_MOV_Y_THRESHOLD_RIGHT = center[1]
							hand_is_moving_right = False
							del timer
							timer = None
							self.encourager.sayCalibLeft()
						else:
							HAND_MOV_X_THRESHOLD_LEFT = center[0]
							HAND_MOV_Y_THRESHOLD_LEFT = center[1]
							self.HAS_CALIBRATED = True
							hand_is_moving_right = True
							self.encourager.sayCalibSuccess()

							# TODO: adapt tolerance values to stored X/Y coordinates from calibration process?
							


			#print "Current center position of glove: ", center
		    
			# Print out the location and size (radius) of the largest detected contour
			#if center != None:
			#print str(center) + " Center - Radius:" + str(radius)   

			# Draw a green circle around the largest enclosed contour
			if center != None:
				cv2.circle(img, center, int(round(radius)), np.array([0,255,0]))

			font = cv2.FONT_HERSHEY_SIMPLEX
			cv2.putText(img,str(loop),(10,50), font, 1,(0,0,255),2)
			cv2.putText(img,"Repetitions: " + str(self.encourager.repetitions),(10,80), font, 1,(0,0,255),2)

			# Show image windows
			cv2.imshow('webcam', img)
			out.write(img)
			cv2.imshow('binary', img_binary)
			cv2.imshow('contours', img_contours)
			if cv2.waitKey(1) & 0xFF == ord('q') or self.encourager.repetitions == self.encourager.repetitions_limit:
				break
		out.release()


# ************************************************************************************
# *********************************** MAIN PROGRAM ***********************************
# ************************************************************************************
if __name__ == '__main__':
	# use the "rospy.myargv" argument vector instead of the built-in "sys.argv" to avoid problems with ROS argument re-mapping
	# (reason: https://groups.google.com/a/rethinkrobotics.com/forum/#!topic/brr-users/ErXVWhRmtNA)
	ex1 = Exercise1Game(rospy.myargv()[1:])
	ex1.startGame()
