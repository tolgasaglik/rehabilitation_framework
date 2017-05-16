#!/usr/bin/env python

import abc
import cv2
import signal
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

# Find the largest contour and use it to compute the min enclosing circle
def get_centroid(frame):  
	# Find center of object using contours instead of blob detection. From:
	# http://www.pyimagesearch.com/2015/09/14/ball-tracking-with-opencv/

	# Minimum required radius of enclosing circle of contour
	MIN_RADIUS = 2

	contours = cv2.findContours(frame, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)[-2]
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
		moment = int(center[0]/(self.camera_resolution[0]/32))
		#print moment
		#if (moment < self._last_moment):
		#	self._last_moment = moment
		#	moment_status = -1
		#elif (moment > self._last_moment):
		#	self._last_moment = moment
		#	moment_status = 1
	#self._last_moment_status = moment_status

	return center

# method that shows a frame given as parameter on a window, performs some pre-processing of the frame before showing it
def show_frame(center, frame):
	# draw a green circle around the largest enclosed contour
	if center != None:
		cv2.circle(img, center, int(round(radius)), np.array([0,255,0]))

	# write number of repetitions done on frame
	font = cv2.FONT_HERSHEY_SIMPLEX
	cv2.putText(img,str(loop),(10,50), font, 1,(0,0,255),2)
	cv2.putText(img,"Repetitions: " + str(self.encourager.repetitions),(10,80), font, 1,(0,0,255),2)

	# Show image windows
	cv2.imshow('webcam', img)
	out.write(img)
	#cv2.imshow('binary', img_binary)
	cv2.imshow('contours', img_contours)

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
		self.exercise_node = Popen(['rosrun', 'reha_game', 'Exercise1.py', '--width', str(width), '--height', str(height), '--calibrate'], stdout=PIPE, stderr=STDOUT)

	def stopExercise(self):
		self.exercise_node.kill()
		self.exercise_node.wait()

	def incRepCounter(self):
		self.repetitions += 1
		self.pub.publish(str(self.repetitions))
		print "Current number of repetitions done: " +str(self.repetitions)
		if self.repetitions == self.repetitions_limit/2:
			self.pub.publish(self.sentences[0])
			print self.sentences[0]
		elif self.repetitions == self.repetitions_limit-1:
			self.pub.publish(self.sentences[1])
			print self.sentences[1]
		elif self.repetitions == self.repetitions_limit:
			self.pub.publish(self.sentences[2])
			print self.sentences[2]

	def sayCalibLeft(self):
		sentence = "Now, move your arm to the left, and hold the position for 5 seconds."
		self.pub.publish(sentence)
		print sentence
	def sayCalibRight(self):
		sentence = "Please lay your arm strechted out on the table in front of you, and hold the position for 5 seconds."
		self.pub.publish(sentence)
		print sentence
	def sayCalibSuccess(self):
		sentence = "Calibration completed! You may begin your exercise now."
		self.pub.publish(sentence)
		print sentence
		


# implementation of a custom timer thread that simply counts seconds up to some defined limit
class Timer(Thread):
	def __init__(self, seconds=1):
		self.seconds = seconds
		Thread.__init__(self)
	def run(self):
		temp = (self.seconds*1.0)/100.0
		for i in xrange(1,100):
			sleep(temp)


# *************************************************************
# EXERCISE CLASSES: used to instantiate the different exercises
# *************************************************************
# some class definitions that represent enums
class Limb:
	_unused, LEFT_ARM, RIGHT_ARM, LEFT_LEG, RIGHT_LEG = range(4)
class RotationType:
	_unused, INTERNAL, EXTERNAL = range(2)
class MotionType:
	_unused, FLEXION, ABDUCTION
class Color:
	_unused, YELLOW, BLUE, BLACK

# returns the HSV thresholds of the color (as a tuple) passed as parameter
def get_hsv_thresholds(self, color):
	if color == Color.BLUE:
		# HSV color thresholds for BLUE
		THRESHOLD_LOW = np.array([110,50,50], dtype=np.uint8)
		THRESHOLD_HIGH = np.array([130,255,255], dtype=np.uint8)
	elif self.COLOR == Color.BLACK:
		# HSV color thresholds for BLACK (not recommended for usage due to huge amounts of noise)
		THRESHOLD_LOW = np.array([60,15,0], dtype=np.uint8)
		THRESHOLD_HIGH = np.array([105,170,110], dtype=np.uint8)
	elif self.COLOR == Color.YELLOW:
		# HSV color thresholds for YELLOW (default)
		THRESHOLD_LOW = np.array([15,210,20], dtype=np.uint8)
		THRESHOLD_HIGH = np.array([35,255,255], dtype=np.uint8)
	else:
		raise ValueError("Unknown or unused color!")
	return (THRESHOLD_LOW, THRESHOLD_HIGH)


# base class for all Exercise classes
class Exercise(object):
	__metaclass__ = abc.ABCMeta

	def __init__(self, repetitions_limit=10, calibration_duration=10, camera_res=(640,480), color="yellow", argv=None, limb=Limb._unused, time_limit=0, path_to_video=""):
		self.repetitions_limit = repetitions_limit
		self._encourager = EncouragerUnit(self.repetitions_limit)
		self.calibration_duration = calibration_duration
		if calibration_duration > 0:
			self._calibration_points = []
		self.camera_resolution = camera_res
		self.color = color
		self.limb = limb
		self.time_limit = time_limit
		self.path_to_video = path_to_video
		if argv != None:
			self.process_args(argv)
		self.init_capture()

	# ****************************** property definitions for the base class ******************************
	@property
	def path_to_video(self):
		return self._path_to_video
	@path_to_video.setter
	def path_to_video(self, path_to_video):
		# TODO: add more checks here! (f.ex. type checking)
		if os.path.isfile(path_to_video):
			self._path_to_video = path_to_video
		else:
			raise ValueError("Specified video file does not exist!")

	@property
	def repetitions_limit(self):
		return self._repetitions_limit
	@repetitions_limit.setter
	def repetitions_limit(self, repetitions_limit):
		if repetitions_limit not in range(1,100):
			raise ValueError("repetitions_limit should be a positive integer value between 1 and 100!")
		else:
			self._repetitions_limit = repetitions_limit

	@property
	def calibration_duration(self):
		return self._calibration_duration
	@calibration_duration.setter
	def calibration_duration(self, calibration_duration):
		if not (type(calibration_duration) is int):
			raise TypeError("Integer value expected!")
		elif calibration_duration not in range(0,30):	# max. caibration duration: 30 seconds
			raise ValueError("calibration_duration should be an integer value between 0 and 30!")
		else:
			self._calibration_duration = calibration_duration

	@property
	def camera_resolution(self):
		return (self._camera_width, self._camera_height)
	@camera_resolution.setter
	def camera_resolution(self, camera_resolution):
		if not type(camera_resolution) is tuple or len(camera_resolution) != 2:
			raise TypeError("camera_resolution argument must be a tuple of two integers!")
		elif not all(type(x) is int for x in camera_resolution):
			raise TypeError("Tuple must only contain integers!")
		elif camera_resolution[0] not in [320,424,640,848,960,1280,1920] or camera_resolution[1] not in [180,240,360,480,540,720,1080]:
			raise ValueError("Invalid camera resolution!")
		else:	
			self._camera_width = camera_resolution[0]
			self._camera_height = camera_resolution[1]

	@property
	def color(self):
		return self._color
	@color.setter
	def color(self, color):
		if not (type(camera_resolution) is Color):
			raise TypeError("Invalid color!")
		else:	
			self._color = color

	@property
	def limb(self):
		return self._limb
	@limb.setter
	def limb(self, limb):
		if not (type(limb) is Limb):
			raise TypeError("Invalid limb type!")
		else:
			self._limb = limb

	@property
	def time_limit(self):
		return self._time_limit
	@time_limit.setter
	def time_limit(self, time_limit):
		if not (type(time_limit) is int):
			raise TypeError("Integer value expected!")
		elif time_limit not in range(1,7200):	# max. time limit: 2 hours
			raise ValueError("time_limit should be an integer value between 0 and 7200!")
		else:
			self._time_limit = time_limit
	# *****************************************************************************************************

	# initializes the camera capture device
	def init_capture(self):
		# define tolerance values for both axis, in case we're calibrating manually
		self._tolerance_x = self.camera_resolution[0] / 8
		self._tolerance_y = self.camera_resolution[1] / 8
		if self.camera_resolution[0] > 2*self.camera_resolution[1]:
			self.camera_resolution[0] = self.camera_resolution[0] / 6
		elif 2*self.camera_resolution[0] < self.camera_resolution[1]:
			self.camera_resolution[1] = self.camera_resolution[1] / 6

		# Initialize camera and get actual resolution
		if self._path_to_video == "":
			self._cap = cv2.VideoCapture(0)
		else:
			self._cap = cv2.VideoCapture(self.path_to_video)

		# check if camera/video file was opened successfully
		if self._cap.isOpened() == False :
			raise Exception("Failed to open video capture stream! Aborting...")

		self._cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.camera_resolution[0])
		self._cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.camera_resolution[1])

	# method that returns the next frame that was retrieved by the capture device
	def get_next_frame(self):
		rval, img = self.cap.read()
		if not rval:
			raise Exception("Failed to get frame from capture device!")

		#VIDEO_WIDTH = cap.get(cv2.CAP_PROP_FRAME_WIDTH)
		#VIDEO_HEIGHT = cap.get(cv2.CAP_PROP_FRAME_HEIGHT)

		# Blur image to remove noise
		img_filter = cv2.GaussianBlur(img.copy(), (3, 3), 0)

		# Convert image from BGR to HSV
		img_filter = cv2.cvtColor(img_filter, cv2.COLOR_BGR2HSV)

		# Set pixels to white if in color range, others to black (binary bitmap)
		thresholds = get_hsv_thresholds(self.color)
		img_binary = cv2.inRange(img_filter.copy(), thresholds[0], thresholds[1])

		# Dilate image to make white blobs larger
		img_binary = cv2.dilate(img_binary, None, iterations = 1)
		return img_binary


	# abstract method that calibrates the camera device with the necessary point coordinates needed for the exercise
	@abstractmethod
	def calibrate(self):
		raise NotImplementedError

	# base class method that starts the exercise
	def start_game(self):
		# check if capture device has been initialized and calibrated
		if not self._cap.isOpened():
			raise Exception("Capture device not initialized!")
		elif len(self._calibration_points) == 0:
			raise Exception("Device was not calibrated!")

		print "Camera dimensions: " +  str(cv2.CAP_PROP_FRAME_WIDTH) + " x " + str(cv2.CAP_PROP_FRAME_HEIGHT)
		print "Press the \"q\" key to quit."

		# Main loop
		index=0
		while True:
			#VIDEO_WIDTH = cap.get(cv2.CAP_PROP_FRAME_WIDTH)
			#VIDEO_HEIGHT = cap.get(cv2.CAP_PROP_FRAME_HEIGHT)

			frame = self.get_frame()
			center = get_center(frame)

			# check if hand movement thresholds have been reached and count repetitions accordingly 
			if center != None :
				if abs(center[0]-self._calibration_points[index][0]) < self._tolerance_x and abs(center[1]-self._calibration_points[index][1]) < self._tolerance_y :
						if index < len(self._calibration_points):
							index += 1
						else:
							index = 0
						self._encourager.incRepCounter()
							
			#print "Current center position of glove: ", center
		    
			show_frame(frame, center)
			if cv2.waitKey(1) & 0xFF == ord('q') or self._encourager.repetitions == self._encourager.repetitions_limit:
				break

	# method that processes the arguments given as a parameter
	def process_args(self, argv):
		if len(argv) > 0:
			parser = argparse.ArgumentParser(description='Reads video footage from a camera or a video file, and performs the Rehazenter exercise on it.')
			parser.add_argument('--width', type=int, dest="width", help='width of the camera window')
			parser.add_argument('--height', type=int, dest="height", help='height of the camera window')
			parser.add_argument('--path-to-video', type=str, dest="path_to_video", help='file path to the video file (default: webcam)')
			parser.add_argument('--calibrate', action="store_true", dest="calibrate", help='pass this flag to enable manual calibration of hand position thresholds')
			#parser.add_argument('--path-to-samples-folder', '-f', type=str, dest="arg_path_to_samples_folder", help='path to the folder where the object\'s image samples are stored')
			parser.add_argument('--color', type=str, dest="color", help='the color of the object that should be tracked (default=yellow)')
			args = parser.parse_args(argv)
			camera_resolution = (args.width, args.height)
			self.camera_resolution = camera_resolution
			self.calibration_enabled = args.calibrate
			self.color = args.color
			self.path_to_video = args.path_to_video


# exercise that counts circular movements performed on the table surface
class RotationExercise(Exercise):
	def __init__(self, repetitions_limit=10, calibration_duration=10, camera_res=(640,480), color="yellow", argv=None, limb=Limb._unused, time_limit=0, rotation_type=RotationType.INTERNAL):
		super(RotationExercise, self).__init__(repetitions_limit, calibration_duration, camera_res, color, argv, limb)
		self._rotation_type = rotation	

	@property
	def rotation_type(self):
		return self._rotation_type
	@rotation_type.setter
	def rotation_type(self, rotation_type):
		if not (type(rotation_type) is RotationType):
			raise TypeError("Invalid rotation type!")
		else:
			self._rotation_type = rotation_type

	#def calibrate(self, calibration_points):
		# do stuff


# exercise that counts simple movements with 2 or 3 point coordinates performed on the table surface
class SimpleMotionExercise(Exercise):
	def __init__(self, repetitions_limit=10, calibration_duration=10, camera_res=(640,480), color="yellow", argv=None, limb=Limb._unused, time_limit=0, motion_type=MotionType.FLEXION):
		super(SimpleMotionExercise, self).__init__(repetitions_limit, calibration_duration, camera_res, color, argv, limb)
		self._motion_type = motion_type

	@property
	def motion_type(self):
		return self._motion_type
	@motion_type.setter
	def motion_type(self, motion_type):
		if not (type(motion_type) is MotionType):
			raise TypeError("Invalid motion type!")
		else:
			self._motion_type = motion_type


	def calibrate(self):
		# check if required properties are set
		if self.motion_type == MotionType._unused or self.color == Color._unused or self.limb == Limb._unused:
			raise Exception("Required properties not set! Please set the color and motion type before calibrating.")
		elif not (hasattr(self, "cap")) or self.cap == None and not (self.cap.isOpened()):
			raise Exception("Capture device not initialized!")

		# define number of calibration points to record (depends on exercise)
		number_of_calibration_points = 0
		if self.motion_type == MotionType.FLEXION or self.motion_type == MotionType.ABDUCTION:
			number_of_calibration_points = 2
		self._calibration_points = []

		# set this variable to whatever time you want (in seconds!)
		CALIBRATION_DURATION_SECS = 10

		# Main calibration loop
		self.encourager.sayCalibRight()
		while len(self._calibration_points) < number_of_calibration_points:
			# get next frame from capture device and compute centroid of the object to track
			frame = self.get_next_frame()
			center = get_centroid(frame)

			# (re-)initialize timer if necessary
			if timer == None:
				timer = Timer(self.CALIBRATION_DURATION_SECS)
				timer.daemon = True
				timer.start()

			# store coordinates when timer has run out
			if timer != None and timer.is_alive() == False:
				self._calibration_points.append((center[0], center[1]))
				del timer
				timer = None
				self.encourager.sayCalibLeft()

			# show frame on a sparate window
			show_frame(frame, center)
		self.encourager.sayCalibSuccess()

	#def process_args(self, argv):
		# do stuff

#class ConesExercise(Exercise)



# ************************************************************************************
# *********************************** MAIN PROGRAM ***********************************
# ************************************************************************************
if __name__ == '__main__':
	# use the "rospy.myargv" argument vector instead of the built-in "sys.argv" to avoid problems with ROS argument re-mapping
	# (reason: https://groups.google.com/a/rethinkrobotics.com/forum/#!topic/brr-users/ErXVWhRmtNA)
	motion_ex = SimpleMotionExercise(argv=rospy.myargv()[1:])
	motion_ex.calibrate()
	motion_ex.start_game()
