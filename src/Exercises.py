#!/usr/bin/env python

from abc import ABCMeta, abstractmethod
import cv2
import signal
import numpy as np
import sys, traceback
import argparse
import roslib; roslib.load_manifest('rehabilitation_framework')
import ast
import roslaunch,rospkg,rospy,rosparam
from std_msgs.msg import String
from sensor_msgs.msg import Image
from os.path import expanduser
import os
import shlex
from subprocess import Popen,PIPE,STDOUT
from threading import Thread
from time import time,sleep
import signal
from rehabilitation_framework.msg import * 
import random
from cv_bridge import CvBridge

#                 |y=-0.1
#        D        |      A
#x=-0.1           |             x=0.1
#------------------------------------
#                 |
#       C         |      B
#                 |y=0.1


# some class definitions that represent enums
class Limb:
    LEFT_ARM, RIGHT_ARM = range(2)
class RobotPosition:
    LEFT, RIGHT, CENTER = range(3)
class RotationType:
    _unused, INTERNAL, EXTERNAL = range(3)
class MotionType:
    _unused, FLEXION, ABDUCTION = range(3)

# method that processes the arguments given as a parameter
def process_args(argv):
# define arguments to parse from the argument vector
    parser = argparse.ArgumentParser(description='Reads video footage from a camera or a video file, and performs the Rehazenter exercise on it.')
    parser.add_argument('--width', type=int, choices=range(320,1980), dest="width", help='width of the camera window')
    parser.add_argument('--height', type=int, choices=range(280,1080), dest="height", help='height of the camera window')
#parser.add_argument('--color_file', type=str, dest="color_file", help='the path to the color file of the object that should be tracked')
    parser.add_argument('--motion_type', type=int, choices=range(0,2), dest="motion_type", help='if specified value is valid, then it creates a simple motion exercise of the specified motion type (0=Flexion, 1=Abduction)')
    parser.add_argument('--rotation_type', type=int, choices=range(0,2), dest="rotation_type", help='if specified value is valid, then it creates a rotation exercise of the specified rotation type (0=Internal, 1=External)')
    parser.add_argument('--number_of_repetitions', type=int, choices=range(1,100), dest="number_of_repetitions", help='amount of repetitions to perform until completion of the exercise session')
    parser.add_argument('--time_limit', type=int, choices=range(0,7200), dest="time_limit", help='time limit that the patient has to complete his exercise')
    #parser.add_argument('--limb', type=int, choices=range(0,1), dest="limb", help='limb of the patient to be used for the exercise')
    parser.add_argument('--calibration_duration', type=int, choices=range(0,30), dest="calibration_duration", help='duration (in seconds) of the calibration procedure for the exercise')
    parser.add_argument('--robot_position', type=str, choices=range(0,2), dest="robot_position", help='position of the robot, in relation to the object that you want to track (0=left, 1=center, 2=right)')
    #parser.add_argument('--calibration_file', type=str, dest="calibration_file", help='path to the file containing the calibration data that will be used for object tracking')
    #parser.add_argument('--calibration_output_file', type=str, dest="calibration_output_file", help='path to the output file containing the calibration data that will be created when calibrating')
    args = parser.parse_args(argv)

    if args.motion_type != None:
        rospy.set_param_raw("motion_type", data.motion_type)
    if args.rotation_type != None:
        rospy.set_param_raw("rotation_type", data.rotation_type)
    if args.camera_width != None and args.camera_height != None:
        rospy.set_param_raw("camera_width", data.camera_width)
        rospy.set_param_raw("camera_height", data.camera_height)
    if args.robot_position != None:
        rospy.set_param_raw("robot_position", data.robot_position)
    if args.repetitions_limit != None:
        rospy.set_param_raw("repetitions_limit", data.repetitions)
    if args.time_limit != None:
        rospy.set_param_raw("time_limit", data.time_limit)

class VideoReader(Thread):
    def __init__(self, rgb_colors, camera_resolution=(640,480), current_calibration_points=[], tolerance_x=0, tolerance_y=0):
        # Initialize camera and get actual resolution
        Thread.__init__(self)
        self.camera_resolution = camera_resolution
        self._center = None
        self._last_valid_center = None
        self._radius = 0
        self._kill_thread = False
        self._tolerance_x = tolerance_x
        self._tolerance_y = tolerance_y

        # calculate HSV (or RGB!) thresholds from rgb_colors array
        self._hsv_threshold_lower = np.array([255,255,255], dtype=np.uint8)
        for color in rgb_colors:
            color_true = np.array([[[color[2],color[1],color[0]]]], dtype=np.uint8)
            hsv_color = cv2.cvtColor(color_true,cv2.COLOR_BGR2HSV)
            self._hsv_threshold_lower = np.array([min(self._hsv_threshold_lower[0],hsv_color[0][0][0]), min(self._hsv_threshold_lower[1],hsv_color[0][0][1]), min(self._hsv_threshold_lower[2],hsv_color[0][0][2])], dtype=np.uint8)
        self._hsv_threshold_upper = np.array([self._hsv_threshold_lower[0]+5,255,255], dtype=np.uint8)
        if int(self._hsv_threshold_lower[0])-10 < 0:
            self._hsv_threshold_lower[0] = 0
        else:
            self._hsv_threshold_lower[0] = self._hsv_threshold_upper[0]-10

        print str("Upper threshold: " + str(self._hsv_threshold_upper))
        print str("Lower threshold: " + str(self._hsv_threshold_lower))

        # define behaviour when SIGINT or SIGTERM received
        signal.signal(signal.SIGINT, self.exit_gracefully)
        signal.signal(signal.SIGTERM, self.exit_gracefully)

        # initialize place holders for calibration points
        self._current_calibration_point_index = 0
        self._current_calibration_points = current_calibration_points

        # initialize camera image publisher
        self._img_mod_pub = rospy.Publisher("/webcam/detected_blobs", Image, queue_size=5)
        self._img_original_pub = rospy.Publisher("/webcam/original_img", Image, queue_size=5)
        self._rate = rospy.Rate(60) # 60hz


    def exit_gracefully(self, signum, frame):
	print "Captured signal, beginning graceful shutdown."
        self.set_kill_thread()

    # ****************************** property definitions for the base class ******************************
    @property
    def img_original(self):
        return self._img_original

    @property
    def img_modified(self):
        return self._img_modified

    @property
    def hsv_thresholds(self):
        return (self._hsv_threshold_lower, self._hsv_threshold_upper)

    @property
    def last_valid_center(self):
        return self._last_valid_center

    @property
    def center(self):
        return self._center

    @property
    def radius(self):
        return self._radius

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
    # *****************************************************************************************************

    def set_kill_thread(self):
        self._kill_thread = True

    def run(self):
        # start capture device
        self._cap = cv2.VideoCapture(0)

        # set camera width and height
        self._cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.camera_resolution[0])
        self._cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.camera_resolution[1])

        # check if camera/video file was opened successfully
        if not self._cap.isOpened():
            raise Exception("Failed to open video capture stream! Aborting...")
        #self.set_kill_thread()

        # Minimum required radius of enclosing circle of contour
        MIN_RADIUS = 12

        # read frames from the capture device until interruption
        bridge = CvBridge()
        while not self._kill_thread:
            rval, img_original = self._cap.read()
            #if not rval:
            #        raise Exception("Failed to get frame from capture device!")
            self._img_original = img_original.copy()

            # Blur image to remove noise
            img_modified = cv2.GaussianBlur(img_original.copy(), (3, 3), 0)

            # Convert image from BGR to HSV
            img_modified = cv2.cvtColor(img_modified, cv2.COLOR_BGR2HSV)

            # Set pixels to white if in color range, others to black (binary bitmap)
            img_modified = cv2.inRange(img_modified, self._hsv_threshold_lower, self._hsv_threshold_upper)

            # Dilate image to make white blobs larger
            self._img_modified = cv2.dilate(img_modified, None, iterations = 1)

            # Find center of object using contours instead of blob detection. From:
            # http://www.pyimagesearch.com/2015/09/14/ball-tracking-with-opencv/
            contours = cv2.findContours(self._img_modified, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)[-2]
            self._center = None
            self._radius = 0
            if len(contours) > 0:
                c = max(contours, key=cv2.contourArea)
                ((x, y), self._radius) = cv2.minEnclosingCircle(c)
                M = cv2.moments(c)
                if M["m00"] > 0:
                    self._center = (int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"]))
                    if self._radius < MIN_RADIUS:
                        self._center = None

            #if center != None:
                #moment = int(center[0]/(self.camera_resolution[0]/32))
                #print moment
                #if (moment < self._last_moment):
                #   self._last_moment = moment
                #   moment_status = -1
                #elif (moment > self._last_moment):
                #   self._last_moment = moment
                #   moment_status = 1
            #self._last_moment_status = moment_status
            # draw a green circle around the largest enclosed contour

            # draw circle arround the detected object and write number of repetitions done on frame
            if self._center != None:
                self._last_valid_center = self._center
                cv2.circle(self._img_original, self._last_valid_center, int(round(self._radius)), np.array([0,255,0]))

            # draw calibration point bounding boxes on original image, if given
            if len(self._current_calibration_points) > 0:
                img_alpha_overlay = self._img_original.copy()
                for i,point in enumerate(self._current_calibration_points):
                    if i == self._current_calibration_point_index:
                        color = np.array([0,255,0])
                    else:
                        color = np.array([0,0,255])
                    cv2.rectangle(img_alpha_overlay, (point[0]+self._tolerance_x,point[1]+self._tolerance_y), (point[0]-self._tolerance_x,point[1]-self._tolerance_y), color, -1)
                cv2.addWeighted(img_alpha_overlay, 0.3, self._img_original, 0.7, 0, self._img_original)
            
            # publish images to ROS topics
            img_original_msg = bridge.cv2_to_imgmsg(self._img_original, encoding="bgr8")
            img_modified_msg = bridge.cv2_to_imgmsg(self._img_modified, encoding="mono8")
            self._img_original_pub.publish(img_original_msg)
            self._img_mod_pub.publish(img_modified_msg)
            self._rate.sleep()

    # returns the latest frame that was retrieve by the capture device
    def get_next_frame(self):
        if hasattr(self, "_frame"):
            return self._frame

    def update_calib_point_index(self):
        self._current_calibration_point_index = (self._current_calibration_point_index+1) % len(self._current_calibration_points)

    def update_calib_points(self, new_calibration_points):
        self._current_calibration_points = new_calibration_points
        self._current_calibration_point_index = 0

class EncouragerUnit(object):
    ENC_SENT_FILELOC = "/config/encouragement_sentences.txt"
    _happy_emotions_list = ["happy", "neutral", "showing_smile", "surprise", "breathing_exercise", "breathing_exercise_nose", "smile", "happy_blinking", "calming_down"]

    @property
    def repetitions_limit(self):
        return self._repetitions_limit

    @property
    def repetitions_arr(self):
        return self._repetitions_arr

    @property
    def spawn_ros_nodes(self):
        return self._spawn_ros_nodes
    @spawn_ros_nodes.setter
    def spawn_ros_nodes(self, spawn_ros_nodes):
        if type(spawn_ros_nodes) is bool:
            self._spawn_ros_nodes = spawn_ros_nodes
        else:
            raise ValueError("Argument spawn_ros_nodes is not boolean!")

    def __init__(self, number_of_blocks, repetitions_limit=10, quantitative_frequency=0, qualitative_frequency=0, emotional_feedbacks=[]):
        self._voice_pub = rospy.Publisher('/robot/voice', String, queue_size=5)
        self._face_pub = rospy.Publisher('/qt_face/setEmotion', String, queue_size=1)
        self._gesture_pub = rospy.Publisher('/robotMovementfromFile', String, queue_size=1)
        self.load_sentences()
        self._repetitions_limit = repetitions_limit
        self._repetitions_arr = [0] * number_of_blocks
        self._emotional_feedback_list = emotional_feedbacks
        self._quantitative_frequency = quantitative_frequency
        self._qualitative_frequency = qualitative_frequency
        random.seed()
        
    def load_sentences(self):
        # retrieve path to reha_game ROS package and load sentences file
        rospack = rospkg.RosPack()
        reha_game_pkg_path = rospack.get_path('rehabilitation_framework')
        sentences_file = open(reha_game_pkg_path + self.ENC_SENT_FILELOC, "r")
        self._sentences = sentences_file.readlines()
        sentences_file.close()

    def inc_repetitions_counter(self, index):
        self._repetitions_arr[index] += 1

        # process emotional feedbacks (= show an emotion on the QT robot face)
        for efb in self._emotional_feedback_list:
            # is feedback fixed? (yes/no)
            if efb[0] == True and self._repetitions_arr[index] == efb[1] or efb[0] == False and self._repetitions_arr[index] % efb[1] == 0 and self._repetitions_arr[index] > 0:
                if efb[2] == "random emotion":
                    self._face_pub.publish(self._happy_emotions_list[random.randint(0,len(self._happy_emotions_list)-1)])
                else:
                    self._face_pub.publish(efb[2])
                if efb[3] == True:
                    self._gesture_pub.publish("testTo5")

        # process quantitative feedback (= tell patient how many repetitions he has done so far)
        if self._quantitative_frequency > 0 and self._repetitions_arr[index] in range(0,self._repetitions_limit-1) and self._repetitions_arr[index] % self._quantitative_frequency == 0:
            self._voice_pub.publish(str(self._repetitions_arr[index]))
            print "Current number of repetitions done: " + str(self._repetitions_arr[index])

        # process qualitative feedback (= tell patient some randomly chosen motivational sentence)
        if self._qualitative_frequency > 0 and self._repetitions_arr[index] in range(0,self._repetitions_limit-1) and self._repetitions_arr[index] % self._qualitative_frequency == 0:
            self._voice_pub.publish(self._sentences[random.randint(0,len(self._sentences)-1)])

    def say(self, sentence):
        self._voice_pub.publish(sentence)
        print "The robot says: \"" + sentence + "\""


# implementation of a custom timer thread that simply counts seconds up to some defined limit
class Timer(Thread):
    def __init__(self, seconds=1):
        self._seconds = seconds
        self._kill_timer = False
        Thread.__init__(self)
    def run(self):
        temp = (self._seconds*1.0)/100.0
        for i in xrange(1,100):
            sleep(temp)
            if self._kill_timer:
                break

    def kill_timer(self):
        self._kill_timer = True


# *************************************************************
# EXERCISE CLASSES: used to instantiate the different exercises
# *************************************************************

# base class for all Exercise classes
class Exercise:
    __metaclass__ = ABCMeta

    def __init__(self, number_of_blocks=1, repetitions_limit=10, calibration_duration=10, camera_resolution=(640,480), time_limit=0, robot_position=RobotPosition.LEFT):
        rospy.init_node('reha_exercise', anonymous=True)
        self.repetitions_limit = repetitions_limit
        self.calibration_duration = calibration_duration
        if calibration_duration > 0:
            self._calibration_points_left_arm = []
            self._calibration_points_right_arm = []
        self.camera_resolution = camera_resolution
        self.limb = Limb.LEFT_ARM
        self._number_of_blocks = number_of_blocks
        self.robot_position = robot_position
        self.time_limit = time_limit

        temp_quant_freq = 0
        temp_quali_freq = 0
        temp_emotional_feedbacks = []

        # retrieve settings from parameter server
        if rospy.has_param('/reha_exercise/camera_width') and rospy.has_param('/reha_exercise/camera_height'):
                self.camera_resolution = (rosparam.get_param("/reha_exercise/camera_width"),rosparam.get_param("/reha_exercise/camera_height"))
        if rospy.has_param('/reha_exercise/robot_position'):
                self._robot_position = rosparam.get_param("/reha_exercise/robot_position")
        if rospy.has_param('/reha_exercise/rgb_colors'):
                rgb_colors = rosparam.get_param("/reha_exercise/rgb_colors")
        else:
            rospy.logerr("Required parameter rgb_colors is not set on server! Aborting.")
            sys.exit()
        if rospy.has_param('/reha_exercise/number_of_blocks'):
            self._number_of_blocks = rosparam.get_param("/reha_exercise/number_of_blocks")
        if rospy.has_param('/reha_exercise/repetitions_limit'):
            self._repetitions_limit = rosparam.get_param("/reha_exercise/repetitions_limit")
        if rospy.has_param('/reha_exercise/time_limit'):
            self._time_limit = rosparam.get_param("/reha_exercise/time_limit")
        if rospy.has_param('/reha_exercise/calibration_duration'):
            self._calibration_duration = rosparam.get_param("/reha_exercise/calibration_duration")
        if rospy.has_param('/reha_exercise/quantitative_frequency'):
            temp_quant_freq = rosparam.get_param("/reha_exercise/quantitative_frequency")
        if rospy.has_param('/reha_exercise/qualitative_frequency'):
            temp_quali_freq = rosparam.get_param("/reha_exercise/qualitative_frequency")
        if rospy.has_param('/reha_exercise/calibration_points_left_arm') and rospy.has_param('/reha_exercise/calibration_points_left_arm'):
            self._calibration_points_left_arm = rosparam.get_param("/reha_exercise/calibration_points_left_arm")
            self._calibration_points_right_arm = rosparam.get_param("/reha_exercise/calibration_points_right_arm")
        if rospy.has_param('/reha_exercise/emotional_feedback_list'):
            temp_emotional_feedbacks = rosparam.get_param("/reha_exercise/emotional_feedback_list")

        # define tolerance values for both axis when calibrating manually
        self._tolerance_x = camera_resolution[0] / 12
        self._tolerance_y = camera_resolution[1] / 12

        self._video_reader = VideoReader(rgb_colors, camera_resolution, self._calibration_points_left_arm, self._tolerance_x, self._tolerance_y)
        self._video_reader.start()
        self._encourager = EncouragerUnit(self._number_of_blocks, self.repetitions_limit, quantitative_frequency=temp_quant_freq, qualitative_frequency=temp_quali_freq, emotional_feedbacks=temp_emotional_feedbacks)
        self._rate = rospy.Rate(60) # 60hz
        sleep(0.2)  # wait some miliseconds until the video reader grabs its first frame from the capture device


    # ****************************** property definitions for the base class ******************************
    @property
    def calibration_points_left_arm(self):
        return self._calibration_points_left_arm
    @property
    def calibration_points_right_arm(self):
        return self._calibration_points_right_arm

    @property
    def calibration_output_file(self):
        return self._calibration_output_file
    @calibration_output_file.setter
    def calibration_output_file(self, calibration_output_file):
        if calibration_output_file == None:
            raise TypeError("cannot process None type object!")
        elif not (type(calibration_output_file) is str):
            raise TypeError("file path string expected!")
        else:
            self._calibration_output_file = calibration_output_file

    @property
    def calibration_file(self):
        return self._calibration_file
    @calibration_file.setter
    def calibration_file(self, calibration_file):
        # TODO: add more checks here! (f.ex. type checking)
        if calibration_file == None:
            self._calibration_file = ""
        elif calibration_file == "" or os.path.isfile(calibration_output_file):
            self._calibration_file = calibration_file
        else:
            raise ValueError("Specified calibration file does not exist!")

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
            print str(calibration_duration)
            raise TypeError("Integer value expected!")
        elif calibration_duration not in range(4,30):   # max. calibration duration: 30 seconds
            print str(calibration_duration)
            raise ValueError("calibration_duration should be an integer value between 0 and 30!")
        else:
            self._calibration_duration = calibration_duration


    @property
    def limb(self):
        return self._limb
    @limb.setter
    def limb(self, limb):
        if not (type(limb) is int) or limb not in range(2):
            raise TypeError("Invalid limb type!")
        else:
            self._limb = limb

    @property
    def robot_position(self):
        return self._robot_position
    @robot_position.setter
    def robot_position(self, robot_position):
        if not (type(robot_position) is int) or robot_position not in range(3):
            raise TypeError("Invalid robot position!")
        else:
            self._robot_position = robot_position

    @property
    def time_limit(self):
        return self._time_limit
    @time_limit.setter
    def time_limit(self, time_limit):
        if not (type(time_limit) is int):
            raise TypeError("Integer value expected!")
        elif time_limit not in range(7200): # max. time limit: 2 hours
            raise ValueError("time_limit should be an integer value between 0 and 7200!")
        else:
            self._time_limit = time_limit
    # *****************************************************************************************************

    # abstract method that calibrates the camera device with the necessary point coordinates needed for the exercise
    @abstractmethod
    def calibrate(self, calib_output_file=""):
        raise NotImplementedError

    # base class method that starts the exercise
    def start_game(self, results_output_file=""):
        # check if capture device has been initialized and calibrated
        if not self._video_reader.is_alive():
            raise Exception("Capture device not initialized!")
        elif len(self._calibration_points_left_arm) == 0:
            raise Exception("Device was not calibrated!")

        print "Camera dimensions: " +  str(cv2.CAP_PROP_FRAME_WIDTH) + " x " + str(cv2.CAP_PROP_FRAME_HEIGHT)
        print "Tolerance values: " + str(self._tolerance_x) + " x " + str(self._tolerance_y) + " pixels"

        # Main loop
        index=0
        current_block=1
        current_block_frame_counter = 0
        total_frame_counter = 0
        time_arr = []
        for i in range(0,self._number_of_blocks):
            temp = [0.0] * self._encourager.repetitions_limit
            time_arr.append(temp)
        trajectory_smoothness_arr = [0.0] * self._number_of_blocks
        reset_start_time_flag = True
        last_repetition_time = time()
        end_time = 0.0
        # sleep for 2 seconds in order to wait for soundplay and video reader to be ready
        sleep(2)
        self._encourager.say("Your exercise will begin soon.")
        sleep(2)
        self._encourager.say("Ready?")
        sleep(2)
        self._encourager.say("Go!")
        session_timer = None
        if self.time_limit > 0:
            session_timer = Timer(self.time_limit)
            session_timer.start()
        while current_block <= self._number_of_blocks or self.time_limit > 0 and session_timer.is_alive():
            total_frame_counter += 1

            # check if last valid detected object coordinates 
            last_center = self._video_reader.last_valid_center
            if self._video_reader.last_valid_center != None :
                # here, we check if the detected object is within the line connecting the current calibration points pair (up to some threshold)
                if self._limb == Limb.LEFT_ARM:
                    x_check = (last_center[0] - (self._calibration_points_left_arm[index])[0]) / ((self._calibration_points_left_arm[(index+1) % len(self._calibration_points_left_arm)])[0] - (self._calibration_points_left_arm[index])[0])
                    y_check = (last_center[1] - (self._calibration_points_left_arm[index])[1]) / ((self._calibration_points_left_arm[(index+1) % len(self._calibration_points_left_arm)])[1] - (self._calibration_points_left_arm[index])[1])
                    if abs(x_check) < 2 and abs(y_check) < 2:
                        current_block_frame_counter += 1
                else:
                    x_check = (last_center[0] - (self._calibration_points_right_arm[index])[0]) / ((self._calibration_points_right_arm[(index+1) % len(self._calibration_points_right_arm)])[0] - (self._calibration_points_right_arm[index])[0])
                    y_check = (last_center[1] - (self._calibration_points_right_arm[index])[1]) / ((self._calibration_points_right_arm[(index+1) % len(self._calibration_points_right_arm)])[1] - (self._calibration_points_right_arm[index])[1])
                    if abs(x_check-y_check) < 2 and abs(x_check-y_check) < 2:
                        current_block_frame_counter += 1

                # check if the next calibration point has been reached
                if (self._limb == Limb.LEFT_ARM and abs(last_center[0]-(self._calibration_points_left_arm[index])[0]) < self._tolerance_x and abs(last_center[1]-(self._calibration_points_left_arm[index])[1]) < self._tolerance_y) or (self._limb == Limb.RIGHT_ARM and abs(last_center[0]-(self._calibration_points_right_arm[index])[0]) < self._tolerance_x and abs(last_center[1]-(self._calibration_points_right_arm[index])[1]) < self._tolerance_y):
                    current_time = time()
                    time_arr[current_block-1][self._encourager.repetitions_arr[current_block-1]] = current_time - last_repetition_time
                    last_repetition_time = current_time
                    index += 1
                    self._video_reader.update_calib_point_index()
                    # check if all calibration points have been reached, if yes then increase repetitions counter
                    # NOTE: both point arrays have the same number of elements, so it doesn't matter which array size we take for the condition below
                    if index == len(self._calibration_points_left_arm):
                        index = 0
                        self._encourager.inc_repetitions_counter(current_block-1)
                        # check if all repetition points have been reached and increase corresponding repetitions counter in array
                        if self._encourager.repetitions_arr[current_block-1] == self._encourager.repetitions_limit:
                            trajectory_smoothness_arr[current_block-1] = (current_block_frame_counter*1.0) / (total_frame_counter*1.0)
                            current_block_frame_counter = 0
                            total_frame_counter = 0
                            self._encourager.say("Block " + str(current_block) + " finished!")
                            self._encourager.say("You did " + str(self._encourager.repetitions_arr[current_block-1]) + " repetitions in this block.")
                            sleep(3)
                            if current_block < self._number_of_blocks:
                                # let patient take a break (probably not needed for now)
                                #self._encourager.say("Take a 20 seconds break.")
                                #sleep(20)
                                enc_sentence = "Now, continue your exercise with your "
                                if self._limb == Limb.LEFT_ARM:
                                    self._limb = Limb.RIGHT_ARM
                                    self._video_reader.update_calib_points(self._calibration_points_right_arm)
                                    enc_sentence += "right arm."
                                else:
                                    self._limb = Limb.LEFT_ARM
                                    self._video_reader.update_calib_points(self._calibration_points_left_arm)
                                    enc_sentence += "left arm."
                                self._encourager.say(enc_sentence)
                            else:
                                break
                            current_block += 1
                            
            # check termination conditions
            if self.time_limit > 0 and not session_timer.is_alive() or not self._video_reader.is_alive():
                break
            self._rate.sleep()

        # kill video reader and timer threads
        if self._video_reader.is_alive():
            self._video_reader.set_kill_thread()
            self._video_reader.join()
        print "Video reader terminated!"
        if self.time_limit > 0:
            if session_timer.is_alive():
                self._encourager.say("Congratulations! You have completed all of the blocks.")
                sleep(2)
                session_timer.kill_timer()
                session_timer.join()
            else:
                self._encourager.say("Time is over!")
                # TODO: play random congratulation sentence depending on performance
            print "Timer thread terminated!"

        # store repetitions and time results
        if results_output_file != "":
	    with open(results_output_file, "w") as res_output_file:
                res_output_file.write("repetitions_results=" + str(self._encourager.repetitions_arr)+ "\n")
                res_output_file.write("time_results=" + str(time_arr)+ "\n")
                res_output_file.write("trajectory_smoothness=" + str(trajectory_smoothness_arr)+ "\n")


# exercise that counts circular movements performed on the table surface
class RotationExercise(Exercise):
    def __init__(self, number_of_blocks=1, repetitions_limit=10, calibration_duration=10, camera_resolution=(640,480), hsv_thresholds=((35, 255, 255),(15, 210, 20)), time_limit=0, robot_position=RobotPosition.LEFT, rotation_type=RotationType.INTERNAL):
        super(RotationExercise, self).__init__(number_of_blocks, repetitions_limit, calibration_duration, camera_resolution, time_limit, robot_position)
        self._rotation_type = rotation_type
        if rospy.has_param('/reha_exercise/rotation_type'):
            self._rotation_type = rospy.get_param("/reha_exercise/rotation_type")

    @property
    def rotation_type(self):
        return self._rotation_type
    @rotation_type.setter
    def rotation_type(self, rotation_type):
        if not (type(rotation_type) is int) or (rotation_type not in range(2)):
            raise TypeError("Invalid rotation type!")
        else:
            self._rotation_type = rotation_type

    #def calibrate(self, calibration_points):
        # do stuff


# exercise that counts simple movements with 2 or 3 point coordinates performed on the table surface
class SimpleMotionExercise(Exercise):
    def __init__(self, number_of_blocks=1, repetitions_limit=10, calibration_duration=10, camera_resolution=(640,480), hsv_thresholds=((35, 255, 255),(15, 210, 20)), time_limit=0, robot_position=RobotPosition.LEFT, motion_type=MotionType.FLEXION):
        super(SimpleMotionExercise, self).__init__(number_of_blocks, repetitions_limit, calibration_duration, camera_resolution, time_limit, robot_position)
        self._motion_type = motion_type
        if rospy.has_param('/reha_exercise/motion_type'):
            self._motion_type = rospy.get_param("/reha_exercise/motion_type")

    @property
    def motion_type(self):
        return self._motion_type
    @motion_type.setter
    def motion_type(self, motion_type):
        if not (type(motion_type) is MotionType):
            raise TypeError("Invalid motion type!")
        else:
            self._motion_type = motion_type


    def calibrate(self, calib_output_file=""):
        # check if capture device running
        if not (self._video_reader.is_alive()):
            raise Exception("Capture device not initialized!")

        # define number of calibration points to record (depends on exercise)
        number_of_calibration_points = 0
        if self.motion_type == MotionType.FLEXION or self.motion_type == MotionType.ABDUCTION:
            number_of_calibration_points = 2
        self._calibration_points_left_arm = []
        self._calibration_points_right_arm = []

        # number of frames to wait when no object detected (before warning user)
        NO_CENTER_FOUND_MAX = 200

        # Main calibration loop
        timer = None
        encourager_guide_flag = True
        no_center_found_counter = 0
        no_center_found_flag = False
        # sleep for 2 seconds in order to wait for soundplay and video reader to be ready
        sleep(2)
        while len(self._calibration_points_right_arm) < number_of_calibration_points:
            #print no_center_found_counter
            #print no_center_found_flag
            #print "-" * 60
            # tell encourager to say a sentence, if needed
            if encourager_guide_flag:
                if self._limb == Limb.LEFT_ARM:
                    if len(self._calibration_points_left_arm) == 0:
                        self._encourager.say("Please lay your left hand on the table, close to you.")
                    else:   
                        if self._motion_type == MotionType.FLEXION:
                            self._encourager.say("Now, stretch your left arm out in front of you on the table.")
                        else:
                            self._encourager.say("Now, stretch your left arm diagonally up and left on the table.")
                elif self._limb == Limb.RIGHT_ARM:
                    if len(self.calibration_points_right_arm) == 0:
                        self._encourager.say("Next, please lay your right hand on the table, close to you.")
                    else:   
                        if self._motion_type == MotionType.FLEXION:
                            self._encourager.say("Now, stretch your right arm out in front of you on the table.")
                        else:
                            self._encourager.say("Now, stretch your right arm diagonally up and right on the table.")
                encourager_guide_flag = False

            # get next frame from capture device
            frame = self._video_reader.img_modified

            # (re-)initialize timer if necessary
            if timer == None and no_center_found_counter == 0:
                timer = Timer(self.calibration_duration)
                timer.start()
                #if (self._limb == Limb.LEFT_ARM and len(self._calibration_points_left_arm) > 0) or (self._limb == Limb.RIGHT_ARM and len(self._calibration_points_right_arm) > 0):
                #encourager_guide_flag = True
                no_center_found_flag = False 
		#print "timer re-init"

            # if no object was found in the video capture for some time, wait for object to reappear
            last_center = self._video_reader.center
            if last_center != None:
                if no_center_found_counter > 0:
                    no_center_found_counter -= 1
                    if no_center_found_counter == 0 and no_center_found_flag:
                        no_center_found_flag = False
                        self._encourager.say("Okay. I can see your object now.")
                        #encourager_guide_flag = True
                # store coordinates when timer has run out
                if timer != None and timer.is_alive() == False:
                    # check if any of the recorded points are too close to each other before inserting
                    if self._limb == Limb.LEFT_ARM: 
                        # check if calibration point is valid for the left arm
                        if len(self._calibration_points_left_arm) > 0 and ((self.robot_position == RobotPosition.LEFT and self.motion_type == MotionType.ABDUCTION or self.robot_position == RobotPosition.CENTER and self.motion_type == MotionType.FLEXION) and (abs((self._calibration_points_left_arm[len(self._calibration_points_left_arm)-1])[1]-last_center[1]) < self._tolerance_y) \
                        or ((self.robot_position == RobotPosition.CENTER and self.motion_type == MotionType.ABDUCTION or self.robot_position == RobotPosition.RIGHT and self.motion_type == MotionType.FLEXION or self.robot_position == RobotPosition.LEFT and self.motion_type == MotionType.FLEXION) and ((abs((self._calibration_points_left_arm[len(self._calibration_points_left_arm)-1])[0]-last_center[0]) < self._tolerance_x) and (abs((self._calibration_points_left_arm[len(self._calibration_points_left_arm)-1])[1]-last_center[1]) < self._tolerance_y-(self._tolerance_y/2))) \
                        or self.robot_position == RobotPosition.RIGHT and self.motion_type == MotionType.ABDUCTION and (abs((self._calibration_points_left_arm[len(self._calibration_points_left_arm)-1])[0]-last_center[0]) < self._tolerance_x))):
                            self._encourager.say("The calibration points are too close to each other, please try again.")
                            self._calibration_points_left_arm = []
                        elif no_center_found_counter == 0:
                            # take last valid centroid that was found by video reader and store coordinates
                            self._calibration_points_left_arm.append(last_center)
                            encourager_guide_flag = True
                            # switch to right arm when all points for the left arm have been calibrated
                            if len(self._calibration_points_left_arm) == number_of_calibration_points:
                                self.limb = Limb.RIGHT_ARM
                    else:
                        # else, check if calibration point is valid for the right arm
                        if len(self._calibration_points_right_arm) > 0 and (self.robot_position == RobotPosition.LEFT and self.motion_type == MotionType.ABDUCTION and (abs((self._calibration_points_right_arm[len(self._calibration_points_right_arm)-1])[0]-last_center[0]) < self._tolerance_x) \
                        or ((self.robot_position == RobotPosition.CENTER and self.motion_type == MotionType.FLEXION or self.robot_position == RobotPosition.RIGHT and self.motion_type == MotionType.ABDUCTION) and (abs((self._calibration_points_right_arm[len(self._calibration_points_right_arm)-1])[1]-last_center[1]) < self._tolerance_y)) \
                        or ((self.robot_position == RobotPosition.CENTER and self.motion_type == MotionType.ABDUCTION or self.robot_position == RobotPosition.RIGHT and self.motion_type == MotionType.FLEXION or self.robot_position == RobotPosition.LEFT and self.motion_type == MotionType.FLEXION) and ((abs((self._calibration_points_right_arm[len(self._calibration_points_right_arm)-1])[0]-last_center[0]) < self._tolerance_x) or (abs((self._calibration_points_right_arm[len(self._calibration_points_right_arm)-1])[1]-last_center[1]) < self._tolerance_y-(self._tolerance_y/2))))):
                            self._encourager.say("The calibration points are too close to each other, please try again.")
                            self._calibration_points_right_arm = []
                        elif no_center_found_counter == 0:
                            # take last valid centroid that was found by video reader and store coordinates
                            self._calibration_points_right_arm.append(last_center)
                            if len(self._calibration_points_right_arm) < number_of_calibration_points:
                                encourager_guide_flag = True
                    timer = None
            elif no_center_found_counter < NO_CENTER_FOUND_MAX and not no_center_found_flag:
                no_center_found_counter += 1
            #elif no_center_found_counter == NO_CENTER_FOUND_MAX and timer != None and timer.is_alive() and not no_center_found_flag:
            elif no_center_found_counter == NO_CENTER_FOUND_MAX and not no_center_found_flag:
                no_center_found_flag = True
                self._encourager.say("I cannot find your object. Please move it closer to the camera.")
                if timer != None:
                    timer.kill_timer()
                    timer.join()
                timer = None


            if len(self._calibration_points_right_arm) == number_of_calibration_points:
                rospy.loginfo("Shutting down video reader...")
                self._video_reader.set_kill_thread()
                self._video_reader.join()
            if not self._video_reader.is_alive():
                if timer != None:
                    rospy.loginfo("Shutting down timer thread...")
                    timer.kill_timer()
                    timer.join()
                break
            self._rate.sleep()

        # write calibration results to file, if property set
        if calib_output_file != "" and len(self._calibration_points_left_arm) == number_of_calibration_points and len(self._calibration_points_right_arm) == number_of_calibration_points:
            self._encourager.say("Calibration completed!")
            with open(calib_output_file, "w") as calib_output_file:
                calib_output_file.write("motion_type=" + str(self._motion_type) + "\n")
                calib_output_file.write("rotation_type=0\n")
                calib_output_file.write("robot_position=" + str(self._robot_position) + "\n")
                calib_output_file.write("camera_width=" + str(self._video_reader.camera_resolution[0]) + "\n")
                calib_output_file.write("camera_height=" + str(self._video_reader.camera_resolution[1]) + "\n")
                calib_output_file.write("calibration_points_left_arm=" + str(self._calibration_points_left_arm)+ "\n")
                calib_output_file.write("calibration_points_right_arm=" + str(self._calibration_points_right_arm)+ "\n")
                print "good"
        else:
            self._encourager.say("Calibration interrupted!")
    	sleep(2.5)



# ************************************************************************************
# *********************************** MAIN PROGRAM ***********************************
# ************************************************************************************
if __name__ == '__main__':
    # parse command-line arguments
    parser = argparse.ArgumentParser(description='Reads video footage from a camera or a video file, and performs the Rehazenter exercise on it.')
    parser.add_argument('--calibrate_only', action="store_true", dest="calibrate_only", help='pass this argument if you only wish to perform the calibration process and not the exercise')
    parser.add_argument('--calibration_output_file', type=str, dest="calibration_output_file", default="/tmp/temp_calib_file.clb", help='resulting file from the calibration process will be stored in this file path')
    parser.add_argument('--results_output_file', type=str, dest="results_output_file", default="/tmp/temp_results_file.tmp", help='results from the exercise will be stored in this file')

    # use the "rospy.myargv" argument vector instead of the built-in "sys.argv" to avoid problems with ROS argument re-mapping
    # (reason: https://groups.google.com/a/rethinkrobotics.com/forum/#!topic/brr-users/ErXVWhRmtNA)
    args = parser.parse_args(rospy.myargv()[1:])

    # parameters checking
    if args.calibrate_only and not (rospy.has_param("/reha_exercise/calibration_duration") and rospy.has_param("/reha_exercise/camera_width") and rospy.has_param("/reha_exercise/camera_height") and rospy.has_param("/reha_exercise/robot_position") and rospy.has_param("/reha_exercise/rgb_colors") and rospy.has_param("/reha_exercise/motion_type") and rospy.has_param("/reha_exercise/rotation_type")):    
        print("ERROR: not all required parameters are set on the parameter server for calibration! Aborting.")
        sys.exit(1)
    else:
        ros_motion_type = rosparam.get_param("/reha_exercise/motion_type")
        ros_rotation_type = rosparam.get_param("/reha_exercise/rotation_type")
        if not args.calibrate_only and not (rospy.has_param("/reha_exercise/quantitative_frequency") or rospy.has_param("/reha_exercise/qualitative_frequency") or rospy.has_param("/reha_exercise/number_of_blocks") or rospy.has_param("/reha_exercise/calibration_points_left_arm") or rospy.has_param("/reha_exercise/calibration_points_right_arm") or rospy.has_param("/reha_exercise/emotional_feedback_list")):
            print("ERROR: not all required parameters are set on the parameter server for the exercise! Aborting.")
            sys.exit(1)
        elif not (ros_motion_type == 0 and ros_rotation_type > 0 or ros_motion_type > 0 and ros_rotation_type == 0):
            print("ERROR: parameter server contains invalid exercise parameters! Aborting.")
            sys.exit(2)
        elif args.calibrate_only and args.calibration_output_file == "":
            print("ERROR: Output calibration file path not set!")
            sys.exit(3)
    try:
        if ros_motion_type == 0 and ros_rotation_type > 0:
            exercise = RotationExercise()
        else:
            exercise = SimpleMotionExercise()
        if args.calibrate_only and args.calibration_output_file != "":
            exercise.calibrate(args.calibration_output_file)
        else:
            exercise.start_game(args.results_output_file)
    except Exception as e:
        print("An error occured when launching the exercise!\n" + str(e))
        traceback.print_exc(file=sys.stdout)
        sys.exit(4)
    sys.exit(0)
