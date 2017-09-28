#!/usr/bin/env python

from abc import ABCMeta, abstractmethod
import numpy as np
import sys, traceback
import argparse
import roslib; roslib.load_manifest('rehabilitation_framework')
import ast
import rospy,rosparam
from std_msgs.msg import String
from sensor_msgs.msg import Image
from threading import Thread
from time import time,sleep
import signal
from rehabilitation_framework.msg import * 
import random
from StreamReader import USBCamReader, OpenCVReader
from EncouragerUnit import EncouragerUnit



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

    def exit_gracefully(self):
	print "Captured signal, beginning graceful shutdown."
        if self._session_timer != None:
            self._session_timer.kill_timer()
        if isinstance(self._video_reader, OpenCVReader):
            self._video_reader.kill_video_reader()

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
        self._session_timer = None
        temp_quant_freq = 0
        temp_quali_freq = 0
        temp_emotional_feedbacks = []

        # define behaviour when SIGINT or SIGTERM received
        signal.signal(signal.SIGINT, self.exit_gracefully)
        signal.signal(signal.SIGTERM, self.exit_gracefully)

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

        self._video_reader = USBCamReader(rgb_colors, camera_resolution, self._calibration_points_left_arm, self._tolerance_x, self._tolerance_y)
        #self._video_reader.start()
        self._encourager = EncouragerUnit(self._number_of_blocks, self.repetitions_limit, quantitative_frequency=temp_quant_freq, qualitative_frequency=temp_quali_freq, emotional_feedbacks=temp_emotional_feedbacks)
        self._rate = rospy.Rate(30) # 30hz
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
        self._session_timer = None
        if self.time_limit > 0:
            self._session_timer = Timer(self.time_limit)
            self._session_timer.start()
        while current_block <= self._number_of_blocks or self.time_limit > 0 and self._session_timer.is_alive():
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
                                self._encourager.say("Congratulations! You have completed all of the blocks.")
                                sleep(2)
                                break
                            current_block += 1
                            
            # check termination conditions
            if self.time_limit > 0 and not self._session_timer.is_alive() or not self._video_reader.is_alive():
                break
            self._rate.sleep()

        # kill video reader and timer threads
        if isinstance(self._video_reader, OpenCVReader) and self._video_reader.is_alive():
            self._video_reader.set_kill_thread()
            self._video_reader.join()
            print "Video reader terminated!"
        if self.time_limit > 0:
            if self._session_timer.is_alive():
                self._session_timer.kill_timer()
                self._session_timer.join()
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
        if isinstance(self._video_reader, OpenCVReader) and not (self._video_reader.is_alive()):
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

            # (re-)initialize timer if necessary
            if self._session_timer == None and no_center_found_counter == 0:
                self._session_timer = Timer(self.calibration_duration)
                self._session_timer.start()
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
                if self._session_timer != None and self._session_timer.is_alive() == False:
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
                    self._session_timer = None
            elif no_center_found_counter < NO_CENTER_FOUND_MAX and not no_center_found_flag:
                no_center_found_counter += 1
            #elif no_center_found_counter == NO_CENTER_FOUND_MAX and timer != None and timer.is_alive() and not no_center_found_flag:
            elif no_center_found_counter == NO_CENTER_FOUND_MAX and not no_center_found_flag:
                no_center_found_flag = True
                self._encourager.say("I cannot find your object. Please move it closer to the camera.")
                if self._session_timer != None:
                    self._session_timer.kill_timer()
                    self._session_timer.join()
                self._session_timer = None


            if isinstance(self._video_reader, OpenCVReader) and len(self._calibration_points_right_arm) == number_of_calibration_points:
                rospy.loginfo("Shutting down video reader...")
                self._video_reader.set_kill_thread()
                self._video_reader.join()
            if isinstance(self._video_reader, OpenCVReader) and not self._video_reader.is_alive():
                if self._session_timer != None:
                    rospy.loginfo("Shutting down timer thread...")
                    self._session_timer.kill_timer()
                    self._session_timer.join()
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
