#!/usr/bin/env python
import rospy
import rosparam
from rehabilitation_framework.msg import *
from rehabilitation_framework.srv import *
from std_msgs.msg import Bool
from subprocess import Popen
from Exercises import MotionType, RotationType, SimpleMotionExercise, RotationExercise

class RosServer(object):
	def __init__(self):
		# initialize properties
		self._exercise_instance = None
		self._NODE_NAME = "reha_exercise"
		
		
		# initialize subscribers and ROS node
		rospy.init_node(self._NODE_NAME, anonymous=True)
		rospy.Subscriber("exercise_init", ExerciseInit, self._exercise_init_callback)
		rospy.Service("calibrate", Calibration, self._calibrate_callback)
		rospy.Subscriber("exercise_stop", Bool, self._exercise_stop_callback)

		# spin() simply keeps python from exiting until this node is stopped
		rospy.spin()

	def _exercise_init_callback(self, data):
		if self._exercise_instance != None:
			rospy.loginfo("Another exercise instance is already running! Aborting exercise creation.")
		elif ((data.motion_type == MotionType.FLEXION or data.motion_type == MotionType.ABDUCTION) and data.rotation_type == RotationType._unused) or ((data.rotation_type == RotationType.INTERNAL or data.rotation_type == RotationType.EXTERNAL) and data.motion_type == MotionType._unused):
			# set parameters on the parameter server (TODO: some params missing still)
			rosparam.set_param_raw(self._NODE_NAME + "/motion_type", data.motion_type)
			rosparam.set_param_raw(self._NODE_NAME + "/rotation_type", data.rotation_type)
			rosparam.set_param_raw(self._NODE_NAME + "/camera_width", data.camera_width)
			rosparam.set_param_raw(self._NODE_NAME + "/camera_height", data.camera_height)
			rosparam.set_param_raw(self._NODE_NAME + "/robot_position", data.robot_position)
			#rosparam.set_param_raw(self._NODE_NAME + "/hsv_thresholds", {'max_hue': data.hsv_thresholds.max_hue, 'max_sat': data.hsv_thresholds.max_sat, 'max_value': data.hsv_thresholds.max_value, 'min_hue': data.hsv_thresholds.min_hue, 'min_sat': data.hsv_thresholds.min_hue, 'min_value': data.hsv_thresholds.min_value})
			rosparam.set_param_raw(self._NODE_NAME + "/repetitions_limit", data.repetitions)
			rosparam.set_param_raw(self._NODE_NAME + "/time_limit", data.time_limit)

			# launch exercise instance as process
			launch_params = ['roslaunch', 'reha_game', 'Exercise_Launcher.launch']
			self._exercise_instance = Popen(launch_params)
		else:
			rospy.loginfo("Received invalid exercise configuration! Aborting exercise creation.")

	def _calibrate_callback(self, req):
		rospy.loginfo("Received calibration request.")
		rospy.loginfo("Message contents:\n" + str(req))
		if self._exercise_instance != None:
			rospy.loginfo("Another exercise instance is already running! Aborting exercise creation.")
			return
		else:
			if (req.motion_type == MotionType.FLEXION or req.motion_type == MotionType.ABDUCTION) and req.rotation_type == RotationType._unused:
				rosparam.set_param_raw(self._NODE_NAME + "/motion_type", req.motion_type)
				rosparam.set_param_raw(self._NODE_NAME + "/rotation_type", 0)
			elif (req.rotation_type == RotationType.INTERNAL or req.rotation_type == RotationType.EXTERNAL) and req.motion_type == MotionType._unused:
				rosparam.set_param_raw(self._NODE_NAME + "/motion_type", 0)
				rosparam.set_param_raw(self._NODE_NAME + "/rotation_type", req.rotation_type)
			else:
				rospy.loginfo("Received invalid exercise configuration! Aborting exercise creation.")
				return

			# set parameters
			rosparam.set_param_raw(self._NODE_NAME + "/calibration_duration", req.calibration_duration)
			rosparam.set_param_raw(self._NODE_NAME + "/camera_width", req.camera_width)
			rosparam.set_param_raw(self._NODE_NAME + "/camera_height", req.camera_height)
			rosparam.set_param_raw(self._NODE_NAME + "/robot_position", req.robot_position)
			local_rgb_colors = []
			for color in req.rgb_color_list.rgb_color_list:
				local_rgb_colors.append((color.red, color.green, color.blue))
			rosparam.set_param_raw(self._NODE_NAME + "/rgb_colors", local_rgb_colors)

			# launch exercise instance as process
			launch_params = ['roslaunch', 'rehabilitation_framework', 'Exercise_Launcher.launch', 'calibration_output_file:=/tmp/temp_calib_file.clb']
			self._exercise_instance = Popen(launch_params)
			self._exercise_instance.wait()

			# TODO: idea for stopping calibration process: send signal to process and check if calibration file exists

			# retrieve created file and send contents over to GUI
			calibration_data = CalibrationResponse()
			with open("/tmp/temp_calib_file.clb", "r") as temp_calib_file:
				for line in temp_calib_file:
					if line.startswith("calibration_points_left_arm="):
						temp_list = ast.literal_eval(line[28:])
						for point in temp_list:
							new_point = CalibrationPoint(point[0], point[1])
							calibration_data.calibration_points_left_arm.append(new_point)
					elif line.startswith("calibration_points_right_arm="):
						temp_list = ast.literal_eval(line[29:])
						for point in temp_list:
							new_point = CalibrationPoint(point[0], point[1])
							calibration_data.calibration_points_right_arm.append(new_point)

			# store calibration settings in parameter server

			# clean up and return service response
			del self._exercise_instance
			self._exercise_instance = None
			return calibration_data

	def _exercise_stop_callback(self, data):
		if data == True:
			if self._exercise_instance == None:
				rospy.loginfo("Received request to finish exercise, but no exercise running! Ignoring.")
			else:
				rospy.loginfo("Received request to finish exercise, cleaning up...")
				self._exercise_instance.terminate()
        			self._exercise_instance.wait()
        			self._exercise_instance = None
				rospy.loginfo("Exercise terminated successfully!")

if __name__ == "__main__":
	server = RosServer()
