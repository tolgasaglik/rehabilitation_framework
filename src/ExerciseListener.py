#!/usr/bin/env python
import rospy
from reha_game.msg import *
from reha_game.srv import *
from std_msgs.msg import String
from Exercises import MotionType, RotationType, SimpleMotionExercise, RotationExercise

class ROSServer(object):
	def __init__(self):
		# initialize properties
		self._exercise_instance = None
		
		
		# initialize subscribers and ROS node
		rospy.init_node("reha_listener", anonymous=True)
		rospy.Subscriber("exercise_init", ExerciseInit, self._exercise_init_callback)
		rospy.Service("calibrate", Calibration, self._calibrate_callback)
		rospy.Subscriber("exercise_stop", bool, self._exercise_stop_callback)

		# spin() simply keeps python from exiting until this node is stopped
		rospy.spin()

	def _exercise_init_callback(self, data):
		if self._exercise_instance != None:
			rospy.loginfo("Another exercise instance is already running! Aborting exercise creation.")
		elif ((data.motion_type == MotionType.FLEXION or data.motion_type == MotionType.ABDUCTION) and data.rotation_type == RotationType._unused) or ((data.rotation_type == RotationType.INTERNAL or data.rotation_type == RotationType.EXTERNAL) and data.motion_type == MotionType._unused):
			# TODO: implement encouragement & blocks changes and change values below
			color_thresholds_tuple = ((data.hsv_thresholds.max_hue, data.hsv_thresholds.max_sat, data.hsv_thresholds.max_value), (data.hsv_thresholds.min_hue, data.hsv_thresholds.min_sat, data.hsv_thresholds.min_value))
			launch_params = ['roslaunch', 'reha_game', 'Exercise_Launcher.launch']
			launch_params.extend(('width:='+str(data.camera_width), 'height:='+str(data.camera_height), 'motion_type:='+str(data.motion_type), 'rotation_type:='+str(data.rotation_type), 'robot_position:='+str(data.robot_position), 'color_thresholds:='+str(color_thresholds_tuple), 'number_of_repetitions:='+str(data.repetitions), "time_limit:="+str(data.time_limit), 'calibration_duration:='+str(data.calibration_data.calibration_duration)))
			self._exercise_instance = Popen(launch_params)
		else:
			rospy.loginfo("Received invalid exercise configuration! Aborting exercise creation.")

	def _calibrate_callback(self, req):
		if self._exercise_instance != None:
			rospy.loginfo("Another exercise instance is already running! Aborting exercise creation.")
			return
		else:
			color_thresholds_tuple = ((req.hsv_thresholds.max_hue, req.hsv_thresholds.max_sat, req.hsv_thresholds.max_value), (req.hsv_thresholds.min_hue, req.hsv_thresholds.min_sat, req.hsv_thresholds.min_value))
			if (req.motion_type == MotionType.FLEXION or req.motion_type == MotionType.ABDUCTION) and req.rotation_type == RotationType._unused:
				self._exercise_instance = SimpleMotionExercise(camera_resolution=(req.camera_width), robot_position=req.robot_position, motion_type=req.motion_type, calibration_duration=req.calibration_duration, color_thresholds=color_thresholds_tuple)
			elif (req.rotation_type == RotationType.INTERNAL or req.rotation_type == RotationType.EXTERNAL) and req.motion_type == MotionType._unused:
				self._exercise_instance = RotationExercise(robot_position=req.robot_position, rotation_type=req.rotation_type, calibration_duration=req.calibration_duration, color_thresholds=color_thresholds_tuple)
			else:
				rospy.loginfo("Received invalid exercise configuration! Aborting exercise creation.")
				return

			# run calibration procedure of exercise instance
			self._exercise_instance.calibrate()
			self._exercise_instance.stop_game()
			
			# retrieve calibration values from exercise instance
			calibration_data = CalibrationResponse()
			for point in self._exercise_instance.calibration_points_left_arm:
				new_point = CalibrationPoint()
				new_point.x = point[0]
				new_point.y = point[1]
				calibration_data.calibration_points_left_arm.append(new_point)
			for point in self._exercise_instance.calibration_points_right_arm:
				new_point = CalibrationPoint()
				new_point.x = point[0]
				new_point.y = point[1]
				calibration_data.calibration_points_right_arm.append(new_point)
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
