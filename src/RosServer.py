#!/usr/bin/env python
import rospy
import rosparam
from rehabilitation_framework.msg import *
from std_msgs.msg import Bool
from subprocess import Popen
import os,ast
from Exercises import MotionType, RotationType, SimpleMotionExercise, RotationExercise

class RosServer(object):
    def __init__(self):
        # initialize properties
        self._exercise_instance = None
        self._NODE_NAME = "reha_exercise"
        
        # initialize subscribers and ROS node
        rospy.init_node(self._NODE_NAME, anonymous=True)
        rospy.Subscriber("exercise_init", ExerciseInit, self._exercise_init_callback)
        rospy.Subscriber("calibration_request", CalibrationRequest, self._calibration_request_callback)
        rospy.Subscriber("exercise_stop", Bool, self._exercise_stop_callback)
        self._calibration_reply_pub = rospy.Publisher("calibration_reply", CalibrationReply, queue_size=1)

        # spin() simply keeps python from exiting until this node is stopped
        rospy.loginfo("ROS Server initialized, listening for messages now...")
        rospy.spin()

    def _exercise_init_callback(self, data):
        rospy.loginfo("Received exercise request.")
        rospy.loginfo("Message contents:\n" + str(data))
        if self._exercise_instance != None:
            rospy.loginfo("Another exercise instance is already running! Aborting exercise creation.")
        elif ((data.motion_type == MotionType.FLEXION or data.motion_type == MotionType.ABDUCTION) and data.rotation_type == RotationType._unused) or ((data.rotation_type == RotationType.INTERNAL or data.rotation_type == RotationType.EXTERNAL) and data.motion_type == MotionType._unused):
            # set parameters on the parameter server 
            rosparam.set_param_raw(self._NODE_NAME + "/motion_type", data.motion_type)
            rosparam.set_param_raw(self._NODE_NAME + "/rotation_type", data.rotation_type)
            rosparam.set_param_raw(self._NODE_NAME + "/camera_width", data.camera_width)
            rosparam.set_param_raw(self._NODE_NAME + "/camera_height", data.camera_height)
            rosparam.set_param_raw(self._NODE_NAME + "/robot_position", data.robot_position)
            rosparam.set_param_raw(self._NODE_NAME + "/number_of_blocks", data.blocks)

            # create array of tuples from color data of ROS message
            rgb_colors = []
            eval_str = "["
            for color in data.rgb_colors:
                eval_str += "(" + str(color.red) + "," + str(color.green) + "," + str(color.blue) + "),"
            rgb_colors = ast.literal_eval(eval_str[:-1] + "]")
            if len(rgb_colors) == 0:
                rospy.loginfo("Received exercise request with empty colors list! Ignoring request.")
                return
            rosparam.set_param_raw(self._NODE_NAME + "/rgb_colors", rgb_colors)
            rosparam.set_param_raw(self._NODE_NAME + "/repetitions_limit", data.repetitions)
            rosparam.set_param_raw(self._NODE_NAME + "/time_limit", data.time_limit)

            # create array of calibration points from ROS message
            calibration_points_left_arm = []
            calibration_points_right_arm = []
            eval_str = "["
            for point in data.calibration_points_left_arm:
                eval_str += "(" + str(point.x) + "," + str(point.y) + "),"
            calibration_points_left_arm = ast.literal_eval(eval_str[:-1] + "]")
            eval_str = "["
            for point in data.calibration_points_right_arm:
                eval_str += "(" + str(point.x) + "," + str(point.y) + "),"
            calibration_points_right_arm = ast.literal_eval(eval_str[:-1] + "]")
            if len(calibration_points_left_arm) != len(calibration_points_right_arm) or len(calibration_points_left_arm) == 0 and len(calibration_points_right_arm) == 0:
                rospy.loginfo("Received exercise request with invalid calibration points! Ignoring request.")
                return
            rosparam.set_param_raw(self._NODE_NAME + "/calibration_points_left_arm", calibration_points_left_arm)
            rosparam.set_param_raw(self._NODE_NAME + "/calibration_points_right_arm", calibration_points_right_arm)
            rosparam.set_param_raw(self._NODE_NAME + "/quantitative_frequency", data.quantitative_frequency)
            rosparam.set_param_raw(self._NODE_NAME + "/qualitative_frequency", data.qualitative_frequency)

            # create array of emotional feedback from ROS message
            emotional_feedback_list = []
            eval_str = "["
            for ef in data.emotional_feedback_list:
                eval_str += "(" + str(ef.is_fixed_feedback) + "," + str(ef.repetitions) + "," + str(ef.face_to_show) + "),"

            # check if emotional feedback list is empty
            if eval_str == "[":
                emotional_feedback_list = [] 
            else:
                emotional_feedback_list = ast.literal_eval(eval_str[:-1] + "]")
                rosparam.set_param_raw(self._NODE_NAME + "/emotional_feedback_list", data.emotional_feedback_list)

            # launch exercise instance as process
            launch_params = ['roslaunch', 'rehabilitation_framework', 'Exercise_Launcher.launch']
            self._exercise_instance = Popen(launch_params)
        else:
            rospy.loginfo("Received invalid exercise configuration! Aborting exercise creation.")

    def _calibration_request_callback(self, req):
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
            for color in req.rgb_color_list:
                local_rgb_colors.append((color.red, color.green, color.blue))
            rosparam.set_param_raw(self._NODE_NAME + "/rgb_colors", local_rgb_colors)

            # launch exercise instance as process
            launch_params = ['roslaunch', 'rehabilitation_framework', 'Exercise_Launcher.launch', 'calibration_output_file:=/tmp/temp_calib_file.clb']
            self._exercise_instance = Popen(launch_params)
            self._exercise_instance.wait()

            # retrieve calibration file written by exercise subprocess and send data to GUI client
            calibration_reply = CalibrationReply()
            if os.path.exists("/tmp/temp_calib_file.clb"):
                # retrieve created file and send contents over to GUI
                left_arm_points = []
                right_arm_points = []
                with open("/tmp/temp_calib_file.clb", "r") as temp_calib_file:
                    for line in temp_calib_file:
                        if line.startswith("calibration_points_left_arm="):
                            temp_list = ast.literal_eval(line[28:])
                            for point in temp_list:
                                new_point = CalibrationPoint(point[0], point[1])
                                left_arm_points.append(new_point)
                        elif line.startswith("calibration_points_right_arm="):
                            temp_list = ast.literal_eval(line[29:])
                            for point in temp_list:
                                new_point = CalibrationPoint(point[0], point[1])
                                right_arm_points.append(new_point)
                calibration_reply.status = 0
                calibration_reply.calibration_points_left_arm = left_arm_points
                calibration_reply.calibration_points_right_arm = right_arm_points
                os.remove("/tmp/temp_calib_file.clb")
                # store calibration settings in parameter server
            else:
                rospy.loginfo("Calibration process was interrupted! Unable to record calibration data...")
                calibration_reply.status = 1
                calibration_reply.calibration_points_left_arm = []
                calibration_reply.calibration_points_right_arm = []

            # clean up and return service response
            self._calibration_reply_pub.publish(calibration_reply)
            del self._exercise_instance
            self._exercise_instance = None

    def _exercise_stop_callback(self, data):
        # data=True: exercise, data=False: calibration only
        if data == True:
            if self._exercise_instance == None:
                rospy.loginfo("Received request to finish exercise, but no exercise running! Ignoring.")
            else:
                rospy.loginfo("Received request to finish exercise, cleaning up...")
                self._exercise_instance.terminate()
                self._exercise_instance.wait()
                self._exercise_instance = None
                rospy.loginfo("Exercise terminated successfully!")
        else:
            if self._exercise_instance == None:
                rospy.loginfo("Received request to stop calibration, but no calibration running! Ignoring.")
            else:
                rospy.loginfo("Received request to stop calibration, cleaning up...")
                self._exercise_instance.terminate()
                if os.path.exists("/tmp/temp_calib_file.clb"):
                    os.remove("/tmp/temp_calib_file.clb")


if __name__ == "__main__":
    server = RosServer()
