from abc import ABCMeta, abstractmethod
import cv2
import numpy as np
import sys, traceback
import rospy
from sensor_msgs.msg import Image
from threading import Thread
from cv_bridge import CvBridge


#                 |y=-0.1
#        D        |      A
#x=-0.1           |             x=0.1
#------------------------------------
#                 |
#       C         |      B
#                 |y=0.1


class StreamReader:
    __metaclass__ = ABCMeta
    MIN_RADIUS = 20

    def __init__(self, rgb_colors, camera_resolution=(640,480), current_calibration_points=[], tolerance_x=0, tolerance_y=0):
        # Initialize camera and get actual resolution
        self.camera_resolution = camera_resolution
        self._center = None
        self._last_valid_center = None
        self._radius = 0
        self._kill_thread = False
        self._tolerance_x = tolerance_x
        self._tolerance_y = tolerance_y
        self._frame = np.zeros((camera_resolution[0],camera_resolution[1]), dtype=np.uint8)
        self._frame_modified = np.zeros((camera_resolution[0],camera_resolution[1]), dtype=np.uint8)

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

        # initialize place holders for calibration points
        self._current_calibration_point_index = 0
        self._current_calibration_points = current_calibration_points

        # initialize publisher
        self._img_msg_pub = rospy.Publisher("/usb_cam/image_modified", Image, queue_size=5) # not sure if this works?
        self._rate = rospy.Rate(30) # 30hz

        # initialize OpenCV bridge
        self._bridge = CvBridge()

    # ****************************** property definitions for the base class ******************************
    @property
    def frame_modified(self):
        return self._frame_modified

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

    def update_calib_point_index(self):
        self._current_calibration_point_index = (self._current_calibration_point_index+1) % len(self._current_calibration_points)

    def update_calib_points(self, new_calibration_points):
        self._current_calibration_points = new_calibration_points
        self._current_calibration_point_index = 0

    def process_frame(self):
        frame_copy = self._frame.copy()

        # Blur image to remove noise
        frame_copy = cv2.GaussianBlur(frame_copy, (3, 3), 0)

        # Convert image from BGR to HSV
        frame_copy = cv2.cvtColor(frame_copy, cv2.COLOR_BGR2HSV)

        # Set pixels to white if in color range, others to black (binary bitmap)
        frame_copy = cv2.inRange(frame_copy, self._hsv_threshold_lower, self._hsv_threshold_upper)

        # Dilate image to make white blobs larger
        frame_copy = cv2.dilate(frame_copy, None, iterations = 1)

        # Find center of object using contours instead of blob detection. From:
        # http://www.pyimagesearch.com/2015/09/14/ball-tracking-with-opencv/
        contours = cv2.findContours(frame_copy, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)[-2]
        self._center = None
        self._radius = 0
        if len(contours) > 0:
            c = max(contours, key=cv2.contourArea)
            ((x, y), self._radius) = cv2.minEnclosingCircle(c)
            M = cv2.moments(c)
            if M["m00"] > 0:
                self._center = (int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"]))
                if self._radius < StreamReader.MIN_RADIUS:
                    self._center = None

        # draw calibration point bounding boxes on original image, if given
        if len(self._current_calibration_points) > 0:
            img_alpha_overlay = self._frame.copy()
            for i,point in enumerate(self._current_calibration_points):
                if i == self._current_calibration_point_index:
                    color = np.array([0,255,0])
                else:
                    color = np.array([0,0,255])
                cv2.rectangle(img_alpha_overlay, (point[0]+self._tolerance_x,point[1]+self._tolerance_y), (point[0]-self._tolerance_x,point[1]-self._tolerance_y), color, -1)
            cv2.addWeighted(img_alpha_overlay, 0.3, self._frame, 0.7, 0, self._frame)

        # draw circle arround the detected object and write number of repetitions done on frame
        if self._center != None:
            self._last_valid_center = self._center
            cv2.circle(self._frame, self._last_valid_center, int(round(self._radius)), np.array([0,255,0])) 

class OpenCVReader(StreamReader,Thread):
    def __init__(self, rgb_colors, camera_resolution=(640,480), current_calibration_points=[], tolerance_x=0, tolerance_y=0):
        Thread.__init__(self)

    def kill_video_reader(self):
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
        while not self._kill_thread:
            rval, self._frame = self._cap.read()
            #if not rval:
            #        raise Exception("Failed to get frame from capture device!")
            # process frame and apply changes to image
            self.process_frame()
            # publish modified image to ROS topic
            img_msg = self._bridge.cv2_to_imgmsg(self._frame, encoding="bgr8")
            self._frame_modified_pub.publish(img_msg)
            self._rate.sleep()

class USBCamReader(StreamReader):
    def __init__(self, rgb_colors, camera_resolution=(640,480), current_calibration_points=[], tolerance_x=0, tolerance_y=0):
        StreamReader.__init__(self, rgb_colors, camera_resolution, current_calibration_points, tolerance_x, tolerance_y)
        rospy.Subscriber("/plain/image_raw/compressed", Image, self.usbcam_img_received_callback)

    def usbcam_img_received_callback(self, data):
        self._frame = self._bridge.imgmsg_to_cv2(data, "bgr8")
        self.process_frame()
        img_msg = self._bridge.cv2_to_imgmsg(self._frame, encoding="bgr8")
        self._img_msg_pub.publish(img_msg)
