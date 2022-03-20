#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
Driver Drowsiness Detection
Copyright ©2022 CONIGITAL LICENSE
@version ...
@author <Mohammad.Abadi@icavtech.com>
"""
# twist stamp
#

# USAGE
# python DMS.py

import sys
# sys.path is a list of absolute path strings

import imutils
import rospy
from sensor_msgs.msg import Image
# from dms_conigital.msg import drowsiness_msgs
from cv_bridge import CvBridge, CvBridgeError
from drowsiness_detection import DrowsinessDetection
# from scripts.drowsiness_detection import DrowsinessDetection
import cv2
import numpy as np


class DMS:
    def __init__(self, ImageTopic):
        self.head_position = None
        self.Yawn_Drowsiness = None
        self.Eye_Drowsiness = None
        self.roll = None
        self.yaw = None
        self.pitch = None
        self.fps = None
        self.frame = None
        self.drowsiness = None
        self.counter = 0
        self.prev_frame_time = 0
        self.new_frame_time = 0

        self.bridge = CvBridge()

        rospy.loginfo("Subscribing to '%s' for Streaming the Camera ...", ImageTopic)
        self.camera_sub = rospy.Subscriber(ImageTopic, Image, self.image_callback, queue_size=1)

    def imgmsg_to_cv2(self, img_msg):
        if img_msg.encoding != "bgr8":
            rospy.logerr("This Coral detect node has been hardcoded to the 'bgr8' encoding.  Come change the code if you're actually trying to implement a new camera")
        dtype = np.dtype("uint8") # Hardcode to 8 bits...
        dtype = dtype.newbyteorder('>' if img_msg.is_bigendian else '<')
        image_opencv = np.ndarray(shape=(img_msg.height, img_msg.width, 3), # and three channels of data. Since OpenCV works with bgr natively, we don't need to reorder the channels.
                        dtype=dtype, buffer=img_msg.data)
        # If the byt order is different between the message and the system.
        if img_msg.is_bigendian == (sys.byteorder == 'little'):
            image_opencv = image_opencv.byteswap().newbyteorder()
        return image_opencv

    def image_callback(self, img_msg):

        self.frame = self.imgmsg_to_cv2(img_msg)
        # self.show_frame()
        self.detect_drowsiness()
        return

    def show_frame(self):

        # img = imutils.resize(img, width=400)
        cv2.imshow('image', self.frame)
        k = cv2.waitKey(1) & 0xFF

    def detect_drowsiness(self):

        self.frame = imutils.resize(self.frame, width=400)
        if self.counter == 0:
            self.drowsiness = DrowsinessDetection(self.frame)
            self.counter = self.counter + 1
        self.pitch, self.yaw, self.roll, self.Eye_Drowsiness, self.Yawn_Drowsiness, self.head_position = \
            self.drowsiness.DrowsinessDetector(self.frame)

        # print(self.pitch, self.yaw, self.roll, self.Eye_Drowsiness, self.Yawn_Drowsiness, self.head_position)


if __name__ == "__main__":

    rospy.init_node('image_listener', anonymous=True)
    # image_topic = "/center_cam"
    image_topic = rospy.get_param("~ImageTopic", "/center_cam")

    dms = DMS(image_topic)
    rospy.spin()
