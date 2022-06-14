#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
Driver Drowsiness Detection
Copyright ©2022 CONIGITAL LICENSE
@version ...
@author <Mohammad.Abadi@conigital.com>
"""

import sys
import os
import imutils
import rospy
from sensor_msgs.msg import Image
from std_msgs.msg import Float32
import cv2
import numpy as np
from pygame import mixer
import time
# from cv_bridge import CvBridge, CvBridgeError

file_dir = os.path.dirname(__file__)
sys.path.append(file_dir)
from drowsiness_detection import DrowsinessDetection


class DMS:
    def __init__(self, ImageTopic, CarTopic):

        script_directory = os.path.dirname(os.path.abspath(__file__))
        self._media_path = script_directory + "/../media/"
        self.eye_close_path = self._media_path + "eyes_close.mp3"
        self.eye_open_path = self._media_path + "eyes_open.mp3"
        self.drive_path = self._media_path + "drive.mp3"

        self.eye_close_tuning = None
        self.speech = False
        self.eye_threshold = None
        self.head_position = None
        self.Yawn_Drowsiness = None
        self.Eye_Drowsiness = None
        self.roll = None
        self.yaw = None
        self.pitch = None
        self.fps = None
        self.frame = None
        self.drowsiness = None
        self.eye_tune = False
        self.counter = 0
        self.prev_frame_time = 0
        self.new_frame_time = 0
        # self.bridge = CvBridge()

        self.t_end_open = time.time() + 5
        self.t_end_close = time.time() + 5

        rospy.loginfo("Subscribing to '%s' for Streaming the Camera ...", ImageTopic)
        self.camera_sub = rospy.Subscriber(ImageTopic, Image, self.image_callback, queue_size=1)

        rospy.loginfo("Subscribing to '%s' for Car Situation ...", CarTopic)
        self.car_sub = rospy.Subscriber(CarTopic, Float32, self.car_situation_callback, queue_size=1)

    def imgmsg_to_cv2(self, img_msg):
        if img_msg.encoding != "bgr8":
            rospy.logerr(
                "This Coral detect node has been hardcoded to the 'bgr8' encoding."
                "  Come change the code if you're actually trying to implement a new camera")
        dtype = np.dtype("uint8")  # Hardcode to 8 bits...
        dtype = dtype.newbyteorder('>' if img_msg.is_bigendian else '<')
        image_opencv = np.ndarray(shape=(img_msg.height, img_msg.width, 3),
                                  # and three channels of data. Since OpenCV works with bgr natively, we don't need to reorder the channels.
                                  dtype=dtype, buffer=img_msg.data)
        # If the byt order is different between the message and the system.
        if img_msg.is_bigendian == (sys.byteorder == 'little'):
            image_opencv = image_opencv.byteswap().newbyteorder()
        return image_opencv

    def image_callback(self, img_msg):

        self.frame = self.imgmsg_to_cv2(img_msg)

        return

    def car_situation_callback(self, msg):

        pass

    def show_frame(self):
        # img = imutils.resize(img, width=400)
        cv2.imshow('image', self.frame)
        k = cv2.waitKey(1) & 0xFF

    def tune_eye_close(self):
        mixer.init()
        print("[INFO] Please Keep Your Eyes Open for Five Seconds ...")
        # rospy.loginfo("Please Keep Your Eyes Open for Five Seconds ...")
        mixer.music.load(self.eye_open_path)
        mixer.music.play()
        while mixer.music.get_busy():  # wait for music to finish playing
            time.sleep(1)

        t_end = time.time() + 5
        while time.time() < t_end:
            eye_open = self.drowsiness.EyeCloseTuning(self.frame, 'open')

        eye_open_ave = sum(eye_open) / len(eye_open)
        # print('Open Average: ', eye_open_ave)

        print("[INFO] Thank You, Now Please Keep Close Your Eyes for Five Seconds ...")
        # rospy.loginfo("Thank You, Now Please Keep Close Your Eyes for Five Seconds ...")
        mixer.music.load(self.eye_close_path)
        mixer.music.play()
        while mixer.music.get_busy():  # wait for music to finish playing
            time.sleep(1)

        t_end = time.time() + 5
        while time.time() < t_end:
            eye_close = self.drowsiness.EyeCloseTuning(self.frame, 'close')

        print('[INFO] Thanks, Now you can drive ...')
        # rospy.loginfo("Thanks, Now you can drive ...")
        mixer.music.load(self.drive_path)
        mixer.music.play()
        while mixer.music.get_busy():  # wait for music to finish playing
            time.sleep(1)

        eye_close_ave = sum(eye_close) / len(eye_close)
        # print('Close Average: ', eye_close_ave)

        eye_threshold = (eye_open_ave - eye_close_ave) * 0.65 + eye_close_ave
        # print('Eye Close Threshold is: ', eye_threshold)

        return eye_threshold, True

    def detect_drowsiness(self):
        while not rospy.is_shutdown():

            if not self.frame is None:

                self.frame = imutils.resize(self.frame, width=400)

                if not self.eye_tune:
                    # if self.counter == 0:
                    self.drowsiness = DrowsinessDetection(self.frame)
                    # self.eye_close_tuning = EyeCloseTune(self.frame)
                    # self.counter = self.counter + 1
                    # self.eye_threshold, self.eye_tune = self.eye_close_tuning.tune_eye_close()
                    self.eye_threshold, self.eye_tune = self.tune_eye_close()
                    self.drowsiness = DrowsinessDetection(self.frame, self.eye_threshold)
                elif self.eye_tune:
                    self.Eye_Drowsiness, self.Yawn_Drowsiness, self.head_position = \
                        self.drowsiness.DrowsinessDetector(self.frame)

            else:
                rospy.logwarn("Waiting for the image...")

        cv2.destroyAllWindows()


if __name__ == "__main__":
    rospy.init_node('image_listener', anonymous=True)
    image_topic = rospy.get_param("~ImageTopic", "/center_cam")
    car_topic = rospy.get_param("~car_topic", "/car_situation")
    dms = DMS(image_topic, car_topic)
    dms.detect_drowsiness()
