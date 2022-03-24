# !/usr/bin/python
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
sys.path.append('/home/abbas/catkin_p3v4/src/dms_conigital')

import imutils
import rospy
from sensor_msgs.msg import Image
from std_msgs.msg import Float32
# from dms_conigital.msg import drowsiness_msgs
from cv_bridge import CvBridge, CvBridgeError
# from drowsiness_detection import DrowsinessDetection
from scripts.drowsiness_detection import DrowsinessDetection
import cv2
import time
from pygame import mixer

eye_close_path = "/home/abbas/catkin_p3v4/src/dms_conigital/media/eyes_close.mp3"
eye_open_path = "/home/abbas/catkin_p3v4/src/dms_conigital/media/eyes_open.mp3"
drive_path = "/home/abbas/catkin_p3v4/src/dms_conigital/media/drive.mp3"


class DMS:
    def __init__(self, ImageTopic, CarTopic):
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
        self.counter = 0
        self.prev_frame_time = 0
        self.new_frame_time = 0

        self.t_end_open = time.time() + 5
        self.t_end_close = time.time() + 5

        self.bridge = CvBridge()

        rospy.loginfo("Subscribing to '%s' for Streaming the Camera ...", ImageTopic)
        self.camera_sub = rospy.Subscriber(ImageTopic, Image, self.image_callback, queue_size=1)

        rospy.loginfo("Subscribing to '%s' for Car Situation ...", CarTopic)
        self.car_sub = rospy.Subscriber(CarTopic, Float32, self.car_situation_callback, queue_size=1)

    def image_callback(self, img_msg):

        self.frame = self.bridge.imgmsg_to_cv2(img_msg, "bgr8")

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
        mixer.music.load(eye_open_path)
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
        mixer.music.load(eye_close_path)
        mixer.music.play()
        while mixer.music.get_busy():  # wait for music to finish playing
            time.sleep(1)

        t_end = time.time() + 5
        while time.time() < t_end:
            eye_close = self.drowsiness.EyeCloseTuning(self.frame, 'close')

        print('[INFO] Thanks, Now you can drive ...')
        # rospy.loginfo("Thanks, Now you can drive ...")
        mixer.music.load(drive_path)
        mixer.music.play()
        while mixer.music.get_busy():  # wait for music to finish playing
            time.sleep(1)

        eye_close_ave = sum(eye_close) / len(eye_close)
        # print('Close Average: ', eye_close_ave)

        eye_threshold = (eye_open_ave - eye_close_ave) * 0.65 + eye_close_ave
        # print('Eye Close Threshold is: ', eye_threshold)

        return eye_threshold

    def detect_drowsiness(self):
        while not rospy.is_shutdown():

            if not self.frame is None:

                self.frame = imutils.resize(self.frame, width=400)

                if self.counter == 0:
                    self.drowsiness = DrowsinessDetection(self.frame)
                    self.counter = self.counter + 1
                    self.eye_threshold = self.tune_eye_close()
                    self.drowsiness = DrowsinessDetection(self.frame, self.eye_threshold)

                self.Eye_Drowsiness, self.Yawn_Drowsiness, self.head_position =\
                    self.drowsiness.DrowsinessDetector(self.frame)

            else:
                rospy.logwarn("Waiting for the image...")

        cv2.destroyAllWindows()


if __name__ == "__main__":
    rospy.init_node('image_listener', anonymous=True)
    # image_topic = "/center_cam"
    image_topic = rospy.get_param("~ImageTopic", "/center_cam")
    car_topic = rospy.get_param("~car_topic", "/car_situation")
    dms = DMS(image_topic, car_topic)
    dms.detect_drowsiness()
    # dms.tune_eye_close()

    # rospy.spin()
    # dms.detect_drowsiness()
    # while True:
    #     rospy.spin()
    #     dms.detect_drowsiness()
    #     # dms.tune_eye_close()

    # cv2.destroyAllWindows()
