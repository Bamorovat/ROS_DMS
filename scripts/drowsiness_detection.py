#!/usr/bin/env python3
# -*- coding: utf-8 -*-

from scipy.spatial import distance as dist
from imutils.video import VideoStream
from imutils import face_utils
import playsound
from threading import Thread
import imutils
import time
import dlib
import cv2

import sys
import os
from pose_estimator import PoseEstimator

class DrowsinessDetection:

    def __init__(self, frame):
        #Add paths
        script_directory = os.path.dirname(os.path.abspath(__file__))
        self._model_path = script_directory + "/../model/"
        self._media_path = script_directory + "/../media/"
        self.Face_Landmark_Model_Path =  self._model_path + "shape_predictor_68_face_landmarks.dat"
        self.Pose_Model_Path = self._model_path + "model.txt"
        self.alarm_path = self._media_path + "alarm.wav"
        self.attention_alarm_path = self._media_path + "Attention.mp3"        
        # used to record the time when we processed last frame
        self.driver_consideration = False
        self.head_position = None
        self.camera_relative_x = 0
        self.camera_relative_y = 0
        self.frame = frame
        self.Yawn_Drowsiness = False
        self.Eye_Drowsiness = False
        self.Yawn_ALARM = False
        self.Eye_ALARM = False
        self.Head_ALARM = False

        self.prev_frame_time = 0
        self.new_frame_time = 0

        self.EYE_AR_THRESH = 0.22
        self.EYE_AR_CONSEC_FRAMES = 15
        self.MOUTH_THRESH = 0.44
        self.Yawn_CONSEC_FRAMES = 10
        self.HEAD_POSE_THRESH = 16
        self.HEAD_CONSEC_FRAMES = 60

        # initialize the frame counter as well as a boolean used to
        # indicate if the alarm is going off
        self.COUNTER = 0
        self.Yawn_COUNTER = 0
        self.head_counter = 0
        self.old_ear = 0
        self.i = 0

        self.pitch = 0.0
        self.yaw = 0.0
        self.roll = 0.0

        # font which we will be using to display FPS
        self.font = cv2.FONT_HERSHEY_SIMPLEX

        # initialize dlib's face detector (HOG-based) and then create
        # the facial landmark predictor
        print("[INFO] loading facial landmark predictor...")
        self.detector = dlib.get_frontal_face_detector()
        self.predictor = dlib.shape_predictor(self.Face_Landmark_Model_Path)  # args["shape_predictor"]

        # height, width, number of channels in image
        self.image_height = self.frame.shape[0]
        self.image_width = self.frame.shape[1]
        # self.image_height = 300
        # self.image_width = 400

        # pose estimator
        print("[INFO] loading Pose Estimator...")
        self.pose_estimator = PoseEstimator(img_size=(self.image_height, self.image_width), model_path=self.Pose_Model_Path)

        # grab the indexes of the facial landmarks for the left and right eye, respectively
        (self.lStart, self.lEnd) = face_utils.FACIAL_LANDMARKS_IDXS["left_eye"]
        (self.rStart, self.rEnd) = face_utils.FACIAL_LANDMARKS_IDXS["right_eye"]
        (self.lipStart, self.lipEnd) = face_utils.FACIAL_LANDMARKS_IDXS["mouth"]
        (self.innerLipStart, self.innerLipEnd) = face_utils.FACIAL_LANDMARKS_IDXS["inner_mouth"]

    def Calculate_Fps(self):
        # time when we finish processing for this frame
        self.new_frame_time = time.time()

        # fps will be number of frame processed in given time frame
        # since their will be most of time error of 0.001 second
        # we will be subtracting it to get more accurate result
        fps = 1 / (self.new_frame_time - self.prev_frame_time)
        self.prev_frame_time = self.new_frame_time

        # converting the fps into integer
        # fps = int(fps)

        # converting the fps to string so that we can display it on frame
        # by using putText function
        fps = str(round(fps, 2))
        # print(fps)

        return fps

    def sound_alarm(self, path):
        # play an alarm sound
        playsound.playsound(path)

    def eye_aspect_ratio(self, eye):
        # compute the euclidean distances between the two sets of
        # vertical eye landmarks (x, y)-coordinates
        A = dist.euclidean(eye[1], eye[5])
        B = dist.euclidean(eye[2], eye[4])

        # compute the euclidean distance between the horizontal
        # eye landmark (x, y)-coordinates
        C = dist.euclidean(eye[0], eye[3])

        # compute the eye aspect ratio
        blink = (A + B) / (2 * C)

        # return the eye aspect ratio
        return blink

    def lip_aspect_ratio(self, lip):
        # compute the euclidean distances between the two sets of
        # vertical eye landmarks (x, y)-coordinates
        A = dist.euclidean(lip[1], lip[7])
        B = dist.euclidean(lip[2], lip[6])
        C = dist.euclidean(lip[3], lip[5])

        # compute the euclidean distance between the horizontal
        # eye landmark (x, y)-coordinates
        D = dist.euclidean(lip[0], lip[4])

        # compute the eye aspect ratio
        yawn = (A + B + C) / (2.0 * D)

        # return the lip aspect ratio
        return yawn

    def head_pose(self):
        x, y = self.pitch, self.yaw
        x = x - self.camera_relative_x
        y = y - self.camera_relative_y

        if y < -self.HEAD_POSE_THRESH:
            self.head_position = 'Right'
        elif y > self.HEAD_POSE_THRESH:
            self.head_position = 'Left'
        elif x < -self.HEAD_POSE_THRESH:
            self.head_position = 'Downward'
        elif x > self.HEAD_POSE_THRESH:
            self.head_position = 'Upward'
        else:
            self.head_position = 'Forward'

        # print('Driver is: ', self.head_position)
        # return self.head_position

    def DrowsinessDetector(self, frame):

        self.frame = frame
        gray = cv2.cvtColor(self.frame, cv2.COLOR_BGR2GRAY)

        # cv2.imshow('image1', gray)
        # detect faces in the grayscale frame
        rects = self.detector(gray, 0)

        # loop over the face detections
        for rect in rects:
            # determine the facial landmarks for the face region, then
            # convert the facial landmark (x, y)-coordinates to a NumPy
            # array
            shape = self.predictor(gray, rect)
            shape = face_utils.shape_to_np(shape)

            # Change the facial marking type to float for pose estimator model
            shape1 = shape.astype('float32')

            # Try pose estimation with 68 points.
            pose = self.pose_estimator.solve_pose_by_68_points(shape1)

            # All done. The best way to show the result would be drawing the
            # pose on the frame in realtime.

            # Show pose annotation?
            # pose_estimator.draw_annotation_box(frame, pose[0], pose[1], color=(0, 255, 0))

            # Show the head axes?
            self.pose_estimator.draw_axes(self.frame, pose[0], pose[1])
            # pose_estimator.draw_axis(frame, pose[0], pose[1])
            (self.pitch, self.yaw, self.roll) = self.pose_estimator.x_y_z_axes(pose[0], pose[1])

            # Show the marks?
            # mark_detector.draw_marks(frame, marks, color=(0, 255, 0))

            # Show facebox?
            # mark_detector.draw_box(frame, [facebox])

            # extract the left and right eye coordinates, then use the
            # coordinates to compute the eye aspect ratio for both eyes
            leftEye = shape[self.lStart:self.lEnd]
            rightEye = shape[self.rStart:self.rEnd]
            upper_lip = shape[self.lipStart:self.lipEnd]
            inner_lip = shape[self.innerLipStart:self.innerLipEnd]

            leftEAR = self.eye_aspect_ratio(leftEye)
            rightEAR = self.eye_aspect_ratio(rightEye)

            # mouth = lip_aspect_ratio(upper_lip)
            mouth = self.lip_aspect_ratio(inner_lip)
            # print("Mouth= ", mouth)

            # average the eye aspect ratio together for both eyes
            ear = (leftEAR + rightEAR) / 2.0

            # compute the convex hull for the left and right eye, then
            # visualize each of the eyes
            leftEyeHull = cv2.convexHull(leftEye)
            rightEyeHull = cv2.convexHull(rightEye)
            cv2.drawContours(self.frame, [leftEyeHull], -1, (0, 255, 0), 1)
            cv2.drawContours(self.frame, [rightEyeHull], -1, (0, 255, 0), 1)

            MouthHull = cv2.convexHull(upper_lip)
            cv2.drawContours(self.frame, [MouthHull], -1, (0, 255, 0), 1)

            # Putting (pitch, yaw, roll) on the Frame
            self.pitch = round(self.pitch, 2)
            self.yaw = round(self.yaw, 2)
            self.roll = round(self.roll, 2)

            self.head_pose()

            # check to see if the eye aspect ratio is below the blink
            # threshold, and if so, increment the blink frame counter

            if ear < self.EYE_AR_THRESH:
                self.COUNTER += 1

                # if the eyes were closed for a sufficient number of
                # then sound the alarm
                if self.COUNTER >= self.EYE_AR_CONSEC_FRAMES:
                    # if the alarm is not on, turn it on
                    self.Eye_Drowsiness = True
                    # self.Eye_ALARM = True

            # otherwise, the eye aspect ratio is not below the blink
            # threshold, so reset the counter and alarm
            else:
                self.Eye_Drowsiness = False
                # draw an alarm on the frame
                self.COUNTER = 0
                self.Eye_ALARM = False

            # draw the computed eye aspect ratio on the frame to help
            # with debugging and setting the correct eye aspect ratio
            # thresholds and frame counters

            if mouth > self.MOUTH_THRESH:
                self.Yawn_COUNTER += 1
                if self.Yawn_COUNTER >= self.Yawn_CONSEC_FRAMES:
                    self.Yawn_Drowsiness = True
                    # self.Yawn_ALARM = True
            else:
                self.Yawn_Drowsiness = False
                self.Yawn_COUNTER = 0
                self.Yawn_ALARM = False

            if self.head_position != "Forward":
                self.head_counter += 1

                if self.head_counter >= self.HEAD_CONSEC_FRAMES:
                    self.driver_consideration = True
            else:
                self.driver_consideration = False
                self.head_counter = 0
                self.Head_ALARM = False

        # Calculating the fps
        fps = self.Calculate_Fps()

        # upscale the frame for output
        self.frame = imutils.resize(self.frame, width=600)

        # putting the INFO on the Frame
        eye_text = "[Info _ Drowsiness] Eye: {}"
        yawn_text = "[Info _ Drowsiness] Yawn: {}"
        fps_text = "[Info _ FPS]:  {}"
        head_pose_text = "[Info _ Driver] Looking {}"
        pitch_position_text = "[Info _ Position] Pitch: {}"
        yaw_position_text = "[Info _ Position] Yaw: {}"
        roll_position_text = "[Info _ Position] Roll: {}"

        if self.Eye_Drowsiness:
            cv2.putText(self.frame, eye_text.format("close"), (10, 30), self.font, 0.45, (0, 0, 255), 1, cv2.LINE_AA)
            cv2.putText(self.frame, "DROWSINESS ALERT!", (10, 140), self.font, 0.5, (0, 0, 255), 1, cv2.LINE_AA)

            if not self.Eye_ALARM:
                self.Eye_ALARM = True
                # check to see if an alarm file was supplied,
                # and if so, start a thread to have the alarm
                # sound played in the background
                t = Thread(target=self.sound_alarm, args=(self.alarm_path,))
                t.deamon = True
                t.start()

        else:
            cv2.putText(self.frame, eye_text.format("open"), (10, 30), self.font, 0.4, (200, 255, 0), 1, cv2.LINE_AA)
            self.Eye_ALARM = False

        if self.Yawn_Drowsiness:
            cv2.putText(self.frame, yawn_text.format("Yes"), (10, 45), self.font, 0.45, (0, 0, 255), 1, cv2.LINE_AA)
            cv2.putText(self.frame, "DROWSINESS ALERT!", (10, 140), self.font, 0.5, (0, 0, 255), 1, cv2.LINE_AA)

            if not self.Yawn_ALARM:
                self.Yawn_ALARM = True
                # check to see if an alarm file was supplied,
                # and if so, start a thread to have the alarm
                # sound played in the background
                t = Thread(target=self.sound_alarm, args=(self.alarm_path,))
                t.deamon = True
                t.start()

        else:
            cv2.putText(self.frame, yawn_text.format("No"), (10, 45), self.font, 0.4, (200, 255, 0), 1, cv2.LINE_AA)
            self.Yawn_ALARM = False

        # putting the FPS count on the frame
        cv2.putText(self.frame, fps_text.format(fps), (10, 60), self.font, 0.4, (200, 255, 0), 1, cv2.LINE_AA)

        if self.driver_consideration:
                cv2.putText(self.frame, head_pose_text.format(self.head_position), (10, 75), self.font,
                            0.4, (0, 0, 255), 1, cv2.LINE_AA)
                cv2.putText(self.frame, "Driver Attention ALERT!", (10, 140), self.font, 0.5, (0, 0, 255), 1, cv2.LINE_AA)

                if not self.Head_ALARM:
                    self.Head_ALARM = True
                    # check to see if an alarm file was supplied,
                    # and if so, start a thread to have the alarm
                    # sound played in the background
                    t = Thread(target=self.sound_alarm, args=(self.attention_alarm_path,))
                    t.deamon = True
                    t.start()

        else:
            cv2.putText(self.frame, head_pose_text.format(self.head_position), (10, 75), self.font,
                    0.4, (200, 255, 0), 1, cv2.LINE_AA)
            self.Head_ALARM = False

        # cv2.putText(self.frame, pitch_position_text.format(self.pitch), (10, 90), self.font,
        #             0.4, (200, 255, 0), 1, cv2.LINE_AA)
        # cv2.putText(self.frame, yaw_position_text.format(self.yaw), (10, 105), self.font,
        #             0.4, (200, 255, 0), 1, cv2.LINE_AA)
        # cv2.putText(self.frame, roll_position_text.format(self.roll), (10, 120), self.font,
        #             0.4, (200, 255, 0), 1, cv2.LINE_AA)

        # show the frame
        cv2.imshow("Frame", self.frame)
        key = cv2.waitKey(1) & 0xFF

        # # if the `q` key was pressed, break from the loop
        # if key == ord("q"):
        #     break

        # cv2.destroyAllWindows()
        # vs.stop()

        return self.pitch, self.yaw, self.roll, self.Eye_Drowsiness, self.Yawn_Drowsiness, self.head_position


def main():

    # construct the argument parse and parse the arguments
    # ap = argparse.ArgumentParser()
    # ap.add_argument("-w", "--webcam", type=int, default=0, help="index of webcam on system")
    # args = vars(ap.parse_args())

    # start the video stream thread
    print("[INFO] starting video stream thread...")
    # vs = VideoStream(src=args["webcam"]).start()
    vs = VideoStream().start()
    frame = vs.read()
    frame = imutils.resize(frame, width=400)

    drowsiness = DrowsinessDetection(frame)

    while True:
        frame = vs.read()
        frame = imutils.resize(frame, width=400)
        drowsiness.DrowsinessDetector(frame)

    cv2.destroyAllWindows()
    vs.stop()


if __name__ == '__main__':
    main()
