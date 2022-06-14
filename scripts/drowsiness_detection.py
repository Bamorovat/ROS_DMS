#!/usr/bin/env python3
# -*- coding: utf-8 -*-

from scipy.spatial import distance as dist
from imutils import face_utils
import imutils
import time
import dlib
import cv2
import os
from pygame import mixer
from pose_estimator import PoseEstimator


class DrowsinessDetection:

    def __init__(self, frame, eye_threshold=0.22):
        # Add paths
        script_directory = os.path.dirname(os.path.abspath(__file__))
        self._model_path = script_directory + "/../model/"
        self._media_path = script_directory + "/../media/"
        self.Face_Landmark_Model_Path = self._model_path + "shape_predictor_68_face_landmarks.dat"
        self.Pose_Model_Path = self._model_path + "model.txt"
        self.alarm_path = self._media_path + "alarm.wav"
        self.attention_alarm_path = self._media_path + "Attention.mp3"

        self.Eye = None
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

        self.EYE_AR_THRESH = eye_threshold
        self.EYE_AR_CONSEC_FRAMES = 15
        self.MOUTH_THRESH = 0.44
        self.Yawn_CONSEC_FRAMES = 15
        self.HEAD_POSE_RIGHT_THRESH = 11
        self.HEAD_POSE_LEFT_THRESH = 20
        self.HEAD_POSE_DOWN_THRESH = 15
        self.HEAD_POSE_UP_THRESH = 13
        self.HEAD_CONSEC_FRAMES = 25

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

        self.Eye_Open = []
        self.Eye_Close = []

        # font which we will be using to display FPS
        self.font = cv2.FONT_HERSHEY_SIMPLEX

        # initialize dlib's face detector (HOG-based) and then create the facial landmark predictor
        print("[INFO] loading facial landmark predictor...")
        self.detector = dlib.get_frontal_face_detector()
        self.predictor = dlib.shape_predictor(self.Face_Landmark_Model_Path)  # args["shape_predictor"]

        # height, width, number of channels in image
        self.image_height = self.frame.shape[0]
        self.image_width = self.frame.shape[1]

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
        fps = 1 / (self.new_frame_time - self.prev_frame_time)
        self.prev_frame_time = self.new_frame_time
        fps = str(round(fps, 2))

        return fps

    def eye_aspect_ratio(self, eye):
        # compute the euclidean distances between the two sets of vertical eye landmarks (x, y)-coordinates
        A = dist.euclidean(eye[1], eye[5])
        B = dist.euclidean(eye[2], eye[4])
        C = dist.euclidean(eye[0], eye[3])
        # compute the eye aspect ratio
        blink = (A + B) / (2 * C)

        return blink

    def lip_aspect_ratio(self, lip):
        # compute the euclidean distances between the two sets of vertical eye landmarks (x, y)-coordinates
        A = dist.euclidean(lip[1], lip[7])
        B = dist.euclidean(lip[2], lip[6])
        C = dist.euclidean(lip[3], lip[5])
        D = dist.euclidean(lip[0], lip[4])
        # compute the eye aspect ratio
        yawn = (A + B + C) / (2.0 * D)

        return yawn

    def head_pose(self):
        x, y = self.pitch, self.yaw
        x = x - self.camera_relative_x
        y = y - self.camera_relative_y

        if y < -self.HEAD_POSE_RIGHT_THRESH:
            self.head_position = 'Right'
        elif y > self.HEAD_POSE_LEFT_THRESH:
            self.head_position = 'Left'
        elif x < -self.HEAD_POSE_DOWN_THRESH:
            self.head_position = 'Downward'
        elif x > self.HEAD_POSE_UP_THRESH:
            self.head_position = 'Upward'
        else:
            self.head_position = 'Forward'

    def DrowsinessDetector(self, frame):

        self.frame = frame
        gray = cv2.cvtColor(self.frame, cv2.COLOR_BGR2GRAY)
        rects = self.detector(gray, 0)

        # loop over the face detections
        for rect in rects:
            shape = self.predictor(gray, rect)
            shape = face_utils.shape_to_np(shape)

            shape1 = shape.astype('float32')

            # Try pose estimation with 68 points.
            pose = self.pose_estimator.solve_pose_by_68_points(shape1)

            # Show the head axes
            self.pose_estimator.draw_axes(self.frame, pose[0], pose[1])
            (self.pitch, self.yaw, self.roll) = self.pose_estimator.x_y_z_axes(pose[0], pose[1])

            # Show the marks
            # mark_detector.draw_marks(frame, marks, color=(0, 255, 0))

            # Show facebox
            # mark_detector.draw_box(frame, [facebox])

            # extract the left and right eye coordinates, then use the coordinates to compute the eye aspect ratio for both eyes
            leftEye = shape[self.lStart:self.lEnd]
            rightEye = shape[self.rStart:self.rEnd]
            upper_lip = shape[self.lipStart:self.lipEnd]
            inner_lip = shape[self.innerLipStart:self.innerLipEnd]

            leftEAR = self.eye_aspect_ratio(leftEye)
            rightEAR = self.eye_aspect_ratio(rightEye)

            # mouth = lip_aspect_ratio(upper_lip)
            mouth = self.lip_aspect_ratio(inner_lip)

            # average the eye aspect ratio together for both eyes
            ear = (leftEAR + rightEAR) / 2.0

            # compute the convex hull for the left and right eye, then visualize each of the eyes
            leftEyeHull = cv2.convexHull(leftEye)
            rightEyeHull = cv2.convexHull(rightEye)
            cv2.drawContours(self.frame, [leftEyeHull], -1, (0, 255, 0), 1)
            cv2.drawContours(self.frame, [rightEyeHull], -1, (0, 255, 0), 1)

            MouthHull = cv2.convexHull(upper_lip)
            cv2.drawContours(self.frame, [MouthHull], -1, (0, 255, 0), 1)

            self.pitch = round(self.pitch, 2)
            self.yaw = round(self.yaw, 2)
            self.roll = round(self.roll, 2)

            self.head_pose()

            if ear < self.EYE_AR_THRESH:
                self.COUNTER += 1

                if self.COUNTER >= self.EYE_AR_CONSEC_FRAMES:
                    self.Eye_Drowsiness = True

            else:
                self.Eye_Drowsiness = False
                self.COUNTER = 0
                self.Eye_ALARM = False

            if mouth > self.MOUTH_THRESH:
                self.Yawn_COUNTER += 1
                if self.Yawn_COUNTER >= self.Yawn_CONSEC_FRAMES:
                    self.Yawn_Drowsiness = True
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
                mixer.music.load(self.alarm_path)
                mixer.music.play()
                # while mixer.music.get_busy():  # wait for music to finish playing
                #     time.sleep(1)

        else:
            cv2.putText(self.frame, eye_text.format("open"), (10, 30), self.font, 0.4, (200, 255, 0), 1, cv2.LINE_AA)
            self.Eye_ALARM = False

        if self.Yawn_Drowsiness:
            cv2.putText(self.frame, yawn_text.format("Yes"), (10, 45), self.font, 0.45, (0, 0, 255), 1, cv2.LINE_AA)
            cv2.putText(self.frame, "DROWSINESS ALERT!", (10, 140), self.font, 0.5, (0, 0, 255), 1, cv2.LINE_AA)

            if not self.Yawn_ALARM:
                self.Yawn_ALARM = True
                mixer.music.load(self.alarm_path)
                mixer.music.play()
                # while mixer.music.get_busy():  # wait for music to finish playing
                #     time.sleep(1)

        else:
            cv2.putText(self.frame, yawn_text.format("No"), (10, 45), self.font, 0.4, (200, 255, 0), 1, cv2.LINE_AA)
            self.Yawn_ALARM = False

        # putting the FPS count on the frame
        cv2.putText(self.frame, fps_text.format(fps), (10, 60), self.font, 0.4, (200, 255, 0), 1, cv2.LINE_AA)

        if self.driver_consideration:
                cv2.putText(self.frame, head_pose_text.format(self.head_position), (10, 75), self.font, 0.4, (0, 0, 255), 1, cv2.LINE_AA)
                cv2.putText(self.frame, "Driver Attention ALERT!", (10, 140), self.font, 0.5, (0, 0, 255), 1, cv2.LINE_AA)

                if not self.Head_ALARM:
                    self.Head_ALARM = True
                    mixer.music.load(self.attention_alarm_path)
                    mixer.music.play()
                    # while mixer.music.get_busy():  # wait for music to finish playing
                    #     time.sleep(1)

        else:
            cv2.putText(self.frame, head_pose_text.format(self.head_position), (10, 75), self.font, 0.4, (200, 255, 0), 1, cv2.LINE_AA)
            self.Head_ALARM = False

        # cv2.putText(self.frame, pitch_position_text.format(self.pitch), (10, 90), self.font, 0.4, (200, 255, 0), 1, cv2.LINE_AA)
        # cv2.putText(self.frame, yaw_position_text.format(self.yaw), (10, 105), self.font, 0.4, (200, 255, 0), 1, cv2.LINE_AA)
        # cv2.putText(self.frame, roll_position_text.format(self.roll), (10, 120), self.font, 0.4, (200, 255, 0), 1, cv2.LINE_AA)

        cv2.imshow("Frame", self.frame)
        key = cv2.waitKey(1) & 0xFF

        return self.Eye_Drowsiness, self.Yawn_Drowsiness, self.head_position

    def EyeCloseTuning(self, frame, eye):
        self.Eye = eye
        self.frame = frame
        gray = cv2.cvtColor(self.frame, cv2.COLOR_BGR2GRAY)

        rects = self.detector(gray, 0)

        for rect in rects:
            shape = self.predictor(gray, rect)
            shape = face_utils.shape_to_np(shape)
            leftEye = shape[self.lStart:self.lEnd]
            rightEye = shape[self.rStart:self.rEnd]

            leftEAR = self.eye_aspect_ratio(leftEye)
            rightEAR = self.eye_aspect_ratio(rightEye)

            ear = (leftEAR + rightEAR) / 2.0

            if self.Eye == 'open':
                self.Eye_Open.insert(0, ear)
            elif self.Eye == 'close':
                self.Eye_Close.insert(0, ear)

            leftEyeHull = cv2.convexHull(leftEye)
            rightEyeHull = cv2.convexHull(rightEye)
            cv2.drawContours(self.frame, [leftEyeHull], -1, (0, 255, 0), 1)
            cv2.drawContours(self.frame, [rightEyeHull], -1, (0, 255, 0), 1)

            self.frame = imutils.resize(self.frame, width=600)
            # show the frame
            cv2.imshow("Frame", self.frame)
            key = cv2.waitKey(1) & 0xFF

            if self.Eye == 'open':
                return self.Eye_Open
            elif self.Eye == 'close':
                return self.Eye_Close
