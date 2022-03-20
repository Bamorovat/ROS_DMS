#!/usr/bin/env python3
import rospy
import cv2
import numpy as np
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError


# Node to obtain call camera data. Separate I/O pipeline
rospy.loginfo('Init Cameras...')
cam_front = cv2.VideoCapture(0)
cam_front.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
cam_front.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
cam_front.set(cv2.CAP_PROP_FOURCC, int(0x47504A4D))

def cv2_to_imgmsg(cv_image):
    img_msg = Image()
    img_msg.height = cv_image.shape[0]
    img_msg.width = cv_image.shape[1]
    img_msg.encoding = "bgr8"
    img_msg.is_bigendian = 0
    img_msg.data = cv_image.tostring()
    img_msg.step = len(img_msg.data) // img_msg.height # That double line is actually integer division, not a comment
    return img_msg

def imagePublisher():
    front_pub = rospy.Publisher('/center_cam', Image, queue_size=1)
    rospy.init_node('camera_publisher', anonymous=True)
    #rate=rospy.Rate(30)#10hz
    bridge = CvBridge()

    while not rospy.is_shutdown():
        _, front_img = cam_front.read()

        # cv2.imshow('image', front_img)        # for debugging purposes
        front_img = cv2_to_imgmsg(front_img)
        # rospy.loginfo("images sent")
        # for debugging purposes, remove if cv2.imshow('imgae', img) is deleted
        k = cv2.waitKey(1) & 0xFF
        if k ==27:
            break

        front_pub.publish(front_img)

    cv2.destroyAllWindows()
    cam_front.release()


if __name__ == '__main__':
    try:
        imagePublisher()
        # print('sending image')
    except rospy.ROSInterruptException:
        pass
