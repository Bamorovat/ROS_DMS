#!/usr/bin/env python3

import rospy
import cv2
from sensor_msgs.msg import Image

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

    while not rospy.is_shutdown():
        _, front_img = cam_front.read()
        front_img = cv2_to_imgmsg(front_img)
        front_pub.publish(front_img)

    cv2.destroyAllWindows()
    cam_front.release()


if __name__ == '__main__':
    try:
        imagePublisher()
    except rospy.ROSInterruptException:
        pass
