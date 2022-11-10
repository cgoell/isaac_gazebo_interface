#!/usr/bin/env python
#subscribing to camera images and creating folder with all the images, flying an eight with the FW over the Map
import time
import os
import rospy
from sensor_msgs.msg import Image
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Pose
# ROS Image message -> OpenCV2 image converter
from cv_bridge import CvBridge, CvBridgeError
# OpenCV2 for saving an image
import cv2


#number for counting images
seq = 0
#saving coordiantes of previous step

path = os.path.expanduser('~') + '/Desktop/imagesonevtol/'


# Instantiate CvBridge
bridge = CvBridge()
def image_callback(msg):
    global seq
    seq+=1
    print(seq)
    try:
        # Convert your ROS Image message to OpenCV2
        cv2_img = bridge.imgmsg_to_cv2(msg, "bgr8")
    except CvBridgeError:
        print("error")
    image_name = '{:06d}.jpeg'.format(seq)
    cv2.imwrite(path+image_name, cv2_img)

def listener():
    rospy.init_node('camera_sub', anonymous=True)
    rospy.Subscriber("/fw1cam1", Image, image_callback)
    rospy.spin()


if __name__ == '__main__':
    try:
        listener()
    except rospy.ROSInterruptException:
        pass

