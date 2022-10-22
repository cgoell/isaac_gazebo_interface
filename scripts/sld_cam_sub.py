#!/usr/bin/env python

import os
import rospy
from sensor_msgs.msg import Image
from geometry_msgs.msg import Pose
# ROS Image message -> OpenCV2 image converter
from cv_bridge import CvBridge, CvBridgeError
# OpenCV2 for saving an image
import cv2

x = 0
y = 0
z = 0
seq = 0
x_old = 1

# Instantiate CvBridge
bridge = CvBridge()

def image_callback(msg):
    print("Received an image!")
    try:
        # Convert your ROS Image message to OpenCV2
        cv2_img = bridge.imgmsg_to_cv2(msg, "bgr8")
    except CvBridgeError:
        print("error")
    else:
        # Save your OpenCV2 image as a jpeg 
        global seq, x_old
        seq += 1
        #print(seq)
        if (x != x_old):
            path = '/home/chris/Desktop/images_cld/'
            cv2.imwrite(path+str(seq)+'_x'+str(x)+'y'+str(y)+'z'+str(z)+'.jpeg', cv2_img)
            print("Wrote image as: "+path+str(seq)+'_x'+str(x)+'y'+str(y)+'z'+str(z)+'.jpeg')
            x_old = x
        rospy.sleep(0.5)

def pose_callback(msg):
    global x, y, z
    x = msg.position.x
    y = msg.position.y
    z = msg.position.z
    print(x)

def listener():
    rospy.init_node('camera_sub', anonymous=True)
    rospy.Subscriber("/sl_detection_cam", Image, image_callback)
    rospy.Subscriber("/sl_detection_pose", Pose, pose_callback)
    rospy.spin()

if __name__ == '__main__':
    try:
        listener()
    except rospy.ROSInterruptException:
        pass