#!/usr/bin/env python
#subscribing to camera images and creating folder with all the images, flying over Isaac Map in a grid

import os
import rospy
from sensor_msgs.msg import Image
from geometry_msgs.msg import Pose
# ROS Image message -> OpenCV2 image converter
from cv_bridge import CvBridge, CvBridgeError
# OpenCV2 for saving an image
import cv2

# Globals
x = 600.0
y = -1000.0
z = 100.0
#number for counting images
seq = 0
#saving coordiantes of previous step
x_old = 600.0
y_old=-1000.0
path = os.path.expanduser('~') + '/Desktop/images_sl/'

# Create and open poses file
file = open(path + "poses.txt", "w+")

# Instantiate CvBridge
bridge = CvBridge()

def image_callback(msg):
    global seq, x_old,y_old, file,x,y,z
    try:
        # Convert your ROS Image message to OpenCV2
        cv2_img = bridge.imgmsg_to_cv2(msg, "bgr8")
    except CvBridgeError:
        print("error")
    
    # Save your OpenCV2 image as a jpeg 
    if (x != x_old ):
        seq += 1
        # Save image
        image_name = '{:06d}.jpeg'.format(seq)
        cv2.imwrite(path+image_name, cv2_img)

        # add to the poses file
        #image which is saved is delayed and always contains the coordiantes one step before
        data_to_save = str(x_old) + "\t" + str(y_old) + "\t" + str(z) + "\t" + str(image_name) + "\n"
        print(data_to_save)
        file.write(data_to_save)
        x_old = x
        y_old=y
    rospy.sleep(0.5)    #wait for 0.5 sec

def pose_callback(msg):
    global x, y, z
    x = msg.position.x
    y = msg.position.y
    z = msg.position.z

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

