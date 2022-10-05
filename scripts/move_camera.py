#!/usr/bin/env python
#from re import I
import rospy
import numpy as np
from isaac_ros_messages.srv import IsaacPose
from isaac_ros_messages.srv import IsaacPoseRequest
from geometry_msgs.msg import Pose
import time


def teleport_client(msg):
    rospy.wait_for_service("teleport")
    try:
        teleport = rospy.ServiceProxy("teleport", IsaacPose)
        teleport(msg)
        return
    except rospy.ServiceException as e:
        print("Service call failed: %s" % e)


i =-0.4
j=-0.34
k=0.57

while(1):
    cam_pose =  Pose()
    cam_pose.position.x = -1.67
    cam_pose.position.y = -6.9
    cam_pose.position.z = 2.94
    cam_pose.orientation.w = i
    cam_pose.orientation.x = j
    cam_pose.orientation.y = k
    cam_pose.orientation.z = 0.62
    i+=0.005
    j+=0.004
    k+=0.001
    teleport_msg = IsaacPoseRequest()
    teleport_msg.names = ["/vessel_a/cam_boat01"]
    teleport_msg.poses = [cam_pose]
    teleport_client(teleport_msg)
    time.sleep(0.1)
    
