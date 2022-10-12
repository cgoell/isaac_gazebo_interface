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


i =-0.5
j=-0.5
k=0.5
l=0.5

while(1):
    cam_pose =  Pose()
    cam_pose.position.x = 0
    cam_pose.position.y = 0
    cam_pose.position.z = 0
    cam_pose.orientation.w = i
    cam_pose.orientation.x = j
    cam_pose.orientation.y = k
    cam_pose.orientation.z = l
    i+=0.003
    j+=0.003
    k-=0.003
    l-=0.003
    teleport_msg = IsaacPoseRequest()
    teleport_msg.names = ["/World/UAVs/standard_vtol1/fixedwing1_cam1"]
    teleport_msg.poses = [cam_pose]
    teleport_client(teleport_msg)
    time.sleep(0.1)
