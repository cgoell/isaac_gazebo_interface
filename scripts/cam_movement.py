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


wangle =-0.5
xangle=-0.5
yangle=0.5
zangle=0.5

while(1):
    cam_pose =  Pose()
    cam_pose.position.x = 0
    cam_pose.position.y = 0
    cam_pose.position.z = 0
    cam_pose.orientation.w = wangle
    cam_pose.orientation.x = xangle
    cam_pose.orientation.y = yangle
    cam_pose.orientation.z = zangle
    teleport_msg = IsaacPoseRequest()
    teleport_msg.names = ["/World/UAVs/standard_vtol1/fixedwing1_cam1"]
    teleport_msg.poses = [cam_pose]
    teleport_client(teleport_msg)
    time.sleep(0.1)
    if wangle ==-0.5:
        time.sleep(1)
        wangle=0.698
        xangle=0.698
        yangle=-0.1106
        zangle=-0.1106