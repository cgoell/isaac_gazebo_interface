#!/usr/bin/env python
import rospy
import numpy as np
from isaac_ros_messages.srv import IsaacPose
from isaac_ros_messages.srv import IsaacPoseRequest
from geometry_msgs.msg import Pose


def teleport_client(msg):
    rospy.wait_for_service("teleport")
    try:
        teleport = rospy.ServiceProxy("teleport", IsaacPose)
        teleport(msg)
        return
    except rospy.ServiceException as e:
        print("Service call failed: %s" % e)

cam_pose =  Pose()
cam_pose.position.x = 90
cam_pose.position.y = 10
cam_pose.position.z = 1
cam_pose.orientation.w = 1
cam_pose.orientation.x = 0
cam_pose.orientation.y = 0
cam_pose.orientation.z = 0

teleport_msg = IsaacPoseRequest()
teleport_msg.names = ["/vessel_a/cam_boat01"]
teleport_msg.poses = [cam_pose]
teleport_client(teleport_msg)

