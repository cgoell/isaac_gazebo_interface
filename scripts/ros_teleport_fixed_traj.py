#!/usr/bin/env python

import rospy
import numpy as np

from isaac_ros_messages.srv import IsaacPose
from isaac_ros_messages.srv import IsaacPoseRequest
from geometry_msgs.msg import Pose
from gazebo_msgs.msg import ModelStates

def teleport_client(msg):
    rospy.wait_for_service("teleport")
    try:
        teleport = rospy.ServiceProxy("teleport", IsaacPose)
        teleport(msg)
        return
    except rospy.ServiceException as e:
        print("Service call failed: %s" % e)


cone_pose = Pose()
cone_pose.position.z = 2.5
cone_pose.position.x = -14.24183
cone_pose.position.y = 42.99404

cone_pose.orientation.x = 0
cone_pose.orientation.y = 0
cone_pose.orientation.z = -0.75188
cone_pose.orientation.w = 0.6593


while (1) :    
    cone_pose.position.y -= 0.09
    #cone_pose.position.y += 0.4


    teleport_msg = IsaacPoseRequest()
    teleport_msg.names = ["/standard_vtol"]
    teleport_msg.poses = [cone_pose] 
    teleport_client(teleport_msg)
