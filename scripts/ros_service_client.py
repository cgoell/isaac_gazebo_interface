#!/usr/bin/env python

import rospy
import numpy as np

from isaac_ros_messages.srv import IsaacPose
from isaac_ros_messages.srv import IsaacPoseRequest
from geometry_msgs.msg import Pose
from gazebo_msgs.msg import ModelStates

def teleport_client(msg):
    #print(msg)
    rospy.wait_for_service("teleport")
    try:
        teleport = rospy.ServiceProxy("teleport", IsaacPose)
        
        teleport(msg)
        return
    except rospy.ServiceException as e:
        print("Service call failed: %s" % e)

def callback(data):
    vdrone_pose.position = data.pose[3].position        #drone  
    vdrone_pose.orientation = data.pose[3].orientation
    va_pose.position = data.pose[4].position  
    va_pose.orientation = data.pose[4].orientation
    vb_pose.position = data.pose[5].position
    vb_pose.orientation = data.pose[5].orientation
    vc_pose.position = data.pose[6].position
    vc_pose.orientation = data.pose[6].orientation
    vd_pose.position = data.pose[7].position
    vd_pose.orientation = data.pose[7].orientation
    ve_pose.position = data.pose[8].position
    ve_pose.orientation = data.pose[8].orientation
    vf_pose.position = data.pose[9].position
    vf_pose.orientation = data.pose[9].orientation
    vg_pose.position = data.pose[10].position
    vg_pose.orientation = data.pose[10].orientation


def listener():
    rospy.init_node('listener', anonymous = True)
    rospy.Subscriber("/gazebo/model_states", ModelStates, callback, queue_size = 10)

teleport_msg = IsaacPoseRequest()
vdrone_pose = Pose()
va_pose = Pose()
vb_pose = Pose()
vc_pose = Pose()
vd_pose = Pose()
ve_pose = Pose()
vf_pose = Pose()
vg_pose = Pose()
listener()

while (1) :         
    for i in range(7):
        teleport_msg.names = ["/hexa_x_tilt", "/vessel_a", "/vessel_b", "/vessel_c", "/vessel_d", "/vessel_e", "/vessel_f", "/vessel_g"]
        teleport_msg.poses = [vdrone_pose, va_pose, vb_pose, vc_pose, vd_pose, ve_pose, vf_pose, vg_pose]
        teleport_client(teleport_msg)
        
