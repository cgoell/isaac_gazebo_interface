#!/usr/bin/env python

import rospy
import numpy as np

from isaac_ros_messages.srv import IsaacPose
from isaac_ros_messages.srv import IsaacPoseRequest
from geometry_msgs.msg import Pose
from gazebo_msgs.msg import ModelStates

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
    move()

def move(): 
    vtol_prim.set_world_pose(position=vdrone_pose.position)
    va_prim.set_world_pose(position=va_pose.position)
    vb_prim.set_world_pose(position=vb_pose.position)
    vc_prim.set_world_pose(position=vc_pose.position)
    vd_prim.set_world_pose(position=vd_pose.position)
    ve_prim.set_world_pose(position=ve_pose.position)
    vf_prim.set_world_pose(position=vf_pose.position)
    vg_prim.set_world_pose(position=vg_pose.position)
    



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
vtol_prim = Robot("/standard_vtol","vtol")
va_prim = Robot("/vessel_a","va")
vb_prim = Robot("/vessel_b","vb")
vc_prim = Robot("/vessel_c","vc")
vd_prim = Robot("/vessel_d","vd")
ve_prim = Robot("/vessel_e","ve")
vf_prim = Robot("/vessel_f","vf")
vg_prim = Robot("/vessel_g","vg")
listener()

      
