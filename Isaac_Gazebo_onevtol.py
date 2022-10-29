#!/usr/bin/env python
import carb
from omni.isaac.kit import SimulationApp

simulation_app = SimulationApp({"renderer": "RayTracedLighting", "headless": False})
import math
import omni
from std_msgs.msg import Float64
from omni.isaac.core import SimulationContext
from omni.isaac.core.utils import viewports, stage, extensions, prims, rotations, nucleus
from omni.isaac.core.utils.extensions import enable_extension
from omni.isaac.core import World

enable_extension("omni.isaac.ros_bridge")
simulation_app.update()

import rosgraph

if not rosgraph.is_master_online():
    carb.log_error("Please run roscore before executing this script")
    simulation_app.close()
    exit()
    
import numpy as np
import rospy
from std_msgs.msg import Empty
import time
from omni.isaac.core.robots import Robot
import omni.isaac.core
import os
from isaac_ros_messages.srv import IsaacPose
from isaac_ros_messages.srv import IsaacPoseRequest
from geometry_msgs.msg import Pose
from geometry_msgs.msg import Quaternion
from gazebo_msgs.msg import ModelStates


pathtoworld = os.path.expanduser('~') + "/Desktop/Coastline_Map_one_VTOL/Terrain/Terrain_Demo.usdc"
omni.usd.get_context().open_stage(pathtoworld, None)
simulation_app.update()
print("Loading stage...")
from omni.isaac.core.utils.stage import is_stage_loading

while is_stage_loading():
    simulation_app.update()

print("Loading Complete")

def getnames(data): 
    #Defining the affiliations in Gazebo
    global flag
    global hexaindex
    global vtol1index
    global vtol2index
    flag =1
    for j in range(len(data.name)):
        if "hexa_x_tilt" in data.name[j]:
            hexaindex = j
        if data.name[j]=="standard_vtol_0":
            vtol1index=j
        if data.name[j]=="standard_vtol_1":
            vtol2index=j


def callback(data):
    #getting positions from Gazebo
    global flag
    if flag==0: 
        getnames(data)
    # vc_pose.position = data.pose[4].position
    # vc_pose.orientation = data.pose[4].orientation
    # vg_pose.position = data.pose[5].position
    # vg_pose.orientation = data.pose[5].orientation
    vtol1_pose.position = data.pose[vtol1index].position        #drone VTOL1  
    vtol1_pose.orientation = data.pose[vtol1index].orientation

def callback_cam1(data):
    #Implement an offset for the camera of fixed wing 1 here to emulate a gimbal
    global camoffset1
    camoffset1 = euler_from_quaternion(data.x, data.y, data.z, data.w)


def heading_cam1(data):
    #Implement an heading for the gimbal camera of fixed wing 1 between -180 and 180 degree
    global headingc1
    headingc1 = data.data
    if headingc1<0: 
        headingc1 = 360+headingc1

    
def get_quaternion_from_euler(roll, pitch, yaw):
     qx = np.sin(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) - np.cos(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
     qy = np.cos(roll/2) * np.sin(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.cos(pitch/2) * np.sin(yaw/2)
     qz = np.cos(roll/2) * np.cos(pitch/2) * np.sin(yaw/2) - np.sin(roll/2) * np.sin(pitch/2) * np.cos(yaw/2)
     qw = np.cos(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
     return [qw, qx, qy, qz]

def euler_from_quaternion(x, y, z, w):
    t0 = +2.0 * (w * x + y * z)
    t1 = +1.0 - 2.0 * (x * x + y * y)
    roll_x = math.atan2(t0, t1)
     
    t2 = +2.0 * (w * y - z * x)
    t2 = +1.0 if t2 > +1.0 else t2
    t2 = -1.0 if t2 < -1.0 else t2
    pitch_y = math.asin(t2)
     
    t3 = +2.0 * (w * z + x * y)
    t4 = +1.0 - 2.0 * (y * y + z * z)
    yaw_z = math.atan2(t3, t4)
     
    return [roll_x, pitch_y, yaw_z] 

def move():
    global cam1frontarray
    global cam2frontarray
    global camoffset1
    global headingc1
    #Streaming positions from gazebo to Isaac Sim
    vtol1_prim.set_world_pose(position=np.array([vtol1_pose.position.x, vtol1_pose.position.y, vtol1_pose.position.z+0.75]),orientation=np.array([vtol1_pose.orientation.w, vtol1_pose.orientation.x, vtol1_pose.orientation.y, vtol1_pose.orientation.z]))
    # vc_prim.set_world_pose(position=np.array([vc_pose.position.x, vc_pose.position.y, vc_pose.position.z]),orientation=get_quaternion_from_euler(vc_pose.orientation.x, vc_pose.orientation.y, vc_pose.orientation.z))
    # vg_prim.set_world_pose(position=np.array([vg_pose.position.x, vg_pose.position.y, vg_pose.position.z]),orientation=get_quaternion_from_euler(vg_pose.orientation.x, vg_pose.orientation.y, vg_pose.orientation.z))
    
    #gimbal for down cameras
    #Fixed Wing 1
    cam1downarray = euler_from_quaternion(vtol1_pose.orientation.x, vtol1_pose.orientation.y, vtol1_pose.orientation.z, vtol1_pose.orientation.w)
    vcamera1down_prim.set_local_pose(translation=np.array([0, 0, 0]),orientation=get_quaternion_from_euler(-cam1downarray[0], -cam1downarray[1], 0))

    #gimbal for front cameras
    #Fixed Wing 1
    cam1frontarray[0]= 0.5*math.pi  
    cam1frontarray[1]= 0 
    cam1frontarray[2]= 0
    if (headingc1 >= 0.0) and (headingc1 <=90.0):
        cam1frontarray[1]= (-1)*(headingc1/180) * math.pi
    elif (headingc1 > 90.0) and (headingc1 < 270.0):
        cam1frontarray[0]=-cam1frontarray[0]
        cam1frontarray[1]=((-180.0+headingc1)/180.0)*math.pi
        cam1frontarray[2]=math.pi
    else: 
        cam1frontarray[1]=((360.0-headingc1)/180.0)*math.pi
    #auxarray1 = get_quaternion_from_euler(cam1frontarray[0], cam1frontarray[1], cam1frontarray[2])
    #auxarray2 =  euler_from_quaternion(auxarray1[1],auxarray1[2],auxarray1[3],auxarray1[0])
    auxarray2=cam1frontarray
    auxarray2[0]+=camoffset1[0]
    auxarray2[1]+=camoffset1[1]
    auxarray2[2]+=camoffset1[2]
    #print(auxarray2)
    auxarray1=get_quaternion_from_euler(auxarray2[0],auxarray2[1],auxarray2[2])
    auxarray1[3]=-auxarray1[3]
    #print(auxarray1)
    vcamera1front_prim.set_world_pose(position=np.array([vtol1_pose.position.x, vtol1_pose.position.y, vtol1_pose.position.z]),orientation=auxarray1)
    
def listener():
    #Initializing Ros nodes and topics
    rospy.init_node('listener', anonymous = True)
    rospy.Subscriber("/movecamfw1", Quaternion, callback_cam1, queue_size = 10)
    rospy.Subscriber("/headingcamfw1", Float64, heading_cam1, queue_size = 10)
    rospy.Subscriber("/gazebo/model_states", ModelStates, callback, queue_size = 10)

#defining global variables
flag =0
hexaindex=0
vtol1index=0
vtol2index=0
cam1frontarray=np.zeros(3,dtype=float)
cam2frontarray=np.zeros(3,dtype=float)
camoffset1 = np.zeros(3,dtype=float)
headingc1=0
teleport_msg = IsaacPoseRequest()
vtol1_pose = Pose()
vc_pose = Pose()
vg_pose = Pose()
vtol1_prim = Robot("/World/UAVs/standard_vtol1","vtol1")
# vc_prim = Robot("/World/Vessels/vessel_c","vc")
# vg_prim = Robot("/World/Vessels/vessel_g","vg")
vcamera1down_prim = Robot("/World/UAVs/standard_vtol1/fixedwing1_cam2","vcam1down")
vcamera1front_prim = Robot("/fixedwing1_cam1","vcam1front")
listener()

while simulation_app.is_running():
    simulation_app.update()
    move()
    
simulation_app.close()
