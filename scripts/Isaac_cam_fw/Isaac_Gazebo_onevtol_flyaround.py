#!/usr/bin/env python
#This script is collecting data for of the front camera of FW1 for shoreline localization, flying over the Isaac Map in an eight

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
from std_msgs.msg import Empty
import time
from omni.isaac.core.robots import Robot
import omni.isaac.core
import os
import rospy
from isaac_ros_messages.srv import IsaacPose
from isaac_ros_messages.srv import IsaacPoseRequest
from sensor_msgs.msg import Image
from geometry_msgs.msg import Pose
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Quaternion
from gazebo_msgs.msg import ModelStates
from datetime import datetime


pathtoworld = "omniverse://128.237.74.24/Projects/Champ/Map/Coastline_Map/Map_Robots.usd"
omni.usd.get_context().open_stage(pathtoworld, None)
simulation_app.update()
print("Loading stage...")
from omni.isaac.core.utils.stage import is_stage_loading

while is_stage_loading():
    simulation_app.update()


print("Loading Complete")

path = os.path.expanduser('~') + '/Desktop/imagesonevtol/'
file = open(path + "positions.txt", "w+")
seq =0

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
    vtol1_pose.position = data.pose[vtol1index].position        #drone VTOL1  
    vtol1_pose.orientation = data.pose[vtol1index].orientation
    
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

def image_callback(msg):
    global seq, file, angvel, linacc
    seq+=1
    #counting the pictures
    print(seq)
    postosave= vcamera1front_prim.get_world_pose()
    postosave = "postion: " + str(postosave[0]) + "   picture: " + str(seq) + "   time: " + str(datetime.now()) + "\n" 
    veltosave = "angular velocity: " +"\n" + str(angvel) + "\n"
    acctosave = "linear acceleration: " + str(linacc) + "\n" + "\n"
    file.write(postosave)
    file.write(veltosave)
    file.write(acctosave)

def imu_callback(msg):
    global angvel,linacc
    angvel = msg.angular_velocity
    linvel = msg.linear_acceleration

def move():
    global cam1frontarray
    #Streaming positions from gazebo to Isaac Sim
    vtol1_prim.set_world_pose(position=np.array([vtol1_pose.position.x, vtol1_pose.position.y, vtol1_pose.position.z]),orientation=np.array([vtol1_pose.orientation.w, vtol1_pose.orientation.x, vtol1_pose.orientation.y, vtol1_pose.orientation.z]))
    
    #gimbal for front cameras
    #Fixed Wing 1
    cam1frontarray[0]= 0.70711  
    cam1frontarray[1]= 0.70711
    cam1frontarray[2]= 0
    cam1frontarray[3]=0
    vcamera1front_prim.set_world_pose(position=np.array([vtol1_pose.position.x, vtol1_pose.position.y, vtol1_pose.position.z]),orientation=cam1frontarray)
def listener():
    #Initializing Ros nodes and topics
    rospy.init_node('listener', anonymous = True)
    rospy.Subscriber("/gazebo/model_states", ModelStates, callback, queue_size = 10)
    rospy.Subscriber("/fw1cam1", Image, image_callback)
    rospy.Subscriber("/uav0/mavros/imu/data", Imu, imu_callback)

#defining global variables
flag =0
hexaindex=0
vtol1index=0
vtol2index=0
cam1frontarray=np.zeros(4,dtype=float)
angvel=np.zeros(3,dtype=float)
linacc=np.zeros(3,dtype=float)
teleport_msg = IsaacPoseRequest()
vtol1_pose = Pose()
vtol1_prim = Robot("/World/UAVs/standard_vtol1","vtol1")
vcamera1front_prim = Robot("/fixedwing1_cam1","vcam1front")
listener()

while simulation_app.is_running():
    simulation_app.update()
    move()
    
simulation_app.close()
file.close()
