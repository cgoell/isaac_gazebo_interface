import carb
from omni.isaac.kit import SimulationApp

simulation_app = SimulationApp({"renderer": "RayTracedLighting", "headless": False})
import math
import omni
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
from gazebo_msgs.msg import ModelStates


pathtoworld = os.path.expanduser('~') + "/Desktop/Coastline_Map/Terrain/Terrain_Demo.usdc"
omni.usd.get_context().open_stage(pathtoworld, None)
simulation_app.update()
print("Loading stage...")
from omni.isaac.core.utils.stage import is_stage_loading

while is_stage_loading():
    simulation_app.update()

print("Loading Complete")

def getnames(data): 
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
        if data.name[j]=="standard_vtol":
            vtol2index=j


def callback(data):
    global flag
    if flag==0: 
        getnames(data)
    va_pose.position = data.pose[3].position  
    va_pose.orientation = data.pose[3].orientation
    vb_pose.position = data.pose[4].position
    vb_pose.orientation = data.pose[4].orientation
    vc_pose.position = data.pose[5].position
    vc_pose.orientation = data.pose[5].orientation
    vd_pose.position = data.pose[6].position
    vd_pose.orientation = data.pose[6].orientation
    ve_pose.position = data.pose[7].position
    ve_pose.orientation = data.pose[7].orientation
    vf_pose.position = data.pose[8].position
    vf_pose.orientation = data.pose[8].orientation
    vg_pose.position = data.pose[9].position
    vg_pose.orientation = data.pose[9].orientation
    vdrone_pose.position = data.pose[hexaindex].position        #drone UAV  
    vdrone_pose.orientation = data.pose[hexaindex].orientation
    vtol1_pose.position = data.pose[vtol1index].position        #drone VTOL1  
    vtol1_pose.orientation = data.pose[vtol1index].orientation
    vtol2_pose.position = data.pose[vtol2index].position        #drone VTOL2  
    vtol2_pose.orientation = data.pose[vtol2index].orientation


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
    global cam1array
    global cam2array
    vtol1_prim.set_world_pose(position=np.array([vtol1_pose.position.x, vtol1_pose.position.y, vtol1_pose.position.z]),orientation=np.array([vtol1_pose.orientation.w, vtol1_pose.orientation.x, vtol1_pose.orientation.y, vtol1_pose.orientation.z]))
    vtol2_prim.set_world_pose(position=np.array([vtol2_pose.position.x, vtol2_pose.position.y, vtol2_pose.position.z]),orientation=np.array([vtol2_pose.orientation.w, vtol2_pose.orientation.x, vtol2_pose.orientation.y, vtol2_pose.orientation.z]))
    vdrone_prim.set_world_pose(position=np.array([vdrone_pose.position.x, vdrone_pose.position.y, vdrone_pose.position.z]),orientation=np.array([vdrone_pose.orientation.w, vdrone_pose.orientation.x, vdrone_pose.orientation.y, vdrone_pose.orientation.z]))
    va_prim.set_world_pose(position=np.array([va_pose.position.x, va_pose.position.y, va_pose.position.z]),orientation=get_quaternion_from_euler(va_pose.orientation.x, va_pose.orientation.y, va_pose.orientation.z))
    vb_prim.set_world_pose(position=np.array([vb_pose.position.x, vb_pose.position.y, vb_pose.position.z]),orientation=get_quaternion_from_euler(vb_pose.orientation.x, vb_pose.orientation.y, vb_pose.orientation.z))
    vc_prim.set_world_pose(position=np.array([vc_pose.position.x, vc_pose.position.y, vc_pose.position.z]),orientation=get_quaternion_from_euler(vc_pose.orientation.x, vc_pose.orientation.y, vc_pose.orientation.z))
    vd_prim.set_world_pose(position=np.array([vd_pose.position.x, vd_pose.position.y, vd_pose.position.z]),orientation=get_quaternion_from_euler(vd_pose.orientation.x, vd_pose.orientation.y, vd_pose.orientation.z))
    ve_prim.set_world_pose(position=np.array([ve_pose.position.x, ve_pose.position.y, ve_pose.position.z]),orientation=get_quaternion_from_euler(ve_pose.orientation.x, ve_pose.orientation.y, ve_pose.orientation.z))
    vf_prim.set_world_pose(position=np.array([vf_pose.position.x, vf_pose.position.y, vf_pose.position.z]),orientation=get_quaternion_from_euler(vf_pose.orientation.x, vf_pose.orientation.y, vf_pose.orientation.z))
    vg_prim.set_world_pose(position=np.array([vg_pose.position.x, vg_pose.position.y, vg_pose.position.z]),orientation=get_quaternion_from_euler(vg_pose.orientation.x, vg_pose.orientation.y, vg_pose.orientation.z))
    cam1array = euler_from_quaternion(vtol1_pose.orientation.x, vtol1_pose.orientation.y, vtol1_pose.orientation.z, vtol1_pose.orientation.w)
    vcamera1_prim.set_local_pose(translation=np.array([0, 0, -0.5]),orientation=get_quaternion_from_euler(-cam1array[0], -cam1array[1], 0))
    cam2array = euler_from_quaternion(vtol2_pose.orientation.x, vtol2_pose.orientation.y, vtol2_pose.orientation.z, vtol2_pose.orientation.w)
    vcamera2_prim.set_local_pose(translation=np.array([0, 0, -0.5]),orientation=get_quaternion_from_euler(-cam2array[0], -cam2array[1], 0))
    
def listener():
    rospy.init_node('listener', anonymous = True)
    rospy.Subscriber("/gazebo/model_states", ModelStates, callback, queue_size = 10)


flag =0
hexaindex=0
vtol1index=0
vtol2index=0
cam1array = np.empty(3,dtype=float) 
cam2array = np.empty(3,dtype=float)
teleport_msg = IsaacPoseRequest()
vdrone_pose = Pose()
vtol1_pose = Pose()
vtol2_pose = Pose()
va_pose = Pose()
vb_pose = Pose()
vc_pose = Pose()
vd_pose = Pose()
ve_pose = Pose()
vf_pose = Pose()
vg_pose = Pose()
vcamera1_pose = Pose()
vcamera2_pose = Pose()
vdrone_prim = Robot("/World/UAVs/hexa_x_tilt","hexa")
vtol1_prim = Robot("/World/UAVs/standard_vtol1","vtol1")
vtol2_prim = Robot("/World/UAVs/standard_vtol2","vtol2")
va_prim = Robot("/World/Vessels/vessel_a","va")
vb_prim = Robot("/World/Vessels/vessel_b","vb")
vc_prim = Robot("/World/Vessels/vessel_c","vc")
vd_prim = Robot("/World/Vessels/vessel_d","vd")
ve_prim = Robot("/World/Vessels/vessel_e","ve")
vf_prim = Robot("/World/Vessels/vessel_f","vf")
vg_prim = Robot("/World/Vessels/vessel_g","vg")
vcamera1_prim = Robot("/World/UAVs/standard_vtol1/fixedwing1_cam2","vcam1")
vcamera2_prim = Robot("/World/UAVs/standard_vtol2/fixedwing2_cam2","vcam2")
listener()

while simulation_app.is_running():
    simulation_app.update()
    move()

simulation_app.close()
