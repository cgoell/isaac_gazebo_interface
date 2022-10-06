import carb
from omni.isaac.kit import SimulationApp

simulation_app = SimulationApp({"renderer": "RayTracedLighting", "headless": False})

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

def teleport_client(msg):
    rospy.wait_for_service("teleport")
    try:
        teleport = rospy.ServiceProxy("teleport", IsaacPose)
        teleport(msg)
        return
    except rospy.ServiceException as e:
        print("Service call failed: %s" % e)

def get_quaternion_from_euler(roll, pitch, yaw):
     qx = np.sin(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) - np.cos(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
     qy = np.cos(roll/2) * np.sin(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.cos(pitch/2) * np.sin(yaw/2)
     qz = np.cos(roll/2) * np.cos(pitch/2) * np.sin(yaw/2) - np.sin(roll/2) * np.sin(pitch/2) * np.cos(yaw/2)
     qw = np.cos(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
     return [qw, qx, qy, qz]

def move():
    vdrone_prim.set_world_pose(position=np.array([vdrone_pose.position.x, vdrone_pose.position.y, vdrone_pose.position.z]),orientation=np.array([vdrone_pose.orientation.w, vdrone_pose.orientation.x, vdrone_pose.orientation.y, vdrone_pose.orientation.z]))
    va_prim.set_world_pose(position=np.array([va_pose.position.x, va_pose.position.y, va_pose.position.z]),orientation=get_quaternion_from_euler(va_pose.orientation.x, va_pose.orientation.y, va_pose.orientation.z))
    vb_prim.set_world_pose(position=np.array([vb_pose.position.x, vb_pose.position.y, vb_pose.position.z]),orientation=get_quaternion_from_euler(vb_pose.orientation.x, vb_pose.orientation.y, vb_pose.orientation.z))
    vc_prim.set_world_pose(position=np.array([vc_pose.position.x, vc_pose.position.y, vc_pose.position.z]),orientation=get_quaternion_from_euler(vc_pose.orientation.x, vc_pose.orientation.y, vc_pose.orientation.z))
    vd_prim.set_world_pose(position=np.array([vd_pose.position.x, vd_pose.position.y, vd_pose.position.z]),orientation=get_quaternion_from_euler(vd_pose.orientation.x, vd_pose.orientation.y, vd_pose.orientation.z))
    ve_prim.set_world_pose(position=np.array([ve_pose.position.x, ve_pose.position.y, ve_pose.position.z]),orientation=get_quaternion_from_euler(ve_pose.orientation.x, ve_pose.orientation.y, ve_pose.orientation.z))
    vf_prim.set_world_pose(position=np.array([vf_pose.position.x, vf_pose.position.y, vf_pose.position.z]),orientation=get_quaternion_from_euler(vf_pose.orientation.x, vf_pose.orientation.y, vf_pose.orientation.z))
    vg_prim.set_world_pose(position=np.array([vg_pose.position.x, vg_pose.position.y, vg_pose.position.z]),orientation=get_quaternion_from_euler(vg_pose.orientation.x, vg_pose.orientation.y, vg_pose.orientation.z))

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
vdrone_prim = Robot("/World/hexa_x_tilt","uav")
va_prim = Robot("/vessel_a","va")
vb_prim = Robot("/vessel_b","vb")
vc_prim = Robot("/vessel_c","vc")
vd_prim = Robot("/vessel_d","vd")
ve_prim = Robot("/vessel_e","ve")
vf_prim = Robot("/vessel_f","vf")
vg_prim = Robot("/vessel_g","vg")
listener()


while simulation_app.is_running():
    simulation_app.update()
    move()
    
    
simulation_app.close()
