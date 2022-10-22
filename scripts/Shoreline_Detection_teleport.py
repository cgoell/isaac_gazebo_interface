#!/usr/bin/env python

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



pathtoworld = os.path.expanduser('~') + "/Desktop/Coastline_Maps/Coastline_Map_cam_only/Terrain/Terrain_Demo.usdc"
omni.usd.get_context().open_stage(pathtoworld, None)
simulation_app.update()
print("Loading stage...")
from omni.isaac.core.utils.stage import is_stage_loading

while is_stage_loading():
    simulation_app.update()

print("Loading Complete")

def callback_cam1(data):
    sl_detection_cam_prim.set_world_pose(position = np.array([data.position.x, data.position.y, data.position.z]), 
                                         orientation = [data.orientation.w, data.orientation.x, data.orientation.y, data.orientation.z])
    
def listener():
    #Initializing Ros nodes and topics
    rospy.init_node('ISAAC_Gazebo', anonymous = True)
    rospy.Subscriber("/sl_detection_pose", Pose, callback_cam1, queue_size = 10)
    

vcamera1down_pose = Pose()
sl_detection_cam_prim = Robot("/sl_detection","sl_detection_cam")
listener()


while simulation_app.is_running():
    simulation_app.update()
    
simulation_app.close()