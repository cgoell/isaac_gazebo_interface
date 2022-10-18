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
from geometry_msgs.msg import Quaternion
from gazebo_msgs.msg import ModelStates


pathtoworld = os.path.expanduser('~') + "/Desktop/Coastline_Map/Terrain/Terrain_Demo.usdc"
omni.usd.get_context().open_stage(pathtoworld, None)
simulation_app.update()
print("Loading stage...")
from omni.isaac.core.utils.stage import is_stage_loading

while is_stage_loading():
    simulation_app.update()

print("Loading Complete")



i=0
t1=(round(time.time() * 1000))

while simulation_app.is_running() and i<1000:
    i+=1
    simulation_app.update()

t2=(round(time.time() * 1000))
print(t2-t1)
simulation_app.close()