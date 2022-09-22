from omni.isaac.kit import SimulationApp

simulation_app = SimulationApp({"headless": False})

from omni.isaac.core import World
from omni.isaac.manipulators import SingleManipulator
from omni.isaac.manipulators.grippers import ParallelGripper
from omni.isaac.core.utils.stage import add_reference_to_stage
from omni.isaac.core.utils.types import ArticulationAction
import numpy as np

my_world = World(stage_units_in_meters=1.0)
#TODO: change this to your own path
asset_path = "~/Desktop/Coastline_Map-20220906T180903Z-001/Coastline_Map/Terrain/Terrain_Demo.usdc"
#add_reference_to_stage(usd_path=asset_path, prim_path="/World/cobotta")
#second_path = "/home/simon/Documents/model/try.usd"
add_reference_to_stage(usd_path=asset_path, prim_path="/World")


i = 0
while simulation_app.is_running():
    my_world.step(render=True)
   

simulation_app.close()
