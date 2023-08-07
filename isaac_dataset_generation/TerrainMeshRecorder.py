import os
import sys
import datetime
import shutil

dt = datetime.datetime.now().strftime('%Y-%m-%d_%H:%M')

from omni.isaac.kit import SimulationApp

simulation_app = SimulationApp({"headless": False})

from omni.isaac.core.utils.stage import open_stage

from utils.terrain_creation import Position_TerrainCreation, Keyboard_TerrainCreation
import yaml
from omni.isaac.core import World, SimulationContext
import omni
from utils.converter import csv_to_json
from utils.terrain_utils import enable_extension
from pxr import Gf
from omni.isaac.core.utils.xforms import reset_and_set_xform_ops
from omni.isaac.core.utils.prims import get_prim_at_path
import asyncio

# load parser
print(sys.argv[1])
stream1 = open(sys.argv[1])
parser = yaml.load(stream1)

os.makedirs(parser[6]['pwd']['data_before']+'data/')


# import world stage with preconfigured action graph
stream2 = open(parser[1]['config']['terrain'])
dictionary = yaml.load(stream2)
# sc = SimulationContext

if dictionary[-1]['controller'] == 'position':
    
    
    open_stage(parser[0]['stage']['position'])
    world = World()

    # Add terrains and trajectory

    num_envs = 1
    num_per_row = 1
    env_spacing = 0.56*2

    Terrain = Position_TerrainCreation(name="TerrainCreation",
                    num_envs=num_envs,
                    num_per_row=num_per_row,
                    env_spacing=env_spacing,
                    dictionary=dictionary)

elif dictionary[-1]['controller'] == 'keyboard':

  
    open_stage(parser[0]['stage']['keyboard'])
    world = World()

    num_envs = 1
    num_per_row = 1
    env_spacing = 0.56*2
  
    Terrain = Keyboard_TerrainCreation(name="TerrainCreation", 
                             num_envs=num_envs, 
                             num_per_row=num_per_row, 
                             env_spacing=env_spacing,
                             dictionary=dictionary)


ext_manager = omni.kit.app.get_app().get_extension_manager()
ext_manager.add_path(parser[3]['extension']['path'])
enable_extension(ext_manager, "sl.sensor.camera")

# reset the world before running play
world.reset()

# Start the recording
os.system(parser[5]['record']['terminal'])

done = False
while simulation_app.is_running():

    world.step(render=True)
    if not world.is_playing():
        if not done:
            csv_to_json()
            print("csv has been converted into json.\n")
            ext_manager.set_extension_enabled_immediate("omni.kit.tool.asset_exporter", True)
            asyncio.ensure_future(Terrain.save_terrain())
            print("\t\tYou can now close the application")
            done = True


if dictionary[0]['num_of_terrains'] > 1:
    newname = parser[6]['pwd']['data_before'] + 'isaac_sequence_' + dt + '_' + dictionary[2]['terrain2']['name'] 
    os.rename(parser[6]['pwd']['data_before']+'data/', newname) 
    shutil.copytree(newname, parser[6]['pwd']['data_after'] + 'isaac_sequence_' + dt + '_' + dictionary[2]['terrain2']['name'])
else:
    newname = parser[6]['pwd']['data_before'] + 'isaac_sequence_' + dt + '_' + dictionary[1]['terrain1']['name'] 
    os.rename(parser[6]['pwd']['data_before']+'data/', newname) 
    shutil.copytree(newname, parser[6]['pwd']['data_after'] + 'isaac_sequence_' + dt + '_' + dictionary[1]['terrain1']['name'])
    
shutil.rmtree(newname)
            
simulation_app.close()


