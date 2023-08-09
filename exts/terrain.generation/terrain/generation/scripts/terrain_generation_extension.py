import omni.ext
import omni.ui as ui
import carb
from omni.isaac.core import World
from .terrain_creation import Position_TerrainCreation, Keyboard_TerrainCreation
from omni.isaac.core.utils.stage import add_reference_to_stage, get_current_stage, clear_stage, open_stage
import yaml
from omni.isaac.core.prims import XFormPrim, GeometryPrim
from pxr import UsdPhysics, Sdf, Gf, PhysxSchema, UsdShade, UsdGeom
from omni.isaac.dynamic_control import _dynamic_control
import numpy as np
from .terrain_utils import *
from threading import Timer




ext_manager = omni.kit.app.get_app().get_extension_manager()
#ASSET_CAMERA_ZED_X_PATH = "/home/user/Documents/zed_isaac_sim/zed-isaac-sim/exts/sl.sensor.camera/sl/sensor/camera/data/zed_x.usd"


def log_info(msg: str):
    carb.log_info(f"{msg}")
    return

# Any class derived from `omni.ext.IExt` in top level module (defined in `python.modules` of `extension.toml`) will be
# instantiated when extension gets enabled and `on_startup(ext_id)` will be called. Later when extension gets disabled
# on_shutdown() is called.

class TerrainGeneration(omni.ext.IExt):
    # ext_id is current extension id. It can be used with extension manager to query additional information, like where
    # this extension is located on filesystem.
    
    
    def on_startup(self, ext_id):
        log_info("Startup")
        self._build_ui()

    def _build_ui(self):
        log_info("Creating UI")
        self._window = ui.Window("GPS Navigation Demo", width=300, height=300)
        with self._window.frame:
            with ui.VStack(spacing=5, height=0):
                ui.Label("Scene Elements")
                with ui.HStack(spacing=5):
                    ui.Button("Load Scene", clicked_fn=self.create_terrain)
                with ui.HStack(spacing=5):
                    ui.Button("Load cam ext", clicked_fn=self.load_camera_extension)
              
                with ui.HStack(spacing=5):
                    ui.Button("Reset Scene", clicked_fn=self.reset_scene)

                
        return
    
    def create_terrain(self):

        stream = open("/home/yohan-sl-intern/isaac/terrain_generation/exts/terrain.generation/terrain/generation/scripts/config.yaml", 'r')
        dictionary = yaml.load(stream)
    
        num_envs = 1
        num_per_row = 1
        env_spacing = 0.56*2

        # get_current_stage()

       

        print(dictionary[1])

        if dictionary[-1]['controller'] == 'keyboard':

            open_stage('/home/yohan-sl-intern/isaac/utils/KEYBOARD_exts_cameracontroller.usd')
            Keyboard_TerrainCreation(name="TerrainCreation", 
                                                    num_envs=num_envs,
                                                    num_per_row=num_per_row,
                                                    env_spacing=env_spacing,
                                                    dictionary=dictionary,
                                                ) 
            
          
        
        elif dictionary[-1]['controller'] == 'position':
             
             open_stage('/home/yohan-sl-intern/isaac/utils/POSITION_exts_cameracontroller.usd')

             Position_TerrainCreation(name="TerrainCreation", 
                                                    num_envs=num_envs,
                                                    num_per_row=num_per_row,
                                                    env_spacing=env_spacing,
                                                    dictionary=dictionary,
                                                ) 
            

        
        return 
    

    def load_camera_extension(self):

        self.enable_extension(ext_manager, "sl.sensor.camera")
        print("The camera extension has been enabled")

        return

    
    
    def disable_extension(self, ext_manager, name : str):
        if ext_manager.is_extension_enabled(name) == True:
            ext_manager.set_extension_enabled(name, False)
        
        return

       

    def enable_extension(self, ext_manager, name: str):
        if ext_manager.is_extension_enabled(name) == False:
            ext_manager.set_extension_enabled(name, True)
        
        return


    def reset_scene(self):

        self.disable_extension(ext_manager, "sl.sensor.camera")
        print("The camera extension has been disabled")
        get_current_stage()
        clear_stage()

        return


    def print_hello(self):
        log_info("hello")
        return

    def on_shutdown(self):
        log_info("Shutdown")
