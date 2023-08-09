import yaml
import omni
from omni.isaac.kit import SimulationApp
import numpy as np
import torch
from abc import abstractmethod
from omni.isaac.core.tasks import BaseTask
from omni.isaac.examples.base_sample import BaseSample
from omni.isaac.core.prims import RigidPrimView, RigidPrim, XFormPrim
from omni.isaac.core import World
from omni.isaac.core.objects import DynamicSphere, DynamicCuboid
from omni.isaac.core.utils.prims import define_prim, get_prim_at_path
from omni.isaac.core.utils.nucleus import find_nucleus_server
from omni.isaac.core.utils.stage import add_reference_to_stage, get_current_stage, clear_stage

from omni.isaac.core.materials import PreviewSurface
from .terrain_generation_extension import *
from omni.isaac.core.utils.xforms import reset_and_set_xform_ops
import carb
import omni.physx as _physx
import omni.graph.core as og
#from omni.isaac.core.controllers import DifferentialController
from omni.isaac.wheeled_robots.controllers.differential_controller import DifferentialController

from pxr import UsdPhysics, UsdLux, UsdShade, Sdf, Gf, UsdGeom, PhysxSchema

from .terrain_utils import *

def log_info(msg: str):
    carb.log_info(f"[sl.playground.demo:INFO] {msg}")
    return


class Keyboard_TerrainCreation(BaseTask):
    def __init__(self, name, num_envs, num_per_row, env_spacing, dictionary, config=None, offset=None,) -> None:
        BaseTask.__init__(self, name=name, offset=offset)

        self._num_envs = num_envs
        self._num_per_row = num_per_row
        self._env_spacing = env_spacing
        self._device = "cpu"
        self.dictionary = dictionary
        
        asyncio.ensure_future(self.set_up_scene(scene=None))
        
       

    async def set_up_scene(self, scene) -> None:
        self._stage = get_current_stage()
      
        j = 0

        for counter in range(self.dictionary[0]['num_of_terrains']):
            
            position = np.array(self.dictionary[counter+1]['terrain'+str(counter+1)]['position'])

            orientation = np.array(self.dictionary[counter+1]['terrain'+str(counter+1)]['orientation'])
            
            scale = np.array(self.dictionary[counter+1]['terrain'+str(counter+1)]['scale'])

            material = self.dictionary[counter+1]['terrain'+str(counter+1)]['material']
            
            self.get_terrain(self.dictionary[counter+1]['terrain'+str(counter+1)]['type'], 
                             position, 
                             orientation, 
                             self.dictionary[counter+1]['terrain'+ str(counter + 1)]['name'], 
                             self.dictionary[counter+1]['terrain' + str(counter + 1)]['width'],
                             self.dictionary[counter + 1]['terrain' + str(counter + 1)]['length'],
                             counter, 
                             scale, 
                             material = material)
            
            await uvs_coordinates("/World/Looks/"+material)
        
        start = get_prim_at_path('/World/CameraPos')
        reset_and_set_xform_ops(start, translation = (-4, -70, 3), orientation = Gf.Quatd(1,0.0,0.0,0.0))

        
        
        
        #camera position trajectory

  

        return

    def get_terrain(self, type, position, orientation, name, width, length, counter, scale, material):

        # create all available terrain types

        num_terrains = 1
        terrain_width = width
        terrain_length = length
        horizontal_scale = 0.25  # [m] #0.25
        vertical_scale = 0.005  # [m] #0.005
        num_rows = int(terrain_width/horizontal_scale)
        num_cols = int(terrain_length/horizontal_scale)
        heightfield = np.zeros((num_terrains*num_rows+2, num_cols+2), dtype=np.int16)

        def new_sub_terrain(): 
            return SubTerrain(width=num_rows, length=num_cols, vertical_scale=vertical_scale, horizontal_scale=horizontal_scale)
        
        mat = self.choose_terrain(new_sub_terrain(),type = type)
        
        heightfield[1:-1, 1:-1] =  mat
        
        vertices, triangles = convert_heightfield_to_trimesh(heightfield, horizontal_scale=horizontal_scale, vertical_scale=vertical_scale, slope_threshold=1.5) 
       
        position = position
        orientation = orientation
        add_terrain_to_stage(stage=self._stage, vertices=vertices, triangles=triangles, position=position, orientation=orientation, name=name+str(counter+1), scale=scale, material = material)
       
    
    
    def choose_terrain(self, new_sub_terrain, type):

       
        if int(type) == 0:
           
            return self.build_random_uniform(new_sub_terrain)
            

        elif int(type) == 1:
            
            return self.build_stepping_stones(new_sub_terrain)
        
        elif  int(type) == 2:
            
            return self.build_pyramid_stairs(new_sub_terrain)
        
        elif int(type) == 3:
            
            return self.build_pyramid_slopped_terrain(new_sub_terrain)
        
        elif int(type) == 4:

            return self.build_discrete_obstacles_terrain(new_sub_terrain)
        
        elif int(type) == 5:

            return self.build_wave_terrain(new_sub_terrain)
        
        elif int(type) == 6:

            return self.build_stairs_terrain(new_sub_terrain)
        
        elif int(type) == 7:

            return self.build_slopped_terrain(new_sub_terrain)
        
        elif int(type) == 8:

            return self.build_flat_terrain(new_sub_terrain)
        

    def build_random_uniform(self, new_sub_terrain):
       
       mesh = random_uniform_terrain(new_sub_terrain, min_height=0, max_height=0.35, step=0.2, downsampled_scale=0.5).height_field_raw
    
       return mesh

    def build_stepping_stones(self, new_sub_terrain):
        
        mesh = stepping_stones_terrain(new_sub_terrain, stone_size=1.,stone_distance=1., max_height=0.2, platform_size=0.).height_field_raw
        
        return mesh
    
    def build_pyramid_stairs(self, new_sub_terrain):

        mesh = pyramid_stairs_terrain(new_sub_terrain, step_width=0.75, step_height=0.5).height_field_raw

        return mesh
    
    def build_pyramid_slopped_terrain(self, new_sub_terrain):

        mesh = pyramid_sloped_terrain(new_sub_terrain, slope=-0.5).height_field_raw

        return mesh
    
    def build_discrete_obstacles_terrain(self, new_sub_terrain):

        mesh = discrete_obstacles_terrain(new_sub_terrain, max_height=0.5, min_size=1., max_size=5., num_rects=20).height_field_raw

        return mesh
    
    def build_wave_terrain(self, new_sub_terrain):

        mesh = wave_terrain(new_sub_terrain, num_waves=2., amplitude=1.).height_field_raw

        return mesh
    
    def build_stairs_terrain(self, new_sub_terrain):

        mesh = stairs_terrain(new_sub_terrain, step_width=0.75, step_height=-0.5).height_field_raw

        return mesh

    def build_slopped_terrain(self, new_sub_terrain):

        mesh = sloped_terrain(new_sub_terrain, slope=-0.5).height_field_raw

        return mesh
    
    def build_flat_terrain(self, new_sub_terrain):

        mesh = random_uniform_terrain(new_sub_terrain, min_height=0, max_height=0, step=0.2, downsampled_scale=0.5).height_field_raw

        return mesh

 


class Position_TerrainCreation(BaseTask): 
    def __init__(self, name, num_envs, num_per_row, env_spacing, dictionary, config=None, offset=None,) -> None:
        BaseTask.__init__(self, name=name, offset=offset)

        self._num_envs = num_envs
        self._num_per_row = num_per_row
        self._env_spacing = env_spacing
        self._device = "cpu"
        self.dictionary = dictionary
        
        asyncio.ensure_future(self.set_up_scene(scene=None))
        
       

    async def set_up_scene(self, scene) -> None:
        self._stage = get_current_stage()
      
        j = 0

        for counter in range(self.dictionary[0]['num_of_terrains']):

            position = np.array(self.dictionary[counter+1]['terrain'+str(counter+1)]['position'])

            orientation = np.array(self.dictionary[counter+1]['terrain'+str(counter+1)]['orientation'])
            
            scale = np.array(self.dictionary[counter+1]['terrain'+str(counter+1)]['scale'])

            material = self.dictionary[counter+1]['terrain'+str(counter+1)]['material']
            
            self.get_terrain(self.dictionary[counter+1]['terrain'+str(counter+1)]['type'], 
                             position, 
                             orientation, 
                             self.dictionary[counter+1]['terrain'+ str(counter + 1)]['name'], 
                             self.dictionary[counter+1]['terrain' + str(counter + 1)]['width'],
                             self.dictionary[counter + 1]['terrain' + str(counter + 1)]['length'],
                             counter, 
                             scale, 
                             material = material)
            
            await uvs_coordinates("/World/Looks/"+material)
        
        
        #camera position trajectory

    
        pos1 = get_prim_at_path('/World/pos1')
        pos2 = get_prim_at_path('/World/pos2')
        target = get_prim_at_path('/World/target')
        reset_and_set_xform_ops(pos1, translation = (-4, -70, 3), orientation = Gf.Quatd(1,0.0,0.0,0.0))
        reset_and_set_xform_ops(pos2, translation = (-400.40, -70, 3),  orientation = Gf.Quatd(1,0.0,0.0,0.0))
        reset_and_set_xform_ops(target, translation = (-448, -140, 0), orientation = Gf.Quatd(1,0.0,0.0,0.0))
       

        return
    
   

    def get_terrain(self, type, position, orientation, name, width, length, counter, scale, material):

        # create all available terrain types

        num_terrains = 1
        terrain_width = width
        terrain_length = length
        horizontal_scale = 0.25  # [m] #0.25
        vertical_scale = 0.005  # [m] #0.005
        num_rows = int(terrain_width/horizontal_scale)
        num_cols = int(terrain_length/horizontal_scale)
        heightfield = np.zeros((num_terrains*num_rows+2, num_cols+2), dtype=np.int16)

        def new_sub_terrain(): 
            return SubTerrain(width=num_rows, length=num_cols, vertical_scale=vertical_scale, horizontal_scale=horizontal_scale)
        
        mat = self.choose_terrain(new_sub_terrain(),type = type)
        
        heightfield[1:-1, 1:-1] =  mat
        
        vertices, triangles = convert_heightfield_to_trimesh(heightfield, horizontal_scale=horizontal_scale, vertical_scale=vertical_scale, slope_threshold=1.5) 
       
        position = position
        orientation = orientation
        add_terrain_to_stage(stage=self._stage, vertices=vertices, triangles=triangles, position=position, orientation=orientation, name=name+str(counter+1), scale=scale, material = material)
       
    
    
    def choose_terrain(self, new_sub_terrain, type):

       
        if int(type) == 0:
           
            return self.build_random_uniform(new_sub_terrain)
            

        elif int(type) == 1:
            
            return self.build_stepping_stones(new_sub_terrain)
        
        elif  int(type) == 2:
            
            return self.build_pyramid_stairs(new_sub_terrain)
        
        elif int(type) == 3:
            
            return self.build_pyramid_slopped_terrain(new_sub_terrain)
        
        elif int(type) == 4:

            return self.build_discrete_obstacles_terrain(new_sub_terrain)
        
        elif int(type) == 5:

            return self.build_wave_terrain(new_sub_terrain)
        
        elif int(type) == 6:

            return self.build_stairs_terrain(new_sub_terrain)
        
        elif int(type) == 7:

            return self.build_slopped_terrain(new_sub_terrain)
        
        elif int(type) == 8:

            return self.build_flat_terrain(new_sub_terrain)
        

    def build_random_uniform(self, new_sub_terrain):
       
       mesh = random_uniform_terrain(new_sub_terrain, min_height=0, max_height=0.35, step=0.2, downsampled_scale=0.5).height_field_raw
    
       return mesh

    def build_stepping_stones(self, new_sub_terrain):
        
        mesh = stepping_stones_terrain(new_sub_terrain, stone_size=1.,stone_distance=1., max_height=0.2, platform_size=0.).height_field_raw
        
        return mesh
    
    def build_pyramid_stairs(self, new_sub_terrain):

        mesh = pyramid_stairs_terrain(new_sub_terrain, step_width=0.75, step_height=0.5).height_field_raw

        return mesh
    
    def build_pyramid_slopped_terrain(self, new_sub_terrain):

        mesh = pyramid_sloped_terrain(new_sub_terrain, slope=-0.5).height_field_raw

        return mesh
    
    def build_discrete_obstacles_terrain(self, new_sub_terrain):

        mesh = discrete_obstacles_terrain(new_sub_terrain, max_height=0.5, min_size=1., max_size=5., num_rects=20).height_field_raw

        return mesh
    
    def build_wave_terrain(self, new_sub_terrain):

        mesh = wave_terrain(new_sub_terrain, num_waves=2., amplitude=1.).height_field_raw

        return mesh
    
    def build_stairs_terrain(self, new_sub_terrain):

        mesh = stairs_terrain(new_sub_terrain, step_width=0.75, step_height=-0.5).height_field_raw

        return mesh

    def build_slopped_terrain(self, new_sub_terrain):

        mesh = sloped_terrain(new_sub_terrain, slope=-0.5).height_field_raw

        return mesh
    
    def build_flat_terrain(self, new_sub_terrain):

        mesh = random_uniform_terrain(new_sub_terrain, min_height=0, max_height=0, step=0.2, downsampled_scale=0.5).height_field_raw

        return mesh

 

    # def setup_graph(self):

    #     keys = og.Controller.Keys
    #     og.Controller.edit(
    #         {"graph_path": "/World/CameraController", "evaluator_name": "execution"}, 
    #         {
    #             keys.CREATE_NODES: [
    #                 ("import_usd_prim_data", "omni.graph.ImportUSDPrim"),
    #                 ("transforms_to_curve", "omni.genproc.core.TransformsToCurve"),
    #                 ("curves_point_on_curve", "omni.genproc.core.PointOnCurve"),
    #                 ("constant_double", "omni.graph.nodes.ConstantDouble"),
    #                 ("divide", "omni.graph.nodes.Divide"),
    #                 ("on_tick", "omni.graph.action.OnTick"),
    #                 ("to_float", "omni.graph.nodes.ToFloat"),
    #                 ("to_double", "omni.graph.nodes.ToDouble"),
    #                 ("CameraPosWriteAttrib", "omni.graph.nodes.WritePrimAttribute"),
    #                 ("CameraOrientationWriteAttrib", "omni.graph.nodes.WritePrimAttribute"),
    #                 ("add", "omni.graph.nodes.Add"),
    #                 ("constant_double_01", "omni.graph.nodes.ConstantDouble"),
    #                 ("divide_01", "omni.graph.nodes.Divide"),
    #                 ("to_float_01", "omni.graph.nodes.ToFloat"),
    #                 ("curves_point_on_curve_01", "omni.genproc.core.PointOnCurve"),
    #                 ("to_double_01", "omni.graph.nodes.ToDouble"),
    #                 ("constant_double3", "omni.graph.nodes.ConstantDouble3"),
    #                 ("constant_double_02", "omni.graph.nodes.ConstantDouble"),
    #                 ("constant_double_03", "omni.graph.nodes.ConstantDouble"),
    #                 ("clamp", "omni.graph.nodes.Clamp"),
    #                 ("subtract", "omni.graph.nodes.Subtract"),
    #                 ("subtract_01", "omni.graph.nodes.Subtract"),
    #                 ("divide_02", "omni.graph.nodes.Divide"),
    #                 ("to_float_02", "omni.graph.nodes.ToFloat"),
    #                 ("easing_function", "omni.graph.nodes.Ease"),
    #                 ("targetReadAttrib", "omni.graph.nodes.ReadPrimAttribute"),
    #                 ("script_node_01", "omni.graph.scriptnode.ScriptNode"),
    #                 ("script_node", "omni.graph.scriptnode.ScriptNode"),
    #             ],
    #             keys.SET_VALUES: [
    #                 ("CameraPosWritteAttrib.inputs:prim", "/World/CameraPos"),
    #                 ("import_usd_prim_data.inputs:prim", "/World/pos1"),
    #                 ("import_usd_prim_data.inputs:prim", "/World/pos2"),   
    #                 ("constant_double.inputs:value", 3000.0),
    #                 ("on_tick.inputs:onlyPlayback", False),
                   
    #                 ("CameraPosWritteAttrib.inputs:name", "xformOp:translate"),
    #                 ("script_node.inputs:usePath", True),
    #                 ("script_node.inputs:scriptPath", "/home/yohan-sl-intern/isaac/script.py"),
    #                 ("CameraOrientationWriteAttrib.inputs:prim", "/World/CameraPos/CameraOrientation"),
    #                 ("constant_double_01.inputs:value", 1.0),
    #                 ("constant_double3.inputs:value", [0.0, 0.0, 1.0]),
    #                 ("constant_double_02.inputs:value", 0),
    #                 ("constant_double_03.inputs:value", 3000),
    #                 ("targetReadAttrib.inputs:prim", "/World/target"),
    #                 ("import_usd_prim_data.inputs:name", "xformOp:translate"),
    #                 ("CameraPosReadAttrib.inputs:prim", "/World/CameraPos"),
    #                 ("CameraPosReadAttrib.inputs:name", "xformOp:translate"),
    #                 ("script_node_01.inputs:usePath", True),
    #                 ("script_node_01.inputs:scriptPath", "/home/yohan-sl-intern/isaac/ground_truth_recorder.py"),
    #             ],

    #             # keys.CONNECT: [
    #             #     ("import_usd_prim_data.outputs_output", "transforms_to_curve.inputs:transformBundle"),
    #             #     ("transforms_to_curve.outputs_curveBundle", "curves_point_on_curve.inputs:curvesBundle"),
    #             #     ("transforms_to_curve.outputs_curveBundle", "curves_point_on_curve_01.inputs:curvesBundle"),
    #             #     ("curves_point_on_curve.outputs:point", "to_double.inputs:value"),
    #             #     ("curves_point_on_curve_01.outputs:point", "to_double_01.inputs:value"),
    #             #     ("curves_point_on_curve_01.outputs:point", "to_double_01.inputs:value"),
    #             #     ("on_tick.outputs:tick", "script_node_01.inputs:execIn"),
    #             #     ("on_tick.outputs:tick", "CameraPosWriteAttrib.inputs:execIn"),
    #             #     ("on_tick.outputs:frame", "divide.inputs:a"),
    #             #     ("on_tick.outputs:frame", "clamp.inputs:input"),
    #             #     ("on_tick.outputs:frame", "add.inputs:a"),
    #             #     ("divide.outputs:result", "to_float.inputs:value"),
    #             #     ("to_float.outputs:converted", "curves_point_on_curve.inputs:uValue"),
    #             #     ("constant_double.inputs:value", "divide.inputs:b"),
    #             #     ("constant_double.inputs:value", "divide_01.inputs:b"),
    #             #     ("constant_double_01.inputs:value", "add.inputs:b"),
    #             #     ("add.outputs:sum", "divide_01.inputs:a"),
    #             #     ("divide_01.outputs:result", "to_float_01.inputs:value"),
    #             #     ("to_float_01.outputs:converted", "curves_point_on_curve_01.inputs:uValue"),
    #             #     ("curves_point_on_curve_01.outputs:point", "to_double_01.inputs:value"),
    #             #     ("curves_point_on_curve.outputs:point", "to_double.inputs:value"),
    #             #     ("to_double.outputs:converted", "CameraPosWriteAttrib.inputs:value"),
    #             #     ("constant_double_02.inputs:value", "clamp.inputs:lower"),
    #             #     ("constant_double_02.inputs:value", "subtract.inputs:b"),
    #             #     ("constant_double_02.inputs:value", "subtract_01.inputs:b"),
    #             #     ("constant_double_03.inputs:value", "clamp.inputs:upper"),
    #             #     ("constant_double_03.inputs:value", "subtract_01.inputs:a"),
    #             #     ("subtract.outputs:difference", "divide_02.inputs:a"),
    #             #     ("subtract_01.outputs:difference", "divide_02.inputs:b"),
    #             #     ("divide_02.outputs:result", "to_float_02.inputs:value"),
    #             #     ("to_double_01.outputs:converted", "easing_function.inputs:start"),
    #             #     ("targetReadAttrib.outputs:value", "easing_function.inputs:end"),
    #             #     ("to_float_02.outputs:converted", "easing_function.inputs:alpha"),
    #             #     ("easing_function.outputs:result", "script_node.inputs:target"),
    #             #     ("constant_double3.inputs:value", "script_node.inputs:up"),
    #             #     ("to_double.outputs:converted", "script_node.inputs:eye"),
    #             #     ("CameraPosWriteAttrib.outputs:execOut", "script_node.inputs:execIn"),
    #             #     ("script_node.outputs:execOut", "CameraOrientationWriteAttrib.inputs:execIn"),
    #             #     ("script_node.ouptuts:quaternion", "CameraOrientationWriteAttrib.inputs:value"),

    #             # ],
    #         },


    #     )













