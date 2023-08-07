import numpy as np
from numpy.random import choice
from scipy import interpolate    
import random 
from cmath import pi, sqrt
import math
from math import sqrt

import omni
import omni.kit.app
from omni.isaac.core.prims import XFormPrim, GeometryPrim
from omni.isaac.core.materials import OmniPBR
from omni.isaac.core.utils.stage import add_reference_to_stage
import warnings
import asyncio
import torch



from pxr import Usd, UsdPhysics, Sdf, Gf, PhysxSchema, UsdShade, UsdGeom,  Vt



ASSET_COUNTRY_GRASS = "http://omniverse-content-production.s3-us-west-2.amazonaws.com/Materials/Base/Natural/Grass_Countryside.mdl"
ASSET_BRICK = "http://omniverse-content-production.s3-us-west-2.amazonaws.com/Materials/Base/Masonry/Brick_Wall_Red.mdl"
ASSET_SOIL = "http://omniverse-content-production.s3-us-west-2.amazonaws.com/Materials/Base/Natural/Soil_Rocky.mdl"
ASSET_SAND = "http://omniverse-content-production.s3-us-west-2.amazonaws.com/Materials/Base/Natural/Sand.mdl"
ASSET_OCTAGON = "http://omniverse-content-production.s3-us-west-2.amazonaws.com/Materials/Base/Stone/Adobe_Octagon_Dots.mdl"




def enable_extension(ext_manager, name: str):
        if ext_manager.is_extension_enabled(name) == False:
            ext_manager.set_extension_enabled(name, True)
        
        # return
def random_uniform_terrain(terrain, min_height, max_height, step=1, downsampled_scale=None,):
    """
    Generate a uniform noise terrain
    Parameters
        terrain (SubTerrain): the terrain
        min_height (float): the minimum height of the terrain [meters]
        max_height (float): the maximum height of the terrain [meters]
        step (float): minimum height change between two points [meters]
        downsampled_scale (float): distance between two randomly sampled points ( musty be larger or equal to terrain.horizontal_scale)
    """

    if downsampled_scale is None:
        downsampled_scale = terrain.horizontal_scale

    # switch parameters to discrete units
    min_height = int(min_height / terrain.vertical_scale)
    max_height = int(max_height / terrain.vertical_scale)
    step = int(step / terrain.vertical_scale)

    heights_range = np.arange(min_height, max_height + step, step)
    height_field_downsampled = np.random.choice(heights_range, (int(terrain.width * terrain.horizontal_scale / downsampled_scale), int(
        terrain.length * terrain.horizontal_scale / downsampled_scale)))

    x = np.linspace(0, terrain.width * terrain.horizontal_scale, height_field_downsampled.shape[0])
    y = np.linspace(0, terrain.length * terrain.horizontal_scale, height_field_downsampled.shape[1])

    f = interpolate.interp2d(y, x, height_field_downsampled, kind='linear')

    x_upsampled = np.linspace(0, terrain.width * terrain.horizontal_scale, terrain.width)
    y_upsampled = np.linspace(0, terrain.length * terrain.horizontal_scale, terrain.length)
    z_upsampled = np.rint(f(y_upsampled, x_upsampled))

    terrain.height_field_raw += z_upsampled.astype(np.int16)
    return terrain

def flat_terrain(terrain):

    return np.asarray(terrain)

def sloped_terrain(terrain, slope=1):
    """
    Generate a sloped terrain
    Parameters:
        terrain (SubTerrain): the terrain
        slope (int): positive or negative slope
    Returns:
        terrain (SubTerrain): update terrain
    """

    x = np.arange(0, terrain.width)
    y = np.arange(0, terrain.length)
    xx, yy = np.meshgrid(x, y, sparse=True)
    xx = xx.reshape(terrain.width, 1)
    max_height = int(slope * (terrain.horizontal_scale / terrain.vertical_scale) * terrain.width)
    terrain.height_field_raw[:, np.arange(terrain.length)] += (max_height * xx / terrain.width).astype(terrain.height_field_raw.dtype)
    return terrain

def pyramid_sloped_terrain(terrain, slope=1, platform_size=1.):
    """
    Generate a sloped terrain
    Parameters:
        terrain (terrain): the terrain
        slope (int): positive or negative slope
        platform_size (float): size of the flat platform at the center of the terrain [meters]
    Returns:
        terrain (SubTerrain): update terrain
    """
    x = np.arange(0, terrain.width)
    y = np.arange(0, terrain.length)
    center_x = int(terrain.width / 2)
    center_y = int(terrain.length / 2)
    xx, yy = np.meshgrid(x, y, sparse=True)
    xx = (center_x - np.abs(center_x-xx)) / center_x
    yy = (center_y - np.abs(center_y-yy)) / center_y
    xx = xx.reshape(terrain.width, 1)
    yy = yy.reshape(1, terrain.length)
    max_height = int(slope * (terrain.horizontal_scale / terrain.vertical_scale) * (terrain.width / 2))
    terrain.height_field_raw += (max_height * xx * yy).astype(terrain.height_field_raw.dtype)

    platform_size = int(platform_size / terrain.horizontal_scale / 2)
    x1 = terrain.width // 2 - platform_size
    x2 = terrain.width // 2 + platform_size
    y1 = terrain.length // 2 - platform_size
    y2 = terrain.length // 2 + platform_size

    min_h = min(terrain.height_field_raw[x1, y1], 0)
    max_h = max(terrain.height_field_raw[x1, y1], 0)
    terrain.height_field_raw = np.clip(terrain.height_field_raw, min_h, max_h)
    return terrain

def discrete_obstacles_terrain(terrain, max_height, min_size, max_size, num_rects, platform_size=1.):
    """
    Generate a terrain with gaps
    Parameters:
        terrain (terrain): the terrain
        max_height (float): maximum height of the obstacles (range=[-max, -max/2, max/2, max]) [meters]
        min_size (float): minimum size of a rectangle obstacle [meters]
        max_size (float): maximum size of a rectangle obstacle [meters]
        num_rects (int): number of randomly generated obstacles
        platform_size (float): size of the flat platform at the center of the terrain [meters]
    Returns:
        terrain (SubTerrain): update terrain
    """
    # switch parameters to discrete units
    max_height = int(max_height / terrain.vertical_scale)
    min_size = int(min_size / terrain.horizontal_scale)
    max_size = int(max_size / terrain.horizontal_scale)
    platform_size = int(platform_size / terrain.horizontal_scale)

    (i, j) = terrain.height_field_raw.shape
    height_range = [-max_height, -max_height // 2, max_height // 2, max_height]
    width_range = range(min_size, max_size, 4)
    length_range = range(min_size, max_size, 4)

    for _ in range(num_rects):
        width = np.random.choice(width_range)
        length = np.random.choice(length_range)
        start_i = np.random.choice(range(0, i-width, 4))
        start_j = np.random.choice(range(0, j-length, 4))
        terrain.height_field_raw[start_i:start_i+width, start_j:start_j+length] = np.random.choice(height_range)

    x1 = (terrain.width - platform_size) // 2
    x2 = (terrain.width + platform_size) // 2
    y1 = (terrain.length - platform_size) // 2
    y2 = (terrain.length + platform_size) // 2
    terrain.height_field_raw[x1:x2, y1:y2] = 0
    return terrain

def wave_terrain(terrain, num_waves=1, amplitude=1.):
    """
    Generate a wavy terrain
    Parameters:
        terrain (terrain): the terrain
        num_waves (int): number of sine waves across the terrain length
    Returns:
        terrain (SubTerrain): update terrain
    """
    amplitude = int(0.5*amplitude / terrain.vertical_scale)
    if num_waves > 0:
        div = terrain.length / (num_waves * np.pi * 2)
        x = np.arange(0, terrain.width)
        y = np.arange(0, terrain.length)
        xx, yy = np.meshgrid(x, y, sparse=True)
        xx = xx.reshape(terrain.width, 1)
        yy = yy.reshape(1, terrain.length)
        terrain.height_field_raw += (amplitude*np.cos(yy / div) + amplitude*np.sin(xx / div)).astype(
            terrain.height_field_raw.dtype)
    return terrain

def stairs_terrain(terrain, step_width, step_height):
    """
    Generate a stairs
    Parameters:
        terrain (terrain): the terrain
        step_width (float):  the width of the step [meters]
        step_height (float):  the height of the step [meters]
    Returns:
        terrain (SubTerrain): update terrain
    """
    # switch parameters to discrete units
    step_width = int(step_width / terrain.horizontal_scale)
    step_height = int(step_height / terrain.vertical_scale)

    num_steps = terrain.width // step_width
    height = step_height
    for i in range(num_steps):
        terrain.height_field_raw[i * step_width: (i + 1) * step_width, :] += height
        height += step_height
    return terrain

def pyramid_stairs_terrain(terrain, step_width, step_height, platform_size=1.):
    """
    Generate stairs
    Parameters:
        terrain (terrain): the terrain
        step_width (float):  the width of the step [meters]
        step_height (float): the step_height [meters]
        platform_size (float): size of the flat platform at the center of the terrain [meters]
    Returns:
        terrain (SubTerrain): update terrain
    """
    # switch parameters to discrete units
    step_width = int(step_width / terrain.horizontal_scale)
    step_height = int(step_height / terrain.vertical_scale)
    platform_size = int(platform_size / terrain.horizontal_scale)

    height = 0
    start_x = 0
    stop_x = terrain.width
    start_y = 0
    stop_y = terrain.length
    while (stop_x - start_x) > platform_size and (stop_y - start_y) > platform_size:
        start_x += step_width
        stop_x -= step_width
        start_y += step_width
        stop_y -= step_width
        height += step_height
        terrain.height_field_raw[start_x: stop_x, start_y: stop_y] = height
    return terrain


def stepping_stones_terrain(terrain, stone_size, stone_distance, max_height, platform_size=1., depth=1):
    """
    Generate a stepping stones terrain
    Parameters:
        terrain (terrain): the terrain
        stone_size (float): horizontal size of the stepping stones [meters]
        stone_distance (float): distance between stones (i.e size of the holes) [meters]
        max_height (float): maximum height of the stones (positive and negative) [meters]
        platform_size (float): size of the flat platform at the center of the terrain [meters]
        depth (float): depth of the holes (default=-10.) [meters]
    Returns:
        terrain (SubTerrain): update terrain
    """
    # switch parameters to discrete units
    stone_size = int(stone_size / terrain.horizontal_scale)
    stone_distance = int(stone_distance / terrain.horizontal_scale)
    max_height = int(max_height / terrain.vertical_scale)
    platform_size = int(platform_size / terrain.horizontal_scale)
    height_range = np.arange(-max_height-1, max_height, step=1)

    start_x = 0
    start_y = 0
    terrain.height_field_raw[:, :] = int(depth / terrain.vertical_scale)
    if terrain.length >= terrain.width:
        while start_y < terrain.length:
            stop_y = min(terrain.length, start_y + stone_size)
            start_x = np.random.randint(0, stone_size)
            # fill first hole
            stop_x = max(0, start_x - stone_distance)
            terrain.height_field_raw[0: stop_x, start_y: stop_y] = 0 #np.random.choice(height_range)
            # fill row
            while start_x < terrain.width:
                stop_x = min(terrain.width, start_x + stone_size)
                terrain.height_field_raw[start_x: stop_x, start_y: stop_y] = 0 #np.random.choice(height_range)
                start_x += stone_size + stone_distance
            start_y += stone_size + stone_distance
    elif terrain.width > terrain.length:
        while start_x < terrain.width:
            stop_x = min(terrain.width, start_x + stone_size)
            start_y = np.random.randint(0, stone_size)
            # fill first hole
            stop_y = max(0, start_y - stone_distance)
            terrain.height_field_raw[start_x: stop_x, 0: stop_y] = np.random.choice(height_range)
            # fill column
            while start_y < terrain.length:
                stop_y = min(terrain.length, start_y + stone_size)
                terrain.height_field_raw[start_x: stop_x, start_y: stop_y] = np.random.choice(height_range)
                start_y += stone_size + stone_distance
            start_x += stone_size + stone_distance

    x1 = (terrain.width - platform_size) // 2
    x2 = (terrain.width + platform_size) // 2
    y1 = (terrain.length - platform_size) // 2
    y2 = (terrain.length + platform_size) // 2
    terrain.height_field_raw[x1:x2, y1:y2] = 0
    return terrain


def generate_vertices(height_field_raw):
    vertices = []
    base = (-1, -0.75, -1)
    size = 2
    max_height = 0.5
    step_x = size/(height_field_raw.shape[0]-1)
    step_y = size/(height_field_raw.shape[1]-1)

    for x in range(height_field_raw.shape[0]):
        for y in range(height_field_raw.shape[1]):
            x_coord = base[0] + step_x*x 
            y_coord = base[1] + max_height*height_field_raw[x][y]
            z_coord = base[2] + step_y*y
            vertices.append((x_coord, y_coord, z_coord))
    print("Vertices generated")
    return np.asarray(vertices)


def generate_tris(height_field_raw):
    edges = []
    surfaces = []

    for x in range(height_field_raw.shape[0]-1):
        for y in range(height_field_raw.shape[1]-1):
            base = x*height_field_raw.shape[0]+y
            a = base
            b = base+1
            c = base+height_field_raw.shape[0]+1
            d = base+height_field_raw.shape[0]
            edges.append((a, b))
            edges.append((b, c))
            edges.append((c, a))
            edges.append((c, d))
            edges.append((d, a))
            surfaces.append((a, b, c))
            surfaces.append((a, c, d))
    print("Edges, surfaces generated")
    return np.asarray(surfaces)


def convert_heightfield_to_trimesh(height_field_raw, horizontal_scale, vertical_scale, slope_threshold=None):

    """
    Convert a heightfield array to a triangle mesh represented by vertices and triangles.
    Optionally, corrects vertical surfaces above the provide slope threshold:
        If (y2-y1)/(x2-x1) > slope_threshold -> Move A to A' (set x1 = x2). Do this for all directions.
                   B(x2,y2)
                  /|
                 / |
                /  |
        (x1,y1)A---A'(x2',y1)
    Parameters:
        height_field_raw (np.array): input heightfield
        horizontal_scale (float): horizontal scale of the heightfield [meters]
        vertical_scale (float): vertical scale of the heightfield [meters]
        slope_threshold (float): the slope threshold above which surfaces are made vertical. If None no correction is applied (default: None)
    Returns:
        vertices (np.array(float)): array of shape (num_vertices, 3). Each row represents the location of each vertex [meters]
        triangles (np.array(int)): array of shape (num_triangles, 3). Each row represents the indices of the 3 vertices connected by this triangle.
    """
    hf = height_field_raw
    num_rows = hf.shape[0]
    num_cols = hf.shape[1]

    y = np.linspace(0, (num_cols-1)*horizontal_scale, num_cols)
    x = np.linspace(0, (num_rows-1)*horizontal_scale, num_rows)
    yy, xx = np.meshgrid(y, x)

    if slope_threshold is not None:

        slope_threshold *= horizontal_scale / vertical_scale
        move_x = np.zeros((num_rows, num_cols))
        move_y = np.zeros((num_rows, num_cols))
        move_corners = np.zeros((num_rows, num_cols))
        move_x[:num_rows-1, :] += (hf[1:num_rows, :] - hf[:num_rows-1, :] > slope_threshold)
        move_x[1:num_rows, :] -= (hf[:num_rows-1, :] - hf[1:num_rows, :] > slope_threshold)
        move_y[:, :num_cols-1] += (hf[:, 1:num_cols] - hf[:, :num_cols-1] > slope_threshold)
        move_y[:, 1:num_cols] -= (hf[:, :num_cols-1] - hf[:, 1:num_cols] > slope_threshold)
        move_corners[:num_rows-1, :num_cols-1] += (hf[1:num_rows, 1:num_cols] - hf[:num_rows-1, :num_cols-1] > slope_threshold)
        move_corners[1:num_rows, 1:num_cols] -= (hf[:num_rows-1, :num_cols-1] - hf[1:num_rows, 1:num_cols] > slope_threshold)
        xx += (move_x + move_corners*(move_x == 0)) * horizontal_scale
        yy += (move_y + move_corners*(move_y == 0)) * horizontal_scale

    # create triangle mesh vertices and triangles from the heightfield grid
    vertices = np.zeros((num_rows*num_cols, 3), dtype=np.float32)
    vertices[:, 0] = xx.flatten()
    vertices[:, 1] = yy.flatten()
    vertices[:, 2] = hf.flatten() * vertical_scale
    triangles = -np.ones((2*(num_rows-1)*(num_cols-1), 3), dtype=np.int32)
    for i in range(num_rows - 1):
        ind0 = np.arange(0, num_cols-1) + i*num_cols
        ind1 = ind0 + 1
        ind2 = ind0 + num_cols
        ind3 = ind2 + 1
        start = 2*i*(num_cols-1)
        stop = start + 2*(num_cols-1)
        triangles[start:stop:2, 0] = ind0
        triangles[start:stop:2, 1] = ind3
        triangles[start:stop:2, 2] = ind1
        triangles[start+1:stop:2, 0] = ind0
        triangles[start+1:stop:2, 1] = ind2
        triangles[start+1:stop:2, 2] = ind3


    return vertices, triangles


def compute_face_normals(faces, vertices):
    n_F = faces.shape[0]
    #n_V = vertices.shape[0]
    face_normals = np.zeros((n_F, 3))
   # vertex_normals = np.zeros((n_V, 3))
    

    for i in range(n_F):
        a = faces[i, 0]
        b = faces[i, 1]
        c = faces[i, 2]

        A = vertices[a, :]
        B = vertices[b, :]
        C = vertices[c, :]

        BA = B - A
        CA = C - A

        face_normal = np.cross(BA, CA)

        face_normal /= np.linalg.norm(face_normal)

        face_normals[i, :] = face_normal
    
    return face_normals


def compute_vertex_normals(faces, vertices):
    n_V = vertices.shape[0]
    vertex_normals = np.zeros((n_V, 3))

    # Compute face normals
    face_normals = compute_face_normals(faces, vertices)

    # Accumulate face normals for each vertex
    for i in range(faces.shape[0]):
        a = faces[i, 0]
        b = faces[i, 1]
        c = faces[i, 2]

        vertex_normals[a, :] += face_normals[i, :]
        vertex_normals[b, :] += face_normals[i, :]
        vertex_normals[c, :] += face_normals[i, :]

    # Normalize vertex normals
    vertex_normals /= np.linalg.norm(vertex_normals, axis=1)[:, np.newaxis]

    return vertex_normals


   
def add_terrain_to_stage(stage, vertices, triangles, position=None, orientation=None, name=str, scale = None, material = None):
 
    # print("Vertice is: ", vertices)
    # print("Triangle is: ", triangles)

    num_faces = triangles.shape[0]
   
    normals = compute_vertex_normals(triangles, vertices)
    
    terrain_mesh = stage.DefinePrim("/World/"+name, "Mesh")

    terrain_mesh.GetAttribute("points").Set(vertices)
    terrain_mesh.GetAttribute("normals").Set(normals)
    terrain_mesh.GetAttribute("faceVertexIndices").Set(triangles.flatten()) 
    terrain_mesh.GetAttribute("faceVertexCounts").Set(np.asarray([3]*num_faces))   
    terrain_mesh.GetAttribute("extent").Set(np.array([vertices[0],vertices[-1]]))

    
    terrain = GeometryPrim(prim_path="/World/"+name,
                           name=name,
                           position=position,
                           scale=scale,
                           orientation=orientation,
                           )
    

    UsdPhysics.CollisionAPI.Apply(terrain.prim)
    collision_api = UsdPhysics.MeshCollisionAPI.Apply(terrain.prim)
    collision_api.CreateApproximationAttr().Set("meshSimplification")
    physx_collision_api = PhysxSchema.PhysxCollisionAPI.Apply(terrain.prim)
    physx_collision_api.GetContactOffsetAttr().Set(0.02)
    physx_collision_api.GetRestOffsetAttr().Set(0.00)

    
    if material == "Country_Grass":

        apply_texture(ASSET_COUNTRY_GRASS, "Grass_Countryside", "/World/Looks/Country_Grass", name)
        
    elif material == "Soil_Rocky":

        apply_texture(ASSET_SOIL, "Soil_Rocky", "/World/Looks/Soil_Rocky", name)
       
    elif material == "Sand":

        apply_texture(ASSET_SAND, "Sand", "/World/Looks/Sand", name)
       

    elif material == "Adobe_Octagon_Dots":

        apply_texture(ASSET_OCTAGON, "Adobe_Octagon_Dots", "/World/Looks/Adobe_Octagon_Dots", name)
        

    elif material == "Brick_Wall_Red":

        apply_texture(ASSET_BRICK, "Brick_Wall_Red", "/World/Looks/Brick_Wall_Red", name)
       
   
     
def apply_texture(mtl_url, mtl_name, mtl_path, name):       

        omni.kit.commands.execute('CreateMdlMaterialPrimCommand',
        mtl_url= mtl_url,
        mtl_name=mtl_name,
        mtl_path=mtl_path)

        omni.kit.commands.execute('BindMaterialCommand',
            prim_path="/World/"+name,
            material_path=mtl_path,
            strength='strongerThanDescendants') 
    
      
async def uvs_coordinates(mtl_path):
    
        selection = omni.usd.get_context().get_selection()
        selection.set_selected_prim_paths([mtl_path+"/Shader"], True)
       
        await omni.kit.app.get_app().next_update_async()
        
    
        omni.kit.commands.execute('ChangeProperty',
            prop_path=Sdf.Path(mtl_path+"/Shader.inputs:project_uvw"),
            value=True,
            prev=False,
         )
       
        

class SubTerrain:
    def __init__(self, terrain_name="terrain", width=256, length=256, vertical_scale=1.0, horizontal_scale=1.0):
        self.terrain_name = terrain_name
        self.vertical_scale = vertical_scale
        self.horizontal_scale = horizontal_scale
        self.width = width
        self.length = length
        self.height_field_raw = np.zeros((self.width, self.length), dtype=np.int16)




#################################################################################################################################
################################################# CODE NOT USED #################################################################
#################################################################################################################################

# def polygon_reduction(vertices, triangles, target_vertices=50000):
#         #Target number of vertex
#         TARGET=target_vertices
#         # Create mesh
#         m = pymeshlab.Mesh(vertices, triangles)
#         # Create a meshset
#         ms = pymeshlab.MeshSet()
#         # Add mesh to meshset
#         ms.add_mesh(m, "terrain")

#         #Estimate number of faces to have 100+10000 vertex using Euler
#         numFaces = 100 + 2*TARGET
#         #Simplify the mesh - only first simplification is aggresive
#         while (ms.current_mesh().vertex_number() > TARGET):
#             ms.apply_filter('meshing_decimation_quadric_edge_collapse', targetfacenum=numFaces, preservenormal=True)
#             print("Decimated to", numFaces, "faces mesh has", ms.current_mesh().vertex_number(), "vertex")
#             #Refine estimation to converge to TARGET vertex number
#             numFaces = numFaces - (ms.current_mesh().vertex_number() - TARGET)

#         # Save final mesh
#         m = ms.current_mesh()
        
#         # Get vertices as float32 (Supported by isaac gym)
#         vertices = m.vertex_matrix().astype('float32')
#         # Get faces as unit32 (Supported by isaac gym)
#         faces =  m.face_matrix().astype('uint32')
#         return vertices, faces

 # if material == "grass":
    #     # mat = UsdShade.Material.Define(stage, "/World/looks/grass")
    #     # texture_file_path = "/home/user/Documents/yohan/terrain_generation/exts/terrain.generation/docs/grass.png"
    #     # texture_attribute = mat.CreateInput("diffuseColor", Sdf.ValueTypeNames.Color3f)
    #     # # Load the texture file and set it as value for the texture attribute
    #     # # This can be done using any image loading library or PySide2.QtGui.QPixmap to load an image
    #     # texture_attribute.Set(Gf.Vec3f(1, 1, 1), texture_file_path)
    #     # material_binding = UsdShade.MaterialBindingAPI.Apply(terrain_mesh)
    #     # material_binding.Bind(mat)
    



    # def cfa(cfa: float, rock_diameter: float):
#     # https://agupubs.onlinelibrary.wiley.com/doi/pdfdirect/10.1029/96JE03319?download=true
#     # https://www.researchgate.net/publication/340235161_Geographic_information_system_based_detection_and_quantification_of_boulders_using_HiRISE_imagery_a_case_study_in_Jezero_Crater
#     q = lambda k: 1.79 + (0.152 / k) # Rate at which the total area covered by rocks decreases with increasing diameter q(k)
#     Fk = lambda D, k: k * math.exp(-q(k)*D) # The cumulative fractional area covered by rocks larger than D

#     return Fk(rock_diameter,cfa)


# def add_rocks_terrain(terrain, rock_height = (0.1,0.2)):

#     k = 0.03    # total fractional area covered by rocks
#     #sample_size = int(0.5 / terrain.horizontal_scale)
#     #probs = np.arange(terrain.horizontal_scale, sample_size, terrain.horizontal_scale)
#     rock = torch.empty(1,4,device="cuda:0") # [x, y, z, radius]
#     rocks = torch.empty(0,4,device="cuda:0")
#     res = terrain.horizontal_scale
#     rock_distribution_step = max(0.10,res)
#     scale = int(rock_distribution_step / res)
#     num_rock_sizes = int(0.5/rock_distribution_step)
#     sampler_halton = qmc.Halton(d=2, scramble=False)
#     for i in range(1,num_rock_sizes):
#         terrain_height_scaler = 1
#         rock_radius = (i * rock_distribution_step) / 2        
#         lower_bound = terrain.length * terrain.width * cfa(k,rock_diameter=i*rock_distribution_step)
#         upper_bound = terrain.length * terrain.width * cfa(k,rock_diameter=(i+1)*rock_distribution_step)
#         num_rocks = int((lower_bound-upper_bound)/(rock_radius*rock_radius*3.1415))

#         random_positions = (sampler_halton.random(n=num_rocks))

#         random_positions[:,0] *= terrain.num_rows
#         random_positions[:,1] *= terrain.num_cols
#         random_positions = random_positions.astype(int)
        
#         kernel = scaleMax(gaussian_kernel(i*scale+1,sigma=1,normalized=False))

#         if kernel[int((i*scale+1)/2)-1,int((i*scale+1)/2)] < 1:
#             kernel[int((i*scale+1)/2),int((i*scale+1)/2)] = kernel[int((i*scale+1)/2)-1,int((i*scale+1)/2)]
#             #terrain_height_scaler = 0.5

#         kernel = np.multiply(np.ones((1+i*scale,1+i*scale)) * random.uniform(rock_height[0], rock_height[1]), scaleMax(kernel) * (1/(1+np.exp(-i*scale*0.3)))*2) * terrain_height_scaler#+ np.ones((1+i,1+i)) * i* scale *0.2
#         for p in random_positions:
#             try:
#                 terrain.height_field_raw[p[0]: p[0]+1+i*scale, p[1]: p[1]+1+i*scale] += (kernel*1/terrain.vertical_scale) * (random.random()*0.4 + 0.6) # 0.6*0.4 is random height scaling
#                 try:
#                     rock[0][0] = p[0] * res + rock_radius # x
#                     rock[0][1] = p[1] * res + rock_radius # y
#                     rock[0][2] = 0                  # z
#                     rock[0][3] = rock_radius        # radius
#                     rocks = torch.cat((rocks, rock))  
#                 except Exception:
#                     print("ERROR POSITION OF ROCKS NOT SAVED")
#                     raise Exception
#             except Exception:
#                 pass

#     return terrain, rocks

# def scaleMax(data) -> np.ndarray:
#     # Scales the maximum value to one
#     return (data / np.max(data))

# def gaussian_kernel(n_samples: int, sigma=0.3, normalized=True) -> np.ndarray:

#     # Take the outer product of a gaussian distribution
#     gaussian_kernel = np.outer(gaussian_distribution(n_samples,sigma,normalized=normalized),gaussian_distribution(n_samples,sigma,normalized=normalized))

#     return gaussian_kernel


# def gaussian_distribution(n_samples: int, sigma=0.3, normalized=True) -> np.ndarray:

#     # Discrete sampling of range [-1:1] with n samples
#     # With step size 1 / ( (n_samples - 1) / 2 )
    
#     step_size = 2 / (n_samples - 1)
#     # Add epsilon to avoid rounding errors influenzing the size of the kernal
#     epsilon = 1e-7
#     sampled_values = np.arange(-1, 1+epsilon, step_size)
#     # Calculate gaussian distribution
#     gaussian_distribution = [(1/(sigma*math.sqrt(2*math.pi))) * math.exp(-0.5*(x/sigma)*(x/sigma))   for x in sampled_values]
    
#     # Normalize data between 0 and 1
#     if normalized == True:
#         gaussian_distribution = NormalizeData(gaussian_distribution)

#     return gaussian_distribution

# def NormalizeData(data) -> np.ndarray:
#     #Normalizes data
#     return (data - np.min(data)) / (np.max(data) - np.min(data))



    # # # Create a UsdGeom Gprim for the terrain
    # # terrain_prim = UsdGeom.Gprim(terrain_mesh)
    # # terrain_prim.CreateDisplayColorAttr((0.5, 0.5, 0.5))

    #     asset_path = ASSET_PATH_REALISTIC_GRASS_FLOOR
    #     cube_mat_shade = UsdShade.Material(asset_path)
    #     UsdShade.MaterialBindingAPI("/World/"+name).Bind(cube_mat_shade, UsdShade.Tokens.strongerThanDescendants)
    #     # #ground = XFormPrim(name="grassground", prim_path = "/World/"+name)
    #     # add_reference_to_stage(usd_path = asset_path, prim_path = "/World/"+name+"/grass")


    # # # Apply a material to the terrain mesh
    # # if material == "grass":
    # #     material_binding = UsdShade.MaterialBindingAPI.Apply(terrain_mesh)
    # #     material_binding.Bind(mat)
    
# ASSET_PATH_REALISTIC_GRASS_FLOOR = "/home/user/Documents/yohan/terrain_generation/exts/terrain.generation/data/GrassFloor_Realistic_6m.usd"
# ASSET_PATH_CROP = "omniverse://localhost/NVIDIA/Assets/Vegetation/Shrub/Century.usd"

# def _process_mesh(mesh_prim, ref_path, attrs, with_materials, with_normals, time=None):

#     time = Usd.TimeCode.Default()
#     cur_first_idx_faces = sum([len(v) for v in attrs.get('vertices', [])])
#     cur_first_idx_uvs = sum([len(u) for u in attrs.get('uvs', [])])
#     mesh = UsdGeom.Mesh(mesh_prim)
#     mesh_vertices = mesh.GetPointsAttr().Get(time=time)
#     mesh_face_vertex_counts = mesh.GetFaceVertexCountsAttr().Get(time=time)
#     mesh_vertex_indices = mesh.GetFaceVertexIndicesAttr().Get(time=time)
#     mesh_st = mesh.GetPrimvar('st')
#     mesh_subsets = UsdGeom.Subset.GetAllGeomSubsets(UsdGeom.Imageable(mesh_prim))
#     mesh_material = UsdShade.MaterialBindingAPI(mesh_prim).ComputeBoundMaterial()[0]
#     transform = torch.from_numpy(np.array(UsdGeom.Xformable(mesh_prim).ComputeLocalToWorldTransform(time), dtype=np.float32))

#     # Parse mesh UVs
#     if mesh_st:
#         mesh_uvs = mesh_st.Get(time=time)
#         mesh_uv_indices = mesh_st.GetIndices(time=time)
#         mesh_uv_interpolation = mesh_st.GetInterpolation()
    
#     mesh_face_normals = mesh.GetNormalsAttr().Get(time=time)

#     # Parse mesh geometry
#     if mesh_vertices:
#         mesh_vertices = torch.from_numpy(np.array(mesh_vertices, dtype=np.float32))
#         mesh_vertices_homo = torch.nn.functional.pad(mesh_vertices, (0, 1), mode='constant', value=1.)
#         mesh_vertices = (mesh_vertices_homo @ transform)[:, :3]
#         attrs.setdefault('vertices', []).append(mesh_vertices)
#     if mesh_vertex_indices:
#         attrs.setdefault('face_vertex_counts', []).append(torch.from_numpy(
#             np.array(mesh_face_vertex_counts, dtype=np.int64)))
#         vertex_indices = torch.from_numpy(np.array(mesh_vertex_indices, dtype=np.int64)) + cur_first_idx_faces
#         attrs.setdefault('vertex_indices', []).append(vertex_indices)
#     if with_normals and mesh_face_normals:
#         attrs.setdefault('face_normals', []).append(torch.from_numpy(np.array(mesh_face_normals, dtype=np.float32)))
#     if mesh_st and mesh_uvs:
#         attrs.setdefault('uvs', []).append(torch.from_numpy(np.array(mesh_uvs, dtype=np.float32)))
#         if mesh_uv_interpolation in ['vertex', 'varying']:
#             if not mesh_uv_indices:
#                 # for vertex and varying interpolation, length of mesh_uv_indices should match
#                 # length of mesh_vertex_indices
#                 mesh_uv_indices = list(range(len(mesh_uvs)))
#             mesh_uv_indices = torch.tensor(mesh_uv_indices) + cur_first_idx_uvs
#             face_uvs_idx = mesh_uv_indices[torch.from_numpy(np.array(mesh_vertex_indices, dtype=np.int64))]
#             attrs.setdefault('face_uvs_idx', []).append(face_uvs_idx)
#         elif mesh_uv_interpolation == 'faceVarying':
#             if not mesh_uv_indices:
#                 # for faceVarying interpolation, length of mesh_uv_indices should match
#                 # num_faces * face_size
#                 # TODO implement default behaviour
#                 mesh_uv_indices = [i for i, c in enumerate(mesh_face_vertex_counts) for _ in range(c)]
#             else:
#                 attrs.setdefault('face_uvs_idx', []).append(torch.tensor(mesh_uv_indices) + cur_first_idx_uvs)
#         # elif mesh_uv_interpolation == 'uniform':
#         else:
#             raise NotImplementedError(f'Interpolation type {mesh_uv_interpolation} is '
#                                         'not currently supported')

    # # Parse mesh materials
    # if with_materials:
    #     subset_idx_map = {}
    #     attrs.setdefault('materials', []).append(None)
    #     attrs.setdefault('material_idx_map', {})
    #     if mesh_material:
    #         mesh_material_path = str(mesh_material.GetPath())
    #         if mesh_material_path in attrs['material_idx_map']:
    #             material_idx = attrs['material_idx_map'][mesh_material_path]
    #         else:
    #             try:
    #                 material = usd_materials.MaterialManager.read_usd_material(mesh_material, stage_dir, time)
    #                 material_idx = len(attrs['materials'])
    #                 attrs['materials'].append(material)
    #                 attrs['material_idx_map'][mesh_material_path] = material_idx
    #             except usd_materials.MaterialNotSupportedError as e:
    #                 warnings.warn(e.args[0])
    #             except usd_materials.MaterialLoadError as e:
    #                 warnings.warn(e.args[0])
    #     if mesh_subsets:
    #         for subset in mesh_subsets:
    #             subset_material, _ = UsdShade.MaterialBindingAPI(subset).ComputeBoundMaterial()
    #             subset_material_metadata = subset_material.GetPrim().GetMetadata('references')
    #             mat_ref_path = ""
    #             if ref_path:
    #                 mat_ref_path = ref_path
    #             if subset_material_metadata:
    #                 asset_path = subset_material_metadata.GetAddedOrExplicitItems()[0].assetPath
    #                 mat_ref_path = os.path.join(ref_path, os.path.dirname(asset_path))
    #             if not os.path.isabs(mat_ref_path):
    #                 mat_ref_path = os.path.join(stage_dir, mat_ref_path)
    #             try:
    #                 kal_material = usd_materials.MaterialManager.read_usd_material(subset_material, mat_ref_path,
    #                                                                                 time)
    #             except usd_materials.MaterialNotSupportedError as e:
    #                 warnings.warn(e.args[0])
    #                 continue
    #             except usd_materials.MaterialLoadError as e:
    #                 warnings.warn(e.args[0])

    #             subset_material_path = str(subset_material.GetPath())
    #             if subset_material_path not in attrs['material_idx_map']:
    #                 attrs['material_idx_map'][subset_material_path] = len(attrs['materials'])
    #                 attrs['materials'].append(kal_material)
    #             subset_indices = np.array(subset.GetIndicesAttr().Get())
    #             subset_idx_map[attrs['material_idx_map'][subset_material_path]] = subset_indices
    #     # Create material face index list
    #     if mesh_face_vertex_counts:
    #         for face_idx in range(len(mesh_face_vertex_counts)):
    #             is_in_subsets = False
    #             for subset_idx in subset_idx_map:
    #                 if face_idx in subset_idx_map[subset_idx]:
    #                     is_in_subsets = True
    #                     attrs.setdefault('materials_face_idx', []).extend(
    #                         [subset_idx] * mesh_face_vertex_counts[face_idx]
    #                     )
    #             if not is_in_subsets:
    #                 if mesh_material:
    #                     attrs.setdefault('materials_face_idx', []).extend(
    #                         [material_idx] * mesh_face_vertex_counts[face_idx]
    #                     )
    #                 else:
    #                     # Assign to `None` material (ie. index 0)
    #                     attrs.setdefault('materials_face_idx', []).extend([0] * mesh_face_vertex_counts[face_idx])
# def add_mesh(stage, scene_path, vertices=None, faces=None, uvs=None, face_uvs_idx=None, face_normals=None,
#              materials_order=None, materials=None, time=None):
#     """Add a mesh to an existing USD stage.
#     Add a mesh to the USD stage. The stage is modified but not saved to disk.
#     Args:
#         stage (Usd.Stage): Stage onto which to add the mesh.
#         scene_path (str): Absolute path of mesh within the USD file scene. Must be a valid ``Sdf.Path``.
#         vertices (torch.FloatTensor, optional): Vertices with shape ``(num_vertices, 3)``.
#         faces (torch.LongTensor, optional): Vertex indices for each face with shape ``(num_faces, face_size)``.
#             Mesh must be homogenous (consistent number of vertices per face).
#         uvs (torch.FloatTensor, optional): of shape ``(num_uvs, 2)``.
#         face_uvs_idx (torch.LongTensor, optional): of shape ``(num_faces, face_size)``. If provided, `uvs` must also
#             be specified.
#         face_normals (torch.Tensor, optional): of shape ``(num_vertices, num_faces, 3)``.
#         materials_order (torch.LongTensor): of shape (N, 2)
#           showing the order in which materials are used over **face_uvs_idx** and the first indices
#           in which they start to be used. A material can be used multiple times.
#         materials (list of Material): a list of materials
#         time (convertible to float, optional): Positive integer defining the time at which the supplied parameters
#             correspond to.
#     Returns:
#         (Usd.Stage)
#     Example:
#         >>> vertices = torch.rand(3, 3)
#         >>> faces = torch.tensor([[0, 1, 2]])
#         >>> stage = create_stage('./new_stage.usd')
#         >>> mesh = add_mesh(stage, '/World/mesh', vertices, faces)
#         >>> stage.Save()
#     """
    
#     if time is None:
#         time = Usd.TimeCode.Default()

#     usd_mesh = UsdGeom.Mesh.Define(stage, scene_path)

#     if faces is not None:
#         print("faces is not none")
#         print(faces)
#         num_faces = faces.size(0)
#         print("num of faces is: ", num_faces)

#         face_vertex_counts = [faces.size(1)] * num_faces
#         faces_list = faces.view(-1).cpu().long().numpy()
#         usd_mesh.GetFaceVertexCountsAttr().Set(face_vertex_counts, time=time)
#         usd_mesh.GetFaceVertexIndicesAttr().Set(faces_list, time=time)
#     if vertices is not None:
#         vertices_list = vertices.detach().cpu().float().numpy()
#         usd_mesh.GetPointsAttr().Set(Vt.Vec3fArray.FromNumpy(vertices_list), time=time)
#     if uvs is not None:
#         uvs_list = uvs.view(-1, 2).detach().cpu().float().numpy()
#         pv = UsdGeom.PrimvarsAPI(usd_mesh.GetPrim()).CreatePrimvar(
#             'st', Sdf.ValueTypeNames.Float2Array)
#         pv.Set(uvs_list, time=time)

#         if vertices is not None and uvs.size(0) == vertices.size(0):
#             pv.SetInterpolation('vertex')
#         elif faces is not None and uvs.view(-1, 2).size(0) == faces.size(0):
#             pv.SetInterpolation('uniform')
#         else:
#             pv.SetInterpolation('faceVarying')

#     if face_uvs_idx is not None:
#         if uvs is not None:
#             pv.SetIndices(Vt.IntArray.FromNumpy(face_uvs_idx.view(-1).cpu().long().numpy()), time=time)
#         else:
#             warnings.warn('If providing "face_uvs_idx", "uvs" must also be provided.')

#     if face_normals is not None:
#         face_normals = face_normals.view(-1, 3).cpu().float().numpy()
#         usd_mesh.GetNormalsAttr().Set(face_normals, time=time)
#         UsdGeom.PointBased(usd_mesh).SetNormalsInterpolation('faceVarying')

#     if faces is not None and materials_order is not None and materials is not None:
#         stage.DefinePrim(f'{scene_path}/Looks', 'Scope')
#         subsets = {}
#         for i in range(len(materials_order)):
#             first_face_idx, mat_idx = materials_order[i]
#             if materials[mat_idx] is None:
#                 continue
#             last_face_idx = materials_order[i + 1][0] if (i + 1) < len(materials_order) else faces.size(0)
#             for face_idx in range(first_face_idx, last_face_idx):
#                 subsets.setdefault(mat_idx, []).append(face_idx)

#         # Create submeshes
#         for i, subset in enumerate(subsets):
#             subset_prim = stage.DefinePrim(f'{scene_path}/subset_{i}', 'GeomSubset')
#             subset_prim.GetAttribute('indices').Set(subsets[subset])
#             # if isinstance(materials[subset], usd_materials.Material):
#             #     materials[subset]._write_usd_preview_surface(stage, f'{scene_path}/Looks/material_{subset}',
#             #                                                  [subset_prim], time, texture_dir=f'material_{subset}',
#             #                                                  texture_file_prefix='')    # TODO allow users to pass root path to save textures to

