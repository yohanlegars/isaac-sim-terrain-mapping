import math
import numpy as np
import omni.usd
from pxr import Usd, UsdGeom, Gf

stage = omni.usd.get_context().get_stage()


camera_frame = stage.GetPrimAtPath("/World/CameraPos/CameraOrientation/camera_keyboard")

W = db.inputs.W
S = db.inputs.S
A = db.inputs.A
D = db.inputs.D

up_arrow = db.inputs.up_arrow
down_arrow = db.inputs.down_arrow

right_arrow = db.inputs.right_arrow
left_arrow = db.inputs.left_arrow

Q = db.inputs.Q
E = db.inputs.E

arr: Gf.Matrix4d = omni.usd.get_world_transform_matrix(camera_frame)
R_camerapos_keyboard =    np.array([   [arr[0][0], arr[1][0], arr[2][0]], 
                                       [arr[0][1], arr[1][1], arr[2][1]], 
                                       [arr[0][2], arr[1][2], arr[2][2]] ])

translate_shift = 0.05
rotation_shift = 0.4

if W != 0:

    forward = np.array([translate_shift, 0, 0]) * W
    db.outputs.translate = R_camerapos_keyboard @ forward

elif S != 0:

    backward = np.array([-translate_shift, 0, 0]) * S
    db.outputs.translate = R_camerapos_keyboard @ backward 

elif D != 0:

    right = np.array([0, -translate_shift, 0]) * D
    db.outputs.translate = R_camerapos_keyboard @ right


elif A != 0:

    left = np.array([0, translate_shift, 0]) * A
    db.outputs.translate =  R_camerapos_keyboard @ left


elif up_arrow != 0:

    up = np.array([0, 0, translate_shift*0.5]) * up_arrow
    db.outputs.translate =  R_camerapos_keyboard @ up


elif down_arrow != 0:

    down = np.array([0, 0, -translate_shift*0.5]) * down_arrow
    db.outputs.translate =  R_camerapos_keyboard @ down

elif right_arrow != 0:

    db.outputs.rotate = np.array([0, 0, -rotation_shift])
   
   

elif left_arrow != 0:

    db.outputs.rotate = np.array([0, 0, rotation_shift])

elif Q != 0:

    db.outputs.rotate = np.array([0, rotation_shift, 0])

elif E != 0:

    db.outputs.rotate = np.array([0 , -rotation_shift, 0])



else:
    db.outputs.translate = np.array([0, 0, 0])
    db.outputs.rotate = np.array([0, 0, 0])





    










