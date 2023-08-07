import math
import numpy as np
import csv
import json
import os
from pxr import Gf
import omni.usd
# from __main__ import parser
import __main__



## Get the homogeneous transform of the selected prim
stage = omni.usd.get_context().get_stage()
prim = stage.GetPrimAtPath("/World/CameraPos/CameraOrientation/offset/zed_x/base_link/ZED_X/CameraLeft/leftcam_orientation")
matrix = omni.usd.get_world_transform_matrix(prim)

# load the data into a csv file

# print(parser[2])
csv_filename = __main__.parser[2]['ground_truth']['csv']

  
# Create File
if not os.path.exists(csv_filename):
    print("No File")
    header = ['first_column', 'second_column', 'third_column', 'fourth_column']
    with open(csv_filename, 'w') as f:
        writer = csv.writer(f)
        writer.writerow(header)
else:

    with open(csv_filename, mode='a') as csv_file:

        writer = csv.writer(csv_file)
        writer.writerow(matrix)
        

    

