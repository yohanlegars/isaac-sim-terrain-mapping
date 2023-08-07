import csv
import json
import numpy as np
import pandas

import numpy as np
# from ..TerrainMeshRecorder import parser
import __main__
# from ..TerrainMeshRecorder import parser
def rotation_matrix(theta1, theta2, theta3, order='xyz'):
    """
    input
        theta1, theta2, theta3 = rotation angles in rotation order (degrees)
        oreder = rotation order of x,y,z　e.g. XZY rotation -- 'xzy'
    output
        3x3 rotation matrix (numpy array)
    """
    c1 = np.cos(theta1 * np.pi / 180)
    s1 = np.sin(theta1 * np.pi / 180)
    c2 = np.cos(theta2 * np.pi / 180)
    s2 = np.sin(theta2 * np.pi / 180)
    c3 = np.cos(theta3 * np.pi / 180)
    s3 = np.sin(theta3 * np.pi / 180)

    if order == 'xzx':
        matrix=[[c2, -c3*s2, s2*s3],
                         [c1*s2, c1*c2*c3-s1*s3, -c3*s1-c1*c2*s3],
                         [s1*s2, c1*s3+c2*c3*s1, c1*c3-c2*s1*s3]]
    elif order=='xyx':
        matrix= [[c2, s2*s3, c3*s2],
                         [s1*s2, c1*c3-c2*s1*s3, -c1*s3-c2*c3*s1],
                         [-c1*s2, c3*s1+c1*c2*s3, c1*c2*c3-s1*s3]]
    elif order=='yxy':
        matrix= [[c1*c3-c2*s1*s3, s1*s2, c1*s3+c2*c3*s1],
                         [s2*s3, c2, -c3*s2],
                         [-c3*s1-c1*c2*s3, c1*s2, c1*c2*c3-s1*s3]]
    elif order=='yzy':
        matrix=[[c1*c2*c3-s1*s3, -c1*s2, c3*s1+c1*c2*s3],
                         [c3*s2, c2, s2*s3],
                         [-c1*s3-c2*c3*s1, s1*s2, c1*c3-c2*s1*s3]]
    elif order=='zyz':
        matrix=[[c1*c2*c3-s1*s3, -c3*s1-c1*c2*s3, c1*s2],
                         [c1*s3+c2*c3*s1, c1*c3-c2*s1*s3, s1*s2],
                         [-c3*s2, s2*s3, c2]]
    elif order=='zxz':
        matrix=[[c1*c3-c2*s1*s3, -c1*s3-c2*c3*s1, s1*s2],
                         [c3*s1+c1*c2*s3, c1*c2*c3-s1*s3, -c1*s2],
                         [s2*s3, c3*s2, c2]]
    elif order=='xyz':
        matrix=[[c2*c3, -c2*s3, s2],
                         [c1*s3+c3*s1*s2, c1*c3-s1*s2*s3, -c2*s1],
                         [s1*s3-c1*c3*s2, c3*s1+c1*s2*s3, c1*c2]]
    elif order=='xzy':
        matrix=[[c2*c3, -s2, c2*s3],
                         [s1*s3+c1*c3*s2, c1*c2, c1*s2*s3-c3*s1],
                         [c3*s1*s2-c1*s3, c2*s1, c1*c3+s1*s2*s3]]
    elif order=='yxz':
        matrix=[[c1*c3+s1*s2*s3, c3*s1*s2-c1*s3, c2*s1],
                         [c2*s3, c2*c3, -s2],
                         [c1*s2*s3-c3*s1, c1*c3*s2+s1*s3, c1*c2]]
    elif order=='yzx':
        matrix=[[c1*c2, s1*s3-c1*c3*s2, c3*s1+c1*s2*s3],
                         [s2, c2*c3, -c2*s3],
                         [-c2*s1, c1*s3+c3*s1*s2, c1*c3-s1*s2*s3]]
    elif order=='zyx':
        matrix=[[c1*c2, c1*s2*s3-c3*s1, s1*s3+c1*c3*s2],
                         [c2*s1, c1*c3+s1*s2*s3, c3*s1*s2-c1*s3],
                         [-s2, c2*s3, c2*c3]]
    elif order=='zxy':
        matrix=[[c1*c3-s1*s2*s3, -c2*s1, c1*s3+c3*s1*s2],
                         [c3*s1+c1*s2*s3, c1*c2, s1*s3-c1*c3*s2],
                         [-c2*s3, s2, c2*c3]]

    return matrix

def csv_to_json():
    sample = []
    df = pandas.read_csv(__main__.parser[4]['converter']['csv'])

    header = ['first_column', 'second_column', 'third_column', 'fourth_column']
    
    for i in range(len(df)):
        arr = []
        for j in header:
            mat = list(eval(df[j][i]))
            arr.append(mat)

        dict = {i: {"position": arr[3][:3],
                    "rotation": [   [arr[0][0], arr[1][0], arr[2][0]], 
                                    [arr[0][1], arr[1][1], arr[2][1]], 
                                    [arr[0][2], arr[1][2], arr[2][2]] ],
                    "Homogeneous_Transform": [[arr[0][0], arr[1][0], arr[2][0], arr[3][0]], 
                                    [arr[0][1], arr[1][1], arr[2][1], arr[3][1]], 
                                    [arr[0][2], arr[1][2], arr[2][2], arr[3][2]],  
                                    [arr[0][3], arr[1][3], arr[2][3], arr[3][3]]]}}
        sample.append(dict)



    with open(__main__.parser[4]['converter']['json'], 'w') as f:
        json.dump(sample, f)



if __name__ == "__main__":

    csv_to_json()

    
