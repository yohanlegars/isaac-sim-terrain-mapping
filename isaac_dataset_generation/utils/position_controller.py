import math
import numpy as np

# Set input values

eye = db.inputs.eye
target = db.inputs.target
up = db.inputs.up

# Get normalized forward vector

forward = target - eye
forward = forward / np.linalg.norm(forward)

# Get normalized right-hand vector

right = np.cross(up, forward)
right = right / np.linalg.norm(right)

# Get normalized up vector

up = np.cross(forward, right)
up = up / np.linalg.norm(up)

# Generate quaternion components values from the three directional vectors
a = forward
b = right
c = up

T = a[0] + b[1] + c[2]
if T > 0:
	s = math.sqrt(T+1) * 2
	X = (b[2] - c[1]) / s
	Y = (c[0] - a[2]) / s
	Z = (a[1] - b[0]) / s
	W = 0.25 * s
elif a[0] > b[1] and a[0] > c[2]:
	s = math.sqrt(1 + a[0] - b[1] - c[2]) * 2
	X = 0.25 * s
	Y = (a[1] + b[0]) / s
	Z = (c[0] + a[2]) / s
	W = (b[2] - c[1]) / s
elif b[1] > c[2]:
	s = math.sqrt(1 + b[1] - a[0] - c[2]) * 2
	X = (a[1] + b[0]) / s
	Y = 0.25 * s 
	Z = (b[2] + c[1]) / s
	W = (c[0] - a[2]) / s
else:
	s = math.sqrt(1 + c[2] - a[0] - b[1]) * 2
	X = (c[0] + a[2]) / s
	Y = (b[2] + c[1]) /s
	Z = 0.25 * s
	W = (a[1] - b[0]) /s

# Set output values
db.outputs.quaternion = np.array([X,Y,Z, W])

# if c[2] < 0:
# 	if a[0] > b[1]:
# 		t = 1 + a[0] - b[1] - c[2]
# 		X = t
# 		Y = b[0] + a[1] 
# 		Z = a[2] + c[0] 
# 		W = c[1] - b[2]

# 	else:
# 		t = 1 - a[0] + b[1] - c[2]
# 		X = b[0] + a[1]
# 		Y = t
# 		Z = c[1] + b[2] 
# 		W = a[2] - c[0]
# else:
# 	if a[0] < -b[1]:
# 		t = 1 - a[0] - b[1] + c[2]
# 		X = a[2] + c[0]
# 		Y = c[1] + b[2] 
# 		Z = t 
# 		W = b[0] - a[1]
	
# 	else:
# 		t = 1 + a[0] + b[1] + c[2]
# 		X = c[1] - b[2]
# 		Y = a[2] - c[0] 
# 		Z = b[0] - a[1] 
# 		W = t

# q = np.array([X,Y,Z,W])
# q *= 0.5/math.sqrt(t)

# db.outputs.quaternion = q



