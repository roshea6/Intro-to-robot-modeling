import numpy as np
import math

# Returns a rotation matrix for rotating around the x axis by the passed in angle
def getXRotMat(angle):
    rot_mat = np.array([
               [1, 0, 0],
               [0, math.cos(angle), -math.sin(angle)],
               [0, math.sin(angle), math.cos(angle)]])
    
    return rot_mat

# Returns a rotation matrix for rotating around the y axis by the passed in angle
def getYRotMat(angle):
    rot_mat = np.array([
               [math.cos(angle), 0, math.sin(angle)],
               [0, 1, 0],
               [-math.sin(angle), 0, math.cos(angle)]])
    
    return rot_mat

# Returns a rotation matrix for rotating around the z axis by the passed in angle
def getZRotMat(angle):
    rot_mat = np.array([
               [math.cos(angle), -math.sin(angle), 0],
               [math.sin(angle), math.cos(angle), 0],
               [0, 0, 1]])
    
    return rot_mat
  
def deg2Rad(deg_angle):
    return (deg_angle * (math.pi/180))  

def rad2Deg(rad_angle):
    return (rad_angle * (180/math.pi))

final_rot_mat = np.matmul(getZRotMat(deg2Rad(16)), np.matmul(getYRotMat(deg2Rad(25)), getXRotMat(deg2Rad(45))))

r11 = final_rot_mat[0][0]
r12 = final_rot_mat[0][1]
r13 = final_rot_mat[0][2]
r21 = final_rot_mat[1][0]
r22 = final_rot_mat[1][1]
r23 = final_rot_mat[1][2]
r31 = final_rot_mat[2][0]
r32 = final_rot_mat[2][1]
r33 = final_rot_mat[2][2]

theta = math.acos((r11 + r22 + r33 - 1)/2)

k = (1/(2*math.sin(theta))) * np.array([r32 - r23,
                                      r13 - r31,
                                      r21 - r12])

print(final_rot_mat)
print("Angle: {}".format(rad2Deg(theta)))
print("Axis: {}".format(k))
