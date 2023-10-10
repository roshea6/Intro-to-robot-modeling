import math

# Define the rotation matrix we derived
rot_mat = [[1, 0, 0],
           [0, 0, 1],
           [0, -1, 0]]

# Calculate the individual components for rotations about each axis
theta_x = math.atan2(rot_mat[2][1], rot_mat[2][2])
theta_y = math.atan2(-rot_mat[2][0], math.sqrt((rot_mat[2][1]**2) + (rot_mat[2][2]**2)))
theta_z = math.atan2(rot_mat[1][0], rot_mat[0][0])

print(theta_x)
print(theta_y)
print(theta_z)