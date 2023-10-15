from math import cos, sin, atan2, sqrt

# Define the rotation matrix we derived
rot_mat = [[1, 0, 0],
           [0, 0, -1],
           [0, 1, 0]]

# Break out the rotation matrix entries to make the theta and k calculations easier
r11 = rot_mat[0][0]
r12 = rot_mat[0][1]
r13 = rot_mat[0][2]
r21 = rot_mat[1][0]
r22 = rot_mat[1][1]
r23 = rot_mat[1][2]
r31 = rot_mat[2][0]
r32 = rot_mat[2][1]
r33 = rot_mat[2][2]

# Calculate the individual components for rotations about each axis
theta = atan2(r32, r33)
phi = atan2(-r31, sqrt(r32**2 + r33**2))
psi = atan2(r21, r11)

print("Rotation Matrix: {} \n".format(rot_mat))

print("Corresponding Euler Angles")
print("Theta around x: {}".format(theta))
print("Phi around y: {}".format(phi))
print("Psi around z: {} \n".format(psi))

# Recreate the rotation matrix from the angles for a sanity checl
rot_mat_from_angles = [[cos(phi)*cos(psi), sin(theta)*sin(phi)*cos(psi) - cos(theta)*sin(psi), cos(theta)*sin(phi)*cos(psi) + sin(theta)*sin(psi)],
                       [cos(phi)*sin(psi), sin(theta)*sin(phi)*sin(psi) - cos(theta)*cos(psi), cos(theta)*sin(phi)*sin(psi) - sin(theta)*cos(psi)],
                       [-sin(phi), sin(theta)*cos(phi), cos(theta)*cos(phi)]]

print("Rotation matrix from the recovered angles: {}".format(rot_mat_from_angles))