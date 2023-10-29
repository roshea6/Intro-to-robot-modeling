import sympy
import numpy as np
import math

# Specity our list of known a and d values
a_list = [0, -0.6127, -0.5716, 0, 0, 0]
d_list = [0.128, 0, 0, 0.1639, 0.1157, 0.0922]

# Create symbols for the the alphas and thetas so we can solve with variables
alpha_0, alpha_1, alpha_2, alpha_3, alpha_4, alpha_5, alpha_n = sympy.symbols("alpha_0, alpha_1, alpha_2, alpha_3, alpha_4, alpha_5, alpha_n")
theta_0, theta_1, theta_2, theta_3, theta_4, theta_5, theta_n = sympy.symbols("theta_0, theta_1, theta_2, theta_3, theta_4, theta_5, theta_n")

alpha_list = [alpha_0, alpha_1, alpha_2, alpha_3, alpha_4, alpha_5, alpha_n]
theta_list = [theta_0, theta_1, theta_2, theta_3, theta_4, theta_5, theta_n]

# Calculated values for alphas and thetas in the home position of the robot
alpha_val_list = [math.pi/2, math.pi, math.pi, -math.pi/2, math.pi/2, 0]
theta_val_list = [math.pi, -math.pi/2, 0, math.pi/2, 0, 0]

# Set to true if you want the matrices to be displayed with thetas as a variable
evaluate_with_vars = False

# If we want to actuate a joint define the joint number and the amount to rotate it by
joint_num_to_rotate = 0
rotate_amount = 0

# Add the rotation to the existing home theta values
theta_val_list[joint_num_to_rotate] += rotate_amount

transformation_mats = []

# Lambda function from Saksham to truncate values in a matrix
# Removes the super small near zero values that result from floating point operations
roundMatrix = lambda m, n: sympy.Matrix([[round(m[x, y], n) for y in range(m.shape[1])] for x in range(m.shape[0])])

# Loop through the table and built the individual transformation matrices from frame to frame
for idx, entry in enumerate(a_list):
    a = entry
    d = d_list[idx]
    alpha = alpha_val_list[idx]
    if evaluate_with_vars:
        theta = theta_list[idx]
    else:
        theta = theta_val_list[idx]
    
    # Create the unique transformation matrix for a given row from the DH table
    trans_mat = sympy.Matrix([[sympy.cos(theta), -sympy.sin(theta)*sympy.cos(alpha), sympy.sin(theta)*sympy.sin(alpha), a*sympy.cos(theta)],
                 [sympy.sin(theta), sympy.cos(theta)*sympy.cos(alpha), -sympy.cos(theta)*sympy.sin(alpha), a*sympy.sin(theta)],
                 [0, sympy.sin(alpha), sympy.cos(alpha), d],
                 [0, 0, 0, 1]])
    
    # trans_mat = trans_mat.evalf(5)
    
    # Remove near 0 values to clean up the matrix
    if not evaluate_with_vars:
        trans_mat = roundMatrix(trans_mat, 5)
    
    print("Transform matrix from frame {} to {}".format(idx, idx+1))
    sympy.pprint(trans_mat)
    
    print()
    
    transformation_mats.append(trans_mat)
    
# Create a 4x4 identity matrix to use our starting point for matrix multiplication
res_mat = sympy.Matrix(np.identity(4))
    
# sympy.pprint(res_mat)

# Multiply the matrices together in the order T1*T2*T3.... to get the final transformation matrix from frame 0 to n
for trans_mat in transformation_mats:
    res_mat = res_mat*trans_mat

# Again remove any near 0 values from floating point operations
if not evaluate_with_vars:
    res_mat = roundMatrix(res_mat, 5)

# res_mat = res_mat.evalf(5)
    
# sympy.simplify(res_mat)
print("Final transformation matrix from frame 0 to n")
sympy.pprint(res_mat)