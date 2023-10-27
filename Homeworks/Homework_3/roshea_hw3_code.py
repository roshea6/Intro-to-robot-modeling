import sympy
import numpy as np
import math

a_list = [0, -0.6127, -0.5716, 0, 0, 0]
d_list = [0.128, 0, 0, 0.1639, 0.1157, 0.0922]

alpha_0, alpha_1, alpha_2, alpha_3, alpha_4, alpha_5, alpha_n = sympy.symbols("alpha_0, alpha_1, alpha_2, alpha_3, alpha_4, alpha_5, alpha_n")
theta_0, theta_1, theta_2, theta_3, theta_4, theta_5, theta_n = sympy.symbols("theta_0, theta_1, theta_2, theta_3, theta_4, theta_5, theta_n")

alpha_list = [alpha_0, alpha_1, alpha_2, alpha_3, alpha_4, alpha_5, alpha_n]
theta_list = [theta_0, theta_1, theta_2, theta_3, theta_4, theta_5, theta_n]

alpha_val_list = [math.pi/2, math.pi, math.pi, -math.pi/2, math.pi/2, 0]
theta_val_list = [math.pi, -math.pi/2, 0, math.pi/2, 0, 0]
# theta_val_list = [0, 0, 0, 0, 0, 0]

transformation_mats = []

for idx, entry in enumerate(a_list):
    a = entry
    d = d_list[idx]
    alpha = alpha_val_list[idx]
    theta = theta_val_list[idx]
    
    # Create the unique transformation matrix for a given row from the DH table
    trans_mat = sympy.Matrix([[sympy.cos(theta), -sympy.sin(theta)*sympy.cos(alpha), sympy.sin(theta)*sympy.sin(alpha), a*sympy.cos(theta)],
                 [sympy.sin(theta), sympy.cos(theta)*sympy.cos(alpha), -sympy.cos(theta)*sympy.sin(alpha), a*sympy.sin(theta)],
                 [0, sympy.sin(alpha), sympy.cos(alpha), d],
                 [0, 0, 0, 1]])
    
    # sympy.pprint(trans_mat)
    
    transformation_mats.append(trans_mat)
    
# Create a 4x4 identity matrix to use our starting point for matrix multiplication
res_mat = sympy.Matrix(np.identity(4))
    
sympy.pprint(res_mat)

for trans_mat in transformation_mats:
    res_mat = trans_mat*res_mat
    
# sympy.simplify(res_mat)
sympy.pprint(res_mat)