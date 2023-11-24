import sympy
import numpy as np
import math
from mpl_toolkits import mplot3d
import matplotlib.pyplot as plt
import random

# Rounds values in sympy expressions. Found online 
def roundExpr(expr, num_digits):
    return expr.xreplace({n : round(n, num_digits) for n in expr.atoms(sympy.Number)})

def deg2Rad(deg_angle):
    return (deg_angle * (math.pi/180))

sympy.init_printing(num_columns=275)
# Lambda function from Saksham to truncate values in a matrix
# Removes the super small near zero values that result from floating point operations
roundMatrix = lambda m, n: sympy.Matrix([[round(m[x, y], n) for y in range(m.shape[1])] for x in range(m.shape[0])])

class JacobianUtils():
    def __init__(self, use_symbols=False, display=False):
        # Specity our list of known a and d values
        self.a_list = [0, -0.8407, -0.9141, 0, 0]
        self.d_list = [0.378, 0.0, 0.0, 0.0, 0.6909]
        
        # Create symbols for the the thetas so we can solve with variables
        theta_0, theta_1, theta_2, theta_3, theta_d, theta_4 = sympy.symbols("theta_0, theta_1, theta_2, theta_3, theta_d, theta_4")
        
        # Define thetas as function of t
        # t = sympy.Symbol('t')
        # theta_0, theta_1, theta_2, theta_3, theta_4, theta_5, theta_n = sympy.Function('theta_0')(t), sympy.Function('theta_1')(t), sympy.Function('theta_2')(t), sympy.Function('theta_3')(t), sympy.Function('theta_4')(t), sympy.Function('theta_5')(t), sympy.Function('theta_n')(t),

        self.theta_list = [theta_0, theta_1, theta_2, theta_3, theta_d, theta_4]
        
        # Small value in radians to be added to each of the starting thetas
        max_espilon = 0.00004

        # Calculated values for alphas and thetas in the home position of the robot
        self.alpha_val_list = [math.pi/2, 0.0, 0.0, math.pi/2, 0]
        self.init_theta_val_list = [math.pi, -math.pi/2, 0.0, -math.pi/2, math.pi/2]
        # Add a small epsilon to each starting angle to prevent large velocity jumps. Recommended by Saksham
        self.init_theta_val_list = [val + max_espilon*random.uniform(-1, 1) for val in self.init_theta_val_list]
        # self.init_theta_val_list = [0, 0, 0, 0, 0, 0]
        self.theta_val_list = self.init_theta_val_list

        # Set to true if you want the matrices to be displayed with thetas as a variable
        self.evaluate_with_vars = use_symbols
        self.display = display
        
        self.transformation_mats = []
        self.successive_trans_mats = []
        self.final_trans_mat = None
        
        self.pseudo_inv_j = None
    
    # Calculates the intermediate i-1 to i homogenous transformation matrices, 0 to i, as well as the final 0 to n matrix
    def calculateTransMats(self):
        # Loop through the table and built the individual transformation matrices from frame to frame
        self.transformation_mats = []

        # Calculate the i-1 to i transformation matrix with values from the DH table
        for idx, entry in enumerate(self.a_list):
            a = entry
            d = self.d_list[idx]
            alpha = self.alpha_val_list[idx]
            if self.evaluate_with_vars:
                theta = self.theta_list[idx]
            else:
                theta = self.theta_val_list[idx]
            
            # Create the unique transformation matrix for a given row from the DH table
            trans_mat = sympy.Matrix([[sympy.cos(theta), -sympy.sin(theta)*sympy.cos(alpha), sympy.sin(theta)*sympy.sin(alpha), a*sympy.cos(theta)],
                        [sympy.sin(theta), sympy.cos(theta)*sympy.cos(alpha), -sympy.cos(theta)*sympy.sin(alpha), a*sympy.sin(theta)],
                        [0, sympy.sin(alpha), sympy.cos(alpha), d],
                        [0, 0, 0, 1]])
            
            # Remove near 0 values to clean up the matrix
            if not self.evaluate_with_vars:
                trans_mat = roundMatrix(trans_mat, 4)
            
            self.transformation_mats.append(trans_mat)
            
        # Create a 4x4 identity matrix to use our starting point for matrix multiplication
        res_mat = sympy.Matrix(np.identity(4))
            
        # sympy.pprint(res_mat)

        # Multiply the matrices together in the order T1*T2*T3.... to get the final transformation matrix from frame 0 to n
        self.successive_trans_mats = []
        for idx, trans_mat in enumerate(self.transformation_mats):
            res_mat = res_mat*trans_mat
            
            if not self.evaluate_with_vars:
                res_mat = roundMatrix(res_mat, 5)
            
            else: 
                res_mat = roundExpr(res_mat, 5)
                
            # Keep track of the 0 to i transformation matrices to be used later for jacobian calculations
            self.successive_trans_mats.append(res_mat)
            
        self.final_trans_mat = res_mat
    
    # Uses the most up to date transformation matrices and joint angles to calculate the jacobian matrix and its inverse
    def calculateInvJacobian(self):
        self.calculateTransMats()
        # Calculate the jacobian vectors J1 through n 
        jacobian_vecs = []
        # Define the Z and O vectors for the 0th frame to simplify calculations for J1
        z_0 = np.array([0, 0, 1])
        O0 = np.array([0, 0, 0])
        # Grab the translation vector from the final transformation matrix for On
        On = [self.final_trans_mat.row(i)[3] for i in range(3)]
        
        # sympy.pprint(self.successive_trans_mats)

        if self.display:
            print("On")
            sympy.pprint(On)
            print()
        
        # Calculate the individual Ji components
        for idx in range(len(self.successive_trans_mats)):
            # Special case for J1 since the identity matrix isn't stored as the first matrix in the list
            if idx == 0:
                z_base = z_0
                O_base = O0
            # Otherwise grab Z and O from the 3rd and 4th columns of the i-1 transformation matrix
            else:
                trans_mat = self.successive_trans_mats[idx-1]
                z_base = np.array([trans_mat.row(i)[2] for i in range(3)])
                O_base = np.array([trans_mat.row(i)[3] for i in range(3)])

            if self.display:
                print("Z{}".format(idx))
                sympy.pprint(z_base)  
                print()
                print("O{}".format(idx))
                sympy.pprint(O_base)
                print()
            # Calculate L (On - Oi-1)
            L = On - O_base
            
            # Calculate the cross product of Zi-1 and L. Technically this would be different if we had prismatic joints as well but 
            # Since it's all revolute joints we're gonna keep it simple
            j_vec_1st_half = np.cross(z_base, L)
            
            j_vec = np.append(j_vec_1st_half, z_base)
            
            jacobian_vecs.append(j_vec)
            
        # Combine the individual vectors into a matrix to form the jacobian
        jacobian = sympy.Matrix(np.array(jacobian_vecs).transpose())

        if self.display:
            sympy.pprint(jacobian)
        
        if self.evaluate_with_vars:
            exit()

        # Calculate the pseudo inverse of the jacobian so we can use it to calculate joint velocities
        # Check the determinant to see if we can use the normal inverse or if we need to use the pseudo inverse instead
        psuedo_inv = jacobian.pinv()
        # det = round(jacobian.det(), 5)
        # if det == 0:
        #     psuedo_inv = jacobian.pinv() #(jacobian.T*jacobian).inv()*jacobian.T 
        # else:
        #     psuedo_inv = jacobian.inv()
        # psuedo_inv = roundExpr(psuedo_inv, 5)

        # sympy.pprint(jacobian)
        
        self.pseudo_inv_j = psuedo_inv
    
    # Updates the current joint angles so they can be used to calculate the jacobian at each time step
    def updateThetas(self, new_theta_val_list):
        self.theta_val_list = [new_theta_val_list[idx] for idx in range(len(new_theta_val_list))]


