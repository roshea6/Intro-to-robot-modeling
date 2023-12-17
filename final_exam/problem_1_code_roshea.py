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

sympy.init_printing(num_columns=400)
# Lambda function from Saksham to truncate values in a matrix
# Removes the super small near zero values that result from floating point operations
roundMatrix = lambda m, n: sympy.Matrix([[round(m[x, y], n) for y in range(m.shape[1])] for x in range(m.shape[0])])

class JacobianUtils():
    def __init__(self, use_symbols=False, display=False):
        # Specity our list of known a and d values
        self.a_list = [0.0, 0.0, -1.0, -1.0]
        self.d_list = [0.2, 1.0, 0.0, 0.0]
        
        # Physical properties for dynamics
        self.link_masses = [1.0, 12.92, 12.92, 12.92]
        # These are the relative center of masses for a link. Their position wrt to the robot base will be calculated at each timestep based on current joint angles
        self.link_cms = [[0.1, 0.1, 0.1],
                         [0.045, 0.045, 0.5],
                         [0.5, 0.045, 0.045],
                         [-0.5, 0.045, 0.045]]
        
        # Create symbols for the the thetas so we can solve with variables
        theta_0, theta_1, theta_2, theta_3= sympy.symbols("theta_0, theta_1, theta_2, theta_3")
        
        # Define thetas as function of t
        # t = sympy.Symbol('t')
        # theta_0, theta_1, theta_2, theta_3, theta_4, theta_5, theta_n = sympy.Function('theta_0')(t), sympy.Function('theta_1')(t), sympy.Function('theta_2')(t), sympy.Function('theta_3')(t), sympy.Function('theta_4')(t), sympy.Function('theta_5')(t), sympy.Function('theta_n')(t),

        # Need to have a theta 0 var for the partial derivatives even though it never changes
        self.theta_list = [theta_0, theta_1, theta_2, theta_3]
        
        # Small value in radians to be added to each of the starting thetas
        max_espilon = 0.004

        # Calculated values for alphas and thetas in the home position of the robot
        self.alpha_val_list = [0, math.pi/2, 0, 0]
        self.init_theta_val_list = [0, 0, math.pi + math.pi/4, 0 - math.pi/4]
        # Add a small epsilon to each starting angle to prevent large velocity jumps. Recommended by Saksham
        # self.init_theta_val_list = [val + max_espilon*random.uniform(-1, 1) for val in self.init_theta_val_list]
        self.theta_val_list = self.init_theta_val_list

        # Set to true if you want the matrices to be displayed with thetas as a variable
        self.evaluate_with_vars = use_symbols
        self.display = display
        
        self.transformation_mats = []
        self.successive_trans_mats = []
        self.final_trans_mat = None
        
        self.var_transformation_mats = []
        self.var_successive_trans_mats = []
        self.var_final_trans_mat = None
        
        self.pseudo_inv_j = None
        self.jacobian_T = None
        
        self.gravity_mat = None
        
        # Used to setup the symbolic transformation matrices on the first run only
        self.first_run = True
    
    # Calculates the intermediate i-1 to i homogenous transformation matrices, 0 to i, as well as the final 0 to n matrix
    def calculateTransMats(self):
        # Loop through the table and built the individual transformation matrices from frame to frame
        self.transformation_mats = []

        # Calculate the i-1 to i transformation matrix with values from the DH table
        for idx, entry in enumerate(self.a_list):
            a = entry
            d = self.d_list[idx]
            alpha = self.alpha_val_list[idx]
           
            theta = self.theta_val_list[idx]
            
            # Create the unique transformation matrix for a given row from the DH table
            trans_mat = sympy.Matrix([[sympy.cos(theta), -sympy.sin(theta)*sympy.cos(alpha), sympy.sin(theta)*sympy.sin(alpha), a*sympy.cos(theta)],
                        [sympy.sin(theta), sympy.cos(theta)*sympy.cos(alpha), -sympy.cos(theta)*sympy.sin(alpha), a*sympy.sin(theta)],
                        [0, sympy.sin(alpha), sympy.cos(alpha), d],
                        [0, 0, 0, 1]])
            
            # trans_mat = roundMatrix(trans_mat, 5)
            
            self.transformation_mats.append(trans_mat)
            
        # Create a 4x4 identity matrix to use our starting point for matrix multiplication
        res_mat = sympy.Matrix(np.identity(4))
            
        # sympy.pprint(res_mat)

        # Multiply the matrices together in the order T1*T2*T3.... to get the final transformation matrix from frame 0 to n
        self.successive_trans_mats = []
        for idx, trans_mat in enumerate(self.transformation_mats):
            res_mat = res_mat*trans_mat
            
            res_mat = roundMatrix(res_mat, 5)
                
            # Keep track of the 0 to i transformation matrices to be used later for jacobian calculations
            self.successive_trans_mats.append(res_mat)
            
        self.final_trans_mat = res_mat
        
        # If we're also evaulating with variables calculate it all again
        if self.evaluate_with_vars:
            # self.first_run = False
             # Loop through the table and built the individual transformation matrices from frame to frame
            self.var_transformation_mats = []

            # Calculate the i-1 to i transformation matrix with values from the DH table
            for idx, entry in enumerate(self.a_list):
                a = entry
                d = self.d_list[idx]
                alpha = self.alpha_val_list[idx]
            
                theta = self.theta_list[idx]
                
                # Create the unique transformation matrix for a given row from the DH table
                trans_mat = sympy.Matrix([[sympy.cos(theta), -sympy.sin(theta)*sympy.cos(alpha), sympy.sin(theta)*sympy.sin(alpha), a*sympy.cos(theta)],
                            [sympy.sin(theta), sympy.cos(theta)*sympy.cos(alpha), -sympy.cos(theta)*sympy.sin(alpha), a*sympy.sin(theta)],
                            [0, sympy.sin(alpha), sympy.cos(alpha), d],
                            [0, 0, 0, 1]])
                
                # trans_mat = roundExpr(trans_mat, 5)
                
                self.var_transformation_mats.append(trans_mat)
                
            # Create a 4x4 identity matrix to use our starting point for matrix multiplication
            res_mat = sympy.Matrix(np.identity(4))
                
            # sympy.pprint(res_mat)

            # Multiply the matrices together in the order T1*T2*T3.... to get the final transformation matrix from frame 0 to n
            self.var_successive_trans_mats = []
            for idx, trans_mat in enumerate(self.var_transformation_mats):
                res_mat = res_mat*trans_mat

                # res_mat = roundExpr(res_mat, 5)
                    
                # Keep track of the 0 to i transformation matrices to be used later for jacobian calculations
                self.var_successive_trans_mats.append(res_mat)
                
            self.var_final_trans_mat = res_mat
    
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
        # sympy.pprint(jacobian)
        
        # if self.evaluate_with_vars:
        #     continue

        # Calculate the pseudo inverse of the jacobian so we can use it to calculate joint velocities
        # Check the determinant to see if we can use the normal inverse or if we need to use the pseudo inverse instead
        # det = round(jacobian.det(), 5)
        # if det == 0:
        psuedo_inv = jacobian.pinv() #(jacobian.T*jacobian).inv()*jacobian.T 
        # else:
        #     psuedo_inv = jacobian.inv()
        # psuedo_inv = roundExpr(psuedo_inv, 5)
        
        self.pseudo_inv_j = psuedo_inv
        
        self.jacobian_T = jacobian.transpose()
        
    # Uses the most up to date transformation matrices and joint angles to calculate the jacobian matrix and its inverse
    def displayVarJacobian(self):
        self.calculateTransMats()
        # Calculate the jacobian vectors J1 through n 
        jacobian_vecs = []
        # Define the Z and O vectors for the 0th frame to simplify calculations for J1
        z_0 = np.array([0, 0, 1])
        O0 = np.array([0, 0, 0])
        # Grab the translation vector from the final transformation matrix for On
        On = [self.var_final_trans_mat.row(i)[3] for i in range(3)]
        
        # sympy.pprint(self.successive_trans_mats)

        if self.display:
            print("On")
            sympy.pprint(On)
            print()
        
        # Calculate the individual Ji components
        for idx in range(len(self.var_successive_trans_mats)):
            # Special case for J1 since the identity matrix isn't stored as the first matrix in the list
            if idx == 0:
                z_base = z_0
                O_base = O0
            # Otherwise grab Z and O from the 3rd and 4th columns of the i-1 transformation matrix
            else:
                trans_mat = self.var_successive_trans_mats[idx-1]
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

        print("Analytical Jacobian")
        sympy.pprint(jacobian)
        
        # Create a list of substitution pairs of the theta variable and the actual theta value
        substitutions = [(theta_var, theta_val) for theta_var, theta_val in zip(self.theta_list, self.theta_val_list)]
        eval_j = jacobian.subs(substitutions)
        
        eval_j = roundExpr(eval_j, 5)
        
        print("Jacobian at home position")
        sympy.pprint(eval_j)
        
    
    # Updates the current joint angles so they can be used to calculate the jacobian at each time step
    def updateThetas(self, new_theta_val_list):
        self.theta_val_list = [new_theta_val_list[idx] for idx in range(len(new_theta_val_list))]
    
    # Calculates the potential energy for each link of the system and then converts it the gravity matrix
    def calcPotEnergy(self):
        total_pot_energy = 0
        # Define the gravity vector
        g_vec = np.array([0, 0, 9.81]).transpose()
        # Zip the link masses, center of masses, and the 0 to i transformation matrices and iterate through them
        for link_mass, link_cm, trans_mat in zip(self.link_masses, self.link_cms, self.var_successive_trans_mats):
            # Extract the translation vector
            t_vec = [trans_mat.row(i)[3] for i in range(3)]
            
            # Calculate rci by adding the link center of mass location to the translation vector
            rci = np.array([link_cm[i] + t_vec[i] for i in range(len(link_cm))]).transpose()

            # Calculate potential energy for the link and add it to the total
            total_pot_energy += link_mass*(np.dot(g_vec, rci))
            
        # print(total_pot_energy)
            
        # Calculate the partial derivative of the potential enerty wrt each of the joint angles
        pot_energy_partials = sympy.Matrix([sympy.diff(total_pot_energy, var) for var in self.theta_list])
        
        
        if self.first_run:
            sympy.pprint(roundExpr(pot_energy_partials, 4))
            self.first_run = False
        
        # Create a list of substitution pairs of the theta variable and the actual theta value
        substitutions = [(theta_var, theta_val) for theta_var, theta_val in zip(self.theta_list, self.theta_val_list)]
        
        # Substitute the values in to get the actualy value of the potential energy partial derivative matrix
        # Take its negative since the Lagrangian is K - P
        self.gravity_mat = -pot_energy_partials.subs(substitutions)
        
        # sympy.pprint(self.gravity_mat)
        
    
    # Calculates the torque at each joint based on the gravity matrix, jacobian, and external force vector at the end effector
    def calcJointTorques(self):
        force_vec = sympy.Matrix(np.array([-10.0, 0.0, 0.0, 0.0, 0.0, 0.0]).transpose())
        joint_torques = self.gravity_mat - self.jacobian_T*force_vec
        
        return joint_torques
    

# Define object for working with the jacobian and calculate the initial one for end effector position estimation
j_utils = JacobianUtils(use_symbols=True, display=False)

j_utils.calculateInvJacobian()
# j_utils.displayVarJacobian()

# for idx, mat in enumerate(j_utils.successive_trans_mats):
#     print("{} to {}".format(0, idx+1))
#     sympy.pprint(mat)
    
# for idx, mat in enumerate(j_utils.transformation_mats):
#     print("{} to {}".format(idx, idx+1))
#     sympy.pprint(mat)
    
# # sympy.pprint(j_utils.final_trans_mat)
    
# exit()

# Whether to plot in  3D or 2D. 3D plot is very slow especially with high number of steps
plot_3d = False

# If all the torques should be plotted on the same graph
plot_single_torque_graph = False

time_to_comp = 20 # seconds to complete the full circle
num_steps = 200 # number of time samples to be taken during time to complete
print_every = 100 # Print current end effector position every n steps

# Generate n timestamps between 0 and the end time
timestamps = np.linspace(0, time_to_comp, num_steps)

# Starting values for the end effector position and velocity
last_stamp = 0
x_pos = 1.707
y_pos = 0.0
z_pos = 1.907
x_list = []
y_list = []
z_list = []
x_dot = 0
z_dot = 0

joint_angles = j_utils.init_theta_val_list
joint_angle_vels = [0, 0, 0, 0]
offset_joint_angles = [0, 0, math.pi/4, -math.pi/4]
joint_angle_list = [[], [], [], []]

torque_list = [[], [], [], []]

# Loop through the timestamps to find the end effector velocity at each timestamp
# Use the end effector velocity to calculate the joint angle velocities
for stamp_num, stamp in enumerate(timestamps):
    time_diff = (stamp - last_stamp)
    
    # Grab the latest position of the end effector with respect to the base frame from the full i to n homogenous transformation matrix
    latest_trans = j_utils.final_trans_mat
    # sympy.pprint(latest_trans)
    x_pos = latest_trans.row(0)[3]
    y_pos = latest_trans.row(1)[3]
    z_pos = latest_trans.row(2)[3]
    
    x_list.append(x_pos)
    y_list.append(y_pos)
    z_list.append(z_pos)
    
    # Calculate the system potential energy and gravity matrix
    j_utils.calcPotEnergy()
    
    # Calculate the joint torques from the updated gravity matrix and record them
    torques = np.array(j_utils.calcJointTorques()).astype(np.float64)
    for joint_torque_list, torque in zip(torque_list, torques):
        joint_torque_list.append(torque[0]) 
    
    # Calculate the end effector x and z velocities from the parametric circle equation derivatives
    # x_dot = -0.00314*np.sin(math.pi/2 + .0314*stamp)
    z_dot = -0.03535
    
    if ((stamp_num + 1) % print_every == 0) or (stamp_num == 0):
        print("Idx: {} \t X: {} \t Y: {} \t Z: {}".format(stamp_num + 1, x_pos, y_pos, z_pos))
    
    # Build the 6x1 end effector state vector
    ee_vel_state = np.array([0, 0, z_dot, 0, 0, 0]).transpose()
    # print(j_utils.pseudo_inv_j.shape)
    
    # Find the joint angles based on the previous state and vel
    for idx, angle in enumerate(joint_angles):
        joint_angles[idx] += joint_angle_vels[idx]*time_diff
        
        # Just for keeping track of the joint angles over time to plot
        offset_joint_angles[idx] += joint_angle_vels[idx]*time_diff
        
    for angle_list, joint_angle in zip(joint_angle_list, offset_joint_angles):
        angle_list.append(joint_angle)
    
        
    # Update the jacobian based on the new angles
    j_utils.updateThetas(joint_angles)
    j_utils.calculateInvJacobian()
    
    # print(joint_angles)
    
    # Calculate the new joint vels based on the end effector vel
    joint_angle_vels = np.matmul(j_utils.pseudo_inv_j, ee_vel_state)
    
    print(joint_angle_vels)
    
    last_stamp = stamp
    
# Produce and display the plot
if plot_3d:
    fig = plt.figure()
    ax = fig.add_subplot(projection='3d')
    ax.scatter(x_list, y_list, z_list, c='blue')
    ax.set_title("X, Y, Z Position")
    ax.set_xlabel("X (m)")
    ax.set_ylabel("Y (m)")
    ax.set_zlabel("Z (m)")
    ax.set_xlim((-.125, .125))
    ax.set_ylim((0.2561, 0.4561))
    ax.set_zlim((1.2, 1.45))
    plt.show()
else:
    plt.plot(x_list, z_list, 'bo')
    plt.title("X, Z Position")
    plt.xlabel("X (m)")
    plt.ylabel("Z (m)")
    # plt.xlim((-.125, .125))
    # plt.ylim((1.2, 1.45))
    plt.show()
    
    # Iterable list of plot colors
    plot_colors = iter(['b', 'g', 'r', 'c', 'm', 'y'])
    joint_names = ["Fixed Base Joint", "Joint 1", "Joint 2", "Joint 3"]
    
    n_rows = 2
    n_cols = 2
    fig, ax = plt.subplots(nrows=n_rows, ncols=n_cols)
    for idx, (offset_joint_angle_list, joint_name) in enumerate(zip(joint_angle_list, joint_names)):
        # Calculates row and col number for each plot based on number of rows and cols
        row_num = 0 if idx < n_rows else 1
        col_num = idx - n_cols*row_num
        ax[row_num, col_num].plot(timestamps, offset_joint_angle_list, label=joint_name, color=next(plot_colors))
        ax[row_num, col_num].set_xlabel("Timestamp (s)")
        ax[row_num, col_num].set_ylabel("Joint angle (degrees)")
        ax[row_num, col_num].set_title(joint_name)
        
    fig.suptitle("Joint Angles Over Time")
    fig.legend()
    plt.show()
    
    # Redefine the color iterable so we don't run out of colors
    plot_colors = iter(['b', 'g', 'r', 'c', 'm', 'y'])
    fig, ax = plt.subplots(nrows=n_rows, ncols=n_cols)
    for idx, (single_torque_list, joint_name) in enumerate(zip(torque_list, joint_names)):
        # Calculates row and col number for each plot based on number of rows and cols
        row_num = 0 if idx < n_rows else 1
        col_num = idx - n_cols*row_num
        ax[row_num, col_num].plot(timestamps, single_torque_list, label=joint_name, color=next(plot_colors))
        ax[row_num, col_num].set_xlabel("Timestamp (s)")
        ax[row_num, col_num].set_ylabel("Torque (N*m)")
        ax[row_num, col_num].set_title(joint_name)
        
    fig.suptitle("Joint Torques Over Time")
    fig.legend()
    plt.show()
    