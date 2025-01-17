#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Joy
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from std_msgs.msg import Float64MultiArray

import math
import numpy as np
import sympy
import matplotlib.pyplot as plt

from aircraft_inspection_robot.jacobian_utils import JacobianUtils

class ControlNode(Node):
    def __init__(self, use_y_vel=False):
        super().__init__('control_node')

        # Publisher for arm joints
        self.joint_position_pub = self.create_publisher(Float64MultiArray, '/position_controller/commands', 10)
        self.velocities_pub = self.create_publisher(Float64MultiArray, '/velocity_controller/commands', 10)

        self.j_utils = JacobianUtils(damped_jacobian=True)
        self.j_utils.calculateInvJacobian()

        time_to_comp = 20 # seconds to complete the full circle
        self.num_steps = 500 # number of time samples to be taken during time to complete
        self.current_step = 0

        # Generate n timestamps between 0 and the end time
        self.timestamps = np.linspace(0, time_to_comp, self.num_steps)

        self.use_y_vel = use_y_vel


    def runTrajectory(self):
        # Whether to plot in  3D or 2D. 3D plot is very slow especially with high number of steps
        plot_3d = True

        time_to_comp = 20 # seconds to complete the full circle
        num_steps = 500 # number of time samples to be taken during time to complete
        print_every = 100 # Print current end effector position every n steps

        # Generate n timestamps between 0 and the end time
        timestamps = np.linspace(0, time_to_comp, num_steps)

        # Starting values for the end effector position and velocity
        last_stamp = 0
        x_pos = 0
        y_pos = 1.0287
        z_pos = 3.0099
        x_list = []
        y_list = []
        z_list = []
        x_dot = 0
        z_dot = 0

        offset_joint_angles = self.j_utils.init_theta_val_list
        current_rel_joint_angles = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        joint_angle_vels = [0, 0, 0, 0, 0, 0]

        # Loop through the timestamps to find the end effector velocity at each timestamp
        # Use the end effector velocity to calculate the joint angle velocities
        for stamp_num, stamp in enumerate(timestamps):
            time_diff = (stamp - last_stamp)
            
            # Grab the latest position of the end effector with respect to the base frame from the full i to n homogenous transformation matrix
            latest_trans = self.j_utils.final_trans_mat
            x_pos = latest_trans.row(0)[3]
            y_pos = latest_trans.row(1)[3]
            z_pos = latest_trans.row(2)[3]
            
            x_list.append(x_pos)
            y_list.append(y_pos)
            z_list.append(z_pos)
            
            # Calculate the end effector x and z velocities from the parametric circle equation derivatives
            x_dot = -0.157*np.sin(math.pi/2 + .314*stamp)
            if self.use_y_vel:
                y_dot = 0.157*np.cos(math.pi/2 + .314*stamp)
            else:
                y_dot = 0
            z_dot = 0.157*np.cos(math.pi/2 + .314*stamp)
            
            if (stamp_num + 1) % print_every == 0:
                print("Idx: {} \t X: {} \t Y: {} \t Z: {}".format(stamp_num + 1, x_pos, y_pos, z_pos))
            
            # Build the 6x1 end effector state vector
            ee_vel_state = np.array([x_dot, y_dot, z_dot, 0, 0, 0]).transpose()
            
            # Find the joint angles based on the previous state and vel
            for idx, angle in enumerate(current_rel_joint_angles):
                current_rel_joint_angles[idx] += joint_angle_vels[idx]*time_diff

            new_joint_angles = [float(offset_joint_angles[i] + current_rel_joint_angles[i]) for i in range(len(current_rel_joint_angles))]
            
            # Update the jacobian based on the new angles
            self.j_utils.updateThetas(new_joint_angles)
            self.j_utils.calculateInvJacobian()
            
            # Calculate the new joint vels based on the end effector vel
            joint_angle_vels = np.matmul(self.j_utils.pseudo_inv_j, ee_vel_state)
            
            last_stamp = stamp
        
            new_joint_positions = Float64MultiArray()

            new_rel_joint_angles = [float(val) for val in current_rel_joint_angles]

            print("New joint angles: {}".format(new_joint_angles))
            # MAKE SURE TO PUBLISH THE RELATIVE ANGLES NOT THE ONES USED IN THE JACOBIAN
            new_joint_positions.data = new_rel_joint_angles

            self.joint_position_pub.publish(new_joint_positions)


        # Produce and display the plot
        if plot_3d:
            fig = plt.figure()
            ax = fig.add_subplot(projection='3d')
            ax.scatter(x_list, y_list, z_list, c='blue')
            ax.set_title("X, Y, Z Position")
            ax.set_xlabel("X (m)")
            ax.set_ylabel("Y (m)")
            ax.set_zlabel("Z (m)")
            # ax.set_xlim((-.125, .125))
            # ax.set_ylim((0.2561, 0.4561))
            # ax.set_zlim((1.2, 1.45))
            plt.show()
        else:
            plt.plot(x_list, z_list, 'bo')
            plt.title("X, Z Position")
            plt.xlabel("X (m)")
            plt.ylabel("Z (m)")
            # plt.xlim((-.125, .125))
            # plt.ylim((1.2, 1.45))
            plt.show()
        



def main(args=None):
    # Set to true to have the end effector perform a 3D tilted circular trajectory instead
    trajectory_3d = False
    rclpy.init(args=args)
    node = ControlNode(use_y_vel=trajectory_3d)
    try:
        node.runTrajectory()
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    # j_utils = JacobianUtils()
    # j_utils.calculateInvJacobian()
    # for idx, mat in enumerate(j_utils.transformation_mats):
    #     print("T {} to {}".format(idx, idx+1))
    #     sympy.pprint(mat)
    # sympy.pprint(j_utils.final_trans_mat)
    # exit()
    main()