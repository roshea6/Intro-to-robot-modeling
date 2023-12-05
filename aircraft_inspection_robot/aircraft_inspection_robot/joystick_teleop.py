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

class JoystickControlNode(Node):
    def __init__(self):
        super().__init__('joystick_control_node')

        # Define a custom QoS profile to match the one provided by the IMU
        qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1
        )
        
        self.joy_sub = self.create_subscription(Joy, "/joy", self.joyCallback, qos_profile=qos_profile)

        self.vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        self.joy_thresh = 0.25
        self.vel_mul = 3.0
        self.command_maxes = {"x_trans": 2.0, "y_trans": 2.0, "z_rot": math.pi/2}

        # Publisher for arm joints
        self.joint_position_pub = self.create_publisher(Float64MultiArray, '/position_controller/commands', 10)
        self.velocities_pub = self.create_publisher(Float64MultiArray, '/velocity_controller/commands', 10)

        # Robot starts in the home configuration with all joints angles set to 0
        self.current_joint_states = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        self.joint_limits = [(-3.14, 3.14), (-1.57, 1.57), (-1.57, 1.57), (-1.57, 1.57), (-3.14, 3.14)]

        self.joint_step_size = 0.5

        self.j_utils = JacobianUtils()
        self.j_utils.calculateInvJacobian()

        self.current_ee_pos = [0.0, 1.0287, 3.0099]
        self.current_ee_vel = [0.0, 0.0, 0.0, 0.0]
        self.current_joint_angles = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0] #self.j_utils.init_theta_val_list
        self.offset_joint_angles = self.j_utils.init_theta_val_list
        self.current_joint_angle_vels = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

        self.test_joint_angles = []

        time_to_comp = 20 # seconds to complete the full circle
        self.num_steps = 2000 # number of time samples to be taken during time to complete
        self.current_step = 0

        # Generate n timestamps between 0 and the end time
        self.timestamps = np.linspace(0, time_to_comp, self.num_steps)


    def joyCallback(self, joy_msg):
        # Create a blank twist message to populate
        twist_msg = Twist()
        
        # Check deadman switch, RB, to see if we should accept joystick input
        if joy_msg.buttons[5]:
            # Extract left stick vertical axis for forwards and backwards
            left_vert = joy_msg.axes[1]
            if left_vert < -self.joy_thresh or left_vert > self.joy_thresh:
                twist_msg.linear.x = self.vel_mul*left_vert*self.command_maxes["x_trans"]
            else:
                twist_msg.linear.x = 0.0

            # Extract left stick horizontal axis for left and right movement
            left_horiz = joy_msg.axes[0] 
            if left_horiz < -self.joy_thresh or left_horiz > self.joy_thresh:
                twist_msg.linear.y = self.vel_mul*left_horiz*self.command_maxes["y_trans"]
            else:
                twist_msg.linear.y = 0.0

            # Extract right stick horizontal axis for rotation about the Z axis
            right_horiz = joy_msg.axes[3]
            if right_horiz < -self.joy_thresh or right_horiz > self.joy_thresh:
                twist_msg.angular.z = self.vel_mul*right_horiz*self.command_maxes["z_rot"]
            else:
                twist_msg.angular.z = 0.0
        # Otherwise publish 0 vels
        else:
            twist_msg.linear.x = 0.0
            twist_msg.linear.y = 0.0
            twist_msg.angular.z = 0.0

        self.vel_pub.publish(twist_msg)

        # Read commands associated with controlling the end effector position
        # Check arm control deadman switch of LB
        if joy_msg.buttons[4]:
            # Extract left stick vertical axis for Z axis movement of the end effector
            left_vert = joy_msg.axes[1]
            if left_vert < -self.joy_thresh or left_vert > self.joy_thresh:
                self.current_ee_vel[2] = self.joint_step_size*left_vert
            else:
                self.current_ee_vel[2] = 0.0

            # Extract left stick horizontal axis for Y axis movement of the end effector
            left_horiz = joy_msg.axes[0] 
            if left_horiz < -self.joy_thresh or left_horiz > self.joy_thresh:
                self.current_ee_vel[1] = self.joint_step_size*left_horiz
            else:
                self.current_ee_vel[1] = 0.0

            # Extract right stick horizontal axis for end effector rotation
            right_horiz = joy_msg.axes[3]
            if right_horiz < -self.joy_thresh or right_horiz > self.joy_thresh:
                self.current_ee_vel[3] = self.joint_step_size*right_horiz
            else:
                self.current_ee_vel[3] = 0.0

            # Extract right stick vertical axis for end effector x movement
            right_vert = joy_msg.axes[4]
            if right_vert < -self.joy_thresh or right_vert > self.joy_thresh:
                self.current_ee_vel[0] = 3*self.joint_step_size*right_vert
            else:
                self.current_ee_vel[0] = 0.0

            # Have to copy the values in a weird way so we don't alter the underlying matrix
            # TODO: Figure out a nicer way to do this
            ee_vel = [val for val in self.current_ee_vel]
            ee_vel.extend([0.0, 0.0])
            ee_vel = [-0.5, 0.0, -0.5, 0.0, 0.0, 0.0]
            # print(ee_vel)
            # ee_vel = np.array(ee_vel).transpose()
            print("Current ee vel {}".format(ee_vel))

            #  Find the joint angles based on the previous state and vel
            # TODO: Update time diff to be calculate time delta between runs
            time_diff = 0.0001
            for idx, angle in enumerate(self.current_joint_angles):
                self.current_joint_angles[idx] += self.current_joint_angle_vels[idx]*time_diff

            new_internal_arm_angles = [self.offset_joint_angles[i] + self.current_joint_angles[i] for i in range(len(self.current_joint_angles))]

            print("Offset arm angles: {}".format(new_internal_arm_angles))

            # Update the jacobian based on the new angles
            self.j_utils.updateThetas(new_internal_arm_angles)
            self.j_utils.calculateInvJacobian()

            # sympy.pprint(self.j_utils.pseudo_inv_j)

            # print(ee_vel)
            
            # print("Inverse Jacobian")
            # sympy.pprint(self.j_utils.pseudo_inv_j)
            # Calculate the new joint vels based on the end effector vel
            self.current_joint_angle_vels = np.matmul(self.j_utils.pseudo_inv_j, ee_vel)

            new_joint_positions = Float64MultiArray()

            # print(self.current_joint_angles)

            new_joint_angles = [float(angle) for angle in self.current_joint_angles]

            print("New joint angles: {}".format(new_joint_angles))
            new_joint_positions.data = new_joint_angles

            self.joint_position_pub.publish(new_joint_positions)

            # for idx, trans_mat in enumerate(self.j_utils.transformation_mats):
            #     print("{} to {}".format(idx, idx+1))
            #     sympy.pprint(trans_mat)
            #     print()


            sympy.pprint(self.j_utils.final_trans_mat)

            print("Joint vels {}".format(self.current_joint_angle_vels))
            print("Relative arm angles {}".format(new_joint_angles))
            print("Internal arm angles {}".format(self.j_utils.theta_val_list))

            ee_pos = [self.j_utils.final_trans_mat.row(i)[3] for i in range(3)]

            print("EE Position {}".format(ee_pos))
            print()

            # ee_x, ee_y, ee_z = ee_pos[0], ee_pos[1], ee_pos[2]

            # self.test_joint_angles.append(ee_pos)

            # fig = plt.figure()
            # ax = fig.add_subplot(projection='3d')
            # ax.scatter(ee_x, ee_y, ee_z, c='blue')
            # ax.set_title("X, Y, Z Position")
            # ax.set_xlabel("X (m)")
            # ax.set_ylabel("Y (m)")
            # ax.set_zlabel("Z (m)")
            # # ax.set_xlim((-.125, .125))
            # # ax.set_ylim((0.2561, 0.4561))
            # # ax.set_zlim((1.2, 1.45))
            # plt.show()

            # new_joint_vels = Float64MultiArray()

            # vels = [float(round(vel, 4)) for vel in self.current_joint_angle_vels]
            # new_joint_vels.data = vels

            # self.velocities_pub.publish(new_joint_vels)
            
    

        # self.velocities_pub.publish()


        # new_joint_positions = Float64MultiArray()

        # new_joint_positions.data = self.current_joint_states

        # self.joint_position_pub.publish(new_joint_positions)


def main(args=None):
    rclpy.init(args=args)
    node = JoystickControlNode()
    try:
        rclpy.spin(node)
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