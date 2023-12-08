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
        self.vel_mul = 2.0
        self.command_maxes = {"x_trans": 2.0, "y_trans": 2.0, "z_rot": math.pi/2}

        # Publisher for arm joints
        self.joint_position_pub = self.create_publisher(Float64MultiArray, '/position_controller/commands', 10)
        self.velocities_pub = self.create_publisher(Float64MultiArray, '/velocity_controller/commands', 10)

        # Robot starts in the home configuration with all joints angles set to 0
        # self.current_joint_states = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        self.joint_limits = [(-3.14, 3.14), (-1.57, 1.57), (-1.57, 1.57), (-1.57, 1.57), (-3.14, 3.14)]

        self.joint_step_size = 0.1
        self.ee_vel_scale = 1.0

        self.j_utils = JacobianUtils(damped_jacobian=True)
        self.j_utils.calculateInvJacobian()

        self.current_ee_pos = [0.0, 1.0287, 3.0099]
        self.current_ee_vel = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        self.current_joint_angles = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0] #self.j_utils.init_theta_val_list
        self.offset_joint_angles = self.j_utils.init_theta_val_list
        self.current_joint_angle_vels = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

        self.test_joint_angles = []

        time_to_comp = 20 # seconds to complete the full circle
        self.num_steps = 2000 # number of time samples to be taken during time to complete
        self.current_step = 0

        # Generate n timestamps between 0 and the end time
        self.timestamps = np.linspace(0, time_to_comp, self.num_steps)

        # Robot params for skid steering setup
        self.wheel_radius = 0.4562
        self.wheel_base = 0.73025

    def joyCallback(self, joy_msg):
        # Create a blank twist message to populate
        # twist_msg = Twist()
        
        # Check deadman switch, RB, to see if we should accept joystick input
        if joy_msg.buttons[5]:
            # Extract left stick vertical axis for forwards and backwards
            left_vert = joy_msg.axes[1]
            if left_vert < -self.joy_thresh or left_vert > self.joy_thresh:
                forward_vel = self.vel_mul*left_vert
            else:
                forward_vel = 0

            # Extract right stick horizontal axis for rotation about the Z axis
            right_horiz = joy_msg.axes[3]
            if right_horiz < -self.joy_thresh or right_horiz > self.joy_thresh:
                rot_vel = 3*self.vel_mul*right_horiz
            else:
                rot_vel = 0

            # Calculate the left and right side wheel velocities
            left_vel = -(forward_vel - rot_vel*self.wheel_base/2.0)/self.wheel_radius
            right_vel = -(forward_vel + rot_vel*self.wheel_base/2.0)/self.wheel_radius

            # Create the joint velocities message, populate, and publish it
            wheel_vel_msg = Float64MultiArray()

            wheel_vels = [right_vel, left_vel, right_vel, left_vel]

            wheel_vel_msg.data = wheel_vels

            self.velocities_pub.publish(wheel_vel_msg)

        # Read commands associated with controlling the end effector position
        # Check arm control deadman switch of LB
        if joy_msg.buttons[4]:
            # Extract left stick vertical axis for Z axis movement of the end effector
            left_vert = joy_msg.axes[1]
            if left_vert < -self.joy_thresh or left_vert > self.joy_thresh:
                self.current_ee_vel[2] = self.ee_vel_scale*left_vert
            else:
                self.current_ee_vel[2] = 0.0

            # Extract left stick horizontal axis for Y axis movement of the end effector
            left_horiz = joy_msg.axes[0] 
            if left_horiz < -self.joy_thresh or left_horiz > self.joy_thresh:
                self.current_ee_vel[1] = self.ee_vel_scale*left_horiz
            else:
                self.current_ee_vel[1] = 0.0

            # Extract right stick vertical axis for end effector x movement
            right_vert = joy_msg.axes[4]
            if right_vert < -self.joy_thresh or right_vert > self.joy_thresh:
                self.current_ee_vel[0] = self.ee_vel_scale*right_vert
            else:
                self.current_ee_vel[0] = 0.0

            # Have to copy the values in a weird way so we don't alter the underlying matrix
            # TODO: Figure out a nicer way to do this
            ee_vel = [val for val in self.current_ee_vel]
            print("Current ee vel {}".format(ee_vel))

            #  Find the joint angles based on the previous state and vel
            # TODO: Update time diff to be calculate time delta between runs
            time_diff = 0.05
            joint_steps = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
            for idx, angle in enumerate(self.current_joint_angles):
                self.current_joint_angles[idx] += self.current_joint_angle_vels[idx]*time_diff

            new_internal_arm_angles = [self.offset_joint_angles[i] + self.current_joint_angles[i] for i in range(len(self.current_joint_angles))]

            print("Offset arm angles: {}".format(new_internal_arm_angles))

            # Update the jacobian based on the new angles
            self.j_utils.updateThetas(new_internal_arm_angles)
            self.j_utils.calculateInvJacobian()

            # Calculate the new joint vels based on the end effector vel
            self.current_joint_angle_vels = np.matmul(self.j_utils.pseudo_inv_j, ee_vel)

            new_joint_positions = Float64MultiArray()

            new_joint_angles = [float(angle) for angle in self.current_joint_angles]

            print("New joint angles: {}".format(new_joint_angles))
            new_joint_positions.data = new_joint_angles

            self.joint_position_pub.publish(new_joint_positions)

            ee_pos = [self.j_utils.final_trans_mat.row(i)[3] for i in range(3)]

            print("EE Position {}".format(ee_pos))
            print()

        # Otherwise set all end effector translation vels to 0
        else:
            for i in range(3):
                self.current_ee_vel[i] = 0

        # Read commands associated with controlling the end effector position
        # Check end effector rotation deadman switch of RT
        if joy_msg.axes[5] < -self.joy_thresh:
            # Extract left stick vertical axis for Z axis rotation of the end effector
            left_vert = joy_msg.axes[1]
            if left_vert < -self.joy_thresh or left_vert > self.joy_thresh:
                self.current_ee_vel[3] = self.ee_vel_scale*left_vert
            else:
                self.current_ee_vel[3] = 0.0

            # Extract left stick horizontal axis for Y axis rotation of the end effector
            left_horiz = joy_msg.axes[0] 
            if left_horiz < -self.joy_thresh or left_horiz > self.joy_thresh:
                self.current_ee_vel[5] = self.ee_vel_scale*left_horiz
            else:
                self.current_ee_vel[5] = 0.0

            # Extract right stick horizontal axis for end effector x rotation
            right_horiz = joy_msg.axes[3]
            if right_horiz < -self.joy_thresh or right_horiz > self.joy_thresh:
                self.current_ee_vel[4] = self.ee_vel_scale*right_horiz
            else:
                self.current_ee_vel[4] = 0.0

            # Have to copy the values in a weird way so we don't alter the underlying matrix
            # TODO: Figure out a nicer way to do this
            ee_vel = [val for val in self.current_ee_vel]
            print("Current ee vel {}".format(ee_vel))

            #  Find the joint angles based on the previous state and vel
            # TODO: Update time diff to be calculate time delta between runs
            time_diff = 0.05
            for idx, angle in enumerate(self.current_joint_angles):
                self.current_joint_angles[idx] += self.current_joint_angle_vels[idx]*time_diff

            new_internal_arm_angles = [self.offset_joint_angles[i] + self.current_joint_angles[i] for i in range(len(self.current_joint_angles))]

            print("Offset arm angles: {}".format(new_internal_arm_angles))

            # Update the jacobian based on the new angles
            self.j_utils.updateThetas(new_internal_arm_angles)
            self.j_utils.calculateInvJacobian()

            # Calculate the new joint vels based on the end effector vel
            self.current_joint_angle_vels = np.matmul(self.j_utils.pseudo_inv_j, ee_vel)

            new_joint_positions = Float64MultiArray()

            new_joint_angles = [float(angle) for angle in self.current_joint_angles]

            print("New joint angles: {}".format(new_joint_angles))
            new_joint_positions.data = new_joint_angles

            self.joint_position_pub.publish(new_joint_positions)

            ee_pos = [self.j_utils.final_trans_mat.row(i)[3] for i in range(3)]

            print("EE Position {}".format(ee_pos))
            print()
        # Otherwise set all end effector translation vels to 0
        else:
            for i in range(3):
                self.current_ee_vel[i+3] = 0



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