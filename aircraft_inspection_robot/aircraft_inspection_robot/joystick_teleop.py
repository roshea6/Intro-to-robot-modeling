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

from aircraft_inspection_robot.jacobian_utils import JacobianUtils

class JotstickControlNode(Node):
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
        self.command_maxes = {"x_trans": 2.0, "y_trans": 2.0, "z_rot": math.pi/2}

        # Publisher for arm joints
        self.joint_position_pub = self.create_publisher(Float64MultiArray, '/position_controller/commands', 10)
        self.velocities_pub = self.create_publisher(Float64MultiArray, '/velocity_controller/commands', 10)

        # Robot starts in the home configuration with all joints angles set to 0
        self.current_joint_states = [0.0, 0.0, 0.0, 0.0, 0.0]
        self.joint_limits = [(-3.14, 3.14), (-1.57, 1.57), (-1.57, 1.57), (-1.57, 1.57), (-3.14, 3.14)]

        self.joint_step_size = 0.1

        self.j_utils = JacobianUtils()
        self.j_utils.calculateInvJacobian()

        self.current_ee_pos = [0.0, 0.0, 2.8237]
        self.current_ee_vel = [0.0, 0.0, 0.0, 0.0]
        self.current_joint_angles = [0.0, 0.0, 0.0, 0.0, 0.0] #self.j_utils.init_theta_val_list
        self.offset_joint_angles = self.j_utils.init_theta_val_list
        self.current_joint_angle_vels = [0.0, 0.0, 0.0, 0.0, 0.0]

    def joyCallback(self, joy_msg):
        # Create a blank twist message to populate
        twist_msg = Twist()
        
        # Check deadman switch, RB, to see if we should accept joystick input
        if joy_msg.buttons[5]:
            # Extract left stick vertical axis for forwards and backwards
            left_vert = joy_msg.axes[1]
            if left_vert < -self.joy_thresh or left_vert > self.joy_thresh:
                twist_msg.linear.x = left_vert*self.command_maxes["x_trans"]
            else:
                twist_msg.linear.x = 0.0

            # Extract left stick horizontal axis for left and right movement
            left_horiz = joy_msg.axes[0] 
            if left_horiz < -self.joy_thresh or left_horiz > self.joy_thresh:
                twist_msg.linear.y = left_horiz*self.command_maxes["y_trans"]
            else:
                twist_msg.linear.y = 0.0

            # Extract right stick horizontal axis for rotation about the Z axis
            right_horiz = joy_msg.axes[3]
            if right_horiz < -self.joy_thresh or right_horiz > self.joy_thresh:
                twist_msg.angular.z = right_horiz*self.command_maxes["z_rot"]
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
                self.current_ee_vel[0] = self.joint_step_size*right_vert
            else:
                self.current_ee_vel[0] = 0.0

            print("Current ee vel {}".format(self.current_ee_vel))

            # Have to copy the values in a weird way so we don't alter the underlying matrix
            # TODO: Figure out a nicer way to do this
            ee_vel = [val for val in self.current_ee_vel]
            ee_vel.extend([0.0, 0.0])
            # print(ee_vel)
            ee_vel = np.array(ee_vel).transpose()

            #  Find the joint angles based on the previous state and vel
            # TODO: Update time diff to be calculate time delta between runs
            time_diff = 0.001
            for idx, angle in enumerate(self.current_joint_angles):
                self.current_joint_angles[idx] += self.current_joint_angle_vels[idx]*time_diff

            new_internal_arm_angles = [self.offset_joint_angles[i] + self.current_joint_angles[i] for i in range(len(self.current_joint_angles))]

            print("Offset arm angles: {}".format(new_internal_arm_angles))

            # Update the jacobian based on the new angles
            self.j_utils.updateThetas(self.current_joint_angles)
            self.j_utils.calculateInvJacobian()

            # sympy.pprint(self.j_utils.pseudo_inv_j)

            # print(ee_vel)
            
            # Calculate the new joint vels based on the end effector vel
            self.current_joint_angle_vels = np.matmul(self.j_utils.pseudo_inv_j, ee_vel)

            new_joint_positions = Float64MultiArray()

            # print(self.current_joint_angles)

            new_joint_angles = [float(round(angle, 4)) for angle in self.current_joint_angles]

            print("New joint angles: {}".format(new_joint_angles))
            new_joint_positions.data = new_joint_angles

            self.joint_position_pub.publish(new_joint_positions)
            
    

        # self.velocities_pub.publish()


        # new_joint_positions = Float64MultiArray()

        # new_joint_positions.data = self.current_joint_states

        # self.joint_position_pub.publish(new_joint_positions)


def main(args=None):
    rclpy.init(args=args)
    node = JotstickControlNode()
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