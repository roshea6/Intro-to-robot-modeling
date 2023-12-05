#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64MultiArray
from sensor_msgs.msg import JointState
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from sensor_msgs.msg import Joy

import sys
import select
import tty
import termios
from pynput import keyboard

import math
import numpy as np
import sympy

from aircraft_inspection_robot.jacobian_utils import JacobianUtils


# Define key codes
LIN_VEL_STEP_SIZE = 1
ANG_VEL_STEP_SIZE = 0.1

class KeyboardControlNode(Node):

    def __init__(self):
        super().__init__('keyboard_control_node')

        self.joint_position_pub = self.create_publisher(Float64MultiArray, '/position_controller/commands', 10)
        self.wheel_velocities_pub = self.create_publisher(Float64MultiArray, '/velocity_controller/commands', 10)
        self.joint_state_pub = self.create_publisher(JointState, '/joint_states', 10)

        self.settings = termios.tcgetattr(sys.stdin)

        self.current_joint_states = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

        self.j_utils = JacobianUtils()
        self.j_utils.calculateInvJacobian()

        self.current_ee_pos = [0.0, 1.0287, 3.0099]
        self.current_ee_vel = [0.0, 0.0, 0.0, 0.0]
        self.current_joint_angles = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0] #self.j_utils.init_theta_val_list
        self.offset_joint_angles = self.j_utils.init_theta_val_list
        self.current_joint_angle_vels = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

        self.base_move_dict = {"w": ("x", 1), "s": ("x", -1), "a": ("y", 1), "d": ("y", -1), "q": ("z", 1), "e": ("z", -1)}

        self.wheel_radius = .1016

    def getKey(self):
        tty.setraw(sys.stdin.fileno())
        rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
        if rlist:
            key = sys.stdin.read(1)
        else:
            key = ''

        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)
        return key

    def run_keyboard_control(self):
        self.msg = """
        Control Your Car!
        ---------------------------
        Moving around:
            w
        a    s    d

        q : force stop

        Esc to quit
        """

        self.get_logger().info(self.msg)
        joint_positions = Float64MultiArray()
        wheel_velocities = Float64MultiArray()
        linear_vel=0.0
        steer_angle=0.0


        while True:
            key = self.getKey()
            if key is not None:
                if key == '\x1b':  # Escape key
                    break
                elif key == 'q':  # Quit
                    linear_vel=0.0
                    steer_angle=0.0
                elif key == 'w':  # Forward
                    linear_vel += LIN_VEL_STEP_SIZE
                elif key == 's':  # Reverse
                    linear_vel -= LIN_VEL_STEP_SIZE
                elif key == 'd':  # Right
                    steer_angle -= ANG_VEL_STEP_SIZE
                elif key == 'a':  # Left
                    steer_angle += ANG_VEL_STEP_SIZE


                if steer_angle>1.5:
                    steer_angle=1.5
                if steer_angle<-1.5:
                    steer_angle=-1.5

                print("Steer Angle",steer_angle)
                print("Linear Velocity",(linear_vel*self.wheel_radius))
                # Publish the twist message
                wheel_velocities.data = [linear_vel,-linear_vel,linear_vel,-linear_vel]
                joint_positions.data = [steer_angle,steer_angle]

                self.joint_position_pub.publish(joint_positions)
                self.wheel_velocities_pub.publish(wheel_velocities)

def main(args=None):
    rclpy.init(args=args)
    node = KeyboardControlNode()
    node.run_keyboard_control()
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()