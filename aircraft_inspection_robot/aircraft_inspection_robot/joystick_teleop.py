#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Joy
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from std_msgs.msg import Float64MultiArray

import math

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

        # Robot starts in the home configuration with all joints angles set to 0
        self.current_joint_states = [0.0, 0.0, 0.0, 0.0, 0.0]
        self.joint_limits = [(-3.14, 3.14), (-1.57, 1.57), (-1.57, 1.57), (-1.57, 1.57), (-3.14, 3.14)]

        self.joint_step_size = 0.01

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
            # Extract left stick vertical axis movement of arm base joint
            left_vert = joy_msg.axes[1]
            if left_vert < -self.joy_thresh or left_vert > self.joy_thresh:
                self.current_joint_states[1] += self.joint_step_size*left_vert

            # Extract left stick horizontal axis for movement of arm link 1 joint
            left_horiz = joy_msg.axes[0] 
            if left_horiz < -self.joy_thresh or left_horiz > self.joy_thresh:
                self.current_joint_states[0] += self.joint_step_size*left_horiz

            # Extract right stick horizontal axis for movement of arm link 2 joint
            right_horiz = joy_msg.axes[3]
            if right_horiz < -self.joy_thresh or right_horiz > self.joy_thresh:
                self.current_joint_states[2] += self.joint_step_size*right_horiz

            # Extract right stick vertical axis for movement of arm link 3 joint
            right_vert = joy_msg.axes[4]
            if right_vert < -self.joy_thresh or right_vert > self.joy_thresh:
                self.current_joint_states[3] += self.joint_step_size*right_vert

            print("Current joint states {}".format(self.current_joint_states))

        new_joint_positions = Float64MultiArray()

        new_joint_positions.data = self.current_joint_states

        self.joint_position_pub.publish(new_joint_positions)


def main(args=None):
    rclpy.init(args=args)
    node = JotstickControlNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()