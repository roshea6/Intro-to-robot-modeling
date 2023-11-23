#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Joy
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy

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

    def joyCallback(self, joy_msg):
        # Create a blank twist message to populate
        twist_msg = Twist()
        
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

        self.vel_pub.publish(twist_msg)

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