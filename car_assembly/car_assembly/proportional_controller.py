#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Imu
from std_msgs.msg import Float64MultiArray
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy

class ProportionalControlNode(Node):

    def __init__(self, goal_vel = 2.0):
        super().__init__('proportional_control_node')

        self.goal_vel = goal_vel

        # Define publishers for the joint and wheel states
        self.joint_position_pub = self.create_publisher(Float64MultiArray, '/position_controller/commands', 10)
        self.wheel_velocities_pub = self.create_publisher(Float64MultiArray, '/velocity_controller/commands', 10)

        # Define a custom QoS profile to match the one provided by the IMU
        qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1
        )

        self.subscription = self.create_subscription(
            Imu,
            'imu_plugin/out',
            self.imu_callback,
            qos_profile=qos_profile)
        
        self.current_vel = 0.0

        self.previous_timestamp = self.get_clock().now().nanoseconds

        self.wheel_radius = 0.1016

        # Proportional component parameter
        self.Kp = 0.75

    def imu_callback(self, imu_msg):
        # Grab just the linear acceleration in the x direction
        lin_x_accel = imu_msg.linear_acceleration.x

        # Get the current time and difference from the previous time
        curr_time = self.get_clock().now().nanoseconds

        time_diff = (curr_time - self.previous_timestamp)/ (10**9)

        self.previous_timestamp = curr_time

        # Calculate current vel based on previous vel, time delta, and accel
        # Negative sign because the robot is technically facing backwards
        self.current_vel += -lin_x_accel * time_diff

        # Calculate the error between goal and current vel
        vel_diff = self.goal_vel - self.current_vel

        print("IMU vel: {} \t Vel Error: {}".format(self.current_vel, vel_diff))

        # Define our new desired velocity based on the error
        # Multipy the difference by our Kp parameter so we take steps towards the goal
        new_vel = self.current_vel + vel_diff*self.Kp

        print("New Goal vel: {} \t Actual Goal Vel: {}".format(new_vel, self.goal_vel))

        # Calculate the wheel velocities from the desired linear velocity
        wheel_vel = new_vel/self.wheel_radius

        print("Wheel vel: {} \n".format(wheel_vel))

        # Joint velocities message
        wheel_velocities = Float64MultiArray()

        wheel_velocities.data = [wheel_vel,-wheel_vel, wheel_vel, -wheel_vel]
        self.wheel_velocities_pub.publish(wheel_velocities)


def main(args=None):
    rclpy.init(args=args)
    node = ProportionalControlNode(goal_vel=1.0)
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()