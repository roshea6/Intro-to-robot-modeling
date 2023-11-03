#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Imu
from std_msgs.msg import Float64MultiArray
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
import math
import numpy as np

class ProportionalControlNode(Node):

    def __init__(self, goal_pose):
        super().__init__('proportional_control_node')

        self.goal_x_pos = goal_pose[0]
        self.goal_y_pos = goal_pose[1]

        self.max_vel = 4.0
        self.vel_step_size = 0.1
        self.max_accel = 1.0

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
        
        self.current_x_pos = 0.0
        self.current_y_pos = 0.0
        self.current_vel = 0.0

        previous_timestamp = self.get_clock().now().nanoseconds
        # previous_timestamp = time.time()

        # Variables for saving the last n timestamps and accelerations so the 
        # accelerations can be averaged
        self.last_n_timestamps = [previous_timestamp]
        self.last_n_accels = [0]
        self.last_n_vels = [0]
        self.n_samples = 5

        self.wheel_radius = 0.1016

        # Proportional component parameter
        self.Kp = 0.2

    # Euler from quaternion helper function from resources found online
    # https://gist.github.com/salmagro/2e698ad4fbf9dae40244769c5ab74434
    def euler_from_quaternion(self, imu_msg):
        x = imu_msg.orientation.x
        y = imu_msg.orientation.y
        z = imu_msg.orientation.z
        w = imu_msg.orientation.w

        roll = np.arctan2(2*(w*x + y*z), 1-2*(x*x +y*y))
        pitch = np.arcsin(2*(w*y - z*x))
        yaw = np.arctan2(2*(w*z + x*y), 1-2*(y*y + z*z))

        return roll, pitch, yaw

    def imu_callback(self, imu_msg):
        # Grab just the linear acceleration in the x direction
        lin_x_accel = imu_msg.linear_acceleration.x

        # Get the rpy values from the quaternion orientation
        # Really only need yaw
        roll, pitch, yaw = self.euler_from_quaternion(imu_msg)
        print("Current yaw: {}".format(yaw))

        # Band pass filter to filter out IMU acceleration noise
        if abs(lin_x_accel) < 0.001:
            lin_x_accel = 0
        elif lin_x_accel > self.max_accel:
            lin_x_accel = self.max_accel 
        elif lin_x_accel < -self.max_accel:
            lin_x_accel = -self.max_accel

        # Get the current time and difference from the previous time
        curr_time = self.get_clock().now().nanoseconds
        # curr_time = time.time()

        self.last_n_accels.append(lin_x_accel)
        self.last_n_timestamps.append(curr_time)

        # If we've saved more than n samples remove the oldest
        if len(self.last_n_accels) > self.n_samples:
            self.last_n_accels.pop(0)
            self.last_n_timestamps.pop(0)

        # Calculate the average acceleration using the last n samples
        avg_accel = sum(self.last_n_accels)/len(self.last_n_accels)
        print("Latest accel: {}".format(lin_x_accel))
        print("Accel: {}".format(avg_accel))

        # Calculate the time diff across the oldest and latest acceleration readings
        # Divide by 10^9 to convert to seconds
        time_diff = (self.last_n_timestamps[-1] - self.last_n_timestamps[0])/(10**9)
        print("Time diff: {}".format(time_diff))

        # Calculate current vel based on previous vel, time delta, and accel
        # Negative sign because the robot is technically facing backwards
        latest_vel = self.current_vel + -avg_accel*time_diff

        self.last_n_vels.append(latest_vel)

        # If we've saved more than n samples remove the oldest
        if len(self.last_n_vels) > self.n_samples:
            self.last_n_vels.pop(0)

        avg_vel = sum(self.last_n_vels)/len(self.last_n_vels)

        # Get the distance traveled since the last timestep using the average velocity 
        # over that time
        self.current_x_pos += -avg_vel*time_diff*math.sin(yaw)
        self.current_y_pos += -avg_vel*time_diff*math.cos(yaw)

        x_dist_err = self.goal_x_pos - self.current_x_pos
        y_dist_err = self.goal_y_pos - self.current_y_pos

        # Calculate the error between goal and current vel
        # vel_diff = self.goal_vel - self.current_vel

        # print("IMU vel: {} \t Vel Error: {}".format(self.current_vel, vel_diff))

        # Define our new desired velocity based on the error
        # Multipy the difference by our Kp parameter so we take steps towards the goal
        new_x_vel = latest_vel*math.sin(yaw) + x_dist_err*self.Kp
        new_y_vel = latest_vel*math.cos(yaw) + y_dist_err*self.Kp


        # Bound the x and y vels between the maxes
        new_x_vel = max(min(new_x_vel, self.max_vel), -self.max_vel)
        new_y_vel = max(min(new_y_vel, self.max_vel), -self.max_vel)

        print("Last vels: {}".format(self.last_n_vels))
        print("New vel: ({}, {}) \t Current vel: {}".format(new_x_vel, new_y_vel, latest_vel))

        print("Current pos: ({}, {}) \t Goal pos: ({}, {})".format(self.current_x_pos, self.current_y_pos, self.goal_x_pos, self.goal_y_pos))

        self.current_vel = latest_vel

        # Calculate the wheel velocities from the desired linear velocity
        right_wheel_vel = new_x_vel/self.wheel_radius
        left_wheel_vel = new_y_vel/self.wheel_radius

        print("Wheel vel: ({}, {}) \n".format(right_wheel_vel, left_wheel_vel))

        # Joint velocities message
        wheel_velocities = Float64MultiArray()

        wheel_velocities.data = [left_wheel_vel,-right_wheel_vel, left_wheel_vel, -right_wheel_vel]
        self.wheel_velocities_pub.publish(wheel_velocities)


def main(args=None):
    rclpy.init(args=args)
    node = ProportionalControlNode(goal_pose = (10, 10))
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()