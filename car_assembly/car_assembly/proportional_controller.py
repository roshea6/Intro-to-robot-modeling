#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Imu
from std_msgs.msg import Float64MultiArray
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
import math
import numpy as np
import time

class ProportionalControlNode(Node):

    def __init__(self, goal_pose):
        super().__init__('proportional_control_node')
        # IMPORTANT! Set this value to the average real time factor that you're 
        # Gazebo sim runs at to account for the time differences
        # Mine runs at like 0.35
        self.real_time_factor = 0.6

        self.goal_x_pos = goal_pose[0]
        self.goal_y_pos = goal_pose[1]

        # Acceptable distance from the goal tolerance
        self.goal_tolerance = 0.5

        self.max_vel = 1.0
        self.vel_step_size = 0.01
        self.max_accel = 0.5

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
        self.previous_vel = 0.0

        self.previous_timestamp = self.get_clock().now().nanoseconds
        # previous_timestamp = time.time()

        # Variables for saving the last n timestamps and accelerations so the 
        # accelerations can be averaged
        self.last_n_timestamps = [self.previous_timestamp]
        self.last_n_accels = [0]
        self.last_n_vels = [0]
        self.n_samples = 5

        self.wheel_radius = 0.1016

        # Proportional component parameter
        self.Kp = 0.05

        # self.last_simulation_time = self.get_clock().now().nanoseconds
        # self.last_wall_clock_time = time.time()*10**9
        # print(self.get_clock().now())
        # print(self.last_simulation_time)
        # print(self.last_wall_clock_time*10**9)
        # self.timer = self.create_timer(1.0, self.calculate_realtime_factor)

    # TODO: Doesn't work but I'll probably get back to this later
    # Calculates the real time factor in Gazebo so we can scale our time appropriately
    # Might not work in ROS 2 cause the clocks are weird
    # def calculate_realtime_factor(self):
    #     current_simulation_time = self.get_clock().now().nanoseconds
    #     simulation_duration = current_simulation_time - self.last_simulation_time
    #     print(simulation_duration)

    #     current_wall_clock_time = time.time()*10**9
    #     wall_clock_duration = current_wall_clock_time - self.last_wall_clock_time
    #     print(wall_clock_duration)

    #     real_time_factor = simulation_duration / wall_clock_duration
    #     self.get_logger().info("Real-time factor: %.2f" % real_time_factor)

    #     self.last_simulation_time = current_simulation_time
    #     self.last_wall_clock_time = current_wall_clock_time

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
        # if abs(lin_x_accel) < 0.1:
        #     lin_x_accel = 0
        # elif lin_x_accel > self.max_accel:
        #     lin_x_accel = self.max_accel 
        # elif lin_x_accel < -self.max_accel:
        #     lin_x_accel = -self.max_accel

        # Get the current time and difference from the previous time
        curr_time = self.get_clock().now().nanoseconds
        # curr_time = time.time()

        # self.last_n_accels.append(lin_x_accel)
        # self.last_n_timestamps.append(curr_time)

        # # If we've saved more than n samples remove the oldest
        # if len(self.last_n_accels) > self.n_samples:
        #     self.last_n_accels.pop(0)
        #     self.last_n_timestamps.pop(0)

        # # Calculate the average acceleration using the last n samples
        # avg_accel = sum(self.last_n_accels)/len(self.last_n_accels)
        # print("Latest accel: {}".format(lin_x_accel))
        # print("Accel: {}".format(avg_accel))

        # Calculate the time diff across the oldest and latest acceleration readings
        # Divide by 10^9 to convert to seconds
        # time_diff = (self.last_n_timestamps[-1] - self.last_n_timestamps[0])/(10**9)
        time_diff = (curr_time - self.previous_timestamp)/(10**9)
        time_diff *= self.real_time_factor
        self.previous_timestamp = curr_time
        print("Time diff: {}".format(time_diff))

        # Calculate current vel based on previous vel, time delta, and accel
        # Negative sign because the robot is technically facing backwards
        # vel_inc = min(max(-avg_accel*time_diff, -self.vel_step_size), self.vel_step_size)
        # print("Vel increment: {}".format(vel_inc))
        # latest_vel = self.current_vel + vel_inc

        # self.last_n_vels.append(latest_vel)

        # If we've saved more than n samples remove the oldest
        # if len(self.last_n_vels) > self.n_samples:
        #     self.last_n_vels.pop(0)

        # avg_vel = sum(self.last_n_vels)/len(self.last_n_vels)

        # Get the distance traveled since the last timestep using the average velocity 
        # over that time
        self.current_x_pos += -self.previous_vel*time_diff*math.sin(yaw)
        self.current_y_pos += -self.previous_vel*time_diff*math.cos(yaw)

        x_dist_err = self.goal_x_pos - self.current_x_pos
        y_dist_err = self.goal_y_pos - self.current_y_pos

        if x_dist_err < self.goal_tolerance and y_dist_err < self.goal_tolerance:
            new_x_vel = 0
            new_y_vel = 0
            print("GOAL REACHED!!!")
        else:
            # Define our new desired velocity based on the error
            # Multipy the difference by our Kp parameter so we take steps towards the goal
            new_x_vel = -self.previous_vel*math.sin(yaw) + x_dist_err*self.Kp
            new_y_vel = -self.previous_vel*math.cos(yaw) + y_dist_err*self.Kp

        # Calculate the error between goal and current vel
        # vel_diff = self.goal_vel - self.current_vel

        # print("IMU vel: {} \t Vel Error: {}".format(self.current_vel, vel_diff))


        # Bound the x and y vels between the maxes
        new_x_vel = max(min(new_x_vel, self.max_vel), -self.max_vel)
        new_y_vel = max(min(new_y_vel, self.max_vel), -self.max_vel)

        # print("Last vels: {}".format(self.last_n_vels))
        print("Current vel: {}".format(self.previous_vel))
        self.previous_vel = math.sqrt(new_x_vel**2 + new_y_vel**2)
        print("New vel: ({}, {}) \t Next vel: {}".format(new_x_vel, new_y_vel, self.previous_vel))

        print("Current pos: ({}, {}) \t Goal pos: ({}, {})".format(self.current_x_pos, self.current_y_pos, self.goal_x_pos, self.goal_y_pos))


        # Calculate the wheel velocities from the desired linear velocity
        right_wheel_vel = -(new_x_vel/math.sin(yaw))/self.wheel_radius
        left_wheel_vel = -(new_y_vel/math.cos(yaw))/self.wheel_radius

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