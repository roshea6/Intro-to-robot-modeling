#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Imu
from std_msgs.msg import Float64MultiArray
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
import time

class ProportionalControlNode(Node):

    def __init__(self, goal_x_pos):
        super().__init__('proportional_control_node')

        self.goal_x_pos = goal_x_pos

        self.max_vel = 4.0
        self.vel_step_size = 0.1
        self.max_accel = 2.0

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
        self.current_vel = 0.0

        previous_timestamp = self.get_clock().now().nanoseconds
        # previous_timestamp = time.time()

        # Variables for saving the last n timestamps and accelerations so the 
        # accelerations can be averaged
        self.last_n_timestamps = [previous_timestamp]
        self.last_n_accels = [0]
        self.last_n_vels = [0]
        self.n_samples = 2

        self.wheel_radius = 0.1016

        # Proportional component parameter
        self.Kp = 0.2

    def imu_callback(self, imu_msg):
        # Grab just the linear acceleration in the x direction
        lin_x_accel = imu_msg.linear_acceleration.x

        # Band pass filter to filter out IMU acceleration noise
        if abs(lin_x_accel) < 0.1:
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
        latest_vel = self.current_vel + -avg_accel * time_diff

        self.last_n_vels.append(latest_vel)

        # If we've saved more than n samples remove the oldest
        if len(self.last_n_vels) > self.n_samples:
            self.last_n_vels.pop(0)

        avg_vel = sum(self.last_n_vels)/len(self.last_n_vels)

        # Get the distance traveled since the last timestep using the average velocity 
        # over that time
        self.current_x_pos += avg_vel*time_diff

        dist_err = self.goal_x_pos - self.current_x_pos

        # Calculate the error between goal and current vel
        # vel_diff = self.goal_vel - self.current_vel

        # print("IMU vel: {} \t Vel Error: {}".format(self.current_vel, vel_diff))

        # Define our new desired velocity based on the error
        # Multipy the difference by our Kp parameter so we take steps towards the goal
        new_vel = latest_vel + dist_err*self.Kp

        if new_vel > self.max_vel:
            new_vel = self.max_vel
        elif new_vel < -self.max_vel:
            new_vel = -self.max_vel
        

        print("New vel: {} \t Current vel: {}".format(new_vel, latest_vel))

        print("Current X pos: {} \t Goal X pos: {}".format(self.current_x_pos, self.goal_x_pos))

        self.current_vel = latest_vel

        # Calculate the wheel velocities from the desired linear velocity
        wheel_vel = new_vel/self.wheel_radius

        print("Wheel vel: {} \n".format(wheel_vel))

        return

        # Joint velocities message
        wheel_velocities = Float64MultiArray()

        wheel_velocities.data = [wheel_vel,-wheel_vel, wheel_vel, -wheel_vel]
        self.wheel_velocities_pub.publish(wheel_velocities)


def main(args=None):
    rclpy.init(args=args)
    node = ProportionalControlNode(goal_x_pos=20.0)
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()