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
import matplotlib.pyplot as plt

class ProportionalControlNode(Node):

    def __init__(self, goal_pose):
        super().__init__('proportional_control_node')
        # IMPORTANT! Set this value to the average real time factor that you're 
        # Gazebo sim runs at to account for the time differences
        # Mine runs at like 0.35 when recording and like 0.75 when not
        self.real_time_factor = 0.80

        self.goal_x_pos = goal_pose[0]
        self.goal_y_pos = goal_pose[1]

        # Acceptable distance from the goal tolerance
        self.goal_tolerance = 0.5

        self.max_vel = 2.0
        self.vel_step_size = 0.01
        self.max_accel = 0.5
        self.max_steer_angle = 0.7854

        self.last_n_accels = [0]
        self.last_n_vels = [0]
        self.n_samples = 2

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
        self.previous_vel = 0.0
        self.current_vel = 0.0
        self.current_steer_angle = 0.0
        self.max_steer_angle_step = 0.01

        self.goal_reached = False

        self.previous_timestamp = self.get_clock().now().nanoseconds

        self.wheel_radius = 0.1016

        # Proportional component parameter
        self.Kp = 0.0005

        self.timestep_list = []
        self.error_list = []
        self.vel_control_list = []
        self.steer_control_list = []

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

        # Get the current time and difference from the previous time
        curr_time = self.get_clock().now().nanoseconds

        # Calculate the time diff across the oldest and latest acceleration readings
        # Divide by 10^9 to convert to seconds
        # time_diff = (self.last_n_timestamps[-1] - self.last_n_timestamps[0])/(10**9)
        time_diff = (curr_time - self.previous_timestamp)/(10**9)
        time_diff *= self.real_time_factor
        self.previous_timestamp = curr_time
        print("Time diff: {}".format(time_diff))

        # Calculate the measured velocity from the IMU 
        measured_vel = self.current_vel + lin_x_accel*time_diff
        
        self.current_vel = measured_vel

        # Get the distance traveled since the last timestep using the average velocity 
        # over that time
        self.current_x_pos += measured_vel*time_diff*math.sin(yaw+self.current_steer_angle)
        self.current_y_pos += measured_vel*time_diff*math.cos(yaw+self.current_steer_angle)

        if not self.goal_reached:
            x_dist_err = self.goal_x_pos - self.current_x_pos
            y_dist_err = self.goal_y_pos - self.current_y_pos
            total_dist_err = math.sqrt((x_dist_err**2) + (y_dist_err**2))
        else:
            x_dist_err = 0
            y_dist_err = 0
            total_dist_err = 0

        # Display the current steer angle and calculate the next one needed to steer us towards the goal
        print("Steer Angle: {}".format(self.current_steer_angle))
        self.steer_control_list.append(self.current_steer_angle)
        new_steer_angle = math.pi/4 - math.atan2(y_dist_err, x_dist_err)

        # First make sure to only increment by the max step size
        if new_steer_angle - self.current_steer_angle > self.max_steer_angle_step:
            new_steer_angle += self.max_steer_angle_step
        elif new_steer_angle - self.current_steer_angle < -self.max_steer_angle_step:
            new_steer_angle -= self.max_steer_angle_step

        # Bound the steering angle to within joint limits
        self.current_steer_angle = max(min(new_steer_angle, self.max_steer_angle), -self.max_steer_angle)

        # If we're within a certain tolerance of the goal consider it reached and set the velocities 
        # to zero
        if x_dist_err < self.goal_tolerance and y_dist_err < self.goal_tolerance:
            new_x_vel = 0
            new_y_vel = 0
            self.current_vel = 0
            self.goal_reached = True
            print("GOAL REACHED!!!")
        else:
            # Define our new desired velocity based on the error
            # Multipy the difference by our Kp parameter so we take steps towards the goal
            new_x_vel = -self.previous_vel*math.sin(yaw) - x_dist_err*self.Kp
            new_y_vel = -self.previous_vel*math.cos(yaw) - y_dist_err*self.Kp
            self.goal_reached = False

        print("Total dist error: {}".format(total_dist_err))

        new_vel = -math.sqrt(new_x_vel**2 + new_y_vel**2)
        # Bound the vel between the maxes
        new_vel = max(min(new_vel, self.max_vel), -self.max_vel)
        # new_y_vel = max(min(new_y_vel, self.max_vel), -self.max_vel)

        # print("Last vels: {}".format(self.last_n_vels))
        print("Current IMU vel: {}".format(measured_vel))
        self.previous_vel = new_vel #math.sqrt(new_x_vel**2 + new_y_vel**2)
        print("New command vel: ({})".format(-new_vel))

        print("Current pos: ({}, {}) \t Goal pos: ({}, {})".format(self.current_x_pos, self.current_y_pos, self.goal_x_pos, self.goal_y_pos))


        # Calculate the wheel velocities from the desired linear velocity
        wheel_vel = -new_vel/self.wheel_radius
        # left_wheel_vel = -(new_y_vel/math.cos(yaw))/self.wheel_radius

        print("Wheel vel: ({}, {}) \n".format(wheel_vel, wheel_vel))

        # Joint velocities message
        wheel_velocities = Float64MultiArray()

        wheel_velocities.data = [wheel_vel,-wheel_vel, wheel_vel, -wheel_vel]
        self.wheel_velocities_pub.publish(wheel_velocities)

        # Steer position message
        steer_angle = Float64MultiArray()
        steer_angle.data = [self.current_steer_angle, self.current_steer_angle]

        self.joint_position_pub.publish(steer_angle)

        # Add the timestep, error, and control to their respective lists
        if len(self.timestep_list) == 0:
            self.timestep_list.append(1)
        else:
            self.timestep_list.append(self.timestep_list[-1] + 1)
        self.error_list.append(total_dist_err)
        self.vel_control_list.append(-self.previous_vel)

    # Plot the distance error and velocity control over time
    def plotControls(self):
        plt.plot(self.timestep_list, self.error_list, label = "Distance Error") 
        plt.plot(self.timestep_list, self.vel_control_list, label = "Velocity Control") 
        plt.plot(self.timestep_list, self.steer_control_list, label = "Steer Control") 
        plt.legend() 
        plt.title("Control & Distance Error Over Time")
        plt.xlabel("Timestep number")
        plt.ylabel("Velocity (m/s), Steer Angle (rad), Distance Error (m)")
        plt.show()


def main(args=None):
    rclpy.init(args=args)
    # Wrap this all in a try finall so that after the node is killed with ctrl +c 
    # it still calls the plotting function
    node = ProportionalControlNode(goal_pose = (10, 10))
    try:
        rclpy.spin(node)
    finally:
        # After the node has been killed call the plotting function
        node.plotControls()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()