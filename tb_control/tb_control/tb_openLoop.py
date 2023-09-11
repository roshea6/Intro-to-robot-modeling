import rclpy
from rclpy.node import Node
from tb_control.srv import SetGoal

from geometry_msgs.msg import Twist

import time
import numpy as np

class VelPubNode(Node):
    def __init__(self):
        super().__init__('openloop_controller')

        # Publisher for the turtlebot's velocity
        self.vel_pub = self.create_publisher(Twist, 'cmd_vel', 10)

        # Define the service and link it to a callback function
        self.commmand_srv = self.create_service(SetGoal, 'set_goal', self.executeServNavCB)

        # Define how much of the designated time period the robot should spend accelerating
        # to and decerating from it's steady state speed
        self.accel_time = .2

        # Define the number of increments between the current speed and the desired speed
        self.num_intermediate_vels = 10

    # Receive the service call and publish the necessary velocities
    def executeServNavCB(self, request, response):
        self.get_logger().info('Incoming request\ndist: %d time: %d velocity profile: %s' % (request.dist, request.time, request.profile))

        # Constant velocity for the entirety of the distance 
        if request.profile == "constant":
            # Calculate the velocity required to cover the distance in the specified
            # amount of time
            desired_vel = float(request.dist)/float(request.time)

            # Create a new twist message and populate it with the desired x velocity
            vel_msg = Twist()
            vel_msg.linear.x = desired_vel

            # Publish the message to the turtlebot
            self.vel_pub.publish(vel_msg)

            # Sleep for the time it should take to get to the goal distance
            # TODO: Find a way to do this with ROS time as they'll be different
            time.sleep(request.time)

            # Create a new message with 0 velocity and publish
            vel_msg = Twist()
            vel_msg.linear.x = 0.0

            self.vel_pub.publish(vel_msg)

            response.status = desired_vel
            return response
        
        # Accelerates linearly up to the steady state speed and then decelerates for a smoother stop
        elif request.profile == "lin_accel":
            # Calculate the steady state velocity based on distance to travel, required time,
            # and the acceleration portion 
            steady_vel = (request.dist * (1 + self.accel_time)) / request.time


            # ACCELERATION
            # Create n evenly spaced points for the intermediate velocities
            # Really this generates like n-2 becaus the status and end point are included
            intermediate_vels = np.linspace(0, steady_vel, self.num_intermediate_vels + 2)

            print(intermediate_vels)

            # Loop through the intermediate velocities, publish one, wait, then publish the next
            # until we've hit the steady state vel
            # Grab only the index 1 value on so we don't grab the starting value
            for vel in intermediate_vels[1:]:
                vel_msg = Twist()
                vel_msg.linear.x = vel

                print(vel)

                self.vel_pub.publish(vel_msg)

                # Calculate how much time should be spent at each intermediate vel
                # accel_time should be percentage of total travel time and each intermediate 
                # step should get an equal portion of that time for linear acceleration
                wait_time = (request.time * self.accel_time) / (self.num_intermediate_vels - 1)

                time.sleep(wait_time)
                
            # STEADY STATE
            # Travel at the steady state speed until we hit the time where we need to begin decelerating
            vel_msg = Twist()
            vel_msg.linear.x = steady_vel

            self.vel_pub.publish(vel_msg)

            # Steady vel time should be the total time - 2*accel_time to account for acceleration up
            # to steady and deceleration from steady state to 0
            time.sleep(request.time*(1 - 2*self.accel_time))

            # DECELERATION
            # Reverse the vel list and grab everything but the first
            intermediate_vels = list(reversed(intermediate_vels))

            print(intermediate_vels)

            # Loop through the intermediate velocities backwards, publish one, wait, then publish the next
            # until we've hit 0
            for vel in intermediate_vels[1:]:
                vel_msg = Twist()
                vel_msg.linear.x = vel

                print(vel)

                self.vel_pub.publish(vel_msg)

                wait_time = (request.time * self.accel_time) / (self.num_intermediate_vels - 1)

                time.sleep(wait_time)

            response.status = steady_vel

            return response
        # Ignore any other velocity profile definitions 
        else:
            self.get_logger().warn("Undefined velocity profile type. Service call ignored.")
            response.status = 0
            return response


def main():
    # Initialize the node
    rclpy.init()

    # Create an instance of the Node
    vel_node = VelPubNode()

    # Start the node to run until it is killed
    rclpy.spin(vel_node)

    # Shutdown the node once it exits
    rclpy.shutdown()


if __name__ == '__main__':
    main()
