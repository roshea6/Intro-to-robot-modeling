import matplotlib.pyplot as plt
import math
import numpy as np

class bicycleKinematics():
    def __init__(self, initial_pose={"x":0, "y":0, "theta":0}):
        self.L = 1.5
        self.r = 0.5/2

        self.pose = initial_pose

        self.current_time = 0

    # Gets the steering angle in radians based on the equation 0.5*sin(pi*t)
    def getSteeringAngle(self, t):
        return 0.5*math.sin(math.pi*t)
    
    # Calculates the derivatives of each of the 3 state values based on 
    # wheel velocity and steering angle
    def calculateStateDerivatives(self, wheel_velocity, steering_angle):
        # Calculate the relative angle based on the steering and robot angle
        relative_angle = steering_angle + self.pose["theta"]
        # print(relative_angle)
        x_dot = wheel_velocity*math.sin(relative_angle)
        y_dot = wheel_velocity*math.cos(relative_angle)
        theta_dot = wheel_velocity*math.sin(steering_angle)/self.L

        return x_dot, y_dot, theta_dot
    
    # Calculates the next state of the bike after a given time delta at a certain 
    # wheel angular speed and steering angle
    def calculateNextState(self, wheel_angular_speed, steering_angle, time_delta):
        wheel_velocity = wheel_angular_speed*self.r

        x_dot, y_dot, theta_dot = self.calculateStateDerivatives(wheel_velocity, steering_angle)

        x_hat = self.pose["x"] + x_dot*time_delta
        y_hat = self.pose["y"] + y_dot*time_delta
        theta_hat = self.pose["theta"] + theta_dot*time_delta

        return x_hat, y_hat, theta_hat
    
    # Plots the state of the bike for a provided set of timestamps and wheel speed
    def plotState(self, wheel_angular_speed, timestamps):
        x_vals = []
        y_vals = []

        # Grab the initial pose
        x_vals.append(self.pose["x"])
        y_vals.append(self.pose["y"])

        for timestamp in timestamps:                
            # Calculate the time delta using the next and current time
            time_delta = timestamp - self.current_time

            # Grab the current steering angle
            steering_angle = self.getSteeringAngle(timestamp)

            x_hat, y_hat, theta_hat = self.calculateNextState(wheel_angular_speed, steering_angle, time_delta)

            # Update the bike's state
            self.pose["x"] = x_hat
            self.pose["y"] = y_hat
            self.pose["theta"] = theta_hat
            self.current_time = timestamp

            # Record the values
            x_vals.append(self.pose["x"])
            y_vals.append(self.pose["y"])

        # Produce and display the plot
        plt.plot(x_vals, y_vals, 'bo')
        plt.title("Bicycle X, Y Position")
        plt.xlabel("X Displacement (m)")
        plt.ylabel("Y Displacement (m)")
        plt.show()



if __name__ == '__main__':
    bike_sim = bicycleKinematics(initial_pose={"x":0, "y":0, "theta":0})

    times = np.linspace(0, 6.0, 100)
    
    bike_sim.plotState(1.0, times)

