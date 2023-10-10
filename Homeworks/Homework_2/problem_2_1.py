import matplotlib.pyplot as plt

class SatelliteTrajectory():
    def __init__(self):
        self.max_ang_vel = 2.0
        
        # Maps axis to the rotation name defines the rotation around them
        self.axis_angle_dict = {"x": ("psi", "x_omega"), "y":("theta", "y_omega"), "z":("phi", "z_omega")}
        
        # Current rotation values for the rotations about the different axis
        self.current_pose = {"psi": 0.0, "theta":0.0, "phi":0.0, "x_omega": 0.0, "y_omega": 0.0, "z_omega": 0.0}
        
        # Intermediate values to plot 
        self.intermediate_vals = {"psi": [0.0], "theta":[0.0], "phi":[0.0], "x_omega": [0.0], "y_omega": [0.0], "z_omega": [0.0]}
        
        # Timesteps to plot against
        self.timesteps = [0.0]
        
        # Iterable list of plot colors
        self.plot_colors = iter(['b', 'g', 'r', 'c', 'm', 'y'])
    
    # Rotates the satellite about the specified axis until the current angle is equal to the goal angle
    def rotate(self, axis_name, goal_angle):
        # Grab the desired angle based on the passed in axis
        angle_name, angle_vel_name = self.axis_angle_dict[axis_name]
        
        # Check distance between current angle and goal angle
        degrees_to_rotate = goal_angle - self.current_pose[angle_name]
        
        if degrees_to_rotate > 0:
            max_vel = self.max_ang_vel # Rotating ccw so using positive rotational velocity
        elif degrees_to_rotate < 0:
            max_vel = -self.max_ang_vel # Rotating cw so using negative rotational velocity
        # Already at goal angle no need to rotate
        else:
            return
        
        # Calculate how many seconds it will take to reach the goal angle at max angular velocity
        # This portion calculates the whole number portion because the int function naturally floors
        num_steps = int(abs(degrees_to_rotate/self.max_ang_vel))
        
        # This calculate the remainder portion which we need to handle differently so we don't overshoot the goal angle
        remainder = abs(degrees_to_rotate) % self.max_ang_vel
    
        # Loop through the whole steps and increment the desired angle while recording the timesteps and pose for that timestep
        for i in range(num_steps):
            self.current_pose[angle_name] += max_vel
            self.current_pose[angle_vel_name] = max_vel
            
            # Add the next timestep which should be one second greater than the previous
            self.timesteps.append(self.timesteps[-1] + 1.0)
            
            # Record current pose
            self.updateIntermediates()
            
        # If there is a remainder then rotate at the max speed for a fraction of a second and update the timestep accordingly
        if remainder > 0.0:
            if max_vel > 0:
                self.current_pose[angle_name] += remainder
            elif max_vel < 0:
                self.current_pose[angle_name] -= remainder
            
            # Set velocity to 0 since we've finished the rotation
            self.current_pose[angle_vel_name] = 0.0
            
            # Add a new timestep that's a fraction of a second
            self.timesteps.append(self.timesteps[-1] + remainder/self.max_ang_vel)
            
            self.updateIntermediates()
            
        # Set velocity to 0 since we've finished the rotation. This already happens in the remainder section but this takes care
        # of it in case there's no remainder
        self.current_pose[angle_vel_name] = 0.0
            
            
    # Adds the current values for the pose to the intermediate values dict
    def updateIntermediates(self):
        for key in self.current_pose.keys():
            self.intermediate_vals[key].append(self.current_pose[key])
            
    # Zeros out all velocities and advances one timestep. Used to show the end state of the satellite
    def zeroVels(self):
        for item in ["x_omega", "y_omega", "z_omega"]:
            self.current_pose[item] = 0.0
            
        self.timesteps.append(self.timesteps[-1] + 1.0)
            
        self.updateIntermediates()
    
    # Nicely displays the current satellite state. Mostly for debugging
    def displayCurrentState(self):
        for key in self.current_pose.keys():
            print("{}: {}".format(key, self.current_pose[key]))
            
        print("time: {}".format(self.timesteps[-1]))
    
    # Plots the 6 state variables over time on a single plot
    def displayStatePlot(self):
        for key in self.intermediate_vals.keys():
            plt.plot(self.timesteps, self.intermediate_vals[key], color=next(self.plot_colors), label=key)
            
        plt.xlabel("Time (seconds)")
        plt.ylabel("Value (degrees and degrees/second)")
        plt.title("Satellite State over Time")
        
        plt.legend()
        
        plt.show()
    
    
if __name__ == '__main__':
    traj = SatelliteTrajectory()
    
    traj.rotate("x", 45.0)
    traj.rotate("y", 25.0)
    traj.rotate("z", 16.0)
    
    traj.zeroVels()
    
    traj.displayCurrentState()
    
    traj.displayStatePlot()
    
    
    