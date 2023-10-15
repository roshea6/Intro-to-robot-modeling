import matplotlib.pyplot as plt
import numpy as np
import math

plt.rcParams.update({'font.size': 22})

# Utility functions to make calculations for the satellite trajectory easier
class Utils():
    def __init__(self):
        pass
    # Returns a rotation matrix for rotating around the x axis by the passed in angle
    def getXRotMat(self, angle):
        rot_mat = np.array([
                [1, 0, 0],
                [0, math.cos(angle), -math.sin(angle)],
                [0, math.sin(angle), math.cos(angle)]])
        
        return rot_mat

    # Returns a rotation matrix for rotating around the y axis by the passed in angle
    def getYRotMat(self, angle):
        rot_mat = np.array([
                [math.cos(angle), 0, math.sin(angle)],
                [0, 1, 0],
                [-math.sin(angle), 0, math.cos(angle)]])
        
        return rot_mat

    # Returns a rotation matrix for rotating around the z axis by the passed in angle
    def getZRotMat(self, angle):
        rot_mat = np.array([
                [math.cos(angle), -math.sin(angle), 0],
                [math.sin(angle), math.cos(angle), 0],
                [0, 0, 1]])
        
        return rot_mat
    
    def deg2Rad(self, deg_angle):
        return (deg_angle * (math.pi/180))  

    def rad2Deg(self, rad_angle):
        return (rad_angle * (180/math.pi))
    
    # Converts desired rotations around the x, y, and z axis into axis angle form
    def caclculateAxisAngleForm(self, x_rot, y_rot, z_rot):
        # Multiply the rotation matrices together in the order Z(YX)
        final_rot_mat = np.matmul(self.getZRotMat(self.deg2Rad(z_rot)), 
                                  np.matmul(self.getYRotMat(self.deg2Rad(y_rot)), 
                                            self.getXRotMat(self.deg2Rad(x_rot))))

        # Break out the rotation matrix entries to make the theta and k calculations easier
        r11 = final_rot_mat[0][0]
        r12 = final_rot_mat[0][1]
        r13 = final_rot_mat[0][2]
        r21 = final_rot_mat[1][0]
        r22 = final_rot_mat[1][1]
        r23 = final_rot_mat[1][2]
        r31 = final_rot_mat[2][0]
        r32 = final_rot_mat[2][1]
        r33 = final_rot_mat[2][2]

        # Calculate the angle and vector for the new axis of rotation
        theta = math.acos((r11 + r22 + r33 - 1)/2)

        k = (1/(2*math.sin(theta))) * np.array([r32 - r23,
                                            r13 - r31,
                                            r21 - r12])
        
        print("Axis angle form: theta={} \t k={}".format(theta, k))

        return self.rad2Deg(theta), k

# Class for simulating the orientation of a satellite as it rotates in space over time
class SatelliteTrajectory():
    def __init__(self):
        self.max_ang_vel = 2.0
        
        # Maps axis to the rotation name defines the rotation around them
        self.axis_angle_dict = {"x": ("psi", "x_omega"), "y":("theta", "y_omega"), "z":("phi", "z_omega")}
        
        # Current rotation values for the rotations about the different axis
        self.current_pose = {"psi": 0.0, "theta":0.0, "phi":0.0, "x_omega": 0.0, "y_omega": 0.0, "z_omega": 0.0}
        
        # Intermediate values to plot 
        self.intermediate_vals = {"psi": [0.0], "theta":[0.0], "phi":[0.0], "x_omega": [0.0], "y_omega": [0.0], "z_omega": [0.0]}
        self.angles = ["psi", "theta", "phi"]
        self.angle_vels = ["x_omega", "y_omega", "z_omega"]
        
        # Timesteps to plot against
        self.timesteps = [0.0]
        
        # Iterable list of plot colors
        self.plot_colors = iter(['b', 'g', 'r', 'c', 'm', 'y'])
    
    # ! IMPORTANT: This is old code from my first attempt at the problem before I tried using axis angle representation.
    # The correctly implemented code is in rotateOtherAxis
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
       
       
    def rotateOtherAxis(self, axis_angle, world_angles):
        # Find total time it would take to rotate around the new axis at the max rotation speed
        min_time = axis_angle/self.max_ang_vel
        
        # Calculate the velocity about each axis based on the time it will take to perform the rotation
        scaled_vels = [vel/min_time for vel in world_angles]
        
        print("Angular velocities (x, y, z): {}".format(scaled_vels))
        
        # Find the whole and partial seconds required to perform the full rotation
        whole_sec = int(axis_angle/self.max_ang_vel)
        partial = abs(axis_angle) % self.max_ang_vel
        
        # Get the intermediate values for the state variables for each whole timestep
        for i in range(whole_sec):
            self.current_pose["psi"] += scaled_vels[0]
            self.current_pose["theta"] += scaled_vels[1]
            self.current_pose["phi"] += scaled_vels[2]
            
            self.current_pose["x_omega"] = scaled_vels[0]
            self.current_pose["y_omega"] = scaled_vels[1]
            self.current_pose["z_omega"] = scaled_vels[2]
            
            self.timesteps.append(self.timesteps[-1] + 1.0)
            self.updateIntermediates()
            
        # Get the final values for the state variables based on the remaining partial timestep
        if partial > 0.0:
            self.current_pose["psi"] += scaled_vels[0] * (partial/self.max_ang_vel)
            self.current_pose["theta"] += scaled_vels[1] * (partial/self.max_ang_vel)
            self.current_pose["phi"] += scaled_vels[2] * (partial/self.max_ang_vel)
            
            self.timesteps.append(self.timesteps[-1] + partial/self.max_ang_vel)
            
            self.updateIntermediates()
            
    
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
        # Create two separate plots, one for angles and one for angular velocities
        fig, ax = plt.subplots(nrows=1, ncols=2)
        for key in self.intermediate_vals.keys():
            # Add to angle plot
            if key in self.angles:
                ax[0].plot(self.timesteps, self.intermediate_vals[key], color=next(self.plot_colors), label=key)
            # Add to angular velocity plot
            elif key in self.angle_vels:
                ax[1].plot(self.timesteps, self.intermediate_vals[key], color=next(self.plot_colors), label=key)
            
        ax[0].set_xlabel("Time (seconds)")
        ax[1].set_xlabel("Time (seconds)")
        ax[0].set_ylabel("Degrees")
        ax[1].set_ylabel("Degrees/Second")
        
        ax[0].set_title("Satellite Orientation Over Time")
        ax[1].set_title("Satellite Angular Velocity Over Time")
        
        fig.suptitle("Satellite State Over Time")
        
        fig.legend()
        
        plt.show()
    
    
if __name__ == '__main__':
    traj = SatelliteTrajectory()
    
    util = Utils()
    
    # angle, vec = util.caclculateAxisAngleForm(45.0, 25.0, 16.0)
    
    # traj.rotateOtherAxis(angle, [45.0, 25.0, 16.0])
    
    traj.rotate("x", 45.0)
    traj.rotate("y", 25.0)
    traj.rotate("z", 16.0)
    
    # traj.zeroVels()
    
    traj.displayCurrentState()
    
    traj.displayStatePlot()
    
    
    