import sympy
import numpy as np
import math
from mpl_toolkits import mplot3d
import matplotlib.pyplot as plt
import random

# plt.rcParams.update({'font.size': 14})

def deg2Rad(deg_angle):
    return (deg_angle * (math.pi/180))

max_rot = math.pi/3
slither_period = 60 # Time to complete one slither cycle in seconds
angle_increment = (2*math.pi)/slither_period

time_to_comp = 180 # Total seconds to run for
num_steps = 20000 # number of time samples to be taken during time to complete
print_every = 1000 # Print current joint angles every n steps

# Generate n timestamps between 0 and the end time
timestamps = np.linspace(0, time_to_comp, num_steps)

# Starting values for the end effector position and velocity
last_stamp = 0

joint_angles = [0, 0, 0, 0, 0, 0, 0, 0, 0, 0]
joint_angle_list = [[] for angle in joint_angles]

# Joint rotation direction with respect to the head frame
# Can be changed to affect the shape of the gait
# Current choice is just based on a sample reverse S shape that can be taken on by the robot
rot_directions = [1, 1, 1, 1, -1, -1, -1, 1, 1, 1]

# Offsets added to each of the joint rotations to make rotation at a given joint more or less extreme
# Essentially can be used to increase the amplitude of the waveform that the trajectory forms
# And to maintain the S like robot shape throughout the motion
degree_offsets = [0, 15, 30, 15, 0, -15, -30, -15, 0, 15]
# Redefine the offsets in radians. It's much easier to originally write it in degrees
offsets = [deg2Rad(angle) for angle in degree_offsets]

joint_angles = [offset for offset in offsets]


# Loop through the timestamps to find joint angles at each timestamp
for stamp_num, stamp in enumerate(timestamps):
    # Record the current joint angles for a given timestamp so they can be plotted later
    for idx, angle in enumerate(joint_angles):
        joint_angle_list[idx].append(angle)
    
    if ((stamp_num + 1) % print_every == 0) or (stamp_num == 0):
        print("Timestamp: {}, Current Joint Angles {}".format(stamp_num + 1, joint_angles))
        
    # Update the joint angles to the next angle based on the current timestamp
    for idx in range(len(joint_angles)):
        # Joint angles will vary over time and oscillate between the max and -max rotation values
        joint_angles[idx] = max_rot*rot_directions[idx]*math.sin(angle_increment*stamp) + offsets[idx]
        
    
    
# Iterable list of plot colors
plot_colors = iter(['b', 'g', 'r', 'c', 'm', 'y', 'k', 'b', 'g', 'r'])
joint_names = ["Joint " + str(idx+1) for idx in range(len(joint_angles))]

n_rows = 2
n_cols = 5
fig, ax = plt.subplots(nrows=n_rows, ncols=n_cols)
for idx, (offset_joint_angle_list, joint_name) in enumerate(zip(joint_angle_list, joint_names)):
    # Calculates row and col number for each plot based on number of rows and cols
    row_num = 0 if idx < n_cols else 1
    col_num = idx - n_cols*row_num
    ax[row_num, col_num].plot(timestamps, offset_joint_angle_list, label=joint_name, color=next(plot_colors))
    ax[row_num, col_num].set_xlabel("Timestamp (s)")
    ax[row_num, col_num].set_ylabel("Joint angle (radians)")
    ax[row_num, col_num].set_title(joint_name)
    
fig.suptitle("Joint Angles Over Time")
fig.legend()
plt.show()
    