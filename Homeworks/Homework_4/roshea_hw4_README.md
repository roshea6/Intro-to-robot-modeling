# ENPM 662 HW4 roshea

# Dependencies
- numpy
- sympy
- random
- matplotlib==3.7.4
- math

# Instructions
For normal plot creation the code can be run as is and should produce a circular plot in 2D with 2000 points. 
- In order to plot in 3D instead change the value of plot_3d on line 182 to true
- In order to change the number of points used change the value of num_steps. Values below 500 will likely cause a visibly incomplete circle
- In order to print out the Z and O vectors in addition to the calculated Jacobian at at each timestep set the the display variable to True in the creation of the j_utils object on line 177
- In order to print out the genral form of the Jacobian and Z and O vectors in the home position also set the use_symbols variable to true on line 177. This will print the Jacobian matrix and vectors before exiting. THIS WILL NOT RUN THROUGH THE CODE AND PRODUCE A GRAPH