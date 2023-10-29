# Instructions
1. To get the successive transformation matrices for the UR-10 in the home position as well as the final transformation matrix between the base link and final frame the code can just be run as is
2. If you want the matrices for a new configuration with a joint rotated then the variables joint_num_to_rotate and rotate_amount should be updated on lines 24 and 25. Joint num should be between 0 and 5 and rotate amount should be a value in radians
3. If you want to evaulate the matrices with theta as a variable then set the variable evaluate_with_vars on line 21 to true. The final matrix for this is gigantic and doesn't display nicely even with sympy.pprint(). 

# Requirements
- numpy
- sympy
- math