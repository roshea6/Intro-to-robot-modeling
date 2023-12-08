import math
import numpy as np
from jacobian_utils import JacobianUtils
import sympy

if __name__ == '__main__':
    j_utils = JacobianUtils(damped_jacobian=False)

    # Define the 3 sets of test joint angles
    # Essentially just adding pi/2 to joints 1, 2, 3
    joint_configs = [[math.pi, -math.pi/2 + math.pi/2, 0, math.pi/2, 0, 0],
                     [math.pi, -math.pi/2, 0 + math.pi/2, math.pi/2, 0, 0],
                     [math.pi, -math.pi/2, 0, math.pi/2 + math.pi/2, 0, 0]]
    
    # Loop through the 3 separate configs and display each
    for idx, config in enumerate(joint_configs):
        j_utils.theta_val_list = config
        j_utils.calculateTransMats()

        print("Config {}".format(idx + 1))
        sympy.pprint(j_utils.final_trans_mat)
        print()
