import math
import numpy as np
from jacobian_utils import JacobianUtils
import sympy

if __name__ == '__main__':
    j_utils = JacobianUtils()

    joint_configs = [[math.pi, -math.pi/2 + math.pi/2, 0, math.pi/2, 0, 0],
                     [math.pi, -math.pi/2, 0 + math.pi/2, math.pi/2, 0, 0],
                     [math.pi, -math.pi/2, 0, math.pi/2 + math.pi/2, 0, 0]]
    
    for idx, config in enumerate(joint_configs):
        j_utils.theta_val_list = config
        j_utils.calculateTransMats()

        print("Config {}".format(idx + 1))
        sympy.pprint(j_utils.final_trans_mat)
        print()
