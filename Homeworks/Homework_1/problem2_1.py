import numpy as np
import sympy

if __name__ == '__main__':
    # theta1, L1, theta3 = sympy.symbols("theta1, L1, theta3")

    t = sympy.Symbol('t')

    # Create theta1, L1, and L3 as functions wrt to time
    theta1 = sympy.Function('theta1')(t)
    L1 = sympy.Function('L1')(t)
    theta3 = sympy.Function('theta3')(t)

    # print(theta1.diff(t))

    # Define the constants for the link lengths and second joint angle
    L2, L3, theta2 = sympy.symbols("L2, L3, theta2")

    # Forward kinematic end effector position
    x_ee  = L1*sympy.cos(theta1) + L2*sympy.cos(theta1 + theta2) + L3*sympy.cos(theta1 + theta2 + theta3)
    y_ee  = L1*sympy.sin(theta1) + L2*sympy.sin(theta1 + theta2) + L3*sympy.sin(theta1 + theta2 + theta3)

    print("End effector X position is defined by: {}".format(x_ee))
    print("End effector Y position is defined by: {}".format(y_ee))

    # Take the derivative of the position equation wrt to time to get the end effector velocity
    x_ee_dot = sympy.diff(x_ee, t)
    y_ee_dot = sympy.diff(y_ee, t)

    print("End effector X velocity is defined by: {}".format(x_ee_dot))
    print("End effector Y velocity is defined by: {}".format(y_ee_dot))