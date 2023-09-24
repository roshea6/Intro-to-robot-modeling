import sympy

if __name__ == '__main__':
    # theta1, L1, theta3 = sympy.symbols("theta1, L1, theta3")

    t = sympy.Symbol('t')

    # Create theta1, L1, and L3 as functions wrt to time
    theta_1 = sympy.Function('theta_1')(t)
    L1 = sympy.Function('L1')(t)
    theta_3 = sympy.Function('theta_3')(t)

    # Define the constants for the link lengths and second joint angle
    L2, L3, theta_2 = sympy.symbols("L2, L3, theta_2")

    # Forward kinematic end effector position
    x_ee  = L1*sympy.cos(theta_1) + L2*sympy.cos(theta_1 + theta_2) + L3*sympy.cos(theta_1 + theta_2 + theta_3)
    y_ee  = L1*sympy.sin(theta_1) + L2*sympy.sin(theta_1 + theta_2) + L3*sympy.sin(theta_1 + theta_2 + theta_3)
    phi_ee = theta_1 + theta_2 + theta_3


    print("End effector X position is defined by: X_e = {} \n".format(sympy.simplify(x_ee)))
    print("End effector Y position is defined by: Y_e = {} \n".format(sympy.simplify(y_ee)))
    print("End effector rotation is defined by: phi_e = {} \n".format(sympy.simplify(phi_ee)))

    # Take the derivative of the position equation wrt to time to get the end effector velocity
    x_ee_dot = sympy.diff(x_ee, t)
    y_ee_dot = sympy.diff(y_ee, t)
    phi_ee_dot = sympy.diff(phi_ee, t)


    print("End effector X velocity is defined by: X_e_dot = {} \n".format(sympy.simplify(x_ee_dot)))
    print("End effector Y velocity is defined by: Y_e_dot = {} \n".format(sympy.simplify(y_ee_dot)))
    print("End effector rotational velocity is defined by: phi_e_dot = {} \n".format(sympy.simplify(phi_ee_dot)))