import sympy

if __name__ == '__main__':
    # Define the velocities for the end effector
    x_dot, y_dot, phi_dot = sympy.symbols("x_dot, y_dot, phi_dot")

    t = sympy.Symbol('t')

    # Create theta_1, L1, and L3 as functions wrt to time
    theta_1 = sympy.Function('theta_1')(t)
    L1 = sympy.Function('L1')(t)
    theta_3 = sympy.Function('theta_3')(t)

    # Define the constants for the link lengths and second joint angle
    L2, L3, theta_2 = sympy.symbols("L2, L3, theta_2")

    # Forward kinematic end effector position
    x_ee  = L1*sympy.cos(theta_1) + L2*sympy.cos(theta_1 + theta_2) + L3*sympy.cos(theta_1 + theta_2 + theta_3)
    y_ee  = L1*sympy.sin(theta_1) + L2*sympy.sin(theta_1 + theta_2) + L3*sympy.sin(theta_1 + theta_2 + theta_3)
    phi_ee = theta_1 + theta_2 + theta_3

    # Take the derivative of the position equation wrt to time to get the end effector velocity
    x_ee_dot = sympy.diff(x_ee, t)
    y_ee_dot = sympy.diff(y_ee, t)
    phi_ee_dot = sympy.diff(phi_ee, t)

    # Define the matrix that maps end input variable (theta_1_dot, L1_dot, theta_3_dot) to the end effector position
    # This matrix would be used to multiply a vector of input variables in order calculate x_ee_dot, y_ee_dot, and phi_ee_dot
    # Derived by hand in notes using the sympy derivatives for forward kinematics
    M_matrix = sympy.Matrix([[-L2*sympy.sin(theta_1 + theta_2) + L3*sympy.sin(theta_1 + theta_2 + theta_3) - L1*sympy.sin(theta_1), sympy.cos(theta_1), L3*sympy.sin(theta_1 + theta_2 + theta_3)],
                [L2*sympy.cos(theta_1 + theta_2) + L3*sympy.cos(theta_1 + theta_2 + theta_3) + L1*sympy.cos(theta_1), sympy.sin(theta_1), L3*sympy.cos(theta_1 + theta_2 + theta_3)],
                [1, 0, 1]])
    
    print("Forward kinematics matrix: {} \n".format(M_matrix))

    # Invert the matrix to get the matrix for inverse kinematics (maps the end effector velocity to the joint velocities)
    inv_M = sympy.simplify(M_matrix.inv())

    # Display the simplified version of the matrix
    print("Inverse kinematics matrix: {} \n ".format(inv_M))

    # Create a vector of the end effector velocities so we can multiply the rows of M inverse by it to 
    # get the joint velocities
    ee_vels = sympy.Matrix([x_ee_dot, y_ee_dot, phi_ee_dot])

    # Calculate and print the joint velocities
    print("Theta 1 velocity is defined by: theta_1_dot = {} \n".format(inv_M.row(0)*ee_vels))
    print("L1 velocity is defined by: L1_dot = {} \n".format(inv_M.row(1)*ee_vels))
    print("Theta 2 velocity is defined by: theta_3_dot = {} \n".format(inv_M.row(2)*ee_vels))
    
