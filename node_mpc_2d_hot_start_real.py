#! /usr/bin/env python3

import time

import rospy
from casadi import *
import numpy as np
from crazyflie_simulator.CrazySim import CrazySim
from crazyflie_manager.CrazyManager import *
from crazy_common_py.dataTypes import Vector3
from crazy_common_py.common_functions import rad2deg, deg2rad
from std_msgs.msg import Empty, Int16
from crazyflie_messages.msg import Position, CrazyflieState, Attitude, MpcOpenLoopTraj


def nlp_solver_2d(mpc_target, actual_state, x_obs, y_obs, r_obs, T_mpc, N_mpc, 
                    r_drone, r_safety, x1_opt, x2_opt, w_des_vel, 
                    w_vel, a_pos, b_pos, d_lb, d_ub):

    # Getting the actual position to set the initial condition
    x_pos = actual_state.position.x
    y_pos = actual_state.position.y
    
    #created folder and file for print debug output

    #filepath = os.path.join('/home/matteo/catkin_ws/src/my_debug_print', 'my_file.txt')

    #if not os.path.exists('/home/matteo/catkin_ws/src/my_debug_print'):
    #    os.makedirs('/home/matteo/catkin_ws/src/my_debug_print')

    #f = open(filepath, "a")

    f = open("/home/matteo/catkin_ws/src/my_debug_print/my_file.txt", "a")

    f.write("TIME = " + str(time_var) + "\n\n")

    f.write("MPC_STATE\n")
    f.write(str(x_pos) + "\n")
    f.write(str(y_pos) + "\n")
    f.write("--------------------------\n\n")
    
    f.close()
    #END - created folder and file for print debug output - END

    # Setting the desired position
    x_des = mpc_target.desired_position.x
    y_des = mpc_target.desired_position.y


    #created folder and file for print debug output
    f = open("/home/matteo/catkin_ws/src/my_debug_print/my_file.txt", "a")

    f.write("TIME = " + str(time_var) + "\n\n")

    f.write("MPC_TARGET_ACQUIRED\n")
    f.write(str(x_des) + "\n")
    f.write(str(y_des) + "\n")
    f.write("--------------------------\n\n")
    
    f.close()
    #END - created folder and file for print debug output - END

    # Setting desired velocity
    vx_des = (x_des-x_pos)/T_mpc
    vy_des = (y_des-y_pos)/T_mpc
    v_des = (vx_des**2 + vy_des**2)**0.5
    if v_des > 0.75:
        v_des = 0.75 #mv: per sicurezza?

    # Declare model variables
    x1 = MX.sym('x1')
    x2 = MX.sym('x2')
    x = vertcat(x1, x2)

    v1 = MX.sym('v1')
    v2 = MX.sym('v2')
    v = vertcat(v1, v2)

    # Model equations
    xdot = vertcat(v1, v2)

    # Distance to be covered
    vector = [x_des-x_pos, y_des-y_pos]
    vector = np.array(vector)
    distance = np.linalg.norm(vector)

    # Defining weights for position cost
    w_pos = a_pos*distance**(-b_pos)
    if distance < d_lb:
        w_pos = a_pos*d_lb**(-b_pos)
    if distance > d_ub:
        w_pos = a_pos*d_ub**(-b_pos)

    # Objective term (minimize control effort)
    L = w_des_vel*(v1**2 + v2**2 - v_des**2)**2 + w_vel*(v1**2+v2**2) +\
        w_pos*(x1-x_des)**2 + w_pos*(x2-y_des)**2

    # Formulate discrete time dynamics
    if False:
        # CVODES from the SUNDIALS suite
        dae = {'x':x, 'p':u, 'ode':xdot, 'quad':L}
        opts = {'tf':T/N}
        F = integrator('F', 'cvodes', dae, opts)

    elif False:
        # Fixed step Runge-Kutta 4 integrator
        M = 4 # RK4 steps per interval
        DT = T_mpc/N_mpc/M
        f = Function('f', [x, v], [xdot, L])
        X0 = MX.sym('X0', 2)
        U = MX.sym('U', 2)
        X = X0
        Q = 0
        for j in range(M):
            k1, k1_q = f(X, U)
            k2, k2_q = f(X + DT/2 * k1, U)
            k3, k3_q = f(X + DT/2 * k2, U)
            k4, k4_q = f(X + DT * k3, U)
            X = X + DT/6*(k1 + 2*k2 + 2*k3 + k4)
            Q = Q + DT/6*(k1_q + 2*k2_q + 2*k3_q + k4_q)
        F = Function('F', [X0, U], [X, Q],['x0','p'],['xf','qf'])

    else:
        # Forward Euler
        DT = T_mpc/N_mpc
        f = Function('f', [x, v], [xdot, L]) #mv:???
        X0 = MX.sym('X0', 2)
        U = MX.sym('U', 2)
        X = X0
        Q = 0
        k, k_q = f(X,U)
        X = X + k*DT
        Q = Q + k_q*DT
        F  = Function('F', [X0, U], [X, Q], ['x0','p'], ['xf','qf'])
        
    # MPC LOOP
    P_0 = [x_pos, y_pos]
    P_N = [x_des, y_des]

    v_ig = [vx_des, vy_des]
    
    # Initializing state estimate at time instant i
    X_i = P_0
    v_i = v_ig

    # Start with an empty NLP at each time step
    w = []
    w0 = []
    lbw = []
    ubw = []
    J = 0
    g = []        # it includes the constraint for obstacle avoidance
    lbg = []
    ubg = []

    # "Lift" initial conditions
    Xk = MX.sym('X0', 2)
    w += [Xk]

    lbw += [X_i[0].__float__(), X_i[1].__float__()]
    ubw += [X_i[0].__float__(), X_i[1].__float__()]
    w0 += [X_i[0].__float__(), X_i[1].__float__()]

    # Formulate the NLP
    for k in range(N_mpc):
        # New NLP variable for the control
        Vk = MX.sym('V_' + str(k), 2)  # creating symbolic expression for the 
                                       # new optimization variable
        w   += [Vk]     
        lbw += [-inf, -inf]
        ubw += [ inf,  inf]
        w0  += [v_i[0].__float__(), v_i[1].__float__()]

        # Integrate till the end of the interval
        Fk = F(x0=Xk, p=Vk)         # we call the integrator
        Xk_end = Fk['xf']
        J=J+Fk['qf']

        # New NLP variable for state at end of interval
        Xk = MX.sym('X_' + str(k+1), 2)
        w   += [Xk]
        lbw += [-inf, -inf]
        ubw += [ inf,  inf]
        w0  += [x1_opt[k+1], x2_opt[k+1]]

        # Add equality constraint (continuity constraint for multiple shooting)
        g   += [Xk_end-Xk]
        lbg += [0, 0]
        ubg += [0, 0]

        # Add inequality constraint for obstacle avoidance
        for ii in range(len(x_obs)):
            g   += [(Xk[0] - x_obs[ii])**2 + (Xk[1] - y_obs[ii])**2]
            lbg += [(r_obs[ii] + r_drone + r_safety)**2]
            ubg += [+inf]

        # Add equality constraint for final position
        if k == N_mpc - 1:
            X_final = P_N
            g += [Xk_end - X_final]
            lbg += [-1, -1]
            ubg += [ 1,  1]

        
        ################## Terminal Penalty Cost ##################

        if k == N_mpc - 1:

            J = J + w_pos*(Xk_end[2*ii]-x_des)**2 + \
                    w_pos*(Xk_end[2*ii+1]-y_des)**2


    prob = {'f': J, 'x': vertcat(*w), 'g': vertcat(*g)}
    solver = nlpsol('solver', 'ipopt', prob);

    # Solve the NLP
    sol = solver(x0=w0, lbx=lbw, ubx=ubw, lbg=lbg, ubg=ubg)
    w_opt = sol['x'].full().flatten()

    # Extracting the optimal control inputs
    x1_opt = []
    x2_opt = []
    v1_opt = []
    v2_opt = []
    x1_opt = w_opt[0::4]
    x2_opt = w_opt[1::4]
    v1_opt = w_opt[2::4]
    v2_opt = w_opt[3::4]

    # Extracting the control inputs to be applied
    v_opt_i = []
    v_opt_i += [v1_opt[0]]
    v_opt_i += [v2_opt[0]]
    v_i = np.array(v_opt_i)
    
    v_desired = v_i
    
    mpc_velocity = Position()

    mpc_velocity.desired_velocity.x = v_desired[0]
    mpc_velocity.desired_velocity.y = v_desired[1]
    mpc_velocity.desired_velocity.z = 0.0 #mv: mpc 3D | da aggiungere!

    mpc_velocity.name = 'cf1'

    return mpc_velocity, x1_opt, x2_opt

###############################################################################

#                  S U B S C R I B E R     C A L L B A C K S

###############################################################################

def mpc_target_sub_callback(msg):
    mpc_target.desired_position.x = msg.desired_position.x
    mpc_target.desired_position.y = msg.desired_position.y
    mpc_target.desired_position.z = msg.desired_position.z
    
    sub_mpc_flag.data = 1   

def state_sub_callback(msg):
    actual_state.position.x = msg.position.x
    actual_state.position.y = msg.position.y
    actual_state.position.z = msg.position.z

    actual_state.velocity.x = msg.velocity.x
    actual_state.velocity.y = msg.velocity.y
    actual_state.velocity.z = msg.velocity.z

    actual_state.orientation.roll = rad2deg(msg.orientation.roll)
    actual_state.orientation.pitch = rad2deg(msg.orientation.pitch)
    actual_state.orientation.yaw = rad2deg(msg.orientation.yaw)

    actual_state.rotating_speed.x = rad2deg(msg.rotating_speed.x)
    actual_state.rotating_speed.y = rad2deg(msg.rotating_speed.y)
    actual_state.rotating_speed.z = rad2deg(msg.rotating_speed.z)



###########################################################################

#                                M A I N

###########################################################################

if __name__ == '__main__':
    
    time_var=time.asctime()

    # Node initialization:
    rospy.init_node('node_mpc_2d', log_level=rospy.DEBUG)

    # Time interval and number of control intervals
    T_mpc = 1
    N_mpc = 5

    # Setting the obstacles' parameters
    x_obs=[1.2]
    y_obs=[0.0]
    r_obs=[0.125]
    r_drone = 0.05
    r_safety = 0.15

    # Weights' parameters
    w_des_vel = 1.0
    w_vel = 0.1
    a_pos = 0.15
    b_pos = 1.3
    d_lb = 0.5
    d_ub = 3

    ###############################################################################

    #                     P U B L I S H E R S   S E T U P

    ###############################################################################

    # Publisher to publish the target velocity (ie controller output velocity)
    mpc_velocity = Position()
    mpc_velocity_pub = rospy.Publisher('/cf1/mpc_velocity', #mv: generalizzare /cf1
                                       Position, queue_size=1)

    # Publisher to publish the optimal trajectory computed by the MPC
    mpc_traj = MpcOpenLoopTraj()
    mpc_traj_pub = rospy.Publisher('/cf1/mpc_traj', #mv: generalizzare /cf1
                                    MpcOpenLoopTraj,queue_size=1)

    for ii in range(N_mpc+1):
        mpc_traj.x_vec.append(Position())

    ###############################################################################

    #                     S U B S C R I B E R S   S E T U P

    ###############################################################################

    # Subscriber to get the mpc target position
    mpc_target_sub = rospy.Subscriber('/cf1/mpc_target', #mv: generalizzare /cf1
                            Position, mpc_target_sub_callback)
    mpc_target = Position()

    # Subscriber to get the actual state of the drone in the simulation:
    state_sub = rospy.Subscriber('/cf1/state', CrazyflieState, #mv: generalizzare /cf1
                                            state_sub_callback)
    actual_state = CrazyflieState()

    # Flag for the mpc target subscriber
    sub_mpc_flag = Int16()
    sub_mpc_flag.data = 0  

    # Initializing the mpc target
    mpc_target.desired_position.x = 0
    mpc_target.desired_position.y = 0

    # Initializing x1_opt, x2_opt to use them as initial guess
    x1_opt_old = np.linspace(actual_state.position.x, 
                            mpc_target.desired_position.x, N_mpc+1)
    x2_opt_old = np.linspace(actual_state.position.y, 
                            mpc_target.desired_position.y, N_mpc+1)

    rate = rospy.Rate(5.0)

    while not rospy.is_shutdown():
        
        if sub_mpc_flag.data == 0:
            # nothing is executed if no mpc target has been published
            pass

        elif sub_mpc_flag.data == 1: #mv: new target acquired
            # When a new target is set, we give the following 
            # trajectories as initial guesses
            x1_opt_old = np.linspace(actual_state.position.x, 
                        mpc_target.desired_position.x, N_mpc+1)
            x2_opt_old = np.linspace(actual_state.position.y, 
                        mpc_target.desired_position.y, N_mpc+1)
            
            sub_mpc_flag.data = 2

        else:
            # If the mpc target remains the same, the nlp solver is 
            # called      
            mpc_velocity, x1_opt, x2_opt = nlp_solver_2d(mpc_target, 
                actual_state, x_obs, y_obs, r_obs, T_mpc, N_mpc, 
                r_drone, r_safety, x1_opt_old, x2_opt_old, w_des_vel, 
                w_vel, a_pos, b_pos, d_lb, d_ub) 

            for ii in range(N_mpc+1):
                mpc_traj.x_vec[ii].desired_position.x = x1_opt[ii]
                mpc_traj.x_vec[ii].desired_position.y = x2_opt[ii]            
            
            # The mpc velocity is published
            mpc_velocity_pub.publish(mpc_velocity)

            mpc_traj_pub.publish(mpc_traj)

            x1_opt_old, x2_opt_old = x1_opt, x2_opt

        rate.sleep() #mv: mpc con frequenza 100Hz
