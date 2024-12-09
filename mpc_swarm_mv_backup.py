#! /usr/bin/env python3

# mv: file originale: swarm_cen_cbf_coll_avoid_node.py

# TODO: PROVARE SPERIMENTALMENTE

# TODO - migliorare MPC (lagrangiana,...)
# TODO - capire come organizzare controllo - controllo ibrido MPC + CBF ? centralizzato / distribuito / <- design architettura controllo !
# TODO: da w_ a gradino a w_ più graduale, in funzione della posizione del centro di massa o di ...

# TODO - integrare pubblicazione efficacie di velocità per ogni drone dello sciame! / swarm_mpc_velocity_pub(mpc_velocity) (?)
# TODO: e se lo sciame è formato da 1 solo drone? N_neigh, ... ! (?)


# TODO: rivedere quale delle due condizioni qui sotto è maggiormente opportuna in base al contesto
"""
CONDIZIONE 1:
if abs(P_0[3 * ii] - x_des) < 1 and abs(P_0[3 * ii + 1] - y_des) < 1 and abs(P_0[3 * ii + 2] - z_des) < 1:

CONDIZIONE 2:
if abs(x_cm - x_des) < 1 and abs(y_cm - y_des) < 1 and abs(z_cm - z_des) < 1:
"""

from math import floor
import rospy
import actionlib
from crazyflie_messages.msg import TakeoffAction, TakeoffResult, TakeoffGoal, SphereData

import time
from casadi import *
import numpy as np
import cvxpy as cp  # cvxpy is a toolbox for convex optimization in python,
# we need to solve a convex QP
from crazyflie_simulator.CrazySim import CrazySim
from crazyflie_manager.CrazyManager import *
from crazy_common_py.dataTypes import Vector3
from crazy_common_py.common_functions import rad2deg, deg2rad
from std_msgs.msg import Empty, Int16, Int32
from crazyflie_messages.msg import Position, CrazyflieState, Attitude
from crazy_common_py.common_functions import standardNameList
from crazyflie_swarm.CrazySwarmReal_prova import CrazySwarmReal_prova
from crazyflie_messages.msg import SwarmStates

from geometry_msgs.msg import Point

import sys

# provvisorio
uris = []
uri = 'radio://0/80/2M/E7E7E7E7E2'
uris.append(uri)
uri = 'radio://0/80/2M/E7E7E7E7E7'
uris.append(uri)

# TODO: CBF ? <---
'''
###################################################################

#      L O W     L E V E L    C B F     C O N T R O L L E R

###################################################################

class CBF_controller():

    def __init__(self, v_mpc, alpha, x):

        self.x = x
        self.v_mpc = v_mpc
        self.alpha = alpha

    def set_obstacle(self):

        ########################################################################################

        # Defining the h function for the known obstacles
        self.h = lambda x1, x2, x_o, r_o: ((x1 - x_o[0]) ** 2 + (x2 - x_o[1]) ** 2
                                           - r_o ** 2) * 0.5

        self.grad_h = lambda x1, x2, x_o, r_o: np.array([x1 - x_o[0], x2 - x_o[1]])

        ########################################################################################

    def get_cbf_v(self, x, N_obs, x_obs, r_obs):

        # Given current state solve pointwise Quadratic Program and get safe
        # control action (velocity)

        v_opt = cp.Variable(2)

        ########################################################################################

        h_num = []
        grad_h_num = []

        for ii in range(N_obs):
            h_num.append(self.h(x[0], x[1], x_obs[ii], r_obs[ii]))
            grad_h_num.append(self.grad_h(x[0], x[1], x_obs[ii], r_obs[ii]))

        ########################################################################################

        v_des = self.v_mpc

        # Solve QP for u_opt
        obj = cp.Minimize((1 / 2) * cp.quad_form(v_opt - v_des, np.eye(2)))

        ########################################################################################

        constraints = []

        for ii in range(N_obs):
            obs_distance = np.linalg.norm(np.array([x[0] - x_obs[ii][0],
                                                    x[1] - x_obs[ii][1]]))
            if obs_distance < (r_obs[ii] + 1.5):
                constraints.append(grad_h_num[ii].T @ v_opt >= -self.alpha * h_num[ii])

        prob = cp.Problem(obj, constraints)
        prob.solve()

        return v_opt.value
'''


###################################################################

#             ( H I G H    L E V E L )    M P C  -  3 D

###################################################################


def nlp_solver_3d(N_cf_, P_N_, P_0_, N_, x_opt_, v_opt_,
                  d_ref_, v_ref_, w_sep_,
                  w_final_, w_vel_, w_t4d_, N_neigh_, obstacles, r_drone_, r_safe_,
                  tgt_choice_, tgt4drone, numerical_method_, gap_modify_weights_, w_sep_near_tgt_,
                  w_sep_near_tgt_t4d_, w_sep_emergency_, w_final_near_tgt_t4d_, w_vel_near_tgt_, w_t4d_near_tgt_,
                  lim_vel_mpc_):
    # -------------------------------------------------------------------

    #                       I N I T I A L I Z A T I O N

    # -------------------------------------------------------------------

    tgt_4_drone = False

    if tgt_choice_ == 2:
        tgt_4_drone = True

    # Getting center of mass of the swarm
    x_cm_ = np.array(P_0_[0::3]).sum() / N_cf_
    y_cm_ = np.array(P_0_[1::3]).sum() / N_cf_
    z_cm_ = np.array(P_0_[2::3]).sum() / N_cf_

    # Desired positions
    x_des = P_N_[0]
    y_des = P_N_[1]
    z_des = P_N_[2]

    # Calculating the reference direction for the drone swarm
    u_ref = [x_des - x_cm_, y_des - y_cm_, z_des - z_cm_]
    u_ref = np.array(u_ref)
    u_norm = np.linalg.norm(u_ref)

    # Calculating the time horizon for the MPC
    T = u_norm / v_ref_

    # Threshold on time horizon, otherwise the swarm oscillates
    if T < 1.0:
        T = 1.0

    # Versor for the reference direction
    u_ref = u_ref / u_norm

    u_refx = u_ref[0]
    u_refy = u_ref[1]
    u_refz = u_ref[2]

    # Setting desired velocity
    vx_des = (x_des - x_cm_) / T
    vy_des = (y_des - y_cm_) / T
    vz_des = (z_des - z_cm_) / T

    # Saturation on desired velocity
    if vx_des > 1:
        vx_des = 1
    if vx_des < -1:
        vx_des = -1

    if vy_des > 1:
        vy_des = 1
    if vy_des < -1:
        vy_des = -1

    if vz_des > 1:
        vz_des = 1
    if vz_des < -1:
        vz_des = -1

    # Declare model variables
    x = MX.sym('x', N_cf_ * 3)

    v = MX.sym('v', N_cf_ * 3)

    # Model equations
    xdot = v

    '''
    
    """
    (forse: tentativo di utilizzare delle matrici per definire quali droni siano "neighbors", 
    per poi costruire la funzione di costo in notazione matriciale)
    """
    
    ################# Building Adjacency Matrix (NOT USED) ########################

    # A_neigh = np.zeros((N_cf, N_cf))

    # index_neigh = []

    # for ii in range(N_cf-1):
    #     list_p_rel = []
    #     for jj in range(ii + 1, N_cf):
    #         p_rel = [P_0[2*ii] - P_0[2*jj], P_0[2*ii+1] - P_0[2*jj+1]]
    #         p_rel = np.array(p_rel)
    #         # Storing p_rel in a list
    #         list_p_rel.append(np.linalg.norm(p_rel))

    #         # Filling Adjacency Matrix
    #         if np.linalg.norm(p_rel) < d_neigh:
    #             A_neigh[ii,jj], A_neigh[jj,ii] = 1, 1

    #     # Getting the sorted indices of list_p_rel to set the order of neighbours
    #     index_neigh.append((np.argsort(list_p_rel)+ii+1).tolist())
    # print('index_neigh is: ', index_neigh)
    # print('A_neigh is: ', A_neigh)

    # # list_neighbours_i = range(N_cf)

    # ################# Ordered list of neighbours (NOT USED) #####################
    # list_list_neighbours = []

    # for ii in range(N_cf-1):

    #     neigh_count = 0

    #     list_neighbours_i = []

    #     for jj in range(N_cf):
    #         if A_neigh[ii,jj] == 1 and jj > ii:
    #             neigh_count += 1

    #     list_neighbours_i = index_neigh[ii][:neigh_count]
    #     print('list_neighbours_i is: ', list_neighbours_i)

    #     list_list_neighbours.append(list_neighbours_i)

    # # list_neighbours_i = [list_neighbours_i for ii in index_neigh[jj]]
    # print('list_list_neighbours is: ', list_list_neighbours)
    # # list_neighbours_i = [x for _, x in sorted(list_neighbours_i,)]

    # ########## Weights for the distances in separation cost (NOT USED) #########
    # list_list_weights = []
    # for ii in range(N_cf-1):
    #     list_weights_i = []
    #     ll = len(list_list_neighbours[ii])
    #     for jj in range(ll):
    #         kk = floor(jj/2)
    #         list_weights_i.append(kk)
    #     list_list_weights.append(list_weights_i)

    # print('list_list_weights is: ', list_list_weights)
    '''

    # ------------- W E I G H T S   M O D I F I C A T I O N -------------

    if abs(x_cm_ - x_des) < gap_modify_weights_ and abs(y_cm_ - y_des) < gap_modify_weights_ and abs(
            z_cm_ - z_des) < gap_modify_weights_:

        w_vel_ = w_vel_near_tgt_

        if tgt_4_drone:
            w_sep_ = w_sep_near_tgt_t4d_
            w_final_ = w_final_near_tgt_t4d_
            w_t4d_ = w_t4d_near_tgt_
        else:
            w_sep_ = w_sep_near_tgt_

    # ------------- L I S T   O F   N E I G H B O U R S -------------
    # Building Ordered list of N_neigh closest neighbors for each drone

    index_neigh = []

    for iii in range(N_cf_):
        list_p_rel = []
        for jj in range(N_cf_):
            p_rel = [P_0_[3 * iii] - P_0_[3 * jj], P_0_[3 * iii + 1] - P_0_[3 * jj + 1],
                     P_0_[3 * iii + 2] - P_0_[3 * jj + 2]]
            p_rel = np.array(p_rel)
            # Storing p_rel in a list
            list_p_rel.append(np.linalg.norm(p_rel))

        # Getting the sorted indices of list_p_rel to set the order of neighbours
        index_neigh_i = (np.argsort(list_p_rel)).tolist()
        index_neigh_i.pop(0)  # elimino il drone stesso - ok
        index_neigh.append(index_neigh_i)

    # ------------- O R D E R E D   L I S T   O F   N E I G H B O U R S -------------
    """
    list_list_neighbours è una lista di liste di vicini ordinati.
    Ogni lista interna contiene gli indici dei droni vicini al drone corrispondente
    nell’ordine di vicinanza crescente.
    """

    list_list_neighbours = []

    for iii in range(N_cf_):
        if N_cf_ > N_neigh_:
            list_neighbours_i = index_neigh[iii][:N_neigh_]
        else:
            list_neighbours_i = index_neigh[iii][:(N_cf_ - 1)]
        list_list_neighbours.append(list_neighbours_i)

    # -------------------------------------------------------------------

    #           B U L D I N G   O B J E C T I V E   F U N C T I O N

    # -------------------------------------------------------------------

    L = 0
    # x_cm = 0
    # y_cm = 0
    # z_cm = 0

    # # 1 - All neighbours
    # for ii in range(N_cf-1):
    #     for kk in range(ii+1, N_cf):
    #         # if ii != kk:
    #         # Separation cost
    #         n_neigh = A_neigh[ii].sum()

    #         d_ref_sep = d_ref #*(1 + 0.2*n_neigh)
    #         L += w_sep*((x[ii*2]-x[kk*2])**2 \
    #             + (x[ii*2+1]-x[kk*2+1])**2\
    #             - d_ref_sep**2)**2
    #     print('n_neigh is: ', n_neigh)

    # 2 - 2 neighbours per drone # TODO ? perché proprio 2 neighbours?

    # ------------- S E P A R A T I O N   T E R M -------------

    for iii in range(N_cf_):
        for kk in list_list_neighbours[iii]:
            if ((P_0_[3 * iii] - P_0_[3 * kk]) ** 2 +
                (P_0_[3 * iii + 1] - P_0_[3 * kk + 1]) ** 2 +
                (P_0_[3 * iii + 2] - P_0_[3 * kk + 2]) ** 2) \
                    < (r_drone_ * 2 + r_safe_) ** 2:
                w_sep_ = w_sep_emergency_  # a fini di sicurezza! rischio collisione!

            # Separation cost
            d_ref_sep = d_ref_

            L += w_sep_ * ((x[iii * 3] - x[kk * 3]) ** 2
                           + (x[iii * 3 + 1] - x[kk * 3 + 1]) ** 2
                           + (x[iii * 3 + 2] - x[kk * 3 + 2]) ** 2
                           - d_ref_sep ** 2) ** 2

    # TODO: controllare...
    '''
    x_cm_sym = 0
    y_cm_sym = 0
    z_cm_sym = 0

    for ii in range(N_cf):
        
        # Navigation cost
        # L += w_nav*(v[ii*2]**2 + v[ii*2+1]**2 - v_ref_new**2)**2

        # Final Position cost
        # TODO: sbagliato! x_des è mpc_target_x in termini di centro di massa, x[ii * 3] è posizione singolo drone
        #  cambiare con x_cm_attuale...

        # TODO: sbagliato # L += w_final * ((x[ii * 3] - x_des) ** 2 + (x[ii * 3 + 1] - y_des) ** 2 + (x[ii * 3 + 2] - z_des) ** 2) ** 2
        
        # # Control input cost
        # w_vel_i = w_vel
        # L += w_vel_i*(v[ii*2]**2 + v[ii*2+1]**2)

        # # Direction cost
        # L += w_dir*(v[ii*2]**2 + v[ii*2+1]**2 - \
        #     (v[ii*2]*u_refx + v[ii*2+1]*u_refy)**2)**2
        '''

    # ------------- F I N A L   P O S I T I O N   T E R M   ( C E N T E R   O F   M A S S ) -------------

    x_cm_sym = 0
    y_cm_sym = 0
    z_cm_sym = 0

    for iii in range(N_cf_):
        x_cm_sym += x[iii * 3]
        y_cm_sym += x[iii * 3 + 1]
        z_cm_sym += x[iii * 3 + 2]

    x_cm_sym /= N_cf_
    y_cm_sym /= N_cf_
    z_cm_sym /= N_cf_

    for iii in range(N_cf_):
        L += w_final_ * ((x_cm_sym - x_des) ** 2 + (y_cm_sym - y_des) ** 2 + (z_cm_sym - z_des) ** 2) ** 2

    # ------------- V E L O C I T Y   P E N A L T Y   T E R M -------------

    for iii in range(N_cf_):
        # Control input cost (greater weight when close to target)
        L += w_vel_ * (v[iii * 3] ** 2 + v[iii * 3 + 1] ** 2 + v[iii * 3 + 2] ** 2)

    # ------------- F I N A L   P O S I T I O N   T E R M   ( T A R G E T   4   D R O N E ) -------------

    if tgt_4_drone:  # if not tgt4drone == 0:

        L_tgt4drone = 0

        for iii in range(N_cf_):

            L_tgt4drone_single = 0

            for jj in range(N_cf_):
                # TODO: controllare...
                L_tgt4drone_single *= (
                        (x[iii * 3] - tgt4drone[jj].x) ** 2 + (x[iii * 3 + 1] - tgt4drone[jj].y) ** 2 + (
                        x[iii * 3 + 2] - tgt4drone[jj].z) ** 2)

            L_tgt4drone_single *= w_t4d_

            L_tgt4drone += L_tgt4drone_single

        L += L_tgt4drone

    # ------------- N U M E R I C A L   M E T H O D   O F   D I S C R E T I Z A T I O N -------------

    # Forward Euler
    if numerical_method_ == 'forward_euler':
        DT = T / N_
        f = Function('f', [x, v], [xdot, L])
        X0 = MX.sym('X0', 3 * N_cf_)
        U = MX.sym('U', 3 * N_cf_)
        X = X0
        Q = 0
        k, k_q = f(X, U)
        X = X + k * DT
        Q = Q + k_q * DT
        F = Function('F', [X0, U], [X, Q], ['x0', 'p'], ['xf', 'qf'])

    # TODO: controllare Inverse Euler...
    # Inverse Euler
    elif numerical_method_ == 'inverse_euler':
        DT = T / N_
        f = Function('f', [x, v], [xdot, L])
        X0 = MX.sym('X0', 3 * N_cf_)
        U = MX.sym('U', 3 * N_cf_)
        X = X0
        Q = 0
        k, k_q = f(X, U)
        X = X0 + k * DT + (X - X0 - k * DT) / DT * DT  # TODO: X + ... o X0 + ... ? SONO LA STESSA COSA, X = X0 ! (?!)
        Q = Q + k_q * DT
        F = Function('F', [X0, U], [X, Q], ['x0', 'p'], ['xf', 'qf'])

        # (X - X0 - k * DT) / DT è la derivata di X rispetto al tempo;
        # Inverse Euler consiste nel calcolare la soluzione del sistema
        # non a partire dallo stato attuale, ma dalla soluzione del sistema nel prossimo istante di tempo.

    else:
        print('ERROR! - numerical method not valid!')
        rospy.is_shutdown()
        sys.exit(1)

    # altro...
    '''
    # Formulate discrete time dynamics
    if False:
        # CVODES from the SUNDIALS suite
        dae = {'x': x, 'p': v, 'ode': xdot, 'quad': L}
        opts = {'tf': T / N}
        F = integrator('F', 'cvodes', dae, opts)

    elif False:
        # Fixed step Runge-Kutta 4 integrator
        M = 4  # RK4 steps per interval
        DT = T / N / M
        f = Function('f', [x, v], [xdot, L])
        X0 = MX.sym('X0', 2 * N_cf)
        U = MX.sym('U', 2 * N_cf)
        X = X0
        Q = 0
        for j in range(M):
            k1, k1_q = f(X, U)
            k2, k2_q = f(X + DT / 2 * k1, U)
            k3, k3_q = f(X + DT / 2 * k2, U)
            k4, k4_q = f(X + DT * k3, U)
            X = X + DT / 6 * (k1 + 2 * k2 + 2 * k3 + k4)
            Q = Q + DT / 6 * (k1_q + 2 * k2_q + 2 * k3_q + k4_q)
        F = Function('F', [X0, U], [X, Q], ['x0', 'p'], ['xf', 'qf'])
    '''

    # -------------------------------------------------------------------

    #           S E T T I N G   U P   T H E   M P C

    # -------------------------------------------------------------------
    """
    We only exploit the first control input within the MPC framework.
    Hence, we define a casadi function which allows us to get the 
    control input at the first time step as output, given the initial state
    as input.
    """

    # MPC LOOP
    P_log = []
    px_log = []
    py_log = []
    V_log = []
    ux_log = []
    uy_log = []

    v_ig = []
    for iii in range(N_cf_):
        v_ig.append(vx_des)
        v_ig.append(vy_des)
        v_ig.append(vz_des)

    P_log = P_0_  # initial state

    # Initializing state estimate at time instant i
    X_i = P_0_
    v_i = v_ig

    # Start with an empty NLP at each time step
    w = []
    w0 = []
    lbw = []
    ubw = []
    J = 0
    g = []
    lbg = []
    ubg = []

    # "Lift" initial conditions # x_measured
    Xk = MX.sym('X0', 3 * N_cf_)
    w += [Xk]

    lbw_k = []
    for iii in range(N_cf_):
        lbw_k.append(X_i[3 * iii].__float__())
        lbw_k.append(X_i[3 * iii + 1].__float__())
        lbw_k.append(X_i[3 * iii + 2].__float__())

    lbw += lbw_k

    ubw_k = []
    for iii in range(N_cf_):
        ubw_k.append(X_i[3 * iii].__float__())
        ubw_k.append(X_i[3 * iii + 1].__float__())
        ubw_k.append(X_i[3 * iii + 2].__float__())

    ubw += ubw_k

    w0_k = []
    for iii in range(N_cf_):
        w0_k.append(X_i[3 * iii].__float__())
        w0_k.append(X_i[3 * iii + 1].__float__())
        w0_k.append(X_i[3 * iii + 2].__float__())

    w0 += w0_k

    # Formulate the NLP
    for k in range(N_):
        # New NLP variable for the control
        Vk = MX.sym('V_' + str(k), 3 * N_cf_)  # creating symbolic expression for the
        # new optimization variable
        w += [Vk]

        lbw_k = []
        for iii in range(N_cf_):
            lbw_k.append(-inf)
            lbw_k.append(-inf)
            lbw_k.append(-inf)

        lbw += lbw_k

        ubw_k = []
        for iii in range(N_cf_):
            ubw_k.append(+inf)
            ubw_k.append(+inf)
            ubw_k.append(+inf)

        ubw += ubw_k

        w0_k = []
        for iii in range(N_cf_):
            w0_k.append(v_opt_[3 * iii][k].__float__())  # TODO: ? vs ros_mpc.py - v_i ?
            w0_k.append(v_opt_[3 * iii + 1][k].__float__())
            w0_k.append(v_opt_[3 * iii + 2][k].__float__())

        w0 += w0_k

        # Integrate till the end of the interval
        Fk = F(x0=Xk, p=Vk)  # we call the integrator
        Xk_end = Fk['xf']
        J = J + Fk['qf']

        # New NLP variable for state at end of interval
        Xk = MX.sym('X_' + str(k + 1), 3 * N_cf_)
        w += [Xk]

        lbw_k = []
        for iii in range(N_cf_):
            lbw_k.append(-inf)
            lbw_k.append(-inf)
            lbw_k.append(-inf)

        lbw += lbw_k

        ubw_k = []
        for iii in range(N_cf_):
            ubw_k.append(+inf)
            ubw_k.append(+inf)
            ubw_k.append(+inf)

        ubw += ubw_k

        w0_k = []
        for iii in range(N_cf_):
            w0_k.append(x_opt_[3 * iii][k + 1].__float__())
            w0_k.append(x_opt_[3 * iii + 1][k + 1].__float__())
            w0_k.append(x_opt_[3 * iii + 2][k + 1].__float__())

        w0 += w0_k

        # Add equality constraint (continuity constraint for multiple shooting)
        g += [Xk_end - Xk]

        lbg_k = []
        for iii in range(N_cf_):
            lbg_k.append(0)
            lbg_k.append(0)
            lbg_k.append(0)

        lbg += lbg_k

        ubg_k = []
        for iii in range(N_cf_):
            ubg_k.append(0)
            ubg_k.append(0)
            ubg_k.append(0)

        ubg += ubg_k

        # TODO ???
        # Add inequality constraint for obstacle avoidance between drones
        # if k > 1:
        # if k > 0:
        # for ii in range(N_cf-1):
        #     for jj in range(ii+1, N_cf):
        #         g   += [(Xk_end[2*ii] - Xk_end[2*jj])**2 +
        #                 (Xk_end[2*ii+1] - Xk_end[2*jj+1])**2]
        #         lbg += [(6*r_drone)**2]
        #         # ubg += [+inf]

        # Add inequality constraint for obstacle avoidance
        '''
        # TODO: OSTACOLI FISICI SFERICI 
        #  tbn: limitazione a soli ostacoli sferici!
        #  vs ostacolo parallelepipedo trattato come area 3D vietata
        #  vedi ros_moc.py, Casadi forum /non funziona
        '''

        for jj in range(N_cf_):
            for iii in range(len(obstacles)):
                # ostacolo sferico
                g += [(Xk[3 * jj] - obstacles[iii].x) ** 2 + (Xk[3 * jj + 1] - obstacles[iii].y) ** 2 +
                      (Xk[3 * jj + 2] - obstacles[iii].z) ** 2]
                lbg += [(obstacles[iii].r + r_drone + r_safe) ** 2]  # TODO: r_safe da rivedere
                ubg += [+inf]

        ################ Center of mass in final target ##########################

        # TODO: Terminal Penalty Cost ? - vedi ros_mpc.py

        if k == N_ - 1:

            if tgt_4_drone:
                """
                TODO: aggiungere posizione finale precisa! (FACOLTATIVO)
                (tbn: difficile, dal momento che non conosco a priori
                dove i droni si debbano posizionare ottimamente)
                soluzione: soft constraint già presente (vedi L_tgt4drone), ok
                """
                pass

            if not tgt_4_drone:
                x_cm_end = 0
                y_cm_end = 0
                z_cm_end = 0

                for iii in range(N_cf_):
                    x_cm_end += Xk_end[3 * iii]
                    y_cm_end += Xk_end[3 * iii + 1]
                    z_cm_end += Xk_end[3 * iii + 2]

                x_cm_end /= N_cf_
                y_cm_end /= N_cf_
                z_cm_end /= N_cf_

                g += [P_N_[0] - x_cm_end, P_N_[1] - y_cm_end, P_N_[2] - z_cm_end]

                lbg += [0, 0, 0]
                ubg += [0, 0, 0]

        ################################################################

        '''
        # --- C O N S T R A I N T S   P O S I Z I O N E   F I N A L E   -   A L T R O ---
        
        ############### Drone 1 on target (NOT USED) ###################

        # if k == N - 1:
        #     g += [P_N[0] - Xk_end[0], P_N[1] - Xk_end[1]]
        #     lbg += [0, 0]
        #     ubg += [0, 0]

        ############## Final target on line (NOT USED) #################

        # if k == N - 1:
        #     X_final = P_N
        #     g += [Xk_end - X_final]

        #     lbg_k = []

        #     for ii in range(N_cf):
        #         lbg_k.append(0)
        #         lbg_k.append(0)
        #         lbg_k.append(0)

        #     lbg += lbg_k

        #     ubg_k = []

        #     for ii in range(N_cf):
        #         ubg_k.append(0)
        #         ubg_k.append(0)
        #         ubg_k.append(0)

        #     ubg += ubg_k

        ########## Final target on circumference (NOT USED) ############

        # if k == N - 1:
        #     x_final = 2
        #     y_final = 1
        #     r_circ = 0.3
        #     for ii in range(N_cf):
        #         g += [(Xk[2*ii] - P_N[0])**2 + \
        #               (Xk[2*ii+1] - P_N[1])**2 - r_circ**2]

        #     lbg_k = []
        #     for ii in range(N_cf):
        #         lbg_k.append(0)

        #     lbg += lbg_k

        #     ubg_k = []
        #     for ii in range(N_cf):
        #         ubg_k.append(0)

        #     ubg += ubg_k

        '''

    # ---------------------------------------------------------------------

    #   S O L V I N G   O P T I M A L   N O N L I N E A R   P R O B L E M

    # ---------------------------------------------------------------------

    # Create an NLP solver
    opts = {}
    opts = {'max_iter_eig': 5, 'ipopt.print_level': 0,
            'print_time': 0}  # TODO: ? max_iter_eig - vedi whatsapp (Giovanni)
    prob = {'f': J, 'x': vertcat(*w), 'g': vertcat(*g)}
    solver = nlpsol('solver', 'ipopt', prob, opts)

    # Solving the NLP
    sol = solver(x0=w0, lbx=lbw, ubx=ubw, lbg=lbg, ubg=ubg)
    w_opt = sol['x'].full().flatten()  # -> x_opt e v_opt
    # lambda_opt = sol['g'].full().flatten()  # -> lambda_opt
    """
    info w_opt by Casadi:
    
    3*N_cf <- x_0 (x y z cf_1, x y z cf_2, ... x y z cf_N_cf)
    3*N_cf <- v_0
    3*N_cf <- x_1
    ...
    3*N_cf <- v_(N_mpc-1)
    3*N_cf <- x_N_mpc
    
    tot. dim. = (3*N_cf*N_mpc)*2 + 3*N_cf
    *3:  x y z
    *2:  x_opt v_opt
    + 3*N_cf: per definizione w-opt by casadi: i primi valori riprendono x_measured input MPC ...
    """

    # Extracting the optimal state
    x_opt_ = []
    for iii in range(3 * N_cf_):
        x_opt_.append(w_opt[iii::6 * N_cf_])
        """
        info x_opt_:
        x_opt_ comprende tutti gli x_opt di ogni drone dello swarm...
        x_opt è una matrice
        x_opt_[0] è un vettore contenente x_opt cf1 @ time = 0, time = 1, ... , time = N_mpc
        x_opt_[1] è un vettore contenente y_opt cf1 @ time = 0, time = 1, ... , time = N_mpc
        x_opt_[5] è un vettore contenente z_opt cf2 @ time = 0, time = 1, ... , time = N_mpc
        ...
        """

    # Extracting the optimal control input
    v_opt_ = []
    for iii in range(3 * N_cf_):
        v_opt_.append(w_opt[(iii + 3 * N_cf_)::6 * N_cf_])
        """
        info v_opt_:
        v_opt_ comprende tutti i v_opt di ogni drone dello swarm...
        v_opt_ è una matrice
        v_opt_[0] è un vettore contenente vx_opt cf1 @ time = 0, time = 1, ... , time = N_mpc-1
        v_opt_[1] è un vettore contenente vy_opt cf1 @ time = 0, time = 1, ... , time = N_mpc-1
        v_opt_[5] è un vettore contenente vz_opt cf2 @ time = 0, time = 1, ... , time = N_mpc-1
        ...
        """

    # Extracting only the first optimal control input
    v_opt_i = []
    for iii in range(N_cf_):
        v_opt_i.append(v_opt_[3 * iii][0])
        v_opt_i.append(v_opt_[3 * iii + 1][0])
        v_opt_i.append(v_opt_[3 * iii + 2][0])

    v_i = v_opt_i

    mpc_velocity_ = []

    for iii in range(N_cf_):
        mpc_velocity_.append(Position())

    for iii in range(N_cf_):
        mpc_velocity_[iii].name = 'cf' + str(
            iii + 1)  # corretto, per come è definito P_0, e conseguentemente w_opt prima riga (!)
        mpc_velocity_[iii].desired_velocity.x = v_i[3 * iii]
        mpc_velocity_[iii].desired_velocity.y = v_i[3 * iii + 1]
        mpc_velocity_[iii].desired_velocity.z = v_i[3 * iii + 2]

    # limito velocità a fini di sicurezza
    for single_velocity in mpc_velocity_:
        if single_velocity.desired_velocity.x > lim_vel_mpc_:
            single_velocity.desired_velocity.x = lim_vel_mpc_
        if single_velocity.desired_velocity.y > lim_vel_mpc_:
            single_velocity.desired_velocity.y = lim_vel_mpc_
        if single_velocity.desired_velocity.z > lim_vel_mpc_:
            single_velocity.desired_velocity.z = lim_vel_mpc_

    return mpc_velocity_, x_opt_, v_opt_


###########################################################################

#                        O T H E R   F U N C T I O N S

###########################################################################

def target_reached():
    print('\n--------- T A R G E T   R E A C H E D --------')

    land_pub.publish(land_trigger)
    rospy.sleep(0.1)

    # termino programma
    rospy.is_shutdown()
    sys.exit(1)


def calcolo_centro_massa_target(array):
    n = len(array)
    cx, cy, cz = 0.0, 0.0, 0.0
    for punto in array:
        cx += punto.x
        cy += punto.y
        cz += punto.z
    cx /= n
    cy /= n
    cz /= n

    mpc_target_ = Position()
    mpc_target_.desired_position.x = cx
    mpc_target_.desired_position.y = cy
    mpc_target_.desired_position.z = cz

    return mpc_target_


def print_mpc_parameters(N_mpc, d_ref, v_ref, N_neigh, w_sep, w_final_cm, w_vel,
                         w_t4d, w_sep_near_tgt_t4d, w_sep_near_tgt, w_sep_emergency,
                         w_final_near_tgt_t4d, w_vel_near_tgt, w_t4d_near_tgt,
                         r_drone, r_safe, pace_mpc, numerical_method,
                         gap_end_loop, gap_modify_weights, lim_vel_mpc):
    print('\n------------- M P C   P A R A M E T E R S -------------')

    print(f"N_mpc: {N_mpc}")
    print(f"d_ref: {d_ref}")
    print(f"v_ref: {v_ref}")
    print(f"N_neigh: {N_neigh}")
    print(f"w_sep: {w_sep}")
    print(f"w_final_cm: {w_final_cm}")
    print(f"w_vel: {w_vel}")
    print(f"w_t4d: {w_t4d}")
    print(f"w_sep_near_tgt_t4d: {w_sep_near_tgt_t4d}")
    print(f"w_sep_near_tgt: {w_sep_near_tgt}")
    print(f"w_sep_emergency: {w_sep_emergency}")
    print(f"w_final_near_tgt_t4d: {w_final_near_tgt_t4d}")
    print(f"w_vel_near_tgt: {w_vel_near_tgt}")
    print(f"w_t4d_near_tgt: {w_t4d_near_tgt}")
    print(f"r_drone: {r_drone}")
    print(f"r_safe: {r_safe}")
    print(f"pace_mpc: {pace_mpc}")
    print(f"numerical_method: {numerical_method}")
    print(f"gap_end_loop: {gap_end_loop}")
    print(f"gap_modify_weights: {gap_modify_weights}")
    print(f"lim_vel_mpc: {lim_vel_mpc}")
    # eventualmente, aggiungere breve descrizione (in inglese)


###########################################################################

#                P U B L I S H E R      F U N C T I O N S

###########################################################################

# mpc_velocity_publishers è lista di mpc_velocity_pub singoli publisher per drone
def make_mpc_velocity_publishers():
    for cf_name in cf_names:
        # viene pubblicata la velocità di un singolo drone nel canale opportuno...
        mpc_velocity_pub = rospy.Publisher('/' + cf_name + '/mpc_velocity',
                                           Position, queue_size=1)  # Position: provvisorio
        mpc_velocity_publishers.append(mpc_velocity_pub)


# pubblico velocità output controllo mpc mediante publisher coerente
def swarm_mpc_velocity_pub(mpc_velocity_):
    index = 0
    for index, mpc_velocity_pub in enumerate(mpc_velocity_publishers):
        mpc_velocity_pub.publish(mpc_velocity_[index])  # TODO: pubblicazione velocità non in parallelo ! (Giovanni)
        time.sleep(0.1)


###########################################################################

#               S U B S C R I B E R     C A L L B A C K S

###########################################################################

# TODO: riceve messaggio ogni 5 Hz
def get_state_callback(msg):
    # provvisorio
    global swarm_states
    swarm_states = msg

    global flag_state
    flag_state = True


###########################################################################

#   S U B S C R I B E R     C A L L B A C K S   -   G E T   I N P U T

###########################################################################

def get_N_cf(msg):
    global N_cf
    N_cf = msg


def get_scelta_target(msg):
    global scelta_target
    scelta_target = msg


def get_mpc_target(msg):
    global mpc_target
    mpc_target = msg


def get_tgt4drone(msg):
    global tgt4drone_array
    tgt4drone_array.append(msg)


def get_obs(msg):
    global obs_array
    obs_array.append(msg)


def get_flag(msg):
    global all_messages_received
    all_messages_received = True


# TODO: provvisorio <---
#  sostituire con condizione più snella
def check_messages_received():
    global all_messages_received

    if flag_go_on:
        if N_cf.data != 0 and scelta_target.data == 1 and len(obs_array) != 0:
            if mpc_target.desired_position.x != 0.0 or mpc_target.desired_position.y != 0.0 or mpc_target.desired_position.z != 0.0:
                all_messages_received = True

        if N_cf.data != 0 and scelta_target.data == 2 and len(obs_array) != 0:
            if len(tgt4drone_array) != 0:
                all_messages_received = True


##################################################################

#                            M A I N

##################################################################

if __name__ == '__main__':

    rospy.init_node('node_mpc_swarm_mv', log_level=rospy.DEBUG)

    all_messages_received = False
    flag_state = False

    print("\n           [ M P C   C O N T R O L L E R   F O R   A   S W A R M   O F   D R O N E S ]\n")

    ##############################################################

    #  S U B S C R I B E R S   S E T U P   -   G E T   I N P U T

    ##############################################################

    N_cf_sub = rospy.Subscriber('/N_cf_topic', Int32, callback=get_N_cf)
    N_cf = Int32()

    scelta_target_sub = rospy.Subscriber('/scelta_target_topic', Int32, callback=get_scelta_target)
    scelta_target = Int32()

    mpc_target_sub = rospy.Subscriber('/mpc_target_topic', Position, callback=get_mpc_target)
    mpc_target = Position()

    tgt4drone_sub = rospy.Subscriber('/tgt4drone_topic', Point, callback=get_tgt4drone)
    tgt4drone_array = []

    obs_sub = rospy.Subscriber('/obs_topic', SphereData, callback=get_obs)
    obs_array = []

    sub_flag_msg_pub = rospy.Subscriber('/flag_topic', Empty, callback=get_flag)
    flag_msg_pub = Empty()

    ##############################################################

    #               P U B L I S H E R S   S E T U P

    ##############################################################

    # per atterrare - provvisorio
    land_pub = rospy.Publisher('/land_topic', Empty, queue_size=1)
    land_trigger = Empty()

    ##############################################################

    #               S U B S C R I B E R S   S E T U P

    ##############################################################

    state_sub = rospy.Subscriber('/swarm/states', SwarmStates, get_state_callback, queue_size=1)
    swarm_states = SwarmStates()

    ##############################################################

    #                   G E T T I N G   I N P U T

    ##############################################################

    print('Waiting for inputs by user...')

    while not all_messages_received:
        time.sleep(0.1)

    print('Inputs by user have been received correctly!')

    ##############################################################

    #       P R I N T I N G   I N P U T   A C Q U I R E D

    ##############################################################

    print('\n--------- R E C E I V E D   M E S S A G E S --------')

    print("number of drones = ", N_cf.data)

    print('modality of target: ', end='')
    if scelta_target.data == 1:
        print('center of mass')
    if scelta_target.data == 2:
        print('each drone')

    if scelta_target.data == 1:
        print("mpc_target = ", mpc_target.desired_position.x, ' ',
              mpc_target.desired_position.y, ' ',
              mpc_target.desired_position.z)

    if scelta_target.data == 2:
        for i in range(len(tgt4drone_array)):
            print("tgt4drone ", i + 1, ':     x = ', tgt4drone_array[i].x,
                  '     y = ', tgt4drone_array[i].y,
                  '     z = ', tgt4drone_array[i].z)
        mpc_target = calcolo_centro_massa_target(tgt4drone_array)
        print("-> mpc_target from tgt4drones = ", mpc_target.desired_position.x, ' ',
              mpc_target.desired_position.y, ' ',
              mpc_target.desired_position.z)

    for i in range(len(obs_array)):
        print("spherical obstacle ", i + 1, ':     x = ', obs_array[i].x,
              '     y = ', obs_array[i].y,
              '     z = ', obs_array[i].z,
              '     r = ', obs_array[i].r)

    ##############################################################

    # Generate a standard list of names:  # OK - solo per creare canali con nome opportunamente
    cf_names = standardNameList(N_cf.data)

    ##############################################################

    #    P U B L I S H E R S   S E T U P   -   C O N T I N U E

    ##############################################################

    # List of velocities used to collect the output of the nlp solver
    mpc_velocity = []
    for ii in range(N_cf.data):
        mpc_velocity.append(Position())  # Position() provvisorio

    # List of mpc_velocity Publishers
    mpc_velocity_publishers = []
    make_mpc_velocity_publishers()

    ##############################################################

    #               W A I T I N G   F O R   S T A T E

    ##############################################################

    print('\n--------- W A I T I N G   F O R   S T A T E --------')
    print('state not yet received - keep waiting...')

    while not flag_state:
        time.sleep(0.1)

    print('state received!')
    # -> state obtained !

    ##############################################################

    #                   M P C   P A R A M E T E R S

    ##############################################################

    N_mpc = 5  # number of MPC time steps
    d_ref = 0.3  # reference distance between agents
    v_ref = 0.25  # reference velocity

    if N_cf.data < 3:
        N_neigh = N_cf.data - 1  # TODO: provvisorio
    else:
        N_neigh = 2
    # N_neigh: number of neighbours per drone

    # Weights for objective function - standard
    w_sep = 10  # separation term
    w_final_cm = 10  # final position penalty term
    w_vel = 30  # velocity control input term
    w_t4d = 1  # target 4 drone term

    # Weights for objective function - exception
    """
    exceptions:
    _near_tgt: near target
    _t4d: caso modalità target assegnata: target definito per ogni drone
    _emergency: caso rischio collisione
    """
    w_sep_near_tgt_t4d = w_sep * 0.1
    w_sep_near_tgt = w_sep * 10
    w_sep_emergency = w_sep * 50
    w_final_near_tgt_t4d = w_final_cm * 0.1
    w_vel_near_tgt = w_vel * 10
    w_t4d_near_tgt = w_t4d * 100

    r_drone = 0.1
    r_safe = 0.2

    pace_mpc = 100  # MPC frequency [Hz]
    # TODO: verificare possibilità di effettuare un certo pace_mpc mediante time time (tic tac)
    # TODO: rate.sleep() non porta ad avere una frequenza = pace_mpc,
    #  è semplicemente equivalente a un time.sleep(), a una pausa! correggere... (stessa cosa per state_pub) <---

    # metodo discretizzazione MPC: 'forward_euler' / 'inverse_euler'
    numerical_method = 'forward_euler'
    # numerical_method = 'inverse_euler'

    gap_end_loop = 0.15  # condizione per uscire da while MPC
    gap_modify_weights = 0.25  # condizione per modificare weights
    lim_vel_mpc = 0.35  # limite velocità output MPC

    print_mpc_parameters(N_mpc, d_ref, v_ref, N_neigh, w_sep, w_final_cm, w_vel,
                         w_t4d, w_sep_near_tgt_t4d, w_sep_near_tgt, w_sep_emergency,
                         w_final_near_tgt_t4d, w_vel_near_tgt, w_t4d_near_tgt,
                         r_drone, r_safe, pace_mpc, numerical_method,
                         gap_end_loop, gap_modify_weights, lim_vel_mpc)

    #################################################################

    #    I N I T I A L I Z I N G   M P C   ( H O T   S T A R T )

    #################################################################

    print('\n--------- M O D E L   P R E D E C T I V E   C O N T R O L --------')

    P_0 = []
    for ii in range(N_cf.data):
        P_0.append(swarm_states.states[ii].position.x)
        P_0.append(swarm_states.states[ii].position.y)
        P_0.append(swarm_states.states[ii].position.z)
        # tbn: P_0 contiene gli stati dello sciame di droni (P_0[0:3]: cf1, P_0[3:6]: cf2, etc.)

        # TODO: MPC solo su posizione, (si potrebbe sfruttare anche la info sulla velocità) (Giovanni)

    P_N = []

    for ii in range(N_cf.data):
        P_N.append(mpc_target.desired_position.x)
        P_N.append(mpc_target.desired_position.y)
        P_N.append(mpc_target.desired_position.z)

    # Initializing the optimal velocity of agents to use it
    # for the hot start initial guess
    v_opt_old = []
    for ii in range(3 * N_cf.data):
        v_opt_old.append(np.zeros(N_mpc + 1))  # TODO: hot start puo essere migliorato... (Giovanni)

    # Initializing optimal positions to use them as initial
    # guess for the hot start initial guess
    x_opt_old = []
    for ii in range(3 * N_cf.data):
        x_opt_old.append(np.linspace(P_0[ii], P_N[ii], N_mpc + 1))

    ##############################################################################

    #                                  M P C

    ##############################################################################

    rate = rospy.Rate(pace_mpc)

    while not rospy.is_shutdown():

        t_mpc_total = time.time()  # tic
        # TODO: riposizionare per ottenere precisamente tempo tra ottenimento di 2 stati...

        # swarm_states è l'ultimo swarm_states messo a disposizione da global swarm_states in get_state_callback
        P_0 = []
        for ii in range(N_cf.data):
            P_0.append(swarm_states.states[ii].position.x)
            P_0.append(swarm_states.states[ii].position.y)
            P_0.append(swarm_states.states[ii].position.z)

        print('\n--- S T A T E   O B T A I N E D ---')
        for ii in range(N_cf.data):
            print('cf' + str(ii + 1) + ':     x = ', P_0[ii * 3],
                  '     y = ', P_0[ii * 3 + 1],
                  '     z = ', P_0[ii * 3 + 2])

        # ++++++++++++ CHECKING IF TARGET HAS BEEN REACHED +++++++++++

        if scelta_target.data == 1:

            x_cm = np.array(P_0[0::3]).sum() / N_cf.data
            y_cm = np.array(P_0[1::3]).sum() / N_cf.data
            z_cm = np.array(P_0[2::3]).sum() / N_cf.data

            distance_from_tgt_cm = ((x_cm - mpc_target.desired_position.x) ** 2 +
                                    (y_cm - mpc_target.desired_position.y) ** 2 +
                                    (z_cm - mpc_target.desired_position.z) ** 2) ** 0.5

            if distance_from_tgt_cm < gap_end_loop:
                target_reached()

        # TODO: creare condizione - vedi condizione_scelta_target_2.py - per atterraggio!
        # TODO: provvisorio - NON FUNZIONA! <---
        """
        tbn: limitazione: target_reached viene chiamato anche nel caso in cui
        tutti i droni siano nei pressi di un unico obiettivo;
        tuttavia MPC e lagrangiana con separation cost dovrebbero evitare ciò  (verificare...)
        """
        if scelta_target.data == 2:

            count = 0

            for i in range(N_cf.data):  # faccio passare ogni drone...
                for j in range(N_cf.data):  # faccio passare ogni obiettivo...
                    distance_from_tgt_4drone = ((P_0[i * 3] - tgt4drone_array[j].x) ** 2 +
                                                (P_0[i * 3 + 1] - tgt4drone_array[j].y) ** 2 +
                                                (P_0[i * 3 + 2] - tgt4drone_array[j].z) ** 2) ** 0.5
                    # calcolo la distanza tra il drone e l'obiettivo

                    if distance_from_tgt_4drone < gap_end_loop:
                        count += 1
                        break
                    # se trovo un drone abbastanza vicino a un obiettivo:
                    # incremento count e passo al drone successivo

            if count == N_cf.data:
                target_reached()
            # se alla fine tutti i droni sono abbastanza vicini a un obiettivo:
            # ho raggiunto l'obiettivo e chiamo target_reached

        # +++++++++++++++++++ MPC CONTROLLER +++++++++++++++++++

        t_mpc_fun = time.time()

        mpc_velocity, x_opt, v_opt = nlp_solver_3d(N_cf.data, P_N,
                                                   P_0, N_mpc, x_opt_old, v_opt_old,
                                                   d_ref, v_ref, w_sep, w_final_cm, w_vel, w_t4d,
                                                   N_neigh, obs_array, r_drone, r_safe,
                                                   scelta_target.data, tgt4drone_array,
                                                   numerical_method, gap_modify_weights, w_sep_near_tgt,
                                                   w_sep_near_tgt_t4d, w_sep_emergency, w_final_near_tgt_t4d,
                                                   w_vel_near_tgt, w_t4d_near_tgt, lim_vel_mpc)

        elapsed_mpc_fun = time.time() - t_mpc_fun  # tac
        print('\nelapsed time (nlp_solver_3d): ', elapsed_mpc_fun)
        print('frequency (nlp_solver_3d): ', 1 / elapsed_mpc_fun)

        x_opt_old, v_opt_old = x_opt, v_opt

        print('\n--- M P C   O U T P U T   V E L O C I T Y ---')
        for velocity in mpc_velocity:
            print(velocity.name, ':     Vx = ', velocity.desired_velocity.x,
                  '     Vy = ', velocity.desired_velocity.y,
                  '     Vz = ', velocity.desired_velocity.z)

        # +++++++++++++++++++ PUBLISHING MPC VELOCITY +++++++++++++++++++

        swarm_mpc_velocity_pub(mpc_velocity)  # commentare per passare da sperimentale a debug
        rate.sleep()

        elapsed_mpc_tot = time.time() - t_mpc_total  # tac
        print('\nelapsed time (mpc total): ', elapsed_mpc_tot)
        print('frequency (mpc total): ', 1 / elapsed_mpc_tot)
