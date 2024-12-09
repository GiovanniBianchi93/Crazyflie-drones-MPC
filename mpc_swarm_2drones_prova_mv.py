#! /usr/bin/env python3

# mv: file originale: swarm_cen_cbf_coll_avoid_node.py

# TODO - integrare pubblicazione efficacie di velocità per ogni drone dello sciame! / swarm_mpc_velocity_pub(mpc_velocity)
# TODO - verificare MPC, indici,... /vedi TODO (sembra ok)
# TODO - migliorare MPC (lagrangiana,...)
# TODO - generalizzare ad uno sciame di N_cf > 2
# TODO - capire come organizzare controllo - controllo ibrido MPC + CBF ?
#  centralizzato / distribuito / <- design architettura controllo !

# TODO: da w_ a gradino a w_ più graduale, in funzione della posizione del centro di massa o di ...

# TODO: rivedere quale delle due condizioni qui sotto è maggiormente opportuna in base al contesto
'''
CONDIZIONE 1:
if abs(P_0[3 * ii] - x_des) < 1 and abs(P_0[3 * ii + 1] - y_des) < 1 and abs(P_0[3 * ii + 2] - z_des) < 1:

CONDIZIONE 2:
if abs(x_cm - x_des) < 1 and abs(y_cm - y_des) < 1 and abs(z_cm - z_des) < 1:
'''

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

# provvisorio
uris = []
uri = 'radio://0/80/2M/E7E7E7E7E2'
uris.append(uri)
uri = 'radio://0/80/2M/E7E7E7E7E7'
uris.append(uri)

# TODO: CBF ?
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

#             H I G H    L E V E L    M P C  -  3 D (mv)

###################################################################


def nlp_solver_3d(N_cf, P_N, P_0, N, x_opt, v_opt,
                  d_ref, v_ref, w_sep,
                  w_final, w_vel, N_neigh, obstacles, r_drone, r_safe,
                  target_assegnato_per_ogni_drone, tgt4drone):
    # Getting center of mass of the swarm
    x_cm = np.array(P_0[0::3]).sum() / N_cf
    y_cm = np.array(P_0[1::3]).sum() / N_cf
    z_cm = np.array(P_0[2::3]).sum() / N_cf

    # Desired positions
    x_des = P_N[0]
    y_des = P_N[1]
    z_des = P_N[2]

    # Calculating the reference direction for the drone swarm
    u_ref = [x_des - x_cm, y_des - y_cm, z_des - z_cm]
    u_ref = np.array(u_ref)
    u_norm = np.linalg.norm(u_ref)

    # Calculating the time horizon for the MPC
    T = u_norm / v_ref

    # Threshold on time horizon, otherwise the swarm oscillates
    if T < 1.0:
        T = 1.0

    # Versor for the reference direction
    u_ref = u_ref / u_norm

    u_refx = u_ref[0]
    u_refy = u_ref[1]
    u_refz = u_ref[2]

    # Setting desired velocity
    vx_des = (x_des - x_cm) / T
    vy_des = (y_des - y_cm) / T
    vz_des = (z_des - z_cm) / T

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
    x = MX.sym('x', N_cf * 3)

    v = MX.sym('v', N_cf * 3)

    # Model equations
    xdot = v

    '''
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

    ############################ List of neighbours ##############################
    '''

    # Building Ordered list of N_neigh closest neighbors for each drone

    index_neigh = []

    for ii in range(N_cf):
        list_p_rel = []
        for jj in range(N_cf):
            p_rel = [P_0[3 * ii] - P_0[3 * jj], P_0[3 * ii + 1] - P_0[3 * jj + 1], P_0[3 * ii + 2] - P_0[3 * jj + 2]]
            p_rel = np.array(p_rel)
            # Storing p_rel in a list
            list_p_rel.append(np.linalg.norm(p_rel))

        # Getting the sorted indices of list_p_rel to set the order of neighbours
        index_neigh_i = (np.argsort(list_p_rel)).tolist()
        ####### index_neigh_i.pop()  # elimino la distanza maggiore (?)  # TODO: ERRORE! sarebbe da eliminare il drone più vicino, ovvero il drone stesso, non quello più lontano!
        index_neigh_i.pop(0)  # elimino il drone stesso - ok
        index_neigh.append(index_neigh_i)

    ###################### Ordered list of neighbours ############################
    # estraggo da matrice i vettori con indici droni "neigh" relativi a ogni singolo drone

    list_list_neighbours = []

    for ii in range(N_cf):
        if N_cf > N_neigh:
            list_neighbours_i = index_neigh[ii][:N_neigh]
        else:
            list_neighbours_i = index_neigh[ii][:(N_cf - 1)]
        list_list_neighbours.append(list_neighbours_i)

        # list_list_neighbours è una lista di liste di vicini ordinati
        # Ogni lista interna contiene gli indici dei droni vicini al drone corrispondente
        # nell’ordine di vicinanza crescente

    ##############################################################################

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

    '''
    # mv: aumento importanza Lsep se in prossimità di obiettivo (vs dispersione)
    if abs(x_cm - x_des) < 1 and abs(y_cm - y_des) < 1 and abs(z_cm - z_des) < 1:
        w_sep = 10 * w_sep    
    else:
        w_sep = w_sep
    '''

    # ------------- I N S E R I R E   T I T O L O -------------

    # TODO: controllare ...
    # TODO: verificare pesi, e limite a fini di sicurezza - attualmente fissato a d_ref * 0.5

    for ii in range(N_cf):
        for kk in list_list_neighbours[ii]:
            if ((P_0[3 * ii] - P_0[3 * kk]) ** 2 +
                (P_0[3 * ii + 1] - P_0[3 * kk + 1]) ** 2 +
                (P_0[3 * ii + 2] - P_0[3 * kk + 2]) ** 2) \
                    < (r_drone * 2 + r_safe) ** 2:
                w_sep_i = w_sep * 200  # a fini di sicurezza! rischio collisione!
            else:
                if target_assegnato_per_ogni_drone:
                    if abs(P_0[3 * ii] - x_des) < 1 and abs(P_0[3 * ii + 1] - y_des) < 1 and abs(
                            P_0[3 * ii + 2] - z_des) < 1:
                        w_sep_i = w_sep * 0.1  # in prossimità di target subentra L_tgt4drone
                    else:
                        w_sep_i = w_sep
                else:
                    if abs(x_cm - x_des) < 1 and abs(y_cm - y_des) < 1 and abs(
                            z_cm - z_des) < 1:
                        w_sep_i = w_sep * 50  # aumento importanza Lsep se in prossimità di obiettivo, vs dispersione

                    else:
                        w_sep_i = w_sep

            # Separation cost
            d_ref_sep = d_ref

            L += w_sep_i * ((x[ii * 3] - x[kk * 3]) ** 2
                            + (x[ii * 3 + 1] - x[kk * 3 + 1]) ** 2
                            + (x[ii * 3 + 2] - x[kk * 3 + 2]) ** 2
                            - d_ref_sep ** 2) ** 2

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

    # TODO: controllare...

    # ------------- I N S E R I R E   T I T O L O -------------

    x_cm_sym = 0
    y_cm_sym = 0
    z_cm_sym = 0

    for ii in range(N_cf):
        x_cm_sym += x[ii * 3]
        y_cm_sym += x[ii * 3 + 1]
        z_cm_sym += x[ii * 3 + 2]

    x_cm_sym = x_cm_sym / N_cf
    y_cm_sym = y_cm_sym / N_cf
    z_cm_sym = z_cm_sym / N_cf

    for ii in range(N_cf):
        if target_assegnato_per_ogni_drone:
            if abs(P_0[3 * ii] - x_des) < 1 and abs(P_0[3 * ii + 1] - y_des) < 1 and abs(P_0[3 * ii + 2] - z_des) < 1:
                w_final_i = 0.1 * w_final
            else:
                w_final_i = w_final
        else:
            w_final_i = w_final

        L += w_final_i * ((x_cm_sym - x_des) ** 2 + (y_cm_sym - y_des) ** 2 + (z_cm_sym - z_des) ** 2) ** 2

    # ------------- I N S E R I R E   T I T O L O -------------

    for ii in range(N_cf):
        # Control input cost (greater weight when close to target)
        if abs(P_0[3 * ii] - x_des) < 1 and abs(P_0[3 * ii + 1] - y_des) < 1 and abs(P_0[3 * ii + 2] - z_des) < 1:
            # TODO: P_0 ? o x_cm? (non x_cm_sym, ovviamente...)
            # w_vel_i = 10 * w_vel
            w_vel_i = w_vel
        else:
            w_vel_i = w_vel

        L += w_vel_i * (v[ii * 3] ** 2 + v[ii * 3 + 1] ** 2 + v[ii * 3 + 2] ** 2)

    # # Final Position cost for center of mass
    # L += 0.02*w_final*(P_N[0] - x_cm)**2
    # L += 0.02*w_final*(P_N[1] - y_cm)**2

    # ------------- I N S E R I R E   T I T O L O -------------

    if target_assegnato_per_ogni_drone:  # if not tgt4drone == 0:
        L_tgt4drone = 0

        for ii in range(N_cf):

            L_tgt4drone_single = 0

            if abs(P_0[3 * ii] - x_des) < 1 and abs(P_0[3 * ii + 1] - y_des) < 1 and abs(P_0[3 * ii + 2] - z_des) < 1:
                # TODO: P_0 ? o x_cm? (non x_cm_sym)
                w_tapod = 100

            else:
                w_tapod = 1

            for jj in range(N_cf):
                L_tgt4drone_single *= w_tapod * (
                        (x[ii * 3] - tgt4drone[jj]['x']) ** 2 + (x[ii * 3 + 1] - tgt4drone[jj]['y']) ** 2 + (
                        x[ii * 3 + 2] - tgt4drone[jj]['z']) ** 2) ** 2
                # TODO: controllare...

            L_tgt4drone += L_tgt4drone_single

        L += L_tgt4drone

    # ------------- S C E G L I   M E T O D O   N U M E R I C O -------------

    # scegli modello numerico pe la risoluzione di ODE...

    # TODO: scelta in my_swarm_node_mv_altro.py

    scelta = 'forward'
    # scelta = 'inverse'
    # scelta = altro...

    if scelta == 'forward':
        # Forward Euler
        DT = T / N
        f = Function('f', [x, v], [xdot, L])
        X0 = MX.sym('X0', 3 * N_cf)
        U = MX.sym('U', 3 * N_cf)
        X = X0
        Q = 0
        k, k_q = f(X, U)
        X = X + k * DT
        Q = Q + k_q * DT
        F = Function('F', [X0, U], [X, Q], ['x0', 'p'], ['xf', 'qf'])

    # TODO: verificare Inverse Euler...
    elif scelta == 'inverse':
        # Inverse Euler
        DT = T / N
        f = Function('f', [x, v], [xdot, L])
        X0 = MX.sym('X0', 3 * N_cf)
        U = MX.sym('U', 3 * N_cf)
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
        print('ERRORE!')
        rospy.is_shutdown()

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

    # Evaluate at a test point
    # Fk = F(x0=[0.2, 0.3],p=[0.4, 2, 9])
    # print(Fk['xf'])
    # print(Fk['qf'])

    # -------------------------------------------------------------------

    #           S E T T I N G   U P   T H E   M P C

    # -------------------------------------------------------------------

    '''
    We only exploit the first control input within the MPC framework.
    Hence, we define a casadi function which allows us to get the 
    control input at the first time step as output, given the initial state
    as input.
    '''

    # MPC LOOP
    P_log = []
    px_log = []
    py_log = []
    V_log = []
    ux_log = []
    uy_log = []

    v_ig = []
    for ii in range(N_cf):
        v_ig.append(vx_des)
        v_ig.append(vy_des)
        v_ig.append(vz_des)

    P_log = P_0  # initial state

    # Initializing state estimate at time instant i
    X_i = P_0
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
    Xk = MX.sym('X0', 3 * N_cf)
    w += [Xk]

    lbw_k = []
    for ii in range(N_cf):
        lbw_k.append(X_i[3 * ii].__float__())
        lbw_k.append(X_i[3 * ii + 1].__float__())
        lbw_k.append(X_i[3 * ii + 2].__float__())

    lbw += lbw_k

    ubw_k = []
    for ii in range(N_cf):
        ubw_k.append(X_i[3 * ii].__float__())
        ubw_k.append(X_i[3 * ii + 1].__float__())
        ubw_k.append(X_i[3 * ii + 2].__float__())

    ubw += ubw_k

    w0_k = []
    for ii in range(N_cf):
        w0_k.append(X_i[3 * ii].__float__())
        w0_k.append(X_i[3 * ii + 1].__float__())
        w0_k.append(X_i[3 * ii + 2].__float__())

    w0 += w0_k

    # Formulate the NLP
    for k in range(N):
        # New NLP variable for the control
        Vk = MX.sym('V_' + str(k), 3 * N_cf)  # creating symbolic expression for the
        # new optimization variable
        w += [Vk]

        lbw_k = []
        for ii in range(N_cf):
            lbw_k.append(-inf)
            lbw_k.append(-inf)
            lbw_k.append(-inf)

        lbw += lbw_k

        ubw_k = []
        for ii in range(N_cf):
            ubw_k.append(+inf)
            ubw_k.append(+inf)
            ubw_k.append(+inf)

        ubw += ubw_k

        w0_k = []
        for ii in range(N_cf):
            w0_k.append(v_opt[3 * ii][k].__float__())  # TODO: ? vs ros_mpc.py - v_i ?
            w0_k.append(v_opt[3 * ii + 1][k].__float__())
            w0_k.append(v_opt[3 * ii + 2][k].__float__())

        w0 += w0_k

        # Integrate till the end of the interval
        Fk = F(x0=Xk, p=Vk)  # we call the integrator
        Xk_end = Fk['xf']
        J = J + Fk['qf']

        # New NLP variable for state at end of interval
        Xk = MX.sym('X_' + str(k + 1), 3 * N_cf)
        w += [Xk]

        lbw_k = []
        for ii in range(N_cf):
            lbw_k.append(-inf)
            lbw_k.append(-inf)
            lbw_k.append(-inf)

        lbw += lbw_k

        ubw_k = []
        for ii in range(N_cf):
            ubw_k.append(+inf)
            ubw_k.append(+inf)
            ubw_k.append(+inf)

        ubw += ubw_k

        w0_k = []
        for ii in range(N_cf):
            w0_k.append(x_opt[3 * ii][k + 1].__float__())
            w0_k.append(x_opt[3 * ii + 1][k + 1].__float__())
            w0_k.append(x_opt[3 * ii + 2][k + 1].__float__())

        w0 += w0_k

        # Add equality constraint (continuity constraint for multiple shooting)
        g += [Xk_end - Xk]

        lbg_k = []
        for ii in range(N_cf):
            lbg_k.append(0)
            lbg_k.append(0)
            lbg_k.append(0)

        lbg += lbg_k

        ubg_k = []
        for ii in range(N_cf):
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

        # TODO: OSTACOLI FISICI SFERICI
        #  tbn: limitazione a soli ostacoli sferici!
        #  ostacolo parallelepipedo trattato come area 3D vietata
        #  vedi ros_moc.py, Casadi forum /non funziona

        '''
        for jj in range(N_cf):
            for ii in range(len(obstacles)):
                # ostacolo sferico
                g += [(Xk[3 * jj] - obstacles[ii]['x']) ** 2 + (Xk[3 * jj + 1] - obstacles[ii]['y']) ** 2 +
                      (Xk[3 * jj + 2] - obstacles[ii]['z']) ** 2]
                lbg += [(obstacles[ii]['r'] + r_drone + r_safe) ** 2]
                ubg += [+inf]
        '''

        ################ Center of mass in final target ##########################

        # TODO: Terminal Penalty Cost ? - vedi ros_mpc.py

        if k == N - 1:

            if target_assegnato_per_ogni_drone:
                # TODO: aggiungere posizione finale precisa! (FACOLTATIVO)
                #  (tbn: difficile, dal momento che non conosco a priori
                #  dove i droni si debbano posizionare ottimalmente)
                #  soluzione: soft constraint già presente (vedi L_tgt4drone), ok
                pass

            else:
                x_cm_end = 0
                y_cm_end = 0
                z_cm_end = 0

                for ii in range(N_cf):
                    x_cm_end += Xk_end[3 * ii]
                    y_cm_end += Xk_end[3 * ii + 1]
                    z_cm_end += Xk_end[3 * ii + 2]

                x_cm_end = x_cm_end / N_cf
                y_cm_end = y_cm_end / N_cf
                z_cm_end = z_cm_end / N_cf

                g += [P_N[0] - x_cm_end, P_N[1] - y_cm_end, P_N[2] - z_cm_end]

                lbg += [0, 0, 0]
                ubg += [0, 0, 0]

        ################################################################

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

        #     lbg += lbg_k

        #     ubg_k = []

        #     for ii in range(N_cf):
        #         ubg_k.append(0)
        #         ubg_k.append(0)

        #     ubg += ubg_k

        ################################################################

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

    ####################################################################

    # Create an NLP solver
    # NLP solver options
    opts = {}
    opts["max_iter_eig"] = 5
    # opts = {'ipopt.print_level': 0, 'print_time': 0}  # TODO: verificare effettiva corretta implementazione...

    prob = {'f': J, 'x': vertcat(*w), 'g': vertcat(*g)}
    solver = nlpsol('solver', 'ipopt', prob, opts)

    # Solve the NLP
    sol = solver(x0=w0, lbx=lbw, ubx=ubw, lbg=lbg, ubg=ubg)
    w_opt = sol['x'].full().flatten()

    # Extracting the optimal state
    x_opt = []
    for ii in range(3 * N_cf):
        x_opt.append(w_opt[ii::6 * N_cf])  # x_opt comprende tutti gli x_opt di ogni drone dello swarm...

    # Extracting the optimal control input
    v_opt = []
    for ii in range(3 * N_cf):
        v_opt.append(w_opt[(ii + 3 * N_cf)::6 * N_cf])  # TODO: ? verificare indici (sembra ok)

    # Extracting only the first optimal control input
    v_opt_i = []
    for ii in range(N_cf):
        v_opt_i.append(v_opt[3 * ii][0])
        v_opt_i.append(v_opt[3 * ii + 1][0])
        v_opt_i.append(v_opt[3 * ii + 2][0])

    v_i = v_opt_i

    mpc_velocity = []

    for ii in range(N_cf):
        mpc_velocity.append(Position())

    for ii in range(N_cf):
        mpc_velocity[ii].name = 'cf' + str(
            ii + 1)  # provvisorio - vs errore in my_swarm_node_mv_altro.py  # TODO: verificare correttezza drone associato a preciso mpc_velocity
        mpc_velocity[ii].desired_velocity.x = v_i[3 * ii]
        mpc_velocity[ii].desired_velocity.y = v_i[3 * ii + 1]
        mpc_velocity[ii].desired_velocity.z = v_i[3 * ii + 2]

        # TODO: aggiungere limitazione MPC velocity!

    print('mpc_velocity prodotta da controllore MPC e\':')
    print(mpc_velocity)

    return mpc_velocity, x_opt, v_opt


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
def swarm_mpc_velocity_pub(mpc_velocity):
    index = 0
    for index, mpc_velocity_pub in enumerate(mpc_velocity_publishers):
        mpc_velocity_pub.publish(mpc_velocity[index])  # TODO: pubblicazione velocità non in parallelo !


def get_state_callback(msg):
    # provvisorio
    global swarm_states
    swarm_states = msg  # TODO (sembra ok)

    global flag_state
    flag_state = True

    # print('\nstato ricevuto \n')


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

    sub_mpc_flag.data = 1

    print('TARGET: ')
    print(mpc_target)


def get_tgt4drone(msg):
    global tgt4drone_array
    tgt4drone_array.append(msg)


def get_obs(msg):
    global obs_array
    obs_array.append(msg)


def check_messages_received():
    global all_messages_received

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

    ##############################################################

    #                       G E T   I N P U T

    ##############################################################

    ##############################################################

    #  S U B S C R I B E R S   S E T U P   -   G E T   I N P U T

    ##############################################################

    N_cf_sub = rospy.Subscriber('N_cf_topic', Int32, callback=get_N_cf)
    N_cf = Int32()

    scelta_target_sub = rospy.Subscriber('scelta_target_topic', Int32, callback=get_scelta_target)
    scelta_target = Int32()

    mpc_target_sub = rospy.Subscriber('/swarm/mpc_target', Position, callback=get_mpc_target)
    mpc_target = Position()

    tgt4drone_sub = rospy.Subscriber('/tgt4drone_topic', Point, callback=get_tgt4drone)
    tgt4drone_array = []

    obs_sub = rospy.Subscriber('/obs_topic', SphereData, callback=get_obs)
    obs_array = []

    print('in attesa di messaggi...')

    while not all_messages_received:
        check_messages_received()
        time.sleep(0.1)

    print('tutti i messaggi sono stati ricevuti correttamente!')

    print('\n--------- M E S S A G G I   R I C E V U T I : --------\n')

    print("N_cf = ", N_cf.data, '\n')
    print("scelta target = ", scelta_target.data, '\n')
    print("mpc_target = ", mpc_target, '\n')
    print('\ntarget centro di massa acquisito: ', mpc_target.desired_position.x, ' ', mpc_target.desired_position.y,
          ' ', mpc_target.desired_position.z)
    print("tgt4drone_array = ", tgt4drone_array, '\n')
    print("ostacoli = ", obs_array, '\n')

    ##############################################################

    #                   F I N E   G E T   I N P U T

    ##############################################################

    # Generate a standard list of names:
    cf_names = standardNameList(N_cf.data)  # TODO: provvisorio, passare il nome effettivo dei droni

    # +++++++++++++++++++ MPC PARAMETERS ++++++++++++++++++++++

    N_mpc = 5  # number of MPC time steps
    d_ref = 0.85  # reference distance between agents
    v_ref = 0.5  # reference velocity
    N_neigh = 1  # number of neighbours per drone / 1: swarm 2 droni...

    # Weights for objective function
    w_sep = 10  # separation term
    w_final = 10  # final position penalty term
    w_vel = 50  # velocity control input term

    r_drone = 0.1
    r_safe = 0.2

    # TODO: inizia a commentare
    '''
    # +++++++++++++++++++ MPC (SPHERICAL) OBSTACLES ++++++++++++++++++++++

    n = 1  # numero ostacoli sferici

    # inizializzo obs
    obs = []
    for i in range(n):
        obs.append({'x': 0.0, 'y': 0.0, 'z': 0.0, 'r': 0.0})  # inizializzo obs lista di dizionari

    # definisco obs
    obs[0] = {'x': 3.0, 'y': 0.0, 'z': 0.5, 'r': 0.1}  # singolo dizionario
    # obs[1] = {'x': 2.2, 'y': 1.0, 'z': 0.1, 'r': 0.9}

    # x_obs1 = obs[0]['x']  # debug # ottengo valore preciso mediante 'notazione a chiave'

    # +++++++++++++++++++ M O D A L I T A'   T A R G E T ++++++++++++++++++++++

    target_assegnato_per_ogni_drone = True
    # target_assegnato_per_ogni_drone = False

    if target_assegnato_per_ogni_drone:
        tgt4drone = []  # contiene i singoli target per ogni drone

        # inizializzo tgt4drone
        for i in range(N_cf):
            tgt4drone.append({'x': 0.0, 'y': 0.0, 'z': 0.0})  # inizializzo obs lista di dizionari

        # assegno valori a tgt4drone da terminale
        print('assegnare target drone per drone')
        for i in range(N_cf):
            tgt4drone[i]['x'] = input("target %d - x:  " % (i+1))
            tgt4drone[i]['y'] = input("target %d - y:  " % (i+1))
            tgt4drone[i]['z'] = input("target %d - z:  " % (i+1))

        # provvisorio - caso N_cf = 2
        tgt4drone[0] = {'x': 1.8, 'y': 0.6, 'z': 0.5}  # target1
        tgt4drone[1] = {'x': 2.2, 'y': 0.6, 'z': 0.5}  # target2

    else:
        tgt4drone = 0
    '''

    ##############################################################

    #               P U B L I S H E R S   S E T U P

    ##############################################################

    # List of velocities used to collect the output of the nlp solver
    mpc_velocity = []
    for ii in range(N_cf.data):
        mpc_velocity.append(Position())  # Position() provvisorio

    # List of mpc_velocity Publishers
    mpc_velocity_publishers = []
    make_mpc_velocity_publishers()

    # TODO: non funziona
    # per atterrare - provvisorio
    land_pub = rospy.Publisher('/land_topic_prova', Empty, queue_size=1)
    land_trigger = Empty()

    ##############################################################

    #               S U B S C R I B E R S   S E T U P

    ##############################################################
    '''
    # Subscriber to get the mpc target position
    mpc_target_sub = rospy.Subscriber('/swarm/mpc_target', Position,
                                      mpc_target_sub_callback)
    mpc_target = Position()
    '''

    state_sub = rospy.Subscriber('/swarm/states', SwarmStates, get_state_callback, queue_size=1)
    swarm_states = SwarmStates()
    ##############################################################

    # Flag for the mpc target subscriber
    sub_mpc_flag = Int16()
    sub_mpc_flag.data = 0

    # Initializing mpc_target
    mpc_target.desired_position.x = 0
    mpc_target.desired_position.y = 0
    mpc_target.desired_position.z = 0

    rate = rospy.Rate(20)

    flag_state = False

    gap = 0.5  # condizione per uscire da while MPC...

    while not rospy.is_shutdown():
        if flag_state:  # procedo se ho in memoria uno stato dello swarm...
            '''
            # global swarm_states
            crazyflie_states = [CrazyflieState(), CrazyflieState()]
            swarm_states = SwarmStates()
            swarm_states.states = crazyflie_states
            '''

            # Initializing the initial position of agents at each step
            # swarm_states è l'ultimo swarm_states messo a disposizione da global swarm_states in get_state_callback
            P_0 = []
            for ii in range(N_cf):
                P_0.append(swarm_states.states[ii].position.x)  # TODO: verificare swarm_states.states (sembra ok)
                P_0.append(swarm_states.states[ii].position.y)
                P_0.append(swarm_states.states[ii].position.z)
            print('l\'ultimo stato ricevuto è:')
            print(P_0)

            if sub_mpc_flag.data == 0:
                # nothing is executed if no mpc target has been published
                pass

            elif sub_mpc_flag.data == 1:  # in case a new target is set
                # Initializing the target position for each drone starting
                # from mpc target
                P_N = []

                for ii in range(N_cf):
                    P_N.append(mpc_target.desired_position.x)
                    P_N.append(mpc_target.desired_position.y)
                    P_N.append(mpc_target.desired_position.z)

                print('il target assunto e\':')
                print(P_N[0:3])

                # Initializing the optimal velocity of agents to use it
                # for the hot start initial guess
                v_opt_old = []
                for ii in range(3 * N_cf):
                    v_opt_old.append(np.zeros(N_mpc + 1))  # TODO: hot start puo essere migliorato...

                # Initializing optimal positions to use them as initial
                # guess for the hot start initial guess
                x_opt_old = []
                for ii in range(3 * N_cf):
                    x_opt_old.append(np.linspace(P_0[ii], P_N[ii], N_mpc + 1))

                sub_mpc_flag.data = 2

            elif sub_mpc_flag.data == 2:
                # Once the flag is set to 2, the nlp solver is called at
                # each iteration until a new mpc target is set and the
                # flag is reset to 1

                # ++++++++++++ SWARM LEVEL MPC CONTROLLER +++++++++++

                x_cm = np.array(P_0[0::3]).sum() / N_cf
                y_cm = np.array(P_0[1::3]).sum() / N_cf
                z_cm = np.array(P_0[2::3]).sum() / N_cf

                if ((x_cm - mpc_target.desired_position.x) ** 2 +
                    (y_cm - mpc_target.desired_position.y) ** 2 +
                    (z_cm - mpc_target.desired_position.z) ** 2) ** 0.5 > gap:

                    print('\nstato attuale centro di massa: ')
                    print(x_cm, y_cm, z_cm)

                    mpc_velocity, x_opt, v_opt = nlp_solver_3d(N_cf, P_N,
                                                               P_0, N_mpc, x_opt_old, v_opt_old,
                                                               d_ref, v_ref, w_sep, w_final, w_vel,
                                                               N_neigh, obs, r_drone, r_safe,
                                                               target_assegnato_per_ogni_drone, tgt4drone)

                    x_opt_old, v_opt_old = x_opt, v_opt

                    print('mpc_velocity output del controllore MPC e\':')
                    print(mpc_velocity)
                    swarm_mpc_velocity_pub(
                        mpc_velocity)  # TODO - integrare pubblicazione efficacie di velocità per ogni drone dello sciame!

                else:
                    # TODO: action non funziona / topic non funziona !
                    '''
                    print('\n\ntarget reached!\n\n')
                    # Invia l'obiettivo al server
                    client.send_goal(goal)  
                    # Attendere il risultato
                    client.wait_for_result()
                    # Recupera il risultato
                    result = client.get_result()
                    print(result)
                    '''
                    # provvisorio: land triggerato mediante topic...
                    print('\ntarget raggiunto')
                    land_pub.publish(land_trigger)
                    rospy.sleep(0.1)

        elif not flag_state:
            print('non ho ancora ricevuto lo stato...')

        rate.sleep()  # all'interno di while not rospy.is_shutdown()
