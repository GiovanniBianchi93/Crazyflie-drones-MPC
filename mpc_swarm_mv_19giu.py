#! /usr/bin/env python3
import math
import time
import sys

import rospy
import actionlib

from math import floor
import numpy as np
from casadi import *
import cvxpy as cp  # cvxpy is a toolbox for convex optimization in python,

from crazyCmd.scripts.Sperimentale.Swarm.MioPersonale.Preferiti.lib_plot_mv import TrajectoryData

# we need to solve a convex QP

from crazyflie_messages.msg import SwarmStates
from crazyflie_messages.msg import SphereData
from geometry_msgs.msg import Point
from std_msgs.msg import Empty, Int32, Float32
from crazyflie_messages.msg import Position
from crazyflie_manager.CrazyManager import *

from crazyCmd.scripts.Sperimentale.Swarm.MioPersonale.Preferiti import lib_mv
from crazy_common_py.common_functions import standardNameList


# TODO: per tutti i commenti, i TODO, gli spunti: vedi edi mpc_swarm_mv_backup24may.py (!!!)

###################################################################

#                 M P C  -  3 D

###################################################################
def nlp_solver_3d(N_cf_, P_N_, P_0_, P_0_dot_, N_, x_opt_, v_opt_,
                  d_ref_, v_ref_, w_sep_,
                  w_final_, w_vel_, w_dir_, w_nav_, w_t4d_, N_neigh_, obstacles, r_drone_, r_safe_,
                  tgt_choice_, tgt4drone, numerical_method_, gap_modify_weights_, w_sep_near_tgt_,
                  w_sep_near_tgt_t4d_, w_sep_emergency_, w_final_near_tgt_t4d_, w_vel_near_tgt_, w_nav_near_tgt_,
                  w_t4d_near_tgt_,
                  w_final_cm_near_tgt_, lim_vel_mpc_, T_lim_):
    # -------------------------------------------------------------------

    #                       I N I T I A L I Z A T I O N

    # -------------------------------------------------------------------

    global counter_iteration

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

    '''
    v_cm = 0.0

    for iii in range(N_cf_):
        vx_4drone = (P_0_dot_[iii * 3])
        vy_4drone = (P_0_dot_[iii * 3 + 1])
        vz_4drone = (P_0_dot_[iii * 3 + 2])
        v_4drone = sqrt(vx_4drone ** 2 + vy_4drone ** 2 + vz_4drone ** 2)
        v_cm += v_4drone

    v_cm /= N_cf_

    print('v_cm: ', v_cm)

    # Calculating the reference direction for the drone swarm
    u_ref = [x_des - x_cm_, y_des - y_cm_, z_des - z_cm_]
    u_ref = np.array(u_ref)
    u_norm = np.linalg.norm(u_ref)
    u_ref /= u_norm
    u_refx = u_ref[0]
    u_refy = u_ref[1]
    u_refz = u_ref[2]

    # print('u_norm: ', u_norm)

    # Calculating the time horizon for the MPC
    # T = u_norm / v_ref_
    T = u_norm / v_cm  # tbn: / v_cm permette di trovare T idoneo per risolvere, eventualmente, infeasability (?!)
    print('T:  ', T)

    # Threshold on time horizon, otherwise the swarm oscillates (mv: !?)
    if T < 0.1:
        T = 0.1  # per DT non troppo piccoli, che non portano a nessun vantaggio...

    if T > T_lim_:
        T = T_lim_  # per infeasability / limite a inizio conteggio...
    # per DT non troppo grandi (vs ostacolo non visto per discretizzazione troppo approssimativa) /NO se g aggiornato, 24mag
    '''

    T = 4

    # Declare model variables
    x = MX.sym('x', N_cf_ * 3)

    v = MX.sym('v', N_cf_ * 3)

    # Model equations
    xdot = v

    # ------------- W E I G H T S / V _ R E F   M O D I F I C A T I O N -------------

    if abs(x_cm_ - x_des) < gap_modify_weights_ and abs(y_cm_ - y_des) < gap_modify_weights_ and abs(
            z_cm_ - z_des) < gap_modify_weights_:

        w_vel_ = w_vel_near_tgt_
        w_nav_ = w_nav_near_tgt_

        v_ref_ = 0.0

        if tgt_4_drone:
            w_sep_ = w_sep_near_tgt_t4d_
            w_final_ = w_final_near_tgt_t4d_
            w_t4d_ = w_t4d_near_tgt_
            # d_ref_ *= 2
        else:
            w_sep_ = w_sep_near_tgt_
            w_final_ = w_final_cm_near_tgt_

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

    # TODO:  migliorare lagrangiana!
    #  - Termini di distribuzione spaziale
    #  - Termini di cambio di direzione
    #  - Termini di regolarità del controllo
    #  - direction cost vedi mpc_swarm_mv_backup24may.py

    """
    Termini di distribuzione spaziale: Se desideri che i droni si distribuiscano 
    in modo uniforme nello spazio, puoi introdurre un termine 
    che penalizzi le deviazioni dalla distribuzione desiderata. 
    Ad esempio, potresti considerare una penalizzazione basata 
    sulla deviazione standard delle posizioni dei droni rispetto a una distribuzione target.
    
    Termini di cambio di direzione: Puoi introdurre un termine che penalizzi 
    i cambi di direzione bruschi o improvvisi dei droni. Ad esempio, 
    potresti penalizzare la variazione accelerazione o la variazione 
    di angolo tra le traiettorie dei droni adiacenti.
    
    Termini di regolarità del controllo: Puoi includere termini che penalizzano 
    le variazioni brusche o irregolari nell'input di controllo dei droni. 
    Questo può favorire l'utilizzo di controlli più regolari e morbidi, 
    migliorando la stabilità del sistema
    
    
    """

    # ------------- S E P A R A T I O N   T E R M -------------

    w_sep_ref = w_sep_  # TODO: aggiunto /sab27maggio

    for iii in range(N_cf_):
        for kk in list_list_neighbours[iii]:
            w_sep_ = w_sep_ref
            '''
            if ((P_0_[3 * iii] - P_0_[3 * kk]) ** 2 +
                (P_0_[3 * iii + 1] - P_0_[3 * kk + 1]) ** 2 +
                (P_0_[3 * iii + 2] - P_0_[3 * kk + 2]) ** 2) \
                    < (r_drone_ * 2 + r_safe_) ** 2:
            '''
            if ((P_0_[3 * iii] - P_0_[3 * kk]) ** 2 +
                (P_0_[3 * iii + 1] - P_0_[3 * kk + 1]) ** 2) \
                    < (r_drone_ * 2 + r_safe_) ** 2:
                w_sep_ = w_sep_emergency_  # a fini di sicurezza! rischio collisione!

            # Separation cost
            d_ref_sep = d_ref_

            '''
            L += w_sep_ * ((x[iii * 3] - x[kk * 3]) ** 2
                           + (x[iii * 3 + 1] - x[kk * 3 + 1]) ** 2
                           + (x[iii * 3 + 2] - x[kk * 3 + 2]) ** 2
                           - d_ref_sep ** 2) ** 2
            '''

            """
            NO Z: vs problema droni che si influenzano con vortice d'aria 
            + kalman filter con stima non da pavimento ma da drone...
            """
            L += w_sep_ * ((x[iii * 3] - x[kk * 3]) ** 2
                           + (x[iii * 3 + 1] - x[kk * 3 + 1]) ** 2
                           - d_ref_sep ** 2) ** 2

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

    # TODO: perche per ogni drone? - correggere / un solo termine, eventualmete con peso maggiore..
    # for iii in range(N_cf_):
    L += w_final_ * ((x_cm_sym - x_des) ** 2 + (y_cm_sym - y_des) ** 2 + (z_cm_sym - z_des) ** 2) ** 2

    '''
    # ------------- V E L O C I T Y   P E N A L T Y   T E R M -------------
    for iii in range(N_cf_):
        # Control input cost (greater weight when close to target)
        L += w_vel_ * (v[iii * 3] ** 2 + v[iii * 3 + 1] ** 2 + v[iii * 3 + 2] ** 2)
    '''

    # ------------- D I R E C T I O N   T E R M -------------
    '''
    # TODO: 28maggio  / verificare!
    # Direction cost
    for iii in range(N_cf_):
        # L += w_dir_ * (v[iii * 3] ** 2 + v[iii * 3 + 1] ** 2 + v[iii * 3 + 2] ** 2 -
        #                (v[iii * 3] * u_refx + v[iii * 3 + 1] * u_refy + v[iii * 3 + 2] * u_refz) ** 2) ** 2

        # if math.copysign(1, v[iii * 3]) == math.copysign(1, u_refx):  # same sign
        # if v[iii * 3] * u_refx > 0:  # same sign
        if casadi.sign(v[iii * 3]) == casadi.sign(u_refx):  # same sign
            u_refx_ = abs(u_refx)
        else:
            u_refx_ = -abs(u_refx)

        # if math.copysign(1, v[iii * 3 + 1]) == math.copysign(1, u_refy):
        if v[iii * 3 + 1] * u_refy > 0:  # same sign
            u_refy_ = abs(u_refy)
        else:
            u_refy_ = -abs(u_refy)

        # if math.copysign(1, v[iii * 3 + 2]) == math.copysign(1, u_refz):
        if v[iii * 3 + 2] * u_refz > 0:  # same sign
            u_refz_ = abs(u_refz)
        else:
            u_refz_ = -abs(u_refz)

        L += w_dir_ * (sqrt(v[iii * 3] ** 2 + v[iii * 3 + 1] ** 2 + v[iii * 3 + 2] ** 2) -
                       sqrt((v[iii * 3] * u_refx_) ** 2 + (v[iii * 3 + 1] * u_refy_) ** 2 + (
                               v[iii * 3 + 2] * u_refz_) ** 2))
    '''
    # ------------- V E L O C I T Y   P E N A L T Y   T E R M -------------
    for iii in range(N_cf_):
        # Control input cost (greater weight when close to target)
        L += w_vel_ * (v[iii * 3] ** 2 + v[iii * 3 + 1] ** 2 + v[iii * 3 + 2] ** 2 - v_ref_ ** 2) ** 2

    # ------------- F I N A L   P O S I T I O N   T E R M   ( T A R G E T   4   D R O N E ) -------------
    """
    tbn: L_tgt4drone viene minimizzata anche quando tutti i droni scelgono di raggiungere lo stesso obiettivo;
    ciò è sbagliato; il separation term della lagrangiana dovrebbe evitare questo scenario...
    """

    if tgt_4_drone:
        """
        cf1 in tgt4d1 o tgt4d2 o..., cf2 in tgt4d1 o tgt4d2 o...,...
        """
        '''
        L_tgt4drone = 0

        for iii in range(N_cf_):
            L_tgt4drone_single = 1

            for jj in range(N_cf_):
                L_tgt4drone_single *= (
                        (x[iii * 3] - tgt4drone[jj].x) ** 2 + (x[iii * 3 + 1] - tgt4drone[jj].y) ** 2 + (
                        x[iii * 3 + 2] - tgt4drone[jj].z) ** 2)

            L_tgt4drone_single *= w_t4d_

            L_tgt4drone += L_tgt4drone_single

        L += L_tgt4drone
        '''

        """
        cf1 in tgt4d1, cf2 in tgt4d2,...
        """
        L_tgt4drone = 0

        for iii in range(N_cf_):
            L_tgt4drone += (
                    (x[iii * 3] - tgt4drone[iii].x) ** 2 + (x[iii * 3 + 1] - tgt4drone[iii].y) ** 2 + (
                    x[iii * 3 + 2] - tgt4drone[iii].z) ** 2) ** 2

        L_tgt4drone *= w_t4d_

        L += L_tgt4drone

    # ------------- N U M E R I C A L   M E T H O D   O F   D I S C R E T I Z A T I O N -------------
    # TODO: migliorare dinamica drone...

    # Forward Euler
    if numerical_method_ == 'forward_euler':
        DT = T / N_
        print('DT: ', DT)
        f = Function('f', [x, v], [xdot, L])
        X0 = MX.sym('X0', 3 * N_cf_)
        U = MX.sym('U', 3 * N_cf_)
        X = X0
        Q = 0
        k, k_q = f(X, U)
        X = X + k * DT
        Q = Q + k_q * DT
        F = Function('F', [X0, U], [X, Q], ['x0', 'p'], ['xf', 'qf'])

    # TODO: Inverse Euler non funziona!
    # Inverse Euler
    elif numerical_method_ == 'inverse_euler':
        DT = T / N_
        f = Function('f', [x, v], [xdot, L])
        X0 = MX.sym('X0', 3 * N_cf_)
        U = MX.sym('U', 3 * N_cf_)
        X = X0
        Q = 0
        k, k_q = f(X, U)
        X = X + k * DT + (X - X0 - k * DT) / DT * DT
        Q = Q + k_q * DT
        F = Function('F', [X0, U], [X, Q], ['x0', 'p'], ['xf', 'qf'])

    else:
        print('ERROR! - numerical method not valid!')
        rospy.signal_shutdown('')

    # -------------------------------------------------------------------

    #           S E T T I N G   U P   T H E   M P C

    # -------------------------------------------------------------------

    # Initializing state estimate at time instant i
    X_i = P_0_

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
            lbw_k.append(-lim_vel_mpc_)
            lbw_k.append(-lim_vel_mpc_)
            lbw_k.append(-lim_vel_mpc_)
        lbw += lbw_k

        ubw_k = []
        for iii in range(N_cf_):
            ubw_k.append(+lim_vel_mpc_)
            ubw_k.append(+lim_vel_mpc_)
            ubw_k.append(+lim_vel_mpc_)
        ubw += ubw_k

        w0_k = []
        for iii in range(N_cf_):
            w0_k.append(
                v_opt_[3 * iii][k].__float__())  # vs ros_mpc.py - v_i / ok, in ros_mpc.py si usa v_des / qui, v_opt, ok
            w0_k.append(v_opt_[3 * iii + 1][k].__float__())
            w0_k.append(v_opt_[3 * iii + 2][k].__float__())

        w0 += w0_k

        # Integrate till the end of the interval
        Fk = F(x0=Xk, p=Vk)  # we call the integrator
        Xk_end = Fk['xf']
        J = J + Fk['qf']

        Xk_old = Xk

        # New NLP variable for state at end of interval
        Xk = MX.sym('X_' + str(k + 1), 3 * N_cf_)
        w += [Xk]

        # limite stanza /rispetto a origine segnata su pavimento lab 1.01
        lbw_k = []
        for iii in range(N_cf_):
            lbw_k.append(-1.2)
            lbw_k.append(-3.6)
            lbw_k.append(0.3)  # limite z > 0.3 - vs slittamento per aria

        lbw += lbw_k

        ubw_k = []
        for iii in range(N_cf_):
            ubw_k.append(+3.6)
            ubw_k.append(+1.2)
            ubw_k.append(+3.0)

        ubw += ubw_k

        w0_k = []
        for iii in range(N_cf_):
            w0_k.append(x_opt_[3 * iii][k + 1].__float__())
            w0_k.append(x_opt_[3 * iii + 1][k + 1].__float__())
            w0_k.append(x_opt_[3 * iii + 2][k + 1].__float__())

        w0 += w0_k

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

        # Add inequality constraint for obstacle avoidance

        '''
        for jj in range(N_cf_):
            for obstacle in obstacles:
                distance = sqrt((Xk[3 * jj] - obstacle.x) ** 2 + (Xk[3 * jj + 1] - obstacle.y) ** 2 + (
                        Xk[3 * jj + 2] - obstacle.z) ** 2)
                g += [distance - (obstacle.r + r_drone_ + r_safe_)]
                lbg += [0]  # Vincolo inferiore: la distanza deve essere maggiore della somma dei raggi
                ubg += [+inf]
        '''

        for jj in range(N_cf_):
            for obstacle in obstacles:
                segment_start = [Xk_old[3 * jj], Xk_old[3 * jj + 1], Xk_old[3 * jj + 2]]
                segment_end = [Xk[3 * jj], Xk[3 * jj + 1], Xk[3 * jj + 2]]

                # Calcola il vettore direzione del segmento
                # segment_dir = segment_end - segment_start
                segment_dir = [x_ - y_ for x_, y_ in zip(segment_end, segment_start)]

                # Calcola la distanza tra il centro della sfera e l'inizio del segmento
                sphere_center = [obstacle.x, obstacle.y, obstacle.z]
                # start_to_center = sphere_center - segment_start
                start_to_center = [x_ - y_ for x_, y_ in zip(sphere_center, segment_start)]

                # Converto formato per poter utilizzare funzioni casadi
                # segment_start = casadi.MX(segment_start)
                segment_start = casadi.vertcat(*segment_start)
                segment_end = casadi.vertcat(*segment_end)
                segment_dir = casadi.vertcat(*segment_dir)  # vercat mantiene proprietà MX? /sembra ok
                start_to_center = casadi.vertcat(*start_to_center)

                # Calcola la proiezione della distanza sul vettore direzione del segmento
                # projection = np.dot(start_to_center, segment_dir) / np.dot(segment_dir, segment_dir)
                projection = casadi.dot(start_to_center, segment_dir) / casadi.dot(segment_dir, segment_dir)

                # Calcola il punto più vicino sul segmento al centro della sfera
                # closest_point = segment_start + np.clip(projection, 0, 1) * segment_dir
                closest_point = segment_start + casadi.fmin(casadi.fmax(projection, 0), 1) * segment_dir

                # Ripristino 3 componenti x y z per corretta valutazione closest_point - sphere_center
                closest_point = [closest_point[i] for i in range(3)]

                # Valutazione closest_point - sphere_center
                diff_closest_point_sphere_center = [x_ - y_ for x_, y_ in zip(closest_point, sphere_center)]
                # diff_closest_point_sphere_center = closest_point - sphere_center

                # Converto formato per poter utilizzare funzioni casadi
                diff_closest_point_sphere_center = casadi.vertcat(*diff_closest_point_sphere_center)

                # Verifica se il punto più vicino è all'interno della sfera
                # distance = np.linalg.norm(diff_closest_point_sphere_center)
                distance = casadi.norm_2(diff_closest_point_sphere_center)  # operazione di sottrazione accettata?

                g += [distance - (obstacle.r + r_drone_ + r_safe_)]
                lbg += [0]  # Vincolo inferiore: la distanza deve essere maggiore della somma dei raggi
                ubg += [+inf]

        ################ Center of mass in final target ##########################

        if k == N_ - 1:

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

            # sia per target cm sia per target tgt4d:
            J = J + w_final_ * ((x_cm_end - x_des) ** 2 + (y_cm_end - y_des) ** 2 + (z_cm_end - z_des) ** 2) ** 2
            # tbn: terminal cost + inequality constraint (vs equality constraint, infattibilità solver)
            # (se ancora infattibilità: togliere constraints e lasciare solo terminal cost)

            '''
            g += [x_des - x_cm_end, y_des - y_cm_end, z_des - z_cm_end]
            lbg += [-0.5, -0.5, -0.5]
            ubg += [0.5, 0.5, 0.5]
            '''

            if tgt_4_drone:
                '''
                """
                cf1 in tgt4d1 o tgt4d2 o..., cf2 in tgt4d1 o tgt4d2 o...,...
                
                TODO: aggiungere posizione finale precisa! (FACOLTATIVO)
                (tbn: difficile, dal momento che non conosco a priori
                dove i droni si debbano posizionare ottimamente)
                soluzione: soft constraint già presente (vedi L_tgt4drone), ok
                """
                pass
                '''

                """
                cf1 in tgt4d1, cf2 in tgt4d2,...
                """
                for iii in range(N_cf_):
                    J = J + w_t4d_ * ((Xk_end[iii * 3] - tgt4drone[iii].x) ** 2 + (
                            Xk_end[iii * 3 + 1] - tgt4drone[iii].y) ** 2 + (
                                              Xk_end[iii * 3 + 2] - tgt4drone[iii].z) ** 2)
                    # tbn: terminal cost + inequality constraint (vs equality constraint, infattibilità solver)
                    # (se ancora infattibilità: togliere constraints e lasciare solo terminal cost)

                    '''
                    g += [Xk_end[iii * 3] - tgt4drone[iii].x, Xk_end[iii * 3 + 1] - tgt4drone[iii].y,
                          Xk_end[iii * 3 + 2] - tgt4drone[iii].z]

                    lbg += [-0.5, -0.5, -0.5]
                    ubg += [0.5, 0.5, 0.5]
                    '''

    ################################################################

    # ---------------------------------------------------------------------

    #   S O L V I N G   O P T I M A L   N O N L I N E A R   P R O B L E M

    # ---------------------------------------------------------------------

    # Create an NLP solver
    opts = {}
    opts = {'max_iter_eig': 50, 'ipopt.print_level': 0,
            'print_time': 0, 'ipopt.max_iter': 50}
    prob = {'f': J, 'x': vertcat(*w), 'g': vertcat(*g)}
    solver = nlpsol('solver', 'ipopt', prob, opts)

    # Solving the NLP
    sol = solver(x0=w0, lbx=lbw, ubx=ubw, lbg=lbg, ubg=ubg)
    w_opt = sol['x'].full().flatten()  # -> x_opt e v_opt
    # lambda_opt = sol['g'].full().flatten()  # -> lambda_opt
    # TODO: provare a cambiare solver - ipopt -> snopt, bonmin, worhp, apopt, knitro
    #  oppure cambiare 'linear_solver': 'mumps' in opt con altro linear solver:
    #  MA27, MA57, MA86, ... <---
    # https://coin-or.github.io/Ipopt/
    # https://help.agi.com/stk/LinkedDocuments/IPOPTreference.pdf

    """
    It is important to keep in mind that usually the
    largest fraction of computation time in the optimizer is spent for solving the linear system,
    and that your choice of the linear solver impacts Ipopt’s speed and robustness. It might be
    worthwhile to try different linear solver to experiment with what is best for your application.
    """

    '''
    print('\n--------------- w_opt ---------------')
    print(w_opt)  # debug
    '''

    if counter_iteration == 0:

        x_plot = [None for i in range(N_cf_)]
        y_plot = [None for i in range(N_cf_)]
        z_plot = [None for i in range(N_cf_)]

        # TODO: RIVEDERE!
        for k in range(N_mpc + 1):  # ? N_mpc + 1
            w_opt_k = w_opt[k * 6 * N_cf_:(k + 1) * 6 * N_cf_]
            for iii in range(N_cf_):
                x_plot[iii] = w_opt_k[iii * 3]
                y_plot[iii] = w_opt_k[iii * 3 + 1]
                z_plot[iii] = w_opt_k[iii * 3 + 2]
            w_opt_trj.save_wopt(x_plot, y_plot, z_plot)

    # Extracting the optimal state
    x_opt_ = []
    for iii in range(3 * N_cf_):
        x_opt_.append(w_opt[iii::6 * N_cf_])

    # Extracting the optimal control input
    v_opt_ = []
    for iii in range(3 * N_cf_):
        v_opt_.append(w_opt[(iii + 3 * N_cf_)::6 * N_cf_])

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
    # print('mpc_velocity_: ', mpc_velocity_)

    counter_iteration += 1

    return mpc_velocity_, x_opt_, v_opt_, DT


###########################################################################

#                        O T H E R   F U N C T I O N S

###########################################################################

def target_reached():
    print('\n--------- T A R G E T   R E A C H E D --------')

    time.sleep(0.33)  # per migliorare plot consegna tesi

    land_pub.publish(land_trigger)
    rospy.sleep(0.1)

    # termino programma
    rospy.signal_shutdown('')


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
        mpc_velocity_pub.publish(mpc_velocity_[index])
        time.sleep(0.1)


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


###########################################################################

#               S U B S C R I B E R     C A L L B A C K S

###########################################################################

# TODO: riceve messaggio ogni 5 Hz circa
def get_state_callback(msg):
    # provvisorio
    global swarm_states
    swarm_states = msg

    global flag_state
    flag_state = True


def emergency_callback(msg):
    print('\n--------- E M E R G E N C Y --------')
    # termino programma
    rospy.signal_shutdown('')
    sys.exit(0)


##################################################################

#                            M A I N

##################################################################

if __name__ == '__main__':

    rospy.init_node('MPC_Node', log_level=rospy.DEBUG)

    filename_trjMPC = '/home/matteo/catkin_ws/src/crazyCmd/scripts' \
                      '/Sperimentale/Swarm/MioPersonale/Preferiti' \
                      '/files_trj/trj_wopt.csv'

    w_opt_trj = TrajectoryData(filename_trjMPC)
    w_opt_trj.create_file()

    all_messages_received = False
    flag_state = False
    counter_iteration = 0

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

    ##############################################################

    #               P U B L I S H E R S   S E T U P

    ##############################################################

    land_pub = rospy.Publisher('/land_topic', Empty, queue_size=1)
    land_trigger = Empty()

    pub_DT = rospy.Publisher('/DT_topic', Float32, queue_size=1)
    DT = Float32()

    ##############################################################

    #               S U B S C R I B E R S   S E T U P

    ##############################################################

    state_sub = rospy.Subscriber('/swarm/swarm_state_topic', SwarmStates, get_state_callback, queue_size=1)
    swarm_states = SwarmStates()

    emergency_sub = rospy.Subscriber('/emergency_topic', Empty, emergency_callback, queue_size=1)

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
        mpc_target = lib_mv.calcolo_centro_massa_target(tgt4drone_array)
        print("-> mpc_target from tgt4drones = ", mpc_target.desired_position.x, ' ',
              mpc_target.desired_position.y, ' ',
              mpc_target.desired_position.z)

    for i in range(len(obs_array)):
        print("spherical obstacle ", i + 1, ':     x = ', obs_array[i].x,
              '     y = ', obs_array[i].y,
              '     z = ', obs_array[i].z,
              '     r = ', obs_array[i].r)

    ##############################################################

    # Generate a standard list of names:
    cf_names = standardNameList(N_cf.data)

    ##############################################################

    #    P U B L I S H E R S   S E T U P   -   C O N T I N U E

    ##############################################################

    # List of velocities used to collect the output of the nlp solver
    mpc_velocity = []
    for ii in range(N_cf.data):
        mpc_velocity.append(Position())

    # List of mpc_velocity Publishers
    mpc_velocity_publishers = []
    make_mpc_velocity_publishers()

    ##############################################################

    #                   M P C   P A R A M E T E R S

    ##############################################################

    N_mpc = 4  # number of MPC time steps
    d_ref = 0.75  # reference distance between agents
    '''
    if N_cf.data == 1:
        N_neigh = 0
    elif N_cf.data == 2:
        N_neigh = 1
    elif N_cf.data == 3:
        N_neigh = 2
    else:
        N_neigh = 3
    '''

    N_neigh = 1
    # N_neigh: number of neighbours per drone

    # Weights for objective function - standard
    w_sep = 100  # separation term
    w_final_cm = 1000  # final position penalty term
    w_vel = 100  # velocity control input term
    w_dir = 1  # direction term  # non utilizzato
    w_nav = 50  # non utilizzato
    w_t4d = 10  # target 4 drone term

    # Weights for objective function - exception
    w_sep_near_tgt_t4d = w_sep * 1
    w_sep_near_tgt = w_sep * 1
    w_sep_emergency = w_sep * 10
    w_final_near_tgt_t4d = w_final_cm * 1
    w_vel_near_tgt = 1000
    w_nav_near_tgt = w_nav * 10  # non utilizzato
    w_t4d_near_tgt = w_t4d * 1
    w_final_cm_near_tgt = w_final_cm * 1

    r_drone = 0.1
    r_safe = 0.3

    pace_mpc = 100  # MPC frequency [Hz]

    # metodo discretizzazione MPC: 'forward_euler' / 'inverse_euler'
    numerical_method = 'forward_euler'
    # numerical_method = 'inverse_euler'

    gap_end_loop = 0.15  # condizione per uscire da while MPC
    gap_modify_weights = 1.0  # condizione per modificare weights
    lim_vel_mpc = 0.35  # limite velocità output MPC

    v_ref = 0.0  # reference velocity  # TODO: provvisorio

    T_lim = 20  # tempo massimo per raggiungere il target -> da cui poi si discretizza!
    # tbn: se troppo basso: infeasability per non raggiungimento target
    #  se troppo alto: raggiungimento target troppo lento
    # ! scegliere con cautela !  # LIMITE MODELLO

    lib_mv.print_mpc_parameters(N_mpc, d_ref, v_ref, N_neigh, w_sep, w_final_cm, w_vel, w_dir, w_nav,
                                w_t4d, w_sep_near_tgt_t4d, w_sep_near_tgt, w_sep_emergency,
                                w_final_near_tgt_t4d, w_vel_near_tgt, w_nav_near_tgt, w_t4d_near_tgt,
                                w_final_cm_near_tgt,
                                r_drone, r_safe, pace_mpc, numerical_method,
                                gap_end_loop, gap_modify_weights, lim_vel_mpc, T_lim)

    ##############################################################

    #               W A I T I N G   F O R   S T A T E

    ##############################################################

    print('\n--------- W A I T I N G   F O R   S T A T E --------')
    print('Waiting for state...')

    while not flag_state:
        time.sleep(0.1)

    print('State has been received correctly!')

    #################################################################

    #    I N I T I A L I Z I N G   M P C   ( H O T   S T A R T )

    #################################################################

    print('\n--------- M O D E L   P R E D E C T I V E   C O N T R O L --------')

    P_0 = []
    for ii in range(N_cf.data):
        P_0.append(swarm_states.states[ii].position.x)
        P_0.append(swarm_states.states[ii].position.y)
        P_0.append(swarm_states.states[ii].position.z)

        # tbn: MPC solo su posizione; si potrebbe invece sfruttare anche la info sulla velocità
        # /sbagliato, essendo un modello single integrator, dunque un modello del primo ordine (?!)

    P_N = []

    for ii in range(N_cf.data):
        P_N.append(mpc_target.desired_position.x)
        P_N.append(mpc_target.desired_position.y)
        P_N.append(mpc_target.desired_position.z)

    # Initializing the optimal velocity of agents to use it
    # for the hot start initial guess
    v_opt_old = []
    for ii in range(3 * N_cf.data):
        v_opt_old.append(np.zeros(N_mpc + 1))

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

        time.sleep(0.75)

        t = time.time()

        # swarm_states è l'ultimo swarm_states messo a disposizione da global swarm_states in get_state_callback
        P_0 = []
        for ii in range(N_cf.data):
            P_0.append(swarm_states.states[ii].position.x)
            P_0.append(swarm_states.states[ii].position.y)
            P_0.append(swarm_states.states[ii].position.z)

        P_0_dot = []
        for ii in range(N_cf.data):
            P_0_dot.append(swarm_states.states[ii].velocity.x)
            P_0_dot.append(swarm_states.states[ii].velocity.y)
            P_0_dot.append(swarm_states.states[ii].velocity.z)

        '''
        print('\n--- S T A T E   O B T A I N E D ---')
        for ii in range(N_cf.data):
            print('cf' + str(ii + 1) + ':     x = ', P_0[ii * 3],
                  '     y = ', P_0[ii * 3 + 1],
                  '     z = ', P_0[ii * 3 + 2])
        for ii in range(N_cf.data):
            print('cf' + str(ii + 1) + ':     vx = ', P_0_dot[ii * 3],
                  '     vy = ', P_0_dot[ii * 3 + 1],
                  '     vz = ', P_0_dot[ii * 3 + 2])
        '''

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

        # TODO: vedi condizione_scelta_target_2.py
        """
        tbn: limitazione: target_reached viene chiamato anche nel caso in cui
        tutti i droni siano nei pressi di un unico obiettivo;
        tuttavia lagrangiana con separation cost dovrebbero evitare ciò...
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
        mpc_velocity, x_opt, v_opt, DT = nlp_solver_3d(N_cf.data, P_N,
                                                       P_0, P_0_dot, N_mpc, x_opt_old, v_opt_old,
                                                       d_ref, v_ref, w_sep, w_final_cm, w_vel, w_dir, w_nav, w_t4d,
                                                       N_neigh, obs_array, r_drone, r_safe,
                                                       scelta_target.data, tgt4drone_array,
                                                       numerical_method, gap_modify_weights, w_sep_near_tgt,
                                                       w_sep_near_tgt_t4d, w_sep_emergency,
                                                       w_final_near_tgt_t4d,
                                                       w_vel_near_tgt, w_nav_near_tgt, w_t4d_near_tgt,
                                                       w_final_cm_near_tgt, lim_vel_mpc,
                                                       T_lim)

        x_opt_old, v_opt_old = x_opt, v_opt
        '''
        print('\n--- M P C   O U T P U T   V E L O C I T Y ---')
        for velocity in mpc_velocity:
            print(velocity.name, ':     Vx = ', velocity.desired_velocity.x,
                  '     Vy = ', velocity.desired_velocity.y,
                  '     Vz = ', velocity.desired_velocity.z)
        '''

        # +++++++++++++++++++ PUBLISHING MPC VELOCITY +++++++++++++++++++

        pub_DT.publish(DT)
        swarm_mpc_velocity_pub(mpc_velocity)  # commentare per passare da sperimentale a debug

        t_elapsed = time.time() - t
        print('t_elapsed: ', t_elapsed)

        rate.sleep()

