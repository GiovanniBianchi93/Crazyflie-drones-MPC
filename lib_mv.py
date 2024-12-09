#! /usr/bin/env python3

import csv
import math
import sys
import time

import rospy
from crazy_common_py.common_functions import standardNameList
from crazyflie_messages.msg import Position, SphereData
from geometry_msgs.msg import Point
from std_msgs.msg import Int32, Empty

import numpy as np


def create_uris_list(cf_names_):
    """
    --- CREATE LIST OF URIS ---

    tbn: utilizzare uri tassativamente crescenti e ordinati!

    esempio:
    cf1 -> radio://0/80/2M/E7E7E7E70
    cf2 -> radio://0/80/2M/E7E7E7E71
    etc.
    """

    uris_ = []

    for name in cf_names_:
        num_ID = int(name[2:]) - 1
        uri = 'radio://0/80/2M/E7E7E7E7E' + hex(num_ID)[-1]
        uris_.append(uri)

    return uris_


def sphere_approximation(x_par, y_par, z_par, a, b, c):
    """
    --- PARALLELEPIPED TO SPHERES APPROXIMATION ---

    Restituisce un elenco di sfere che approssimano il parallelepipedo
    con centro di massa definito in x_par, y_par, z_par e di dimensione a, b, c.
    Restituisce un elenco di (x, y, z, r), dove x, y, z sono le coordinate del centro della sfera
    e r è il suo raggio.

    tbn: in occasione dell'approssimazione, scegliere math.floor o math.ceil in base alle proprie esigenze:

    math.floor:
    le sfere rimangono all'interno del volume del parallelepipedo
    le sfere non si sovrappongono mai, si creano degli spazi vuoti tra le sfere
    minor numero di sfere

    math.ceil:
    le sfere possono oltrepassare il volume del parallelepipedo
    le sfere si sovrappongono; non si creano degli spazi vuoti tra le sfere
    maggior numero di sfere

    per evitare un eccessivo sforzo computazionale e per come è definito g in MPC:
    si consiglia l'utilizzo di math.floor
    """

    # Calcola il raggio ottimo per approssimare le sfere
    max_r = min(a, b, c) / 2

    max_d = max_r * 2

    n_a = a / max_d
    n_b = b / max_d
    n_c = c / max_d

    n_a = math.floor(n_a)
    n_b = math.floor(n_b)
    n_c = math.floor(n_c)

    '''
    n_a = math.ceil(n_a)
    n_b = math.ceil(n_b)
    n_c = math.ceil(n_c)
    '''

    xs = [a / n_a * (0.5 + ii) for ii in range(n_a)]
    ys = [b / n_b * (0.5 + ii) for ii in range(n_b)]
    zs = [c / n_c * (0.5 + ii) for ii in range(n_c)]

    # Genera lista (di liste) con tutte le possibilità...
    sph_res = [[x, y, z, max_r] for x in xs for y in ys for z in zs]

    # print('Sph_res: ', sph_res)

    # traslo opportunamente: - a / 2 per centro di massa, + x_par per traslazione
    for sphere in sph_res:
        sphere[0] += x_par - a / 2
        sphere[1] += y_par - b / 2
        sphere[2] += z_par - c / 2

    # print('Sph_res_translated: ', sph_res)
    print('Result: ', sph_res)

    return sph_res


def get_input_terminal():
    # vedi lib_mv_backup.py
    pass


def get_input_file():
    # vedi lib_mv_backup.py
    pass


def get_input_script():
    # ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
    #                                    G E T T I N G   I N P U T
    # ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

    print("\n                   [ I N P U T   B Y   U S E R ]\n")

    print('------------- N .   D R O N E S   I N   T H E   S W A R M -------------')

    local_N_cf = 4  # <---

    cf_names = standardNameList(local_N_cf)

    uris = create_uris_list(cf_names)

    print("Number of drones in the swarm: ", local_N_cf)
    print("Names of the drones: ", cf_names)
    print("URIs of the drones: ", uris)

    print('\n------------- O F F S E T -------------')

    offset = []

    for i in range(local_N_cf):
        offset.append({'x': 0.0, 'y': 0.0, 'z': 0.0})  # inizializzo lista di dizionari

    # cf1
    offset[0]['x'] = 0.0  # <---
    offset[0]['y'] = 0.0

    # cf2
    offset[1]['x'] = 0.0
    offset[1]['y'] = -1.0

    # cf3
    offset[2]['x'] = 0.0
    offset[2]['y'] = -2.0

    # cf4
    offset[3]['x'] = 0.0
    offset[3]['y'] = -3.0

    # ... # in funzione di local_N_cf

    print("Offset acquired: ", offset)

    print('\n------------- T A R G E T -------------')

    local_scelta_target = 1
    # local_scelta_target = 2

    print('Target modality: ', end='')
    if local_scelta_target == 1:
        print('center of mass')
    if local_scelta_target == 2:
        print('each drone')

    local_mpc_target = Position()
    local_tgt4drone_array = []  # lista di variabili Point()

    if local_scelta_target == 1:
        print('\n-------- T A R G E T :   C E N T E R   O F   M A S S --------')

        mpc_target_list = []

        tgt_cm = {'x': 2.5, 'y': -1.5, 'z': 0.5}  # <---

        mpc_target_list.append(tgt_cm['x'])
        mpc_target_list.append(tgt_cm['y'])
        mpc_target_list.append(tgt_cm['z'])
        print('Target center of mass acquired: ', mpc_target_list)

        # passo a variabile ROS...
        local_mpc_target.desired_position.x = mpc_target_list[0]
        local_mpc_target.desired_position.y = mpc_target_list[1]
        local_mpc_target.desired_position.z = mpc_target_list[2]

    if local_scelta_target == 2:

        print('\n-------- T A R G E T :   E A C H   D R O N E --------')

        tgt4drone = []  # contiene i singoli target per ogni drone

        # inizializzo tgt4drone
        for i in range(local_N_cf):
            tgt4drone.append({'x': 0.0, 'y': 0.0, 'z': 0.0})  # inizializzo lista di dizionari

        # tgt4drone1
        tgt4drone[0]['x'] = 2.4
        tgt4drone[0]['y'] = -1.2
        tgt4drone[0]['z'] = 0.5

        # tgt4drone2
        tgt4drone[1]['x'] = 1.2
        tgt4drone[1]['y'] = 0.0
        tgt4drone[1]['z'] = 0.5

        # tgt4drone3
        tgt4drone[2]['x'] = 0.0
        tgt4drone[2]['y'] = -1.2
        tgt4drone[2]['z'] = 0.5
        
        # tgt4drone4
        tgt4drone[3]['x'] = 1.2
        tgt4drone[3]['y'] = - 2.4
        tgt4drone[3]['z'] = 0.5

        # ... # in funzione di local_N_cf

        print('Target for each drone acquired: ', tgt4drone)

        for i in range(local_N_cf):
            tgt4drone_single = Point()
            tgt4drone_single.x = tgt4drone[i]['x']
            tgt4drone_single.y = tgt4drone[i]['y']
            tgt4drone_single.z = tgt4drone[i]['z']
            local_tgt4drone_array.append(tgt4drone_single)

    print('\n------------- O B S T A C L E S   ( P A R A L L E L E P I P E D ) -------------')

    num_obs = 2  # <---

    info_par = []
    obs = []
    obs_array = []  # lista di variabili Point()

    for i in range(num_obs):
        info_par.append({'x': 0.0, 'y': 0.0, 'z': 0.0, 'a': 0.0, 'b': 0.0, 'c': 0.0})  # inizializzo

    # obs1
    info_par[0]['x'] = 1.5
    info_par[0]['y'] = -0.5
    info_par[0]['z'] = 0.5
    info_par[0]['a'] = 0.5
    info_par[0]['b'] = 0.5
    info_par[0]['c'] = 1.0

    # obs2
    info_par[1]['x'] = 1.5
    info_par[1]['y'] = -2.5
    info_par[1]['z'] = 0.5
    info_par[1]['a'] = 0.5
    info_par[1]['b'] = 0.5
    info_par[1]['c'] = 1.0

    '''
    # obs3
    info_par[2]['x'] = 1.6
    info_par[2]['y'] = - 0.6
    info_par[2]['z'] = 0.5
    info_par[2]['a'] = 0.5
    info_par[2]['b'] = 0.5
    info_par[2]['c'] = 0.5
    '''

    # ... # in funzione di num_obs

    print('Parallelepiped acquired: ', info_par)

    for i in range(num_obs):
        obs += sphere_approximation(info_par[i]['x'], info_par[i]['y'], info_par[i]['z'],
                                    info_par[i]['a'], info_par[i]['b'], info_par[i]['c'])

    for i in range(len(obs)):
        obs_single = SphereData()
        obs_single.x = obs[i][0]
        obs_single.y = obs[i][1]
        obs_single.z = obs[i][2]
        obs_single.r = obs[i][3]
        obs_array.append(obs_single)

    return local_N_cf, cf_names, uris, offset, local_scelta_target, local_mpc_target, local_tgt4drone_array, obs_array


def pub_input(local_N_cf, local_scelta_target, local_mpc_target, local_tgt4drone_array,
              obs_array):
    # ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
    #                 P U B L I S H E R S  S E T U P
    # ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

    N_cf_pub = rospy.Publisher('/N_cf_topic', Int32, queue_size=1)
    scelta_target_pub = rospy.Publisher('/scelta_target_topic', Int32, queue_size=1)
    mpc_target_pub = rospy.Publisher('/mpc_target_topic', Position, queue_size=1)
    pub_tgt4drone = rospy.Publisher('/tgt4drone_topic', Point, queue_size=1)
    pub_obs = rospy.Publisher('/obs_topic', SphereData, queue_size=1)
    pub_flag_msg_pub = rospy.Publisher('/flag_topic', Empty, queue_size=1)
    time.sleep(1)

    # ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
    #                                    P U B L I S H I N G   I N P U T
    # ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
    print("\n                   [ P U B L I S H I N G   I N P U T ]\n")

    N_cf_pub.publish(local_N_cf)
    time.sleep(0.1)
    print("Number of drones in the swarm: published")

    scelta_target_pub.publish(local_scelta_target)
    time.sleep(0.1)
    print('Target modality: published')

    mpc_target_pub.publish(local_mpc_target)
    time.sleep(0.1)
    print('Target center of mass: published')

    for i in range(len(local_tgt4drone_array)):
        pub_tgt4drone.publish(local_tgt4drone_array[i])  # pubblico singola variabile Point()
        time.sleep(0.1)
    print('Target for each drone: published')

    for i in range(len(obs_array)):
        pub_obs.publish(obs_array[i])  # pubblico singola variabile SphereData()
        time.sleep(0.1)
    print('Obstacles: published')

    flag_msg_pub = Empty()
    pub_flag_msg_pub.publish(flag_msg_pub)
    time.sleep(0.1)
    print('All messages published!')


def security(states_):  # rispetto all'origine segnata su pavimento!
    lim_inf_x = - 1.2
    lim_sup_x = 3.6
    lim_inf_y = -3.6
    lim_sup_y = 1.2
    lim_inf_z = -0.5
    lim_sup_z = 3.0

    for state in states_.states:

        x = state.position.x
        y = state.position.y
        z = state.position.z

        # security condition
        # TODO: inserire condizione definitiva, segnando su pavimento origine e eventualmente inserendo offset ache su drone cf1
        if not (lim_inf_x <= x <= lim_sup_x and lim_inf_y <= y <= lim_sup_y and lim_inf_z <= z <= lim_sup_z):
            return True

    return False


def is_segment_intersecting_sphere(segment_start, segment_end, sphere_center, sphere_radius):
    # Calcola il vettore direzione del segmento
    segment_dir = segment_end - segment_start

    # Calcola la distanza tra il centro della sfera e l'inizio del segmento
    start_to_center = sphere_center - segment_start

    # Calcola la proiezione della distanza sul vettore direzione del segmento
    projection = np.dot(start_to_center, segment_dir) / np.dot(segment_dir, segment_dir)

    # Calcola il punto più vicino sul segmento al centro della sfera
    closest_point = segment_start + np.clip(projection, 0, 1) * segment_dir

    # Verifica se il punto più vicino è all'interno della sfera
    distance = np.linalg.norm(closest_point - sphere_center)
    if distance <= sphere_radius:
        return True
    else:
        return False


def print_mpc_parameters(N_mpc, d_ref, v_ref, N_neigh, w_sep, w_final_cm, w_vel, w_dir, w_nav,
                         w_t4d, w_sep_near_tgt_t4d, w_sep_near_tgt, w_sep_emergency,
                         w_final_near_tgt_t4d, w_vel_near_tgt, w_nav_near_tgt, w_t4d_near_tgt, w_final_cm_near_tgt,
                         r_drone, r_safe, pace_mpc, numerical_method,
                         gap_end_loop, gap_modify_weights, lim_vel_mpc, T_lim):
    print('\n------------- M P C   P A R A M E T E R S -------------')

    print(f"N_mpc: {N_mpc}")
    print(f"d_ref: {d_ref}")
    print(f"N_neigh: {N_neigh}")
    print(f"w_sep: {w_sep}")
    print(f"w_final_cm: {w_final_cm}")
    print(f"w_vel: {w_vel}")
    print(f"w_dir: {w_dir}")
    print(f"w_nav: {w_nav}")
    print(f"w_t4d: {w_t4d}")
    print(f"w_sep_near_tgt_t4d: {w_sep_near_tgt_t4d}")
    print(f"w_sep_near_tgt: {w_sep_near_tgt}")
    print(f"w_sep_emergency: {w_sep_emergency}")
    print(f"w_final_near_tgt_t4d: {w_final_near_tgt_t4d}")
    print(f"w_vel_near_tgt: {w_vel_near_tgt}")
    print(f"w_nav_near_tgt: {w_nav_near_tgt}")
    print(f"w_t4d_near_tgt: {w_t4d_near_tgt}")
    print(f"w_final_cm_near_tgt: {w_final_cm_near_tgt}")
    print(f"r_drone: {r_drone}")
    print(f"r_safe: {r_safe}")
    print(f"pace_mpc: {pace_mpc}")
    print(f"numerical_method: {numerical_method}")
    print(f"gap_end_loop: {gap_end_loop}")
    print(f"gap_modify_weights: {gap_modify_weights}")
    print(f"lim_vel_mpc: {lim_vel_mpc}")
    print(f"v_ref: {v_ref}")
    print(f"T_lim: {T_lim}")


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
