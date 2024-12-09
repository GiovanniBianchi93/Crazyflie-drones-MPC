#! /usr/bin/env python3

import sys

import math

import cflib.crtp
from cflib.crazyflie import Commander
from cflib.crazyflie.log import LogConfig
from cflib.crazyflie.swarm import Swarm, _Factory
from cflib.crazyflie.syncLogger import SyncLogger
from cflib.positioning.motion_commander import MotionCommander

import numpy as np

from matplotlib import pyplot as plt, animation
import mpl_toolkits.mplot3d.axes3d as p3

import rospy
import actionlib

from crazy_common_py.common_functions import standardNameList
from crazyflie_messages.msg import TakeoffAction, TakeoffResult, SwarmStates, CrazyflieState, Position, SphereData
from std_msgs.msg import Empty
from std_msgs.msg import Int32

from geometry_msgs.msg import Point

import time


# TODO - riordinare inserendo in un unico posto tutti i dati (target, offset, pesi, ...)
#  e poi pubblicare opportunamente ed avviare mpc solo in tal caso


# ++++++++++++++++++++++++ CREATE LIST OF URIS +++++++++++++++++++++++++++++++++

# tbn: utilizzare uri tassativamente crescenti e ordinati:
# cf1 -> radio://0/80/2M/E7E7E7E70
# cf2 -> radio://0/80/2M/E7E7E7E71
# etc.

def create_uris_list():
    uris_ = []

    for name in cf_names:
        num_ID = int(name[2:]) - 1
        uri = 'radio://0/80/2M/E7E7E7E7E' + hex(num_ID)[-1]
        uris_.append(uri)

    return uris_


# ==================================================================================================================
#
#        L O G G E R S     C O N F I G U R A T I O N    M E T H O D S
#
# ==================================================================================================================

# Dictionaries are used to store the SyncLogger instances and extract data
# within the 100 Hz callback
# The key used to identify the drones is the URI string

# ++++++++++++++++++++++++ CREATE STATE LOGGERS +++++++++++++++++++++++++++++++++

def state_logger_drone(scf):
    # TODO: provvisorio / non generalizzato nel caso di sciamo con num droni > 2 !
    '''
    if scf._link_uri == uris[0]:
        state_logger = SyncLogger(scf, state_logger_config_1)
        state_logger.connect()
        state_loggers[scf._link_uri] = state_logger

    elif scf._link_uri == uris[1]:
        state_logger = SyncLogger(scf, state_logger_config_2)
        state_logger.connect()
        state_loggers[scf._link_uri] = state_logger
    '''

    # TODO: provvisorio, da verificare sperimentalmente - per generalizzare a sciame con N_cf > 2
    #  controllare...
    for j, uri in enumerate(uris):
        if uri == scf._link_uri:
            break  # j trovato
    nome_variabile_ = f"config_cf{j + 1}"
    state_logger = SyncLogger(scf, state_logger_configs[nome_variabile_])
    state_logger.connect()
    state_loggers[scf._link_uri] = state_logger


def state_loggers_swarm():
    swarm.parallel_safe(state_logger_drone)


# ==================================================================================================================
#
#         I N I T I A L  O P E R A T I O N S  M E T H O D S
#
# ==================================================================================================================

# ++++++++ MOTION COMMANDER AND COMMANDER INSTANTIATION METHOD +++++++++++++++++

# Creating a list of motion commanders to be used for the single
# drones and for the swarm

def create_commanders_dict_drone(scf):
    # Motion commander instance
    motion_commander = MotionCommander(scf)
    mc_dict[scf._link_uri] = motion_commander

    # Commander instance to send control setpoints
    commander = Commander(scf.cf)
    commander.set_client_xmode(enabled=True)

    c_dict[scf._link_uri] = commander


def create_commanders_dict_swarm():
    swarm.parallel_safe(create_commanders_dict_drone)  # parallel_safe -> tutti i droni all'interno dello swarm
    # cioé: create_commanders_dict_drone è effettuato per ogni drone definito da preciso scf, create_commanders_dict_drone(scf)
    # scf gestito ad alto livello da Swarm


# ++++++++ STATE SUBSCRIBER LIST METHOD +++++++++++++++++++

def make_states_list():
    for cf_name in cf_names:
        states.append(CrazyflieState())
        # -> states diventa array di variabili di tipo CrazyflieState()


# per pubblicare lo stato del singolo drone su canali separati /cf1/state, /cf2/state, ...
'''
# +++++++++ STATE PUBLISHERS LIST METHOD ++++++++++++

# viene creato states_pubs, lista di publisher per pubblicare lo stato del singolo drone
def make_states_publishers():
    states_pubs = []
    for cf_name in cf_names:
        global states_pubs # soluzione non ottima dal punto di vista dello stile
        tmp_pub = rospy.Publisher('/' + cf_name + '/state',
                                  CrazyflieState, queue_size=1)
        states_pubs.append(tmp_pub)


# ++++++++++ STATE PUBLISHERS PUBLISH METHOD +++++++++

def states_pub_(states_):
    index = 0
    for index, state_pub in enumerate(states_pubs):  # state_pub considera un singolo publisher nella lista states_pubs
        state_pub.publish(states_.states[index])  # viene pubblicato lo stato del singolo drone
'''


# ++++++++ MPC VELOCITY SUBSCRIBER LIST METHOD ++++++++++++

def make_mpc_velocity_subs():
    for cf_name in cf_names:
        # Subscribers to read the desired velocity computed by the MPC controller
        tmp_sub = rospy.Subscriber('/' + cf_name +
                                   '/mpc_velocity', Position,
                                   mpc_velocity_sub_callback)
        mpc_velocity_subs.append(tmp_sub)


###########################################################################

#                        O T H E R   F U N C T I O N S

###########################################################################
# initialization function: plot the background of each frame
def init():
    for ii in range(local_N_cf):
        lines[ii].set_data_3d([], [], [])
    return lines


# animation function.  This is called sequentially
def animate(frame_i):
    for ii in range(local_N_cf):
        x_plot[ii] = np.random.random()  # TODO: provvisorio
        y_plot[ii] = np.random.random()
        z_plot[ii] = np.random.random()
        lines[ii].set_data_3d(np.append(lines[ii]._verts3d[0], x_plot[ii]),
                              np.append(lines[ii]._verts3d[1], y_plot[ii]),
                              np.append(lines[ii]._verts3d[2], z_plot[ii]))
    return lines

def sphere_approximation(x_par, y_par, z_par, a, b, c):
    """
    --- sphere_approximation: parallelepiped to spheres ---

    Restituisce un elenco di sfere approssimative per il parallelepipedo
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

    '''
    n_a = math.floor(n_a)
    n_b = math.floor(n_b)
    n_c = math.floor(n_c)
    '''

    n_a = math.ceil(n_a)
    n_b = math.ceil(n_b)
    n_c = math.ceil(n_c)

    xs = [a / n_a * (0.5 + ii) for ii in range(n_a)]
    ys = [b / n_b * (0.5 + ii) for ii in range(n_b)]
    zs = [c / n_c * (0.5 + ii) for ii in range(n_c)]

    # Genera lista (di liste) con tutte le possibilità...
    sph_res = [[x, y, z, max_r] for x in xs for y in ys for z in zs]

    print('Sph_res: ', sph_res)

    # traslo opportunamente: - a / 2 per centro di massa, + x_par per traslazione
    for sphere in sph_res:
        sphere[0] += x_par - a / 2
        sphere[1] += y_par - b / 2
        sphere[2] += z_par - c / 2

    print('Sph_res_translated: ', sph_res)

    return sph_res


# ==================================================================================================================
#
#                                     C A L L B A C K  M E T H O D S  (T O P I C S)
#
# ==================================================================================================================
# ------------------------------------------------------------------------------------------------------------------
#
#                                       P A C E _ S U B _ C A L L B A C K
#
# This callback is called whenever an Empty message is published by pace_state node; it is used to develop the main
# routine.
# ------------------------------------------------------------------------------------------------------------------
def pace_sub_callback(msg, offset_cb):
    t_trigger = time.time()  # tic

    global x_plot, y_plot, z_plot

    if initialOperationsEnded:

        # Initializing states message:
        states_ = SwarmStates()

        cf_index = 0

        for uri in uris:
            # Extracting information from the loggers
            state_data_dict[uri] = state_loggers[uri].next()  # TODO: errore con pace = 1 Hz ??? <---

            # TODO: dove è definita states ?
            # States
            states[cf_index].name = 'cf' + str(cf_index + 1)
            states[cf_index].position.x = \
                state_data_dict[uri][1]['stateEstimate.x']
            states[cf_index].position.y = \
                state_data_dict[uri][1]['stateEstimate.y']
            states[cf_index].position.z = \
                state_data_dict[uri][1]['stateEstimate.z']
            states[cf_index].velocity.x = \
                state_data_dict[uri][1]['stateEstimate.vx']
            states[cf_index].velocity.y = \
                state_data_dict[uri][1]['stateEstimate.vy']
            states[cf_index].velocity.z = \
                state_data_dict[uri][1]['stateEstimate.vz']

            cf_index += 1

        # ------------- A G G I U N G O   O F F S E T -------------

        for ii in range(len(states)):
            states[ii].position.x = states[ii].position.x + offset_cb[ii]['x']
            states[ii].position.y = states[ii].position.y + offset_cb[ii]['y']
            states[ii].position.z = states[ii].position.z + offset_cb[ii]['z']

        states_.states = states

        for ii in range(len(states)):
            x_plot[ii] = states[ii].position.x
            y_plot[ii] = states[ii].position.y
            z_plot[ii] = states[ii].position.z

        print('\n--- A C T U A L   S T A T E ---')
        for state in states_.states:
            print(state.name, ' :   x = ', state.position.x, ' y = ', state.position.y, ' z = ', state.position.z)

        '''
        # ------------- A T T E R R A G G I O    D I    E M E R G E N Z A -------------
        for state in states_.states:
            if state.position.x > 2 or state.position.x < - 1 \
                    or state.position.y > 2 or state.position.y < -2 \
                    or state.position.z > 1:
                print('\n\nEMERGENCY LANDING!')
                msg_ = Empty()
                swarm_land_act_callback(msg_)
                # swarm_land_act_callback è una ROS callback attivata da un messaggio di tipo Empty!
        '''
        # ------------- P U B B L I C O   S T A T O   S W A R M  ----------------------

        states_pub.publish(states_)  # pubblica stato complessivo swarm
        time.sleep(0.01)  # TODO <--- cosa succede se metto time.sleep piu alto in relazione a pace_state?
        # states_pub_(states_) # pubblica stato singolo drone

        t_state_pub = time.time()  # tic
        print('\npubblico state a t = ', t_state_pub)

        elapsed_state = t_state_pub - t_trigger
        print('elapsed time (state): ', elapsed_state)
        print('frequency (state): ', 1 / elapsed_state)


# ------------------------------------------------------------------------------------------------------------------
#
#         M P C _ V E L O C I T Y  _ S U B _ C A L L B A C K
#
# This callback gets the desired velocity computed by the mpc controller
# and sets the velocity target so that the velocity commands are given as
# velocity setpoints to the drones composing the swarm
# ------------------------------------------------------------------------------------------------------------------
def mpc_velocity_sub_callback(msg):
    cf_name = msg.name
    num_ID = int(cf_name[2:]) - 1
    uri = 'radio://0/80/2M/E7E7E7E7E' + hex(num_ID)[-1]  # [-1] è l'ultimo elemento di hex(num_ID)

    desired_vx[uri] = msg.desired_velocity.x
    desired_vy[uri] = msg.desired_velocity.y
    desired_vz[uri] = msg.desired_velocity.z
    # desired_yaw_rate[uri] = msg.desired_yaw_rate  # ? - mpc_velocity output MPC controllore non ha info su yaw_rate !

    velocity_setpoint_swarm()


# ==================================================================================================================
#
#            C A L L B A C K  M E T H O D S  (A C T I O N S)
#
# ==================================================================================================================
# ------------------------------------------------------------------------------------------------------------------
#
#         S W A R M _ T A K E O F F _ A C T _ C A L L B A C K
#
# This method is used as the swarm takeoff action.
# ------------------------------------------------------------------------------------------------------------------

def swarm_takeoff_act_callback():
    # Output:
    # result = TakeoffResult()

    print('\nAbout to take off...')

    print("Take off in", end="")
    for ii in range(3, 0, -1):
        print(" %s" % ii, end="")
        time.sleep(1)
    print(" - taking off!", end="")

    takeoff_swarm()

    print('Take off: done\n')

    rospy.sleep(1)

    # swarm_takeoff_act.set_succeeded(result)


# +++++++++++++++++++++++ TAKEOFF METHOD ++++++++++++++++++++++++++++++++++++++

# gestisce il decollo del singolo drone
def takeoff_drone4swarm(scf):
    mc_dict[scf._link_uri].take_off(height=0.3)  # <- modifica altezza decollo /a discrezione dell'utente


# gestisce il decollo a livello di swarm
def takeoff_swarm():
    # print('takeoff action')
    swarm.parallel_safe(takeoff_drone4swarm)


# ------------------------------------------------------------------------------------------------------------------
#
#         S W A R M _ L A N D _ A C T _ C A L L B A C K
#
# This method is used as the swarm land action.
# ------------------------------------------------------------------------------------------------------------------
def swarm_land_act_callback(msg):
    # Output:
    # result = TakeoffResult()

    print('About to land...')

    land_swarm()

    print('Landing: done\n')

    rospy.sleep(1)

    # TODO: ogni volta che atterro: termino anche il programma (?)
    rospy.is_shutdown()
    sys.exit(1)  # TODO: non funziona

    # result.result = True  #tbn: mancava nel codice di Fuso !
    # swarm_land_act.set_succeeded(result)


# +++++++++++++++++++++++++ LAND METHOD +++++++++++++++++++++++++++++++++++++++

# a livello di drone
def land_drone4swarm(scf):
    mc_dict[scf._link_uri].land()


# a livello di swarm
def land_swarm():
    # print('land action')
    swarm.parallel_safe(land_drone4swarm)


# +++++++++++++++++++++ VELOCITY SETPOINT METHOD ++++++++++++++++++++++++++++++

# ? esegue mpc_velocity a drone corrispondente ?
def velocity_setpoint_drone4swarm(scf):
    '''
    c_dict[scf._link_uri].send_velocity_world_setpoint(
        desired_vx[scf._link_uri],
        desired_vy[scf._link_uri],
        desired_vz[scf._link_uri],
        desired_yaw_rate[scf._link_uri])
    '''

    # chiamo send_velocity_world_setpoint senza definire desired_yaw_rate, dal momento che MPC controller
    # dà in output solo Vx, Vy, Vz ...

    c_dict[scf._link_uri].send_velocity_world_setpoint(
        desired_vx[scf._link_uri],
        desired_vy[scf._link_uri],
        desired_vz[scf._link_uri],
        0)  # desired_yaw_rate[scf._link_uri])
    # desired_yaw_rate imposto = 0 (non è output MPC)

    # debug  # TODO: essendo all'interno di parallel_safe: print esce mischiata - da migliorare
    print(' vx = ', desired_vx[scf._link_uri],
          ' vy = ', desired_vy[scf._link_uri],
          ' vz = ', desired_vz[scf._link_uri],
          ' - uri: ', scf._link_uri)

    # TODO: così facendo si considerano le velocità in output da MPC come inerenti a SR assoluto (?!) (Giovanni) <---
    # ? corretto? o meglio linear_motion by MotionCommander, che considera SR relativo drone?
    # ? come fa il drone ad essere a conoscenza di un SR assoluto? /esso è affidabile?


def velocity_setpoint_swarm():
    print('\n--- E X E C U T E D   V E L O C I T Y ---')
    swarm.parallel_safe(velocity_setpoint_drone4swarm)


##################################################################

#                            M A I N

##################################################################

if __name__ == '__main__':

    rospy.init_node('node_swarm', log_level=rospy.DEBUG)

    initialOperationsEnded = False

    # ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
    #                 P U B L I S H E R S  S E T U P
    # ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

    N_cf_pub = rospy.Publisher('N_cf_topic', Int32, queue_size=1)
    scelta_target_pub = rospy.Publisher('scelta_target_topic', Int32, queue_size=1)
    mpc_target_pub = rospy.Publisher('/swarm/mpc_target', Position, queue_size=1)
    pub_tgt4drone = rospy.Publisher('/tgt4drone_topic', Point, queue_size=1)
    pub_obs = rospy.Publisher('/obs_topic', SphereData, queue_size=1)
    pub_flag_msg_pub = rospy.Publisher('/flag_topic', Empty, queue_size=1)

    # ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
    #                                    I N P U T
    # ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

    print("\n                   [ I N P U T   B Y   U S E R ]\n")

    print('------------- N .   D R O N E S   I N   T H E   S W A R M -------------')

    local_N_cf = int(input("Enter the number of drones in the swarm: "))

    # Generate a standard list of names
    cf_names = standardNameList(local_N_cf)

    uris = create_uris_list()

    print("Number of drones in the swarm: ", local_N_cf)
    print("Names of the drones: ", cf_names)
    print("URIs of the drones: ", uris)

    N_cf_pub.publish(local_N_cf)
    time.sleep(0.1)
    print("Number of drones in the swarm: published\n")

    # -------------------------- O F F S E T --------------------------

    print('------------- O F F S E T -------------')

    # inizializzo offset

    offset = []

    for i in range(local_N_cf):
        offset.append({'x': 0.0, 'y': 0.0, 'z': 0.0})  # inizializzo lista di dizionari

    print(
        'Enter offset (x y z) [m] for each drone - offset compared to drone %s ( uri: %s )' % (cf_names[0], uris[0]))
    print('x > 0 / < 0 : forward / backward ')
    print('y > 0 / < 0 : left / right ')
    print('z > 0 / < 0 : above / under ')
    for i in range(1, local_N_cf):
        offset[i]['x'] = float(input("Offset %s ( uri: %s) -  x:  " % (cf_names[i], uris[i])))
        offset[i]['y'] = float(input("Offset %s ( uri: %s) -  y:  " % (cf_names[i], uris[i])))
        offset[i]['z'] = float(input("Offset %s ( uri: %s) -  z:  " % (cf_names[i], uris[i])))

    print("Offset acquired: ", offset)

    # -------------------------- T A R G E T --------------------------

    print('\n------------- T A R G E T -------------')

    print('Enter target modality')
    print('1: center of mass')
    print('2: each drone')

    while True:
        try:
            local_scelta_target = int(input("Enter 1 or 2: "))
            if local_scelta_target != 1 and local_scelta_target != 2:
                raise ValueError
            break
        except ValueError:
            print("Error: enter 1 or 2")

    print('Target modality: ', end='')
    if local_scelta_target == 1:
        print('center of mass')
    if local_scelta_target == 2:
        print('each drone')

    scelta_target_pub.publish(local_scelta_target)
    time.sleep(0.1)
    print('Target modality: published')

    if local_scelta_target == 1:
        print('\n-------- T A R G E T :   C E N T E R   O F   M A S S --------\n')

        mpc_target_list = []

        print('Enter target center of mass coordinates (x y z): ')
        mpc_target_list.append(float(input("x:  ")))
        mpc_target_list.append(float(input("y:  ")))
        mpc_target_list.append(float(input("z:  ")))
        print('\nTarget center of mass acquired: ', mpc_target_list)
        # eventualmente: ottimizzare mpc_target con unica variabile

        # passo a variabile ROS...
        local_mpc_target = Position()
        local_mpc_target.desired_position.x = mpc_target_list[0]
        local_mpc_target.desired_position.y = mpc_target_list[1]
        local_mpc_target.desired_position.z = mpc_target_list[2]

        mpc_target_pub.publish(local_mpc_target)
        time.sleep(0.1)
        print('Target center of mass: published')

    if local_scelta_target == 2:

        print('\n-------- T A R G E T :   E A C H   D R O N E --------\n')

        tgt4drone = []  # contiene i singoli target per ogni drone

        # inizializzo tgt4drone
        for i in range(local_N_cf):
            tgt4drone.append({'x': 0.0, 'y': 0.0, 'z': 0.0})  # inizializzo lista di dizionari

        print('Enter target for each drone coordinates (x y z):')
        for i in range(local_N_cf):
            tgt4drone[i]['x'] = float(input("%s - x:  " % (i + 1)))
            tgt4drone[i]['y'] = float(input("%s - y:  " % (i + 1)))
            tgt4drone[i]['z'] = float(input("%s - z:  " % (i + 1)))

        print('Target for each drone acquired: ', tgt4drone)

        local_tgt4drone_array = []  # lista di variabili Point()

        for i in range(local_N_cf):
            tgt4drone_single = Point()
            tgt4drone_single.x = tgt4drone[i]['x']
            tgt4drone_single.y = tgt4drone[i]['y']
            tgt4drone_single.z = tgt4drone[i]['z']
            local_tgt4drone_array.append(tgt4drone_single)

        # pubblicazione effettiva via ROS...
        '''
        alternativamente: creare variabile ad hoc su ROS
        vedi https://stackoverflow.com/questions/29290188/how-to-append-to-personalised-ros-array-message-in-python
        vedi tutorial TheConstruct
        '''
        for i in range(len(local_tgt4drone_array)):
            pub_tgt4drone.publish(local_tgt4drone_array[i])  # pubblico singola variabile Point()
            time.sleep(0.1)  # TODO: importante ! - per dare il tempo di pubblicare il messaggio...
        print('Target for each drone: published')
        # ---------------------------------------------------------------------------------

    print('\n------------- O B S T A C L E S   ( P A R A L L E L E P I P E D ) -------------\n')
    while True:
        try:
            num_obs = int(input("Enter number of obstacles: "))
            break
        except ValueError:
            print("Error: enter number of obstacles")

    # +++++++++++++++++++ MPC (PARALLELEPIPED) OBSTACLES ++++++++++++++++++++++

    obs = []
    obs_array = []  # lista di variabili Point()

    # TODO: provvisorio
    # se non ci sono ostacoli
    if num_obs == 0:
        obs_single = SphereData()
        obs_single.x = -999
        obs_single.y = -999
        obs_single.z = -999
        obs_single.r = 0.1
        obs_array.append(obs_single)

    for i in range(num_obs):
        # inizializzo parallelepiped
        info_par = {'x': 0.0, 'y': 0.0, 'z': 0.0, 'a': 0.0, 'b': 0.0, 'c': 0.0, }  # inizializzo

        print('Parallelepiped %s - Enter center of mass (x y z) and dimensions (a b c): ' % (i + 1))
        info_par['x'] = float(input("x:  "))
        info_par['y'] = float(input("y:  "))
        info_par['z'] = float(input("z:  "))
        info_par['a'] = float(input("a:  "))
        info_par['b'] = float(input("b:  "))
        info_par['c'] = float(input("c:  "))
        print('Parallelepiped %s acquired: %s' % (i + 1, info_par))

        obs += sphere_approximation(info_par['x'], info_par['y'], info_par['z'],
                                    info_par['a'], info_par['b'], info_par['c'])

    for i in range(len(obs)):
        obs_single = SphereData()
        obs_single.x = obs[i][0]
        obs_single.y = obs[i][1]
        obs_single.z = obs[i][2]
        obs_single.r = obs[i][3]
        obs_array.append(obs_single)

    # pubblicazione effettiva via ROS...
    for i in range(len(obs_array)):
        pub_obs.publish(obs_array[i])  # pubblico singola variabile SphereData()
        time.sleep(0.1)  # TODO: importante ! - per dare il tempo di pubblicare il messaggio...

    print('Obstacles: published')

    print('\n---------------------------------------------------------\n')

    # pubblicazione flag
    flag_msg_pub = Empty()
    pub_flag_msg_pub.publish(flag_msg_pub)
    time.sleep(0.1)
    print('All messages published!')

    # ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
    #                                    I N P U T   -   E N D
    # ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

    # ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
    #                                    G E N E R A L   S E T U P
    # ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

    print("\n                   [ G E N E R A L   S E T U P ]\n")

    # Dictionaries for commanders initialization
    mc_dict = dict()
    c_dict = dict()

    # Logger period
    logger_period = 200  # ms

    '''
    # provvisorio / non generalizzato nel caso di sciame con con N_cf > 2

    # State logger configuration (6 floats => 24/26 bytes)
    state_logger_config_1 = LogConfig(name='state_conf',
                                      period_in_ms=logger_period)
    state_logger_config_1.add_variable('stateEstimate.x',
                                       'float')
    state_logger_config_1.add_variable('stateEstimate.y',
                                       'float')
    state_logger_config_1.add_variable('stateEstimate.z',
                                       'float')
    state_logger_config_1.add_variable('stateEstimate.vx',
                                       'float')
    state_logger_config_1.add_variable('stateEstimate.vy',
                                       'float')
    state_logger_config_1.add_variable('stateEstimate.vz',
                                       'float')

    # bug - vedi discussione Bitcraze
    state_logger_config_2 = LogConfig(name='state_conf',
                                      period_in_ms=logger_period)
    state_logger_config_2.add_variable('stateEstimate.x',
                                       'float')
    state_logger_config_2.add_variable('stateEstimate.y',
                                       'float')
    state_logger_config_2.add_variable('stateEstimate.z',
                                       'float')
    state_logger_config_2.add_variable('stateEstimate.vx',
                                       'float')
    state_logger_config_2.add_variable('stateEstimate.vy',
                                       'float')
    state_logger_config_2.add_variable('stateEstimate.vz',
                                       'float')
    '''

    """
    # TODO: provvisorio, da verificare sperimentalmente - per generalizzare a sciame con N_cf > 2
    #  /mediante dizionario
    #  tbn: è garantito che LogConfig, chiamato N_cf volte,
    #  non si inizializzi sulla stessa area di memoria di un precedente LogConfig ? (!?) /SEMBRA OK!
    """
    state_logger_configs = {}

    for i in range(local_N_cf):
        nome_variabile = f"config_cf{i + 1}"
        state_logger_config_i = LogConfig(name='state_conf',
                                          period_in_ms=logger_period)
        state_logger_config_i.add_variable('stateEstimate.x',
                                           'float')
        state_logger_config_i.add_variable('stateEstimate.y',
                                           'float')
        state_logger_config_i.add_variable('stateEstimate.z',
                                           'float')
        state_logger_config_i.add_variable('stateEstimate.vx',
                                           'float')
        state_logger_config_i.add_variable('stateEstimate.vy',
                                           'float')
        state_logger_config_i.add_variable('stateEstimate.vz',
                                           'float')
        state_logger_configs[nome_variabile] = state_logger_config_i

    # Dictionaries for logger data
    state_data_dict = dict()

    mpc_velocity_subs = []
    desired_vx = dict()
    desired_vy = dict()
    desired_vz = dict()
    desired_yaw_rate = dict()
    make_mpc_velocity_subs()

    # ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
    #                 P L O T   3 D   T R A J E C T O R Y
    # ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
    x_plot = [0.0 for i in range(local_N_cf)]
    y_plot = [0.0 for i in range(local_N_cf)]
    z_plot = [0.0 for i in range(local_N_cf)]

    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')

    # Set plot properties
    ax.set_xlabel('X')
    ax.set_ylabel('Y')
    ax.set_zlabel('Z')
    ax.set_title('3D Trajectory Animation')

    lines = []
    for i in range(local_N_cf):
        line, = ax.plot3D([], [], [], lw=1, label=f'cf {i + 1}')  # creo istanza Line3D
        lines.append(line)

    # call the animator.  blit=True means only re-draw the parts that have changed.
    anim = animation.FuncAnimation(fig, animate, init_func=init,
                                   frames=None, interval=50, blit=True)

    # save the animation as an mp4.  This requires ffmpeg or mencoder to be
    # installed.  The extra_args ensure that the x264 codec is used, so that
    # the video can be embedded in html5.  You may need to adjust this for
    # your system: for more information, see
    # http://matplotlib.sourceforge.net/api/animation_api.html
    # anim.save('basic_animation.mp4', fps=30, extra_args=['-vcodec', 'libx264'])

    ax.legend()
    plt.show()
    # TODO: il programma si ferma - aggiungere thread (?)

    # ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
    #                 P U B L I S H E R S  S E T U P
    # ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
    # States publisher:
    states_pub = rospy.Publisher('/swarm/states', SwarmStates, queue_size=1)

    # List of states
    # make_states_publishers()

    # ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
    #                 S U B S C R I B E R S  S E T U P
    # ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
    # Subscriber to pace:
    pace_sub = rospy.Subscriber('/pace_state', Empty, callback=pace_sub_callback, callback_args=offset, queue_size=1)

    # Subscriber to land:
    land_sub = rospy.Subscriber('/land_topic', Empty, swarm_land_act_callback, queue_size=1)

    '''
    # ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
    #                    A C T I O N S  S E T U P
    # ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
    # tbn: si può anche non utilizzare le action in questo caso, e chiamare direttamente le callback
    # oppure: mantenere la struttura delle action lato server in un file a parte, e qui in MAIN avere il lato client...
    
    # Action to make the entire swarm take off :
    swarm_takeoff_act = actionlib.SimpleActionServer('/swarm/takeoff_actn', TakeoffAction,swarm_takeoff_act_callback, False)
    swarm_takeoff_act.start()

    # Action to make the entire swarm land:
    swarm_land_act = actionlib.SimpleActionServer('/swarm/land_actn', TakeoffAction, swarm_land_act_callback, False)
    swarm_land_act.start()
    '''

    print('General setup: done')

    # ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
    #             I N I T I A L  O P E R A T I O N S
    # ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
    print("\n               [ I N I T I A L   O P E R A T I O N S ]\n")

    # Drivers initialization:
    cflib.crtp.init_drivers()

    # Instantiation of Swarm:
    swarm = Swarm(uris, factory=_Factory())  # da cui ottengo scf per ogni drone dello sciame; scf è l'oggetto Crazyflie

    print('Swarm: initialized')

    #  Opening communication
    swarm.open_links()

    print('Connection: established')

    # Resetting estimators
    swarm.reset_estimators()

    print('Position estimators: stable position found')

    # Loggers configuration for all the agents
    state_loggers = dict()

    # Configuring the loggers
    state_loggers_swarm()

    print('Loggers configuration: done')

    # List of states as CrazyflieState messages:
    states = []
    make_states_list()

    # Create commanders for the swarm
    create_commanders_dict_swarm()

    print('Commanders configuration: done')

    # provvisorio /no azioni
    swarm_takeoff_act_callback()  # TODO: provvisorio

    initialOperationsEnded = True

    print('Initial operations ended: %s\n' % initialOperationsEnded)

    rospy.spin()
