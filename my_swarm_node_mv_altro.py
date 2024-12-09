#! /usr/bin/env python3

import sys

import cflib.crtp
from cflib.crazyflie import Commander
from cflib.crazyflie.log import LogConfig
from cflib.crazyflie.swarm import Swarm, _Factory
from cflib.crazyflie.syncLogger import SyncLogger
from cflib.positioning.motion_commander import MotionCommander

import actionlib
import rospy

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
# cf1 -> radio://0/80/2M/E7E7E7E70 | DRONE 7
# cf2 -> radio://0/80/2M/E7E7E7E71 | DRONE 4
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
    if scf._link_uri == uris[0]:
        state_logger = SyncLogger(scf, state_logger_config_1)
        state_logger.connect()
        state_loggers[scf._link_uri] = state_logger

    elif scf._link_uri == uris[1]:
        state_logger = SyncLogger(scf, state_logger_config_2)
        state_logger.connect()
        state_loggers[scf._link_uri] = state_logger

    # TODO: provvisorio, da verificare sperimentalmente - per generalizzare a sciame con N_cf > 2
    #  controllare...
    '''
    for j, uri in enumerate(uris):
        if uri == scf._link_uri:
            break  # j trovato
    nome_variabile_ = f"config_cf{j + 1}"
    state_logger = SyncLogger(scf, state_logger_configs[nome_variabile_])
    state_logger.connect()
    state_loggers[scf._link_uri] = state_logger
    '''


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
        # Subscribers to read the desired velocity
        # computed by the MPC controller
        tmp_sub = rospy.Subscriber('/' + cf_name +
                                   '/mpc_velocity', Position,
                                   mpc_velocity_sub_callback)
        mpc_velocity_subs.append(tmp_sub)


# ==================================================================================================================
#
#                                     C A L L B A C K  M E T H O D S  (T O P I C S)
#
# ==================================================================================================================
# ------------------------------------------------------------------------------------------------------------------
#
#                                  __P A C E _ 1 0 0 H Z _ S U B _ C A L L B A C K
#
# This callback is called whenever an Empty message is published by pace_100Hz_node; it is used to develop the main
# routine.
# ------------------------------------------------------------------------------------------------------------------
def pace_sub_callback(msg, offset_cb):
    if initialOperationsEnded:

        # Initializing states message:
        states_ = SwarmStates()

        cf_index = 0

        for uri in uris:
            # Extracting information from the loggers
            state_data_dict[uri] = state_loggers[uri].next()  # TODO: errore con pace = 1 Hz ???

            # debug
            # print(uri, state_data_dict[uri])

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

        for state in states:  # TODO: controllare sperimentalmente!
            states[state].position.x = states[state].position.x + offset_cb[state]['x']
            states[state].position.y = states[state].position.y + offset_cb[state]['y']
            states[state].position.z = states[state].position.z + offset_cb[state]['z']

        '''
        # rispettare offset nella disposizione iniziale dei droni!
        # offset[1]['x'] = 0.0  # offset x del drone [1] (ie cf2)
        offset[1]['y'] = - 0.6

        # states[1].position.x = states[1].position.x + offset[1]['x']
        states[1].position.y = states[1].position.y + offset[1]['y']
        '''

        states_.states = states

        print('\nl\'ultimo stato ricevuto è:\n')
        for state in states_.states:
            print(states_.states[state])  # TODO: controllare... - stampa correttamente?

        # ------------- A T T E R R A G G I O    D I    E M E R G E N Z A -------------

        for state in states_.states:
            if state.position.x > 2.4 or state.position.x < -0.5 \
                    or state.position.y > 2.0 or state.position.y < -2.0:
                print('\n\natterraggio di emergenza...')
                swarm_land_act_callback()
                rospy.is_shutdown()
                sys.exit(1)  # TODO: verificare...

        # ------------- P U B B L I C O   S T A T O   S W A R M  ----------------------

        states_pub.publish(states_)  # pubblica stato complessivo swarm
        # states_pub_(states_) # pubblica stato singolo drone


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

    print('about to take off...')

    takeoff_swarm()

    rospy.sleep(2)

    # swarm_takeoff_act.set_succeeded(result)


# +++++++++++++++++++++++ TAKEOFF METHOD ++++++++++++++++++++++++++++++++++++++

# gestisce il decollo del singolo drone
def takeoff_drone4swarm(scf):
    mc_dict[scf._link_uri].take_off(height=0.5)  # <- modifica altezza decollo /a discrezione dell'utente


# gestisce il decollo a livello di swarm
def takeoff_swarm():
    print('takeoff action')
    swarm.parallel_safe(takeoff_drone4swarm)


# ------------------------------------------------------------------------------------------------------------------
#
#         S W A R M _ L A N D _ A C T _ C A L L B A C K
#
# This method is used as the swarm land action.
# ------------------------------------------------------------------------------------------------------------------
def swarm_land_act_callback():
    # Output:
    # result = TakeoffResult()

    print('about to land...')

    land_swarm()

    rospy.sleep(2)

    # result.result = True  #tbn: mancava nel codice di Fuso !

    # swarm_land_act.set_succeeded(result)


# +++++++++++++++++++++++++ LAND METHOD +++++++++++++++++++++++++++++++++++++++

# a livello di drone
def land_drone4swarm(scf):
    mc_dict[scf._link_uri].land()


# a livello di swarm
def land_swarm():
    print('land action')
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

    # debug
    print('\nho eseguito vx = ', desired_vx[scf._link_uri],
          ' vy = ', desired_vy[scf._link_uri],
          ' vz = ', desired_vz[scf._link_uri],
          ' - drone di uri: ', scf._link_uri)

    # TODO: così facendo si considerano le velocità in output da MPC come inerenti a SR assoluto (?!)
    # ? corretto? o meglio linear_motion by MotionCommander, che considera SR relativo drone?
    # ? come fa il drone ad essere a conoscenza di un SR assoluto? /esso è affidabile?


def velocity_setpoint_swarm():
    # print('... sending velocity setpoints')
    swarm.parallel_safe(velocity_setpoint_drone4swarm)


##################################################################

#                            M A I N

##################################################################

if __name__ == '__main__':

    rospy.init_node('node_swarm', log_level=rospy.DEBUG)

    print('nodo inizializzato\n')

    initialOperationsEnded = False

    print('initial operations ended: %s\n' % initialOperationsEnded)

    # ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
    #                 P U B L I S H E R S  S E T U P
    # ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

    # States publisher:
    N_cf_pub = rospy.Publisher('N_cf_topic', Int32, queue_size=1)
    scelta_target_pub = rospy.Publisher('scelta_target_topic', Int32, queue_size=1)
    mpc_target_pub = rospy.Publisher('/swarm/mpc_target', Position, queue_size=1)
    pub_tgt4drone = rospy.Publisher('/tgt4drone_topic', Point, queue_size=1)
    pub_obs = rospy.Publisher('/obs_topic', SphereData, queue_size=1)

    # ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
    #                                    I N P U T
    # ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

    print('------------- N .   D R O N I   S C I A M E -------------')

    local_N_cf = int(input("inserire numero di droni dello sciame:  "))

    # Generate a standard list of names
    cf_names = standardNameList(local_N_cf)

    uris = create_uris_list()

    print("numero di droni dello sciame: ", local_N_cf)
    print("nomi dei droni: ", cf_names)
    print("URIs dei droni: ", uris)
    print('\n')

    N_cf_pub.publish(local_N_cf)
    time.sleep(0.1)
    print('numero di droni dello sciame: pubblicato')

    # -------------------------- O F F S E T --------------------------

    print('------------- O F F S E T -------------')

    # inizializzo offset

    offset = []

    for i in range(local_N_cf):
        offset.append({'x': 0.0, 'y': 0.0, 'z': 0.0})  # inizializzo lista di dizionari

    print(
        'inserire offset (x y z) per ogni drone - offset rispetto a drone %s ( uri: %s )' % (cf_names[0], uris[0]))
    for i in range(1, local_N_cf):
        offset[i]['x'] = float(input("offset drone %s - coordinata x:  " % cf_names[i]))
        offset[i]['y'] = float(input("offset drone %s - coordinata y:  " % cf_names[i]))
        offset[i]['z'] = float(input("offset drone %s - coordinata z:  " % cf_names[i]))

    print("\noffset acquisito: ", offset)

    # TODO: posizionamento provvisorio...
    # ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
    #                 S U B S C R I B E R S  S E T U P
    # ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
    # Subscriber to pace:
    pace_sub = rospy.Subscriber('/pace_state', Empty, callback=pace_sub_callback, callback_args=offset, queue_size=1)

    # -------------------------- T A R G E T --------------------------

    print('\n------------- T A R G E T -------------\n')

    print('inserire modalità di definizione target')
    print('1: target definito per centro di massa')
    print('2: target definito per ogni drone')

    while True:
        try:
            local_scelta_target = int(input("Inserisci 1 o 2: "))
            if local_scelta_target != 1 and local_scelta_target != 2:
                raise ValueError
            break
        except ValueError:
            print("Errore: inserire 1 o 2")

    print('\nmodalità di definizione target: ', end='')
    if local_scelta_target == 1:
        print('centro di massa')
    if local_scelta_target == 2:
        print('ogni drone')

    scelta_target_pub.publish(local_scelta_target)
    time.sleep(0.1)
    print('modalità di definizione target: pubblicata')

    if local_scelta_target == 1:
        print('\n-------- T A R G E T :   C E N T R O   D I   M A S S A --------\n')

        print('inserire coordinate target centro di massa (x y z): ')
        mpc_target_x = float(input("coordinata x:  "))
        mpc_target_y = float(input("coordinata y:  "))
        mpc_target_z = float(input("coordinata z:  "))
        print('\ntarget centro di massa acquisito: ', mpc_target_x, ' ', mpc_target_y, ' ', mpc_target_z)
        # eventualmente: ottimizzare mpc_target con unica variabile

        # passo a variabile ROS...
        local_mpc_target = Position()
        local_mpc_target.desired_position.x = mpc_target_x
        local_mpc_target.desired_position.y = mpc_target_y
        local_mpc_target.desired_position.z = mpc_target_z

        mpc_target_pub.publish(local_mpc_target)
        time.sleep(0.1)
        print('target centro di massa: pubblicato')

    if local_scelta_target == 2:

        print('\n-------- T A R G E T :   P E R   O G N I   D R O N E --------\n')

        tgt4drone = []  # contiene i singoli target per ogni drone

        # inizializzo tgt4drone
        for i in range(local_N_cf):
            tgt4drone.append({'x': 0.0, 'y': 0.0, 'z': 0.0})  # inizializzo lista di dizionari

        print('inserire coordinate target drone per drone (x y z):')
        for i in range(local_N_cf):
            tgt4drone[i]['x'] = float(input("target drone %s - coordinata x:  " % cf_names[i]))
            tgt4drone[i]['y'] = float(input("target drone %s - coordinata y:  " % cf_names[i]))
            tgt4drone[i]['z'] = float(input("target drone %s - coordinata z:  " % cf_names[i]))

        print('target drone per drone acquisito: ', tgt4drone)

        local_tgt4drone_array = []  # lista di variabili Point()

        for i in range(local_N_cf):
            tgt4drone_single = Point()
            tgt4drone_single.x = tgt4drone[i]['x']
            tgt4drone_single.y = tgt4drone[i]['y']
            tgt4drone_single.z = tgt4drone[i]['z']
            local_tgt4drone_array.append(tgt4drone_single)

        # pubblicazione effettiva via ROS...
        # TODO: alternativamente: creare variabile ad hoc su ROS
        #  vedi https://stackoverflow.com/questions/29290188/how-to-append-to-personalised-ros-array-message-in-python
        #  vedi tutorial TheConstruct
        for i in range(len(local_tgt4drone_array)):
            pub_tgt4drone.publish(local_tgt4drone_array[i])  # pubblico singola variabile Point()
            time.sleep(0.1)  # TODO: importante ! - per dare il tempo di pubblicare il messaggio...
        print('target drone per drone: pubblicato')
        # ---------------------------------------------------------------------------------

    print('\n------------- O S T A C O L I   P R E S E N T I -------------\n')
    while True:
        try:
            num_ostacoli = int(input("Inserisci numero di ostacoli presenti: "))
            break
        except ValueError:
            print("Errore: inserire numero valido")

    # +++++++++++++++++++ MPC (SPHERICAL) OBSTACLES ++++++++++++++++++++++

    # inizializzo obs
    obs = []
    for i in range(num_ostacoli):
        obs.append({'x': 0.0, 'y': 0.0, 'z': 0.0, 'r': 0.0})  # inizializzo obs lista di dizionari

    print('inserire info ostacolo - coordinate (x y z) e raggio (r):')

    for i in range(num_ostacoli):
        obs[i]['x'] = float(input("ostacolo %s - coordinata x:  " % (i + 1)))
        obs[i]['y'] = float(input("ostacolo %s - coordinata y:  " % (i + 1)))
        obs[i]['z'] = float(input("ostacolo %s - coordinata z:  " % (i + 1)))
        obs[i]['r'] = float(input("ostacolo %s - raggio r :  " % (i + 1)))

    print('ostacoli acquisiti: ', obs)

    obs_array = []  # lista di variabili Point()

    # TODO: per soddisfare condizione len(obs_array) != 0 in prova_sub.py /obs_array correttamente ricevuto, anche se di lunghezza = 0,
    #  dunque differente da obs_array = [] inizializzato in prova_sub.py
    if num_ostacoli == 0:
        obs_single = SphereData()
        obs_single.x = -999
        obs_single.y = -999
        obs_single.z = -999
        obs_single.r = -999
        obs_array.append(obs_single)
        pub_obs.publish(obs_array[0])
        time.sleep(0.1)

    else:
        for i in range(num_ostacoli):
            obs_single = SphereData()
            obs_single.x = obs[i]['x']
            obs_single.y = obs[i]['y']
            obs_single.z = obs[i]['z']
            obs_single.r = obs[i]['r']

            obs_array.append(obs_single)

        # pubblicazione effettiva via ROS...
        for i in range(num_ostacoli):
            pub_obs.publish(obs_array[i])  # pubblico singola variabile SphereData()
            time.sleep(0.1)  # TODO: importante ! - per dare il tempo di pubblicare il messaggio...

    print('info ostacoli: pubblicate')

    print('\n------------- I N P U T   A C Q U I S I T I -------------\n')

    # ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
    #                                    F I N E   I N P U T
    # ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

    # ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
    #                               S E T U P   G E N E R A L E
    # ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

    # Dictionaries for commanders initialization
    mc_dict = dict()
    c_dict = dict()

    # Logger period
    logger_period = 200  # ms

    # TODO: provvisorio / non generalizzato nel caso di sciame con con N_cf > 2
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

    # TODO: provvisorio, da verificare sperimentalmente - per generalizzare a sciame con N_cf > 2
    #  /mediante dizionario
    #  tbn: è garantito che LogConfig, chiamato N_cf volte,
    #  non si inizializzi sulla stessa area di memoria di un precedente LogConfig ? (!?) /SEMBRA OK!
    '''
    state_logger_configs = {}
    
    for i in range(N_cf):
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
    '''

    # Dictionaries for logger data
    state_data_dict = dict()

    mpc_velocity_subs = []
    desired_vx = dict()
    desired_vy = dict()
    desired_vz = dict()
    desired_yaw_rate = dict()
    make_mpc_velocity_subs()

    # ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
    #                 P U B L I S H E R S  S E T U P
    # ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
    # States publisher:
    states_pub = rospy.Publisher('/swarm/states', SwarmStates, queue_size=1)

    # List of states
    # make_states_publishers()

    '''
    # States publisher:
    mpc_target_pub = rospy.Publisher('/swarm/mpc_target', Position, queue_size=1)
    mpc_target = Position()
    '''

    # ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
    #                 S U B S C R I B E R S  S E T U P
    # ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

    land_sub = rospy.Subscriber('/land_topic', Empty, swarm_land_act_callback, queue_size=1)

    # si può anche non utilizzare le action in questo caso, e chiamare direttamente le callback
    # oppure: mantenere la struttura delle action lato server in un file a parte, e qui in MAIN avere il lato client...

    # ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
    #                    A C T I O N S  S E T U P
    # ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
    '''
    # Action to make the entire swarm take off :
    swarm_takeoff_act = actionlib.SimpleActionServer('/swarm/takeoff_actn', TakeoffAction,swarm_takeoff_act_callback, False)
    swarm_takeoff_act.start()
    '''

    '''
    # Action to make the entire swarm land:
    swarm_land_act = actionlib.SimpleActionServer('/swarm/land_actn', TakeoffAction, swarm_land_act_callback, False)
    swarm_land_act.start()
    '''

    # ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
    #             I N I T I A L  O P E R A T I O N S
    # ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

    print('loggers: inizializzati')

    # Drivers initialization:
    cflib.crtp.init_drivers()

    # Instantiation of Swarm:
    swarm = Swarm(uris, factory=_Factory())  # da cui ottengo scf per ogni drone dello sciame; scf è l'oggetto Crazyflie

    print('swarm: inizializzato')

    #  Opening communication
    swarm.open_links()

    print('connessione stabilita')

    # Resetting estimators
    swarm.reset_estimators()

    print('posizione di riferimento trovata')

    # Loggers configuration for all the agents
    state_loggers = dict()

    # Configuring the loggers
    state_loggers_swarm()

    print('logger configurati')

    # List of states as CrazyflieState messages:
    states = []
    make_states_list()

    # Create commanders for the swarm
    create_commanders_dict_swarm()

    print('commanders configurati')

    # provvisorio /no azioni
    swarm_takeoff_act_callback()  #TODO:  provvisorio

    initialOperationsEnded = True

    print('initial operations ended: %s\n' % initialOperationsEnded)

    rospy.spin()

    '''
    while not rospy.is_shutdown():
        pass

    # rospy.sleep(5)

    # swarm_land_act_callback()

    # rospy.is_shutdown(
    '''
