#! /usr/bin/env python3

import csv
import sys
import time

import cflib.crtp
from PyQt5.QtWidgets import QApplication
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
from crazyCmd.scripts.Sperimentale.Swarm.MioPersonale.Preferiti import lib_mv
from crazyCmd.scripts.Sperimentale.Swarm.MioPersonale.Preferiti import lib_plot_mv
from crazyCmd.scripts.Sperimentale.Swarm.MioPersonale.Preferiti.lib_plot_mv import TrajectoryData

from crazyflie_messages.msg import TakeoffAction, TakeoffResult, SwarmStates, CrazyflieState, Position, SphereData
from std_msgs.msg import Empty, Float32

"""
Copia di my_swarm_mv_backup.py

tbn: alcune parti di my_swarm_mv, momentaneamente non utilizzate, sono state eliminate;
tali parti sono ancora presenti in my_swarm_mv, che assurge a ruolo di backup...
"""

'''
17maggio2013

WIP:  ragionare su sincronizzazione eventi, ritardo con frequenza bassa, .next,... <---
TODO:
- risolvere .next()
- risolvere salvataggio su file trj.cvs / anche in background
- aggiungere extended kalman migliore su asse z
- aggiungere w pesi lagrangiana che cambiano gradualmente
- nuovi grafici per consegna tesi
- migliorare codice

ALTRO:
- pace_state non necessario
- animation 3D + salvataggio
- inserire inputs by user da file (WIP, vedi anche input_file_prova.py)
/ alternativa: lib_mv.get_input_script(), da script, provvisorio ok

EXTRA:
- rilevazione ostacolo
'''

'''
- scrivere tesi! /consegna luglio: deposito online termine: 05/06/2023 - 26/06/2023 !!!

'''


# ==================================================================================================================
#
#        L O G G E R S     C O N F I G U R A T I O N    M E T H O D S
#
# ==================================================================================================================

# ++++++++++++++++++++++++ CREATE STATE LOGGERS +++++++++++++++++++++++++++++++++

def state_logger_drone(scf):
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
# -> vedi my_swarm_mv_backup.py


# ++++++++ MPC VELOCITY SUBSCRIBER LIST METHOD ++++++++++++

def make_mpc_velocity_subs():
    for cf_name in cf_names:
        # Subscribers to read the desired velocity computed by the MPC controller
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
#                                       P A C E _ S U B _ C A L L B A C K
#
# This callback is called whenever an Empty message is published by pace_state node; it is used to develop the main
# routine.
# ------------------------------------------------------------------------------------------------------------------
def pace_sub_callback(msg, offset_cb):
    '''
    t_trigger = time.time()  # tic
    '''

    global actual_time_plot

    if initialOperationsEnded and not everythingFinished:

        # `pace_state = 100  # TODO
        # rate = rospy.Rate(pace_state)

        states_ = SwarmStates()

        cf_index = 0

        actual_time = time.time() - start_time

        for uri in uris:

            # TODO /provvisorio <--- vedi discussione Bitcraze (?)
            #  tbn: _queue.empty() è definito non reliable (!)
            last_data = None
            flag_last_data = False

            while not flag_last_data:
                while True:
                    if not state_loggers[uri]._queue.empty():
                        last_data = state_loggers[uri].next()
                        flag_last_data = True
                    else:
                        break

            # print(last_data)  # debug
            state_data_dict[uri] = last_data

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

        # ------------- A D D I N G   O F F S E T -------------

        for ii in range(local_N_cf):
            states[ii].position.x = states[ii].position.x + offset_cb[ii]['x']
            states[ii].position.y = states[ii].position.y + offset_cb[ii]['y']
            states[ii].position.z = states[ii].position.z + offset_cb[ii]['z']

        states_.states = states

        # ------------- E M E R G E N C Y   L A N D I N G -------------
        emergency = lib_mv.security(states_)
        if emergency:
            print('\n\nEMERGENCY LANDING!\n')
            msg_ = Empty()
            emergency_pub.publish(msg_)
            swarm_land(msg_)

        # ------------- P U B B L I C O   S T A T O   S W A R M  ----------------------
        if takeoffDone:  # pubblica stato complessivo swarm a MPC solo dopo aver completato takeoff
            states_pub.publish(states_)
            time.sleep(0.05)

        '''
        # ------------- P R I N T I N G   S T A T E -------------

        print('\n--- A C T U A L   S T A T E ---')
        for state in states_.states:
            print(state.name, ' :   x = ', state.position.x, ' y = ', state.position.y, ' z = ', state.position.z)
        '''

        # ------------- S A V I N G   S T A T E   O N   F I L E -------------

        for ii in range(local_N_cf):
            x_plot[ii] = states_.states[ii].position.x
            y_plot[ii] = states_.states[ii].position.y
            z_plot[ii] = states_.states[ii].position.z
            actual_time_plot[ii] = actual_time
            vx_plot[ii] = states_.states[ii].velocity.x
            vy_plot[ii] = states_.states[ii].velocity.y
            vz_plot[ii] = states_.states[ii].velocity.z

        trj_data.save_trj(x_plot, y_plot, z_plot, actual_time_plot, vx_plot, vy_plot, vz_plot)

        '''
        t_state_pub = time.time()  # tic
        elapsed_state = t_state_pub - t_trigger
        print('elapsed time (state): ', elapsed_state)
        print('frequency (state): ', 1 / elapsed_state)
        '''

        # rate.sleep()


# ------------------------------------------------------------------------------------------------------------------
#
#         M P C _ V E L O C I T Y  _ S U B _ C A L L B A C K
#
# This callback gets the desired velocity computed by the mpc controller
# and sets the velocity target so that the velocity commands are given as
# velocity setpoints to the drones composing the swarm
# ------------------------------------------------------------------------------------------------------------------
def mpc_velocity_sub_callback(msg):
    global count_mpc_vel_received

    cf_name = msg.name
    num_ID = int(cf_name[2:]) - 1
    uri = 'radio://0/80/2M/E7E7E7E7E' + hex(num_ID)[-1]  # [-1] è l'ultimo elemento di hex(num_ID)

    desired_vx[uri] = msg.desired_velocity.x
    desired_vy[uri] = msg.desired_velocity.y
    desired_vz[uri] = msg.desired_velocity.z

    # count_mpc_vel_received += 1

    # if count_mpc_vel_received == local_N_cf:
    # chiamo velocity_setpoint_swarm solo quando
    # ho ricevuto tutte le mpc_velocity per ogni drone  # TODO: provvisorio, da verificare

    global clock

    if cf_name == 'cf1':
        if clock != 0.0:
            t_elapsed = time.time() - clock
            print('TIME CLOCK = ', t_elapsed)
        clock = time.time()

    velocity_setpoint_swarm()


# ------------------------------------------------------------------------------------------------------------------
#
#                       D T _ C B
#
# ------------------------------------------------------------------------------------------------------------------
def DT_cb(msg):
    global DT
    DT = msg


# ------------------------------------------------------------------------------------------------------------------
#
#               S W A R M _ T A K E O F F
#
# This method is used as the swarm takeoff action.
# ------------------------------------------------------------------------------------------------------------------

def swarm_takeoff():
    global takeoffDone
    global start_time
    global start_takeoff, finish_takeoff
    '''
    print('\nAbout to take off...')
    print("Take off in", end="")
    for ii in range(3, 0, -1):
        print(" %s" % ii, end="")
        time.sleep(1)
    print(" - taking off!")
    '''
    time.sleep(1)

    start_takeoff = time.time() - start_time

    takeoff_swarm()
    print('Take off: done\n')
    time.sleep(1)

    takeoffDone = True
    finish_takeoff = time.time() - start_time


# +++++++++++++++++++++++ TAKEOFF METHOD ++++++++++++++++++++++++++++++++++++++

# gestisce il decollo del singolo drone
def takeoff_drone4swarm(scf):
    mc_dict[scf._link_uri].take_off(height=0.3)


# gestisce il decollo a livello di swarm
def takeoff_swarm():
    swarm.parallel_safe(takeoff_drone4swarm)


# ------------------------------------------------------------------------------------------------------------------
#
#              S W A R M _ L A N D
#
# This method is used as the swarm land action.
# ------------------------------------------------------------------------------------------------------------------
def swarm_land(msg):
    global everythingFinished
    global start_time
    global start_landing, finish_landing

    print('About to land...')

    start_landing = time.time() - start_time

    land_swarm()

    print('Landing: done\n')

    finish_landing = time.time() - start_time

    everythingFinished = True


# +++++++++++++++++++++++++ LAND METHOD +++++++++++++++++++++++++++++++++++++++

# a livello di drone
def land_drone4swarm(scf):
    mc_dict[scf._link_uri].land()


# a livello di swarm
def land_swarm():
    # print('land action')
    swarm.parallel_safe(land_drone4swarm)


# ------------------------------------------------------------------------------------------------------------------
#
#                            O T H E R
#
# ------------------------------------------------------------------------------------------------------------------

# +++++++++++++++++++++ VELOCITY SETPOINT METHOD ++++++++++++++++++++++++++++++

# ? esegue mpc_velocity a drone corrispondente ?
def velocity_setpoint_drone4swarm(scf):
    global DT

    '''
    time_sleep = DT.data
    if time_sleep > 0.2:
        time_sleep = 0.2  # limite per tenere sotto controllo drone /rule of thumb
    '''
    # time_sleep = 0.3

    # anche tenendo conto di tempo necessario a MPC per pubblicare nuova velocità
    # limite inferiore: già assegnato mediante T_lim in mpc_swarm_mv_19giu.py
    '''
    # tbn: c_dict by Commander crea conflitto con mc_dict by MotionCommander -> vincolo altezza takeoff *
    c_dict[scf._link_uri].send_velocity_world_setpoint(
        desired_vx[scf._link_uri],
        desired_vy[scf._link_uri],
        desired_vz[scf._link_uri],
        0)  # desired_yaw_rate[scf._link_uri])
    # desired_yaw_rate imposto = 0 (non è output MPC)
    '''

    # alternativa vs vincolo altezza takeoff *
    mc_dict[scf._link_uri].start_linear_motion(
        desired_vx[scf._link_uri],
        desired_vy[scf._link_uri],
        desired_vz[scf._link_uri],
        0)  # desired_yaw_rate[scf._link_uri])
    # print('DT = ', DT.data)
    time.sleep(DT.data)
    # time.sleep(DT.data-0.1)  # rule of thumb per ovviare a semplicità modello single integrator
    mc_dict[scf._link_uri].stop()

    # desired_yaw_rate imposto = 0 (non è output MPC)

    '''
    # essendo all'interno di parallel_safe: print esce mischiata - da migliorare
    print(' vx = ', desired_vx[scf._link_uri],
          ' vy = ', desired_vy[scf._link_uri],
          ' vz = ', desired_vz[scf._link_uri],
          ' - uri: ', scf._link_uri)
    '''

    # TODO: così facendo si considerano le velocità in output da MPC come inerenti a SR assoluto (?!) (Giovanni) <---
    # ? corretto? o meglio linear_motion by MotionCommander, che considera SR relativo drone?
    # ? come fa il drone ad essere a conoscenza di un SR assoluto? /esso è affidabile?


def velocity_setpoint_swarm():
    global count_mpc_vel_received

    # count_mpc_vel_received = 0

    # print('\n--- E X E C U T E D   V E L O C I T Y ---')
    swarm.parallel_safe(velocity_setpoint_drone4swarm)


###########################################################################

#                        O T H E R   F U N C T I O N S

###########################################################################

##################################################################

#                            M A I N

##################################################################

if __name__ == '__main__':

    rospy.init_node('Swarm_Node', log_level=rospy.DEBUG)

    initialOperationsEnded = False
    takeoffDone = False
    everythingFinished = False

    clock = 0.0

    # ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
    #                                    I N P U T
    # ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
    # su altro modulo, importato - per migliore visione codice
    # local_N_cf, cf_names, uris, offset, local_scelta_target, local_mpc_target, local_tgt4drone_array, obs_array = lib_mv.get_input_terminal()  # da terminale
    # local_N_cf, cf_names, uris, offset, local_scelta_target, local_mpc_target, local_tgt4drone_array, obs_array = lib_mv.get_input_file()  # da file # WIP, da continuare
    local_N_cf, cf_names, uris, \
    offset, local_scelta_target, \
    local_mpc_target, local_tgt4drone_array, obs_array = lib_mv.get_input_script()  # da script

    lib_mv.pub_input(local_N_cf, local_scelta_target, local_mpc_target, local_tgt4drone_array, obs_array)

    # ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
    #           T R A J E C T O R Y   A N I M A T I O N   -   I N   B A C K G R O U N D
    # +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

    # vedi my_swarm_mv_3Danimation_backup.py

    # ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
    #                                    G E N E R A L   S E T U P
    # ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

    print("\n                   [ G E N E R A L   S E T U P ]\n")

    app = QApplication(sys.argv)

    x_plot = [None for i in range(local_N_cf)]
    y_plot = [None for i in range(local_N_cf)]
    z_plot = [None for i in range(local_N_cf)]
    actual_time_plot = [0.0 for i in range(local_N_cf)]
    vx_plot = [None for i in range(local_N_cf)]
    vy_plot = [None for i in range(local_N_cf)]
    vz_plot = [None for i in range(local_N_cf)]

    filename_trj = '/home/matteo/catkin_ws/src/crazyCmd/scripts' \
                   '/Sperimentale/Swarm/MioPersonale/Preferiti' \
                   '/files_trj/trj.csv'

    trj_data = TrajectoryData(filename_trj)
    trj_data.create_file()

    # Dictionaries for commanders initialization
    mc_dict = dict()
    c_dict = dict()

    # Logger period
    logger_period = 50  # ms
    # logger_period = 2000  # ms

    #  TODO: è garantito che LogConfig, chiamato N_cf volte,
    #   non si inizializzi sulla stessa area di memoria di un precedente LogConfig ? (!?) /SEMBRA OK!
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

    count_mpc_vel_received = 0
    DT = Float32()

    # ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
    #                 P U B L I S H E R S   S E T U P
    # ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
    # States publisher:
    states_pub = rospy.Publisher('/swarm/swarm_state_topic', SwarmStates, queue_size=1)

    # Emergency publisher:
    emergency_pub = rospy.Publisher('/emergency_topic', Empty, queue_size=1)

    # List of states
    # make_states_publishers()

    # ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
    #                 S U B S C R I B E R S   S E T U P
    # ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
    # Subscriber to pace:
    pace_sub = rospy.Subscriber('/pace_state_topic', Empty, callback=pace_sub_callback, callback_args=offset,
                                queue_size=1)

    # Subscriber to land:
    land_sub = rospy.Subscriber('/land_topic', Empty, swarm_land, queue_size=1)

    sub_DT = rospy.Subscriber('/DT_topic', Float32, DT_cb, queue_size=1)

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

    initialOperationsEnded = True
    print('Initial operations ended: %s\n' % initialOperationsEnded)

    start_time = time.time()
    start_takeoff = None
    finish_takeoff = None
    start_landing = None
    finish_landing = None

    swarm_takeoff()  # commentare per passare da sperimentale a debug
    # takeoffDone = True  # commentare per passare da debug a sperimentale

    while True:
        if everythingFinished:

            filename_wopt = '/home/matteo/catkin_ws/src/crazyCmd/scripts' \
                            '/Sperimentale/Swarm/MioPersonale/Preferiti' \
                            '/files_trj/trj_wopt.csv'

            filename_trjMPC_iter0 = '/home/matteo/catkin_ws/src/crazyCmd/scripts' \
                                    '/Sperimentale/Swarm/MioPersonale/Preferiti' \
                                    '/files_trj/trj_wopt_iter0.csv'
            
            # ------------------------------------------------------------------------------------------------------------------
            #
            #       P L O T   T R A J E C T O R Y
            #
            # ------------------------------------------------------------------------------------------------------------------

            N_mpc = 4  # <---

            if local_scelta_target == 2:
                local_mpc_target = lib_mv.calcolo_centro_massa_target(local_tgt4drone_array)

            # lib_plot_mv.plot_trj(local_N_cf, obs_array, local_mpc_target)
            lib_plot_mv.plot_trj_wopt(local_N_cf, obs_array, N_mpc, filename_trjMPC_iter0, local_mpc_target, local_tgt4drone_array)
            # lib_plot_mv.plot_trj_wopt(local_N_cf, obs_array, N_mpc, filename_wopt, local_mpc_target, local_tgt4drone_array)

            # lib_plot_mv.plot_trj2D(local_N_cf, obs_array, local_mpc_target)
            lib_plot_mv.plot_trj2D_wopt(local_N_cf, obs_array, N_mpc, filename_trjMPC_iter0, local_mpc_target)
            # lib_plot_mv.plot_trj2D_wopt(local_N_cf, obs_array, N_mpc, filename_wopt, local_mpc_target)
            # lib_plot_mv.plot_trj2D_wopt_t4d(local_N_cf, obs_array, N_mpc, filename_trjMPC_iter0, local_mpc_target, local_tgt4drone_array)
            # lib_plot_mv.plot_trj2D_wopt_t4d(local_N_cf, obs_array, N_mpc, filename_wopt, local_mpc_target, local_tgt4drone_array)

            # ------------------------------------------------------------------------------------------------------------------
            #
            #       P L O T   A N I M A T I O N
            #
            # ------------------------------------------------------------------------------------------------------------------

            # lib_plot_mv.plot_trj2D_anim(local_N_cf, obs_array, local_mpc_target)
            lib_plot_mv.plot_trj2D_anim_MPC(local_N_cf, obs_array, N_mpc, local_mpc_target)

            # lib_plot_mv.plot_trj2D_anim_MPC_t4d(local_N_cf, obs_array, N_mpc, local_mpc_target, local_tgt4drone_array)
            # lib_plot_mv.plot_trj2D_anim_xz_MPC_t4d(local_N_cf, obs_array, N_mpc, local_mpc_target, local_tgt4drone_array)


            # ------------------------------------------------------------------------------------------------------------------
            #
            #       P L O T   T A R G E T
            #
            # ------------------------------------------------------------------------------------------------------------------

            lib_plot_mv.plot_cm_xyz(local_N_cf, start_takeoff, finish_takeoff, start_landing, finish_landing,
                                    local_mpc_target)
            lib_plot_mv.plot_distance_tgtcm(local_N_cf, start_takeoff, finish_takeoff, start_landing, finish_landing,
                                            local_mpc_target)
            # lib_plot_mv.plot_distance_tgt4d(local_N_cf, start_takeoff, finish_takeoff, start_landing, finish_landing,
            #                                 local_tgt4drone_array)

            # ------------------------------------------------------------------------------------------------------------------
            #
            #       P L O T   D I S T A N C E S   I N T E R - A G E N T
            #
            # ------------------------------------------------------------------------------------------------------------------

            # CONSEGNA TESI - NON MODIFICARE
            r_drone = 0.1  # <--- stessa d_ref di MPC
            r_safe = 0.3  # <--- stessa d_ref di MPC
            d_ref = 0.75  # <--- stessa d_ref di MPC
            d_coll = r_drone * 2
            d_emer = d_coll + r_safe

            lib_plot_mv.plot_distance_ij(local_N_cf, start_takeoff, finish_takeoff, start_landing, finish_landing,
                                         d_ref, d_emer, d_coll)
            lib_plot_mv.plot_distance_maxminavg(local_N_cf, start_takeoff, finish_takeoff, start_landing, finish_landing, d_ref, d_emer, d_coll)
            lib_plot_mv.plot_distance_min(local_N_cf, start_takeoff, finish_takeoff, start_landing, finish_landing, d_ref, d_emer, d_coll)

            # ------------------------------------------------------------------------------------------------------------------
            #
            #       P L O T     V E L O C I T I E S
            #
            # ------------------------------------------------------------------------------------------------------------------

            # CONSEGNA TESI - NON MODIFICARE
            v_lim = 0.2

            lib_plot_mv.plot_vel_maxminavg(local_N_cf, start_takeoff, finish_takeoff, start_landing, finish_landing,
                                            v_lim)
            lib_plot_mv.plot_vel4drone(local_N_cf, start_takeoff, finish_takeoff, start_landing, finish_landing, v_lim)
            lib_plot_mv.plot_v_MPC(local_N_cf, start_takeoff, finish_takeoff, start_landing, finish_landing, v_lim)

            # ------------------------------------------------------------------------------------------------------------------
            #
            #       P L O T     O B S T A C L E S
            #
            # ------------------------------------------------------------------------------------------------------------------

            lib_plot_mv.plot_distance_obs(local_N_cf, start_takeoff, finish_takeoff, start_landing, finish_landing,
                                           obs_array, r_drone, r_safe)
            # lib_plot_mv.plot_distance_closestobs(local_N_cf, start_takeoff, finish_takeoff, start_landing,
            #                                       finish_landing, obs_array, r_drone, r_safe)
            lib_plot_mv.plot_distance_closestobs_normradius(local_N_cf, start_takeoff, finish_takeoff, start_landing,
                                                              finish_landing, obs_array, r_drone, r_safe)

            # tbn: per produrre correttamente plot, rimanere all'interno del main() thread, ie if __name__ == '__main__':

            break
        time.sleep(1)

    print("\n                   [ E N D   O F   P R O G R A M ]\n")
    rospy.signal_shutdown('')
    sys.exit(0)
