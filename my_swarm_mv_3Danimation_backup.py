#! /usr/bin/env python3
import csv
import sys

import math
import threading

import pdb

import time

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
from crazyCmd.scripts.Sperimentale.Swarm.MioPersonale.Preferiti import lib_mv
from crazyCmd.scripts.Sperimentale.Swarm.MioPersonale.Preferiti.lib_mv import TrajectoryData

from crazyflie_messages.msg import TakeoffAction, TakeoffResult, SwarmStates, CrazyflieState, Position, SphereData
from std_msgs.msg import Empty

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
    pace_state = 100  # TODO <---
    rate = rospy.Rate(pace_state)

    global x_plot, y_plot, z_plot
    # global trj_data_obj

    '''
    t_trigger = time.time()  # tic
    '''

    if initialOperationsEnded and not everythingFinished:

        states_ = SwarmStates()

        cf_index = 0

        for uri in uris:

            # TODO /provvisorio <--- vedi discussione Bitcraze
            #  tbn: _queue.empty() è definito non reliable (!)

            # TODO: verificare...
            last_data = None
            flag_last_data = False

            while not flag_last_data:
                while True:
                    if not state_loggers[uri]._queue.empty():
                        last_data = state_loggers[uri].next()
                        flag_last_data = True
                    else:
                        break

            print(last_data)  # debug  # ? None?
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

        states_pub.publish(states_)  # pubblica stato complessivo swarm
        time.sleep(0.05)

        # ------------- P R I N T I N G   S T A T E -------------

        print('\n--- A C T U A L   S T A T E ---')
        for state in states_.states:
            print(state.name, ' :   x = ', state.position.x, ' y = ', state.position.y, ' z = ', state.position.z)

        # ------------- S A V I N G   S T A T E   O N   F I L E -------------

        for ii in range(local_N_cf):
            x_plot[ii] = states_.states[ii].position.x
            y_plot[ii] = states_.states[ii].position.y
            z_plot[ii] = states_.states[ii].position.z

        # trj_data_obj.save_trj(x_plot, y_plot, z_plot)

        '''
        t_state_pub = time.time()  # tic
        elapsed_state = t_state_pub - t_trigger
        print('elapsed time (state): ', elapsed_state)
        print('frequency (state): ', 1 / elapsed_state)
        '''

        rate.sleep()


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

    velocity_setpoint_swarm()


# ------------------------------------------------------------------------------------------------------------------
#
#               S W A R M _ T A K E O F F
#
# This method is used as the swarm takeoff action.
# ------------------------------------------------------------------------------------------------------------------

def swarm_takeoff():
    print('\nAbout to take off...')
    print("Take off in", end="")
    for ii in range(3, 0, -1):
        print(" %s" % ii, end="")
        time.sleep(1)
    print(" - taking off!")

    takeoff_swarm()
    print('Take off: done\n')

    time.sleep(1)


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

    print('About to land...')

    land_swarm()
    print('Landing: done\n')

    everythingFinished = True

    plot_trj_()

    rospy.signal_shutdown('')
    time.sleep(1)
    sys.exit(0)  # TODO ?


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
    c_dict[scf._link_uri].send_velocity_world_setpoint(
        desired_vx[scf._link_uri],
        desired_vy[scf._link_uri],
        desired_vz[scf._link_uri],
        0)  # desired_yaw_rate[scf._link_uri])
    # desired_yaw_rate imposto = 0 (non è output MPC)

    # TODO: essendo all'interno di parallel_safe: print esce mischiata - da migliorare
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


###########################################################################

#                        O T H E R   F U N C T I O N S

###########################################################################


def plot_trj(obs_array_):
    """
    RuntimeError: main thread is not in main loop

    Questo errore si verifica quando si tenta di creare una finestra di grafica
    in un thread separato dal thread principale.

    Per risolvere questo problema, puoi provare a creare una finestra di grafica
    nel thread principale e passare l'oggetto della finestra di grafica al thread separato.
    In questo modo, il thread separato può accedere alla finestra di grafica creata
    dal thread principale senza causare errori.

    tbn: se si vuole inserire plot_trj in un nuovo modulo da importare,
    al fine di avere codice più pulito, si potrebbe provare a utilizzare
    librerie come PyQt o PySide per creare finestre di grafica in thread separati.
    Queste librerie gestiscono  automaticamente la creazione della finestra
    di grafica nel thread appropriato. (da provare(?))
    """
    # TODO: PyQt o PySide per creare finestre di grafica in thread separati
    # TODO: provare a tornare a versione Matplotlib precedente - Matplotlib version 3.1.2 non dava problemi,
    #  a parte per il salvataggio, che però puo essere fatto mediante cattura schermo

    global x_plot, y_plot, z_plot

    fig_ = plt.figure()
    ax = fig_.add_subplot(111, projection='3d')

    # Set plot properties - general
    ax.set_title('Trajectory Animation in 3D Space')
    ax.set_xlabel('X [m]')
    ax.set_ylabel('Y [m]')
    ax.set_zlabel('Z [m]')
    ax.set_xlim([-0.5, 3.0])
    ax.set_ylim([-2.5, 2.5])
    ax.set_zlim([0.0, 1.0])
    ax.scatter(0, 0, 0, color='r', label='Origin')

    # +++++++++++++++++++++++++ PLOTTING OBSTACLES +++++++++++++++++++++++++++++++++++++++
    """
    u e v sono due griglie, tali che, per ogni punto (u[i,j], v[i,j]), 
    si può ottenere una singola coordinata in longitudine e latitudine, cioè (theta, phi), dove:
    theta = u[i,j] , phi = v[i,j]
    
    Combinando tutti i valori di u e v, puoi coprire tutte le possibili coordinate sulla sfera.
    
    Esempio: se si ha una griglia u di dimensione 3x3 e una griglia v di dimensione 3x3, 
    si ottiene 9x9=81 coordinate possibili.
    """
    # TODO: modificare lunghezza array u v se plot sfera troppo pesante ...

    spheres = obs_array_

    # drawing each sphere
    for sphere in spheres:
        x = sphere.x
        y = sphere.y
        z = sphere.z
        r = sphere.r
        u, v = np.mgrid[0:2 * np.pi:2 * np.pi / 20,
               0:np.pi:np.pi / 10]  # u: angolo di longitudine, v: angolo di latitudine
        x = x + r * np.cos(u) * np.sin(v)
        y = y + r * np.sin(u) * np.sin(v)
        z = z + r * np.cos(v)
        ax.scatter(x, y, z, color='black')

    ax.scatter([], [], [], color='black', label='Obstacles')  # Creazione di un punto fittizio per la legenda

    # +++++++++++++++++++++++++ PLOTTING 3D TRAJECTORIES +++++++++++++++++++++++++++++++++++++++

    lines = []
    for i in range(local_N_cf):
        line, = ax.plot3D([], [], [], lw=1, label=f'cf{i + 1}')  # creo istanza Line3D
        lines.append(line)

    # initialization function: plot the background of each frame
    def init():
        for ii in range(local_N_cf):
            lines[ii].set_data_3d([], [], [])
        return lines

    # animation function: this is called sequentially
    def animate(frame_i):
        for ii in range(local_N_cf):
            lines[ii].set_data_3d(np.append(lines[ii]._verts3d[0], x_plot[ii]),
                                  np.append(lines[ii]._verts3d[1], y_plot[ii]),
                                  np.append(lines[ii]._verts3d[2], z_plot[ii]))
        return lines

    # +++++++++++++++++++++++++ ANIMATION +++++++++++++++++++++++++++++++++++++++

    # call the animator. blit=True means only re-draw the parts that have changed.
    anim = animation.FuncAnimation(fig_, animate, init_func=init,
                                   frames=None, interval=50, blit=True, save_count=100)

    ax.legend()  # aggiungo legenda

    # salvo animazione
    FFMpegWriter = animation.writers['ffmpeg']
    writer_ = FFMpegWriter(fps=60)
    anim.save('/home/matteo/catkin_ws/src/crazyCmd/scripts'
              '/Sperimentale/Swarm/MioPersonale/Preferiti'
              '/animations/3d_trj.mp4', writer=writer_)

    plt.show()

    """
    Per salvare l'animazione, seguire le istruzioni di seguito:
    
    save the animation as an mp4.  This requires ffmpeg or mencoder to be
    installed.  The extra_args ensure that the x264 codec is used, so that
    the video can be embedded in html5.  You may need to adjust this for
    your system: for more information, see
    http://matplotlib.sourceforge.net/api/animation_api.html
    anim.save('basic_animation.mp4', fps=30, extra_args=['-vcodec', 'libx264'])
    """  # TODO: <---


def save_trj():
    global x_plot, y_plot, z_plot
    global trj_data_obj

    while True:
        if initialOperationsEnded and not everythingFinished:
            trj_data_obj.save_trj(x_plot, y_plot, z_plot)
        time.sleep(0.1)


def plot_trj_():
    lib_mv.plot_trj_from_file(filename, local_N_cf, obs_array)


##################################################################

#                            M A I N

##################################################################

if __name__ == '__main__':

    rospy.init_node('node_swarm', log_level=rospy.DEBUG)

    initialOperationsEnded = False
    everythingFinished = False

    # ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
    #                                    I N P U T
    # ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
    # su altro modulo, importato - per migliore visione codice
    # local_N_cf, cf_names, uris, offset, local_scelta_target, local_mpc_target, local_tgt4drone_array, obs_array = lib_mv.get_input_terminal()  # da terminale
    # local_N_cf, cf_names, uris, offset, local_scelta_target, local_mpc_target, local_tgt4drone_array, obs_array = lib_mv.get_input_file()  # da file # TODO: WIP, da continuare
    local_N_cf, cf_names, uris, \
    offset, local_scelta_target, \
    local_mpc_target, local_tgt4drone_array, obs_array = lib_mv.get_input_script()  # da script

    lib_mv.pub_input(local_N_cf, local_scelta_target, local_mpc_target, local_tgt4drone_array, obs_array)

    # ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
    #           T R A J E C T O R Y   A N I M A T I O N   -   I N   B A C K G R O U N D
    # +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

    '''
    # TODO: non funziona
    # animazione in background
    thr = threading.Thread(target=plot_trj, args=(obs_array,))
    thr.start()
    '''

    # ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
    #                                    G E N E R A L   S E T U P
    # ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

    print("\n                   [ G E N E R A L   S E T U P ]\n")

    x_plot = [0.0 for i in range(local_N_cf)]
    y_plot = [0.0 for i in range(local_N_cf)]
    z_plot = [0.0 for i in range(local_N_cf)]

    filename = '/home/matteo/catkin_ws/src/crazyCmd/scripts' \
               '/Sperimentale/Swarm/MioPersonale/Preferiti' \
               '/files_trj/trj.csv'

    trj_data_obj = TrajectoryData(filename)
    trj_data_obj.create_file()

    thr = threading.Thread(target=save_trj)
    thr.start()

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

    # ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
    #                 P U B L I S H E R S   S E T U P
    # ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
    # States publisher:
    states_pub = rospy.Publisher('/swarm/states', SwarmStates, queue_size=1)

    # Emergency publisher:
    emergency_pub = rospy.Publisher('/emergency', Empty, queue_size=1)

    # List of states
    # make_states_publishers()

    # ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
    #                 S U B S C R I B E R S   S E T U P
    # ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
    # Subscriber to pace:
    pace_sub = rospy.Subscriber('/pace_state', Empty, callback=pace_sub_callback, callback_args=offset, queue_size=1)

    # Subscriber to land:
    land_sub = rospy.Subscriber('/land_topic', Empty, swarm_land, queue_size=1)

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

    swarm_takeoff()  # commentare per passare da sperimentale a debug

    initialOperationsEnded = True

    print('Initial operations ended: %s\n' % initialOperationsEnded)

    rospy.spin()
