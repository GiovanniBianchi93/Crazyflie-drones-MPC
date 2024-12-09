#! /usr/bin/env python3

from crazy_common_py.common_functions import standardNameList

import rospy
from crazyflie_messages.msg import Position, SphereData

from geometry_msgs.msg import Point
from std_msgs.msg import Int32

import time


def create_uris_list():
    uris_ = []

    for name in cf_names:
        num_ID = int(name[2:]) - 1
        uri = 'radio://0/80/2M/E7E7E7E7E' + hex(num_ID)[-1]
        uris_.append(uri)

    return uris_


##################################################################

#                            M A I N

##################################################################

if __name__ == '__main__':

    rospy.init_node('node_swarm')

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
    #                 S U B S C R I B E R S  S E T U P
    # ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
    # Subscriber to pace:
    pace_sub = rospy.Subscriber('/pace_state', Empty, callback=pace_sub_callback, callback_args=offset, queue_size=1)

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
