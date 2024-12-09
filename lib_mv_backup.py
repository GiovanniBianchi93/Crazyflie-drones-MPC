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
from matplotlib import pyplot as plt
import mpl_toolkits.mplot3d.axes3d as p3


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


def get_input_terminal():
    # ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
    #                                    G E T T I N G   I N P U T
    # ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

    print("\n                   [ I N P U T   B Y   U S E R ]\n")

    print('------------- N .   D R O N E S   I N   T H E   S W A R M -------------')

    local_N_cf = int(input("Enter the number of drones in the swarm: "))

    cf_names = standardNameList(local_N_cf)

    uris = create_uris_list(cf_names)

    print("Number of drones in the swarm: ", local_N_cf)
    print("Names of the drones: ", cf_names)
    print("URIs of the drones: ", uris)

    print('\n------------- O F F S E T -------------')

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

    local_mpc_target = Position()
    local_tgt4drone_array = []  # lista di variabili Point()

    if local_scelta_target == 1:
        print('\n-------- T A R G E T :   C E N T E R   O F   M A S S --------')

        mpc_target_list = []

        print('Enter target center of mass coordinates (x y z) [m]: ')
        mpc_target_list.append(float(input("x:  ")))
        mpc_target_list.append(float(input("y:  ")))
        mpc_target_list.append(float(input("z:  ")))
        print('Target center of mass acquired: ', mpc_target_list)
        # eventualmente: ottimizzare mpc_target con unica variabile

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

        print('Enter target for each drone coordinates (x y z) [m]:')
        for i in range(local_N_cf):
            tgt4drone[i]['x'] = float(input("%s - x:  " % (i + 1)))
            tgt4drone[i]['y'] = float(input("%s - y:  " % (i + 1)))
            tgt4drone[i]['z'] = float(input("%s - z:  " % (i + 1)))

        print('Target for each drone acquired: ', tgt4drone)

        for i in range(local_N_cf):
            tgt4drone_single = Point()
            tgt4drone_single.x = tgt4drone[i]['x']
            tgt4drone_single.y = tgt4drone[i]['y']
            tgt4drone_single.z = tgt4drone[i]['z']
            local_tgt4drone_array.append(tgt4drone_single)

    print('\n------------- O B S T A C L E S   ( P A R A L L E L E P I P E D ) -------------')
    while True:
        try:
            num_obs = int(input("Enter number of obstacles: "))
            break
        except ValueError:
            print("Error: enter number of obstacles")

    obs = []
    obs_array = []  # lista di variabili Point()

    for i in range(num_obs):
        # inizializzo parallelepiped
        info_par = {'x': 0.0, 'y': 0.0, 'z': 0.0, 'a': 0.0, 'b': 0.0, 'c': 0.0}  # inizializzo

        print('Parallelepiped %s - Enter center of mass (x y z) [m] and dimensions (a b c) [m]: ' % (i + 1))
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

    return local_N_cf, cf_names, uris, offset, local_scelta_target, local_mpc_target, local_tgt4drone_array, obs_array


def get_input_file():
    with open('./users_files/get_input_file', 'r') as file:

        for line in file:
            if line.startswith('number drones:'):
                local_N_cf = int(line.replace('number drones:', '').strip())
                break

        cf_names = standardNameList(local_N_cf)

        uris = create_uris_list(cf_names)

        offset = []

        for i in range(local_N_cf):
            offset.append({'x': 0.0, 'y': 0.0, 'z': 0.0})  # inizializzo lista di dizionari

        for line in file:
            if line.startswith('offset (x y z) [m] for each drone:'):
                offset_str = line.replace('offset (x y z) [m] for each drone:', '').strip().split()
                break

        # debug
        print('offset_str: ', offset_str)

        if len(offset_str) != 3 * local_N_cf:
            print('errore - dati mancanti')
            sys.exit(1)

        for i in range(local_N_cf):
            offset[i]['x'] = float(offset_str[i * 3])
            offset[i]['y'] = float(offset_str[i * 3 + 1])
            offset[i]['z'] = float(offset_str[i * 3 + 2])

        for line in file:
            if line.startswith('target modality (1/2):'):
                local_scelta_target = int(line.replace('target modality (1/2):', '').strip())
                break

        if local_scelta_target not in [1, 2]:
            print('errore - scelta non valida')
            sys.exit(1)

        for line in file:
            if line.startswith('target center of mass (x y z) [m]:'):
                mpc_target_str = line.replace('offset (x y z) [m] for each drone:', '').strip().split()
                break

        if local_scelta_target == 1:
            if len(mpc_target_str) != 3:
                print('errore - dati mancanti')
                sys.exit(1)

        # debug
        print('mpc_target_str: ', mpc_target_str)

        tgt4drone = []

        for i in range(local_N_cf):
            tgt4drone.append({'x': 0.0, 'y': 0.0, 'z': 0.0})

        for line in file:
            if line.startswith('target (x y z) [m] for each drone:'):
                tgt4drone_str = line.replace('target (x y z) [m] for each drone:', '').strip().split()
                break

        # debug
        print('tgt4drone_str: ', tgt4drone_str)

        if local_scelta_target == 2:
            if len(tgt4drone_str) != 3 * local_N_cf:
                print('errore - dati mancanti')
                sys.exit(1)

        for i in range(local_N_cf):
            tgt4drone[i]['x'] = float(tgt4drone_str[i * 3])
            tgt4drone[i]['y'] = float(tgt4drone_str[i * 3 + 1])
            tgt4drone[i]['z'] = float(tgt4drone_str[i * 3 + 2])

        for line in file:
            if line.startswith('number parallelepiped obstacles:'):
                num_obs = int(line.replace('number parallelepiped obstacles:', '').strip())
                break

        for i in range(num_obs):
            info_par = {'x': 0.0, 'y': 0.0, 'z': 0.0, 'a': 0.0, 'b': 0.0, 'c': 0.0}

        for line in file:
            if line.startswith('center of mass (x y z) [m] and dimensions (a b c) [m] for each obstacle:'):
                info_par_str = line.replace('center of mass (x y z) [m] and dimensions (a b c) [m] for each obstacle:',
                                            '').strip().split()
                break

        # debug
        print('info_par_str: ', info_par_str)

        if len(info_par_str) != 6 * num_obs:
            print('errore - dati mancanti')
            sys.exit(1)

        for i in range(num_obs):
            info_par[i]['x'] = float(info_par_str[i * 6])
            info_par[i]['y'] = float(info_par_str[i * 6 + 1])
            info_par[i]['z'] = float(info_par_str[i * 6 + 2])
            info_par[i]['a'] = float(info_par_str[i * 6 + 3])
            info_par[i]['b'] = float(info_par_str[i * 6 + 4])
            info_par[i]['c'] = float(info_par_str[i * 6 + 5])

    # TODO: WIP - da continuare...

    return local_N_cf, cf_names, uris, offset, local_scelta_target, local_mpc_target, local_tgt4drone_array, obs_array


def get_input_script():
    # ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
    #                                    G E T T I N G   I N P U T
    # ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

    print("\n                   [ I N P U T   B Y   U S E R ]\n")

    print('------------- N .   D R O N E S   I N   T H E   S W A R M -------------')

    local_N_cf = 1

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
    # ...
    '''
    # cf2
    offset[1]['x'] = 0
    offset[1]['y'] = 1.2

    # cf3
    offset[2]['x'] = 1.2
    offset[2]['y'] = 0

    # cf4
    offset[3]['x'] = 1.2
    offset[3]['y'] = 1.2
    '''
    '''
    # cf5
    offset[4]['x'] = 1.2
    offset[4]['y'] = 1.2
    '''

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

        tgt_cm = {'x': 1.8, 'y': -0.6, 'z': 0.3}

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
        tgt4drone[1]['x'] = 2.4
        tgt4drone[1]['y'] = -1.6
        tgt4drone[1]['z'] = 0.5

        # tgt4drone3
        tgt4drone[2]['x'] = 3.0
        tgt4drone[2]['y'] = -0.8
        tgt4drone[2]['z'] = 0.7

        # tgt4drone4
        tgt4drone[3]['x'] = 3.0
        tgt4drone[3]['y'] = -1.6
        tgt4drone[3]['z'] = 1.2

        # tgt4drone5
        # tgt4drone[4]['x'] = 3.0
        # tgt4drone[4]['y'] = -1.2
        # tgt4drone[4]['z'] = 1.2

        # ... # in funzione di local_N_cf

        print('Target for each drone acquired: ', tgt4drone)

        for i in range(local_N_cf):
            tgt4drone_single = Point()
            tgt4drone_single.x = tgt4drone[i]['x']
            tgt4drone_single.y = tgt4drone[i]['y']
            tgt4drone_single.z = tgt4drone[i]['z']
            local_tgt4drone_array.append(tgt4drone_single)

    print('\n------------- O B S T A C L E S   ( P A R A L L E L E P I P E D ) -------------')

    num_obs = 0

    info_par = []
    obs = []
    obs_array = []  # lista di variabili Point()

    for i in range(num_obs):
        info_par.append({'x': 0.0, 'y': 0.0, 'z': 0.0, 'a': 0.0, 'b': 0.0, 'c': 0.0})  # inizializzo

    # obs1
    '''
    info_par[0]['x'] = 1.2
    info_par[0]['y'] = 0
    info_par[0]['z'] = 0.5
    info_par[0]['a'] = 0.5
    info_par[0]['b'] = 0.4
    info_par[0]['c'] = 1.0
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


def security(states_):
    lim_inf_x = -1.0
    lim_sup_x = 2.4
    lim_inf_y = -2.0
    lim_sup_y = 2.0
    # lim_inf_z = -1.0
    lim_sup_z = 2.0

    for state in states_.states:

        x = state.position.x
        y = state.position.y
        z = state.position.z

        # security condition
        # TODO: inserire condizione definitiva, segnando su pavimento origine e eventualmente inserendo offset ache su drone cf1
        if not (lim_inf_x <= x <= lim_sup_x and lim_inf_y <= y <= lim_sup_y and z <= lim_sup_z):
            return True

    return False


class TrajectoryData:
    def __init__(self, filename):
        self.filename = filename
        self.writer = None

    def create_file(self):
        with open(self.filename, 'w', newline='') as csvfile:
            pass
        print(f"The CSV file '{self.filename}' has been created or overwritten correctly.")

    def save_trj(self, x_plot, y_plot, z_plot):
        trj_data = list(zip(x_plot, y_plot, z_plot))
        with open(self.filename, 'a', newline='') as csvfile:
            self.writer = csv.writer(csvfile)
            self.writer.writerows(trj_data)


def plot_trj_from_file(filename_, N_cf_, obs_array_):
    fig_ = plt.figure()
    ax = fig_.add_subplot(111, projection='3d')

    # Set plot properties - general
    ax.set_title('Trajectory in 3D Space')
    ax.set_xlabel('X [m]')
    ax.set_ylabel('Y [m]')
    ax.set_zlabel('Z [m]')
    ax.set_xlim([-0.5, 3.0])
    ax.set_ylim([-2.5, 2.5])
    ax.set_zlim([0.0, 1.0])
    ax.scatter(0, 0, 0, color='r', label='Origin')

    # +++++++++++++++++++++++++ PLOTTING OBSTACLES +++++++++++++++++++++++++++++++++++++++

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

    lines = []  # lista di oggetti Line3D
    for i in range(N_cf_):
        line, = ax.plot3D([], [], [], lw=1, label=f'cf{i + 1}')  # creo istanza Line3D
        lines.append(line)

    # inizializzo
    for i in range(N_cf_):
        lines[i].set_data_3d([], [], [])

    # ottengo dati
    with open(filename_, 'r') as csvfile:
        reader = csv.reader(csvfile, delimiter=',')  # se file CVS con numeri separati da COMMA
        # reader = csv.reader(csvfile, delimiter='\t')  # se file CVS con numeri separati da TAB

        # Leggi le colonne dal file
        x_tot = []
        y_tot = []
        z_tot = []

        for riga in reader:
            x_tot.append(float(riga[0]))
            y_tot.append(float(riga[1]))
            z_tot.append(float(riga[2]))

    # carico dati in lines
    x_4drone = []
    y_4drone = []
    z_4drone = []

    for i in range(N_cf_):
        x_4drone.append(x_tot[i::N_cf_])
        y_4drone.append(y_tot[i::N_cf_])
        z_4drone.append(z_tot[i::N_cf_])

    for i in range(N_cf_):
        lines[i].set_data_3d(np.append(lines[i]._verts3d[0], x_4drone[i]),
                             np.append(lines[i]._verts3d[1], y_4drone[i]),
                             np.append(lines[i]._verts3d[2], z_4drone[i]))

    ax.legend()
    plt.show()
