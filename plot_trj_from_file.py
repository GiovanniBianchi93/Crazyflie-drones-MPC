#! /usr/bin/env python3

import csv
import math
import sys
import time

import rospy
from crazyCmd.scripts.Sperimentale.Swarm.MioPersonale.Preferiti import lib_mv
from crazy_common_py.common_functions import standardNameList
from crazyflie_messages.msg import Position, SphereData
from geometry_msgs.msg import Point
from std_msgs.msg import Int32, Empty

import numpy as np
from matplotlib import pyplot as plt, cm
import mpl_toolkits.mplot3d.axes3d as p3


def plot_trj_from_file(filename_, N_cf_, obs_array_):
    fig_ = plt.figure()
    ax = fig_.add_subplot(111, projection='3d')

    # Set plot properties - general
    ax.set_title('Trajectory in 3D Space')
    ax.set_xlabel('X [m]')
    ax.set_ylabel('Y [m]')
    ax.set_zlabel('Z [m]')
    ax.set_xlim([-1.2, 3.6])
    ax.set_ylim([-3.6, 1.2])
    ax.set_zlim([0.0, 3.0])
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

    # traiettorie in x y z per ogni drone
    x_4drone = []
    y_4drone = []
    z_4drone = []

    for i in range(N_cf_):
        x_4drone.append(x_tot[i::N_cf_])
        y_4drone.append(y_tot[i::N_cf_])
        z_4drone.append(z_tot[i::N_cf_])

    # lista di oggetti Line3D
    lines = []
    for i in range(N_cf_):
        line, = ax.plot3D([], [], [], lw=2, label=f'cf{i + 1}')  # creo istanza Line3D
        lines.append(line)

    # inizializzo oggetti Line3D
    for i in range(N_cf_):
        lines[i].set_data_3d([], [], [])

    # carico traiettorie
    for i in range(N_cf_):
        lines[i].set_data_3d(np.append(lines[i]._verts3d[0], x_4drone[i]),
                             np.append(lines[i]._verts3d[1], y_4drone[i]),
                             np.append(lines[i]._verts3d[2], z_4drone[i]))

    ax.legend()
    plt.show()


if __name__ == '__main__':

    filename = '/home/matteo/catkin_ws/src/crazyCmd/scripts' \
               '/Sperimentale/Swarm/MioPersonale/Preferiti' \
               '/files_trj/trj.csv'

    local_N_cf = 3

    num_obs = 2  # <---

    info_par = []
    obs = []
    obs_array = []  # lista di variabili Point()

    for i in range(num_obs):
        info_par.append({'x': 0.0, 'y': 0.0, 'z': 0.0, 'a': 0.0, 'b': 0.0, 'c': 0.0})  # inizializzo

    # obs1
    info_par[0]['x'] = 1.2
    info_par[0]['y'] = - 1.2
    info_par[0]['z'] = 0.5
    info_par[0]['a'] = 1.0
    info_par[0]['b'] = 1.0
    info_par[0]['c'] = 1.0

    # obs2
    info_par[1]['x'] = 1.6
    info_par[1]['y'] = - 2.4
    info_par[1]['z'] = 0.5
    info_par[1]['a'] = 0.8
    info_par[1]['b'] = 0.8
    info_par[1]['c'] = 0.8

    '''
    # obs3
    info_par[2]['x'] = 1.6
    info_par[2]['y'] = - 0.6
    info_par[2]['z'] = 0.5
    info_par[2]['a'] = 0.5
    info_par[2]['b'] = 0.5
    info_par[2]['c'] = 0.5
    '''

    for i in range(num_obs):
        obs += lib_mv.sphere_approximation(info_par[i]['x'], info_par[i]['y'], info_par[i]['z'],
                                           info_par[i]['a'], info_par[i]['b'], info_par[i]['c'])

    for i in range(len(obs)):
        obs_single = SphereData()
        obs_single.x = obs[i][0]
        obs_single.y = obs[i][1]
        obs_single.z = obs[i][2]
        obs_single.r = obs[i][3]
        obs_array.append(obs_single)

    plot_trj_from_file(filename, local_N_cf, obs_array)
