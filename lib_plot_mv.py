#! /usr/bin/env python3
import csv
import math
import os
from datetime import datetime

import numpy as np
from matplotlib import pyplot as plt, animation
import mpl_toolkits.mplot3d.axes3d as p3
from matplotlib.colors import Normalize
from matplotlib.patches import Circle
from matplotlib.ticker import FormatStrFormatter

from crazyflie_messages.msg import Position

from scipy.interpolate import interp2d, interp1d
from scipy.interpolate import griddata

folder = '/home/matteo/Desktop/Tesi/Images/Experiments'


class TrajectoryData:
    def __init__(self, filename):
        self.filename = filename
        self.writer = None

    def create_file(self):
        with open(self.filename, 'w', newline='') as csvfile:
            pass
        print(f"The CSV file '{self.filename}' has been created or overwritten correctly.")

    def save_trj(self, x_plot, y_plot, z_plot, actual_time_plot, vx_plot, vy_plot, vz_plot):
        trj_data = list(zip(actual_time_plot, x_plot, y_plot, z_plot, vx_plot, vy_plot, vz_plot))
        with open(self.filename, 'a', newline='') as csvfile:
            self.writer = csv.writer(csvfile)
            self.writer.writerows(trj_data)

    def save_wopt(self, x_plot, y_plot, z_plot):
        trj_data = list(zip(x_plot, y_plot, z_plot))
        with open(self.filename, 'a', newline='') as csvfile:
            self.writer = csv.writer(csvfile)
            self.writer.writerows(trj_data)


def plot_trj(N_cf_, obs_array_, local_mpc_target):
    filename_trj_ = '/home/matteo/catkin_ws/src/crazyCmd/scripts' \
                    '/Sperimentale/Swarm/MioPersonale/Preferiti' \
                    '/files_trj/trj.csv'

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
    # ax.scatter(0, 0, 0, color='r', label='Origin')

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

    # ax.scatter([], [], [], color='black', label='Obstacles')  # Creazione di un punto fittizio per la legenda
    ax.scatter(local_mpc_target.desired_position.x,
               local_mpc_target.desired_position.y,
               local_mpc_target.desired_position.z, color='red', label='tgt')

    # +++++++++++++++++++++++++ PLOTTING 3D TRAJECTORIES +++++++++++++++++++++++++++++++++++++++

    lines = []  # lista di oggetti Line3D
    for i in range(N_cf_):
        line, = ax.plot3D([], [], [], lw=2, label=f'trj_cf{i + 1}')  # creo istanza Line3D
        lines.append(line)

    # inizializzo
    for i in range(N_cf_):
        lines[i].set_data_3d([], [], [])

    # ottengo dati
    with open(filename_trj_, 'r') as csvfile:
        reader = csv.reader(csvfile, delimiter=',')  # se file CVS con numeri separati da COMMA
        # reader = csv.reader(csvfile, delimiter='\t')  # se file CVS con numeri separati da TAB

        # Leggi le colonne dal file
        x_tot = []
        y_tot = []
        z_tot = []

        for riga in reader:
            x_tot.append(float(riga[1]))
            y_tot.append(float(riga[2]))
            z_tot.append(float(riga[3]))

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

    # salvo immagine
    fig_arg = "plot_trj"
    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    filename = f"{timestamp}_{fig_arg}"
    full_path = os.path.join(folder, filename)
    plt.savefig(full_path)

    plt.show()


def plot_trj_wopt(N_cf_, obs_array_, N_mpc, filename_wopt_, local_mpc_target, local_tgt4drone_array):
    filename_trj_ = '/home/matteo/catkin_ws/src/crazyCmd/scripts' \
                    '/Sperimentale/Swarm/MioPersonale/Preferiti' \
                    '/files_trj/trj.csv'

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
    # ax.scatter(0, 0, 0, color='r', label='Origin')

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

    # SE PRESENTI OSTACOLI
    # ax.scatter([], [], [], color='black', label='Obstacles')  # Creazione di un punto fittizio per la legenda

    # SE MODALITA TARGET CENTRO DI MASSA
    '''
    ax.scatter(local_mpc_target.desired_position.x,
               local_mpc_target.desired_position.y,
               local_mpc_target.desired_position.z, color='red', label='tgt')
    '''
    '''
    # SE MODALITA TARGET 4 DRONE
    # local_tgt4drone_array
    for i in range(N_cf_):
        ax.scatter(local_tgt4drone_array[i].x,
                   local_tgt4drone_array[i].y,
                   local_tgt4drone_array[i].z, label=f'tgt_cf{i + 1}')
    '''
    # +++++++++++++++++++++++++ PLOTTING 3D TRAJECTORIES +++++++++++++++++++++++++++++++++++++++

    lines = []  # lista di oggetti Line3D
    for i in range(N_cf_):
        line, = ax.plot3D([], [], [], lw=2, label=f'trj_cf{i + 1}')  # creo istanza Line3D
        lines.append(line)

    # inizializzo
    for i in range(N_cf_):
        lines[i].set_data_3d([], [], [])

    # ottengo dati
    with open(filename_trj_, 'r') as csvfile:
        reader = csv.reader(csvfile, delimiter=',')  # se file CVS con numeri separati da COMMA
        # reader = csv.reader(csvfile, delimiter='\t')  # se file CVS con numeri separati da TAB

        # Leggi le colonne dal file
        x_tot = []
        y_tot = []
        z_tot = []

        for riga in reader:
            x_tot.append(float(riga[1]))
            y_tot.append(float(riga[2]))
            z_tot.append(float(riga[3]))

    # carico dati in lines
    x_4drone = []
    y_4drone = []
    z_4drone = []

    for i in range(N_cf_):
        x_4drone.append(x_tot[i::N_cf_])
        y_4drone.append(y_tot[i::N_cf_])
        z_4drone.append(z_tot[i::N_cf_])

    x_cm_plot = []

    for i in range(N_cf_):
        lines[i].set_data_3d(np.append(lines[i]._verts3d[0], x_4drone[i]),
                             np.append(lines[i]._verts3d[1], y_4drone[i]),
                             np.append(lines[i]._verts3d[2], z_4drone[i]))

    # +++++++++++++++++++++++++ PLOTTING WOPT TRAJECTORIES +++++++++++++++++++++++++++++++++++++++

    # ottengo dati
    with open(filename_wopt_, 'r') as csvfile:
        reader = csv.reader(csvfile, delimiter=',')  # se file CVS con numeri separati da COMMA
        # reader = csv.reader(csvfile, delimiter='\t')  # se file CVS con numeri separati da TAB

        # Leggi le colonne dal file
        x_tot_ = []
        y_tot_ = []
        z_tot_ = []

        for riga in reader:
            x_tot_.append(float(riga[0]))
            y_tot_.append(float(riga[1]))
            z_tot_.append(float(riga[2]))

    # carico dati in lines
    x_tot_drone_tot_iteration = []
    y_tot_drone_tot_iteration = []
    z_tot_drone_tot_iteration = []

    for i in range(N_cf_):
        x_tot_drone_tot_iteration.append(x_tot_[i::N_cf_])
        y_tot_drone_tot_iteration.append(y_tot_[i::N_cf_])
        z_tot_drone_tot_iteration.append(z_tot_[i::N_cf_])

    x_4drone_totiteration = []
    y_4drone_totiteration = []
    z_4drone_totiteration = []

    num_iter = len(x_tot_drone_tot_iteration[0]) / (N_mpc + 1)
    num_iter = int(num_iter)

    for drone in x_tot_drone_tot_iteration:
        for i in range(num_iter):
            x_4drone_totiteration.append(drone[i * (N_mpc + 1):(i + 1) * (N_mpc + 1)])  # ?

    for drone in y_tot_drone_tot_iteration:
        for i in range(num_iter):
            y_4drone_totiteration.append(drone[i * (N_mpc + 1):(i + 1) * (N_mpc + 1)])

    for drone in z_tot_drone_tot_iteration:
        for i in range(num_iter):
            z_4drone_totiteration.append(drone[i * (N_mpc + 1):(i + 1) * (N_mpc + 1)])

    lines_wopt = []  # lista di oggetti Line3D

    for i in range(N_cf_):
        for j in range(num_iter):
            line_wopt, = ax.plot3D([], [], [], lw=1, linestyle='dashed',
                                   label=f'cf{i + 1}_pred{j * 5}')  # creo istanza Line3D
            line_wopt.set_marker('o')
            line_wopt.set_markersize(3)
            lines_wopt.append(line_wopt)

    # inizializzo
    for i in range(N_cf_):
        for j in range(num_iter):
            lines_wopt[i * num_iter + j].set_data_3d([], [], [])

    for i in range(N_cf_):
        for j in range(num_iter):
            lines_wopt[i * num_iter + j].set_data_3d(
                np.append(lines_wopt[i * num_iter + j]._verts3d[0], x_4drone_totiteration[i * num_iter + j]),
                np.append(lines_wopt[i * num_iter + j]._verts3d[1], y_4drone_totiteration[i * num_iter + j]),
                np.append(lines_wopt[i * num_iter + j]._verts3d[2], z_4drone_totiteration[i * num_iter + j]))

    ax.legend()

    # salvo immagine
    fig_arg = "plot_trj_wopt"
    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    filename = f"{timestamp}_{fig_arg}"
    full_path = os.path.join(folder, filename)
    plt.savefig(full_path)

    plt.show()


def plot_trj2D(N_cf_, obs_array, local_mpc_target):
    filename_trj_ = '/home/matteo/catkin_ws/src/crazyCmd/scripts' \
                    '/Sperimentale/Swarm/MioPersonale/Preferiti' \
                    '/files_trj/trj.csv'

    # Inizializzazione
    x_coords = []
    y_coords = []
    time_coords = []

    # Leggi il file CSV
    with open(filename_trj_, 'r') as csvfile:
        reader = csv.reader(csvfile, delimiter=',')
        for row in reader:
            time_coords.append(float(row[0]))
            x_coords.append(float(row[1]))
            y_coords.append(float(row[2]))

    time_coords = time_coords[0::N_cf_]

    x_4drone = []
    y_4drone = []

    for i in range(N_cf_):
        x_4drone.append(x_coords[i::N_cf_])
        y_4drone.append(y_coords[i::N_cf_])

    # target
    tgt_x = local_mpc_target.desired_position.x
    tgt_y = local_mpc_target.desired_position.y

    # calcolo cm finale
    cm_fin_x, cm_fin_y = 0.0, 0.0
    for i in range(N_cf_):
        cm_fin_x += x_4drone[i][len(time_coords) - 1]
        cm_fin_y += y_4drone[i][len(time_coords) - 1]
    cm_fin_x /= N_cf_
    cm_fin_y /= N_cf_

    # Inizializzazione - END

    # Plotto traiettorie
    for i in range(N_cf_):
        plt.plot(x_4drone[i], y_4drone[i], '-o', lw=1, markersize=1, label=f'trj_cf{i + 1}')

    # Plotto cm finale
    # plt.plot(cm_fin_x, cm_fin_y, 'bx', label='cm_fin')
    plt.plot(cm_fin_x, cm_fin_y, 'bx', label='final_pos')

    # Plotto target
    # plt.plot(tgt_x, tgt_y, 'rx', label='tgt_cm')
    plt.plot(tgt_x, tgt_y, 'rx', label='tgt')

    # Plotto target threshold
    gap_end_loop = 0.15
    tgt_ts = Circle((tgt_x, tgt_y), gap_end_loop, edgecolor='black', alpha=0.3, linestyle='dashed', facecolor='none',
                    label='tgt_threshold')
    plt.gca().add_patch(tgt_ts)

    # Plotto ostacoli
    for i in range(len(obs_array)):
        cx, cy, r = obs_array[i].x, obs_array[i].y, obs_array[i].r
        obs = Circle((cx, cy), r, edgecolor='black', facecolor='gray')
        plt.gca().add_patch(obs)
    # plt.gca().add_patch(Circle((0, 0), 0, edgecolor='black', facecolor='gray', label='obs'))  # etichetta

    # Formattazione
    plt.xlabel('X [m]')
    plt.ylabel('Y [m]')
    plt.xlim(-1.2, 3.6)
    plt.ylim(-3.6, 1.2)
    plt.legend()
    plt.grid(True)
    plt.title("Trajectory in X-Y Plane")

    # salvo immagine
    fig_arg = "plot_trj2D"
    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    filename = f"{timestamp}_{fig_arg}"
    full_path = os.path.join(folder, filename)
    plt.savefig(full_path)

    plt.show()


def plot_trj2D_wopt(N_cf_, obs_array, N_mpc, filename_wopt_, local_mpc_target):
    filename_trj_ = '/home/matteo/catkin_ws/src/crazyCmd/scripts' \
                    '/Sperimentale/Swarm/MioPersonale/Preferiti' \
                    '/files_trj/trj.csv'

    # +++++++++++++++++++++++++ PLOTTING 2D TRAJECTORIES +++++++++++++++++++++++++++++++++++++++

    # Inizializzazione
    x_coords = []
    y_coords = []
    time_coords = []

    # Leggi il file CSV
    with open(filename_trj_, 'r') as csvfile:
        reader = csv.reader(csvfile, delimiter=',')
        for row in reader:
            time_coords.append(float(row[0]))
            x_coords.append(float(row[1]))
            y_coords.append(float(row[2]))

    time_coords = time_coords[0::N_cf_]

    x_4drone = []
    y_4drone = []

    for i in range(N_cf_):
        x_4drone.append(x_coords[i::N_cf_])
        y_4drone.append(y_coords[i::N_cf_])

    # target
    tgt_x = local_mpc_target.desired_position.x
    tgt_y = local_mpc_target.desired_position.y

    # MODALITA' TARGET CENTER OF MASS
    # calcolo cm finale
    cm_fin_x, cm_fin_y = 0.0, 0.0
    for i in range(N_cf_):
        cm_fin_x += x_4drone[i][len(time_coords) - 1]
        cm_fin_y += y_4drone[i][len(time_coords) - 1]
    cm_fin_x /= N_cf_
    cm_fin_y /= N_cf_

    # Inizializzazione - END

    # Plotto traiettorie
    for i in range(N_cf_):
        plt.plot(x_4drone[i], y_4drone[i], '-o', lw=1, markersize=1, label=f'trj_cf{i + 1}')

    # MODALITA' TARGET CENTER OF MASS
    # Plotto cm finale
    plt.plot(cm_fin_x, cm_fin_y, 'bx', label='final_pos_cm')

    # MODALITA' TARGET CENTER OF MASS
    # Plotto target
    # plt.plot(tgt_x, tgt_y, 'rx', label='tgt_cm')
    plt.plot(tgt_x, tgt_y, 'rx', label='tgt')

    # MODALITA' TARGET CENTER OF MASS
    # Plotto target threshold
    gap_end_loop = 0.15
    tgt_ts = Circle((tgt_x, tgt_y), gap_end_loop, edgecolor='black', alpha=0.3, linestyle='dashed', facecolor='none',
                    label='tgt_threshold')
    plt.gca().add_patch(tgt_ts)

    # Plotto ostacoli
    for i in range(len(obs_array)):
        cx, cy, r = obs_array[i].x, obs_array[i].y, obs_array[i].r
        obs = Circle((cx, cy), r, edgecolor='black', facecolor='gray')
        plt.gca().add_patch(obs)
    # plt.gca().add_patch(Circle((0, 0), 0, edgecolor='black', facecolor='gray', label='obs'))  # etichetta

    # +++++++++++++++++++++++++ PLOTTING WOPT TRAJECTORIES +++++++++++++++++++++++++++++++++++++++

    # Leggi il file CSV
    with open(filename_wopt_, 'r') as csvfile:
        reader = csv.reader(csvfile, delimiter=',')

        # Leggi le colonne dal file
        x_tot_ = []
        y_tot_ = []

        for riga in reader:
            x_tot_.append(float(riga[0]))
            y_tot_.append(float(riga[1]))

    # carico dati in lines
    x_tot_drone_tot_iteration = []
    y_tot_drone_tot_iteration = []

    for i in range(N_cf_):
        x_tot_drone_tot_iteration.append(x_tot_[i::N_cf_])
        y_tot_drone_tot_iteration.append(y_tot_[i::N_cf_])

    x_4drone_totiteration = []
    y_4drone_totiteration = []

    num_iter = len(x_tot_drone_tot_iteration[0]) / (N_mpc + 1)
    num_iter = int(num_iter)

    for drone in x_tot_drone_tot_iteration:
        for i in range(num_iter):
            x_4drone_totiteration.append(drone[i * (N_mpc + 1):(i + 1) * (N_mpc + 1)])  # ?

    for drone in y_tot_drone_tot_iteration:
        for i in range(num_iter):
            y_4drone_totiteration.append(drone[i * (N_mpc + 1):(i + 1) * (N_mpc + 1)])

    # Inizializzazione - END

    # Plotto traiettorie
    for i in range(N_cf_):
        for j in range(num_iter):
            plt.plot(x_4drone_totiteration[i * num_iter + j],
                     y_4drone_totiteration[i * num_iter + j],
                     '-o', linestyle='dashed', lw=1, markersize=3, label=f'cf{i + 1}_pred{j * 5}')

    # +++++++++++++++++++++++++ FORMATTAZIONE +++++++++++++++++++++++++++++++++++++++

    plt.xlabel('X [m]')
    plt.ylabel('Y [m]')
    plt.xlim(-0.5, 4.0)
    plt.ylim(-3.6, 1.2)
    plt.legend()
    plt.grid(True)
    plt.title("Trajectory in X-Y Plane")

    # salvo immagine
    fig_arg = "plot_trj2D_wopt"
    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    filename = f"{timestamp}_{fig_arg}"
    full_path = os.path.join(folder, filename)
    plt.savefig(full_path)

    plt.show()


def plot_trj2D_wopt_t4d(N_cf_, obs_array, N_mpc, filename_wopt_, local_mpc_target, local_tgt4drone_array):
    filename_trj_ = '/home/matteo/catkin_ws/src/crazyCmd/scripts' \
                    '/Sperimentale/Swarm/MioPersonale/Preferiti' \
                    '/files_trj/trj.csv'

    # +++++++++++++++++++++++++ PLOTTING 2D TRAJECTORIES +++++++++++++++++++++++++++++++++++++++

    # Inizializzazione
    x_coords = []
    y_coords = []
    time_coords = []

    # Leggi il file CSV
    with open(filename_trj_, 'r') as csvfile:
        reader = csv.reader(csvfile, delimiter=',')
        for row in reader:
            time_coords.append(float(row[0]))
            x_coords.append(float(row[1]))
            y_coords.append(float(row[2]))

    time_coords = time_coords[0::N_cf_]

    x_4drone = []
    y_4drone = []

    for i in range(N_cf_):
        x_4drone.append(x_coords[i::N_cf_])
        y_4drone.append(y_coords[i::N_cf_])

    # MODALITA' TARGET 4 DRONE
    final_pos_x = []
    final_pos_y = []

    for i in range(N_cf_):
        final_pos_x.append(x_4drone[i][len(time_coords) - 1])
        final_pos_y.append(y_4drone[i][len(time_coords) - 1])

    # Inizializzazione - END

    # Plotto traiettorie
    for i in range(N_cf_):
        plt.plot(x_4drone[i], y_4drone[i], '-o', lw=1, markersize=1, label=f'trj_cf{i + 1}')

    # MODALITA' TARGET 4 DRONE
    for i in range(N_cf_):
        if i == 0:
            plt.plot(final_pos_x[i], final_pos_y[i], 'bx', label=f'final_pos_cf{i + 1}')
        if i == 1:
            plt.plot(final_pos_x[i], final_pos_y[i], 'x', color='orange', label=f'final_pos_cf{i + 1}')

    # MODALITA' TARGET 4 DRONE
    for i in range(N_cf_):
        plt.plot(local_tgt4drone_array[i].x, local_tgt4drone_array[i].y, 'rx', label=f'tgt_cf{i + 1}')

    # MODALITA' TARGET 4 DRONE
    # Plotto target threshold
    gap_end_loop = 0.15
    for i in range(N_cf_):
        tgt_ts = Circle((local_tgt4drone_array[i].x, local_tgt4drone_array[i].y), gap_end_loop, edgecolor='black',
                        alpha=0.3, linestyle='dashed', facecolor='none')
        plt.gca().add_patch(tgt_ts)
    plt.gca().add_patch(Circle((0, 0), 0, edgecolor='black', alpha=0.3, linestyle='dashed', facecolor='none',
                               label='tgt_threshold'))  # etichetta una tantum

    # Plotto ostacoli
    for i in range(len(obs_array)):
        cx, cy, r = obs_array[i].x, obs_array[i].y, obs_array[i].r
        obs = Circle((cx, cy), r, edgecolor='black', facecolor='gray')
        plt.gca().add_patch(obs)
    # plt.gca().add_patch(Circle((0, 0), 0, edgecolor='black', facecolor='gray', label='obs'))  # etichetta

    # +++++++++++++++++++++++++ PLOTTING WOPT TRAJECTORIES +++++++++++++++++++++++++++++++++++++++

    # Leggi il file CSV
    with open(filename_wopt_, 'r') as csvfile:
        reader = csv.reader(csvfile, delimiter=',')

        # Leggi le colonne dal file
        x_tot_ = []
        y_tot_ = []

        for riga in reader:
            x_tot_.append(float(riga[0]))
            y_tot_.append(float(riga[1]))

    # carico dati in lines
    x_tot_drone_tot_iteration = []
    y_tot_drone_tot_iteration = []

    for i in range(N_cf_):
        x_tot_drone_tot_iteration.append(x_tot_[i::N_cf_])
        y_tot_drone_tot_iteration.append(y_tot_[i::N_cf_])

    x_4drone_totiteration = []
    y_4drone_totiteration = []

    num_iter = len(x_tot_drone_tot_iteration[0]) / (N_mpc + 1)
    num_iter = int(num_iter)

    for drone in x_tot_drone_tot_iteration:
        for i in range(num_iter):
            x_4drone_totiteration.append(drone[i * (N_mpc + 1):(i + 1) * (N_mpc + 1)])  # ?

    for drone in y_tot_drone_tot_iteration:
        for i in range(num_iter):
            y_4drone_totiteration.append(drone[i * (N_mpc + 1):(i + 1) * (N_mpc + 1)])

    # Inizializzazione - END

    # Plotto traiettorie
    for i in range(N_cf_):
        for j in range(num_iter):
            plt.plot(x_4drone_totiteration[i * num_iter + j],
                     y_4drone_totiteration[i * num_iter + j],
                     '-o', linestyle='dashed', lw=1, markersize=3, label=f'cf{i + 1}_pred{j * 5}')

    # +++++++++++++++++++++++++ FORMATTAZIONE +++++++++++++++++++++++++++++++++++++++

    plt.xlabel('X [m]')
    plt.ylabel('Y [m]')
    plt.xlim(-0.5, 4.0)
    plt.ylim(-3.6, 1.2)
    plt.legend()
    plt.grid(True)
    plt.title("Trajectory in X-Y Plane")

    # salvo immagine
    fig_arg = "plot_trj2D_wopt_t4d"
    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    filename = f"{timestamp}_{fig_arg}"
    full_path = os.path.join(folder, filename)
    plt.savefig(full_path)

    plt.show()


def plot_trj2D_anim(N_cf_, obs_array, local_mpc_target):
    # TODO: PER PRESENTAZIONE!

    filename_trj_ = '/home/matteo/catkin_ws/src/crazyCmd/scripts' \
                    '/Sperimentale/Swarm/MioPersonale/Preferiti' \
                    '/files_trj/trj.csv'

    # Inizializzazione
    x_coords = []
    y_coords = []
    time_coords = []

    # Leggi il file CSV
    with open(filename_trj_, 'r') as csvfile:
        reader = csv.reader(csvfile, delimiter=',')
        for row in reader:
            time_coords.append(float(row[0]))
            x_coords.append(float(row[1]))
            y_coords.append(float(row[2]))

    time_coords = time_coords[0::N_cf_]

    x_4drone = []
    y_4drone = []

    for i in range(N_cf_):
        x_4drone.append(x_coords[i::N_cf_])
        y_4drone.append(y_coords[i::N_cf_])

    # target
    tgt_x = local_mpc_target.desired_position.x
    tgt_y = local_mpc_target.desired_position.y

    # calcolo cm finale
    cm_fin_x, cm_fin_y = 0.0, 0.0
    for i in range(N_cf_):
        cm_fin_x += x_4drone[i][len(time_coords) - 1]
        cm_fin_y += y_4drone[i][len(time_coords) - 1]
    cm_fin_x /= N_cf_
    cm_fin_y /= N_cf_

    gap_end_loop = 0.15

    # Inizializzazione - END

    # Crea la figura e gli assi
    fig, ax = plt.subplots()

    # Plotto cm finale
    plt.plot(cm_fin_x, cm_fin_y, 'bx', label='cm_fin')

    # Plotto target
    plt.plot(tgt_x, tgt_y, 'rx', label='tgt_cm')

    # Plotto target threshold
    tgt_ts = Circle((tgt_x, tgt_y), gap_end_loop, edgecolor='black', alpha=0.3, linestyle='dashed', facecolor='none',
                    label='tgt_threshold')
    plt.gca().add_patch(tgt_ts)

    # Plotto ostacoli
    for i in range(len(obs_array)):
        cx, cy, r = obs_array[i].x, obs_array[i].y, obs_array[i].r
        obs = Circle((cx, cy), r, edgecolor='black', facecolor='gray')
        plt.gca().add_patch(obs)
    # plt.gca().add_patch(Circle((0, 0), 0, edgecolor='black', facecolor='gray', label='obs'))  # etichetta

    # Animazione

    # Inizializzo traiettorie
    lines = []
    for i in range(N_cf_):
        line, = ax.plot([], [], '-o', lw=1, markersize=1, label=f'trj_cf{i + 1}')  # creo istanza Line3D
        lines.append(line)

    # Inizializzo il testo del tempo
    time_text = ax.text(0.02, 0.95, '', transform=ax.transAxes)

    # initialization function: plot the background of each frame
    def init():
        for ii in range(N_cf_):
            lines[ii].set_data([], [])
        return lines + [time_text]  # return lines, time_text non funziona !?

    # animation function: this is called sequentially
    def update(frame):
        for ii in range(N_cf_):
            lines[ii].set_data(x_4drone[ii][:frame], y_4drone[ii][:frame])
            time_text.set_text('Time: {:.2f} [s]'.format(time_coords[frame]))  # Aggiorna il testo del tempo
        return lines + [time_text]  # return lines, time_text non funziona !?

    # +++++++++++++++++++++++++ ANIMATION +++++++++++++++++++++++++++++++++++++++

    # call the animator. blit=True means only re-draw the parts that have changed.
    anim = animation.FuncAnimation(fig, update, frames=len(time_coords), init_func=init,
                                   blit=True, interval=50)
    # Animation - END

    # Formattazione
    plt.xlabel('X [m]')
    plt.ylabel('Y [m]')
    plt.xlim(-1.2, 3.6)
    plt.ylim(-3.6, 1.2)
    plt.legend()
    plt.grid(True)
    plt.title("Trajectory - Animation")
    plt.legend()

    # Mostra l'animazione
    plt.show()
    # TODO: salvare animazione


def plot_trj2D_anim_MPC(N_cf_, obs_array, N_mpc, local_mpc_target):
    # TODO: PER PRESENTAZIONE!

    filename_trj_ = '/home/matteo/catkin_ws/src/crazyCmd/scripts' \
                    '/Sperimentale/Swarm/MioPersonale/Preferiti' \
                    '/files_trj/trj.csv'

    filename_wopt_ = '/home/matteo/catkin_ws/src/crazyCmd/scripts' \
                     '/Sperimentale/Swarm/MioPersonale/Preferiti' \
                     '/files_trj/trj_wopt.csv'

    # Inizializzazione Traiettorie Sperimentali

    x_coords = []
    y_coords = []
    time_coords = []

    # Leggi il file CSV
    with open(filename_trj_, 'r') as csvfile:
        reader = csv.reader(csvfile, delimiter=',')
        for row in reader:
            time_coords.append(float(row[0]))
            x_coords.append(float(row[1]))
            y_coords.append(float(row[2]))

    time_coords = time_coords[0::N_cf_]

    x_4drone = []
    y_4drone = []

    for i in range(N_cf_):
        x_4drone.append(x_coords[i::N_cf_])
        y_4drone.append(y_coords[i::N_cf_])

    # MODALITA' TARGET CENTER OF MASS
    # calcolo cm finale
    cm_fin_x, cm_fin_y = 0.0, 0.0
    for i in range(N_cf_):
        cm_fin_x += x_4drone[i][len(time_coords) - 1]
        cm_fin_y += y_4drone[i][len(time_coords) - 1]
    cm_fin_x /= N_cf_
    cm_fin_y /= N_cf_

    # Inizializzazione Traiettorie Sperimentali - END

    # Inizializzazione Traiettorie MPC

    # Leggi il file CSV
    with open(filename_wopt_, 'r') as csvfile:
        reader = csv.reader(csvfile, delimiter=',')

        # Leggi le colonne dal file
        x_tot_ = []
        y_tot_ = []

        for riga in reader:
            x_tot_.append(float(riga[0]))
            y_tot_.append(float(riga[1]))

    # carico dati in lines
    x_tot_drone_tot_iteration = []
    y_tot_drone_tot_iteration = []

    for i in range(N_cf_):
        x_tot_drone_tot_iteration.append(x_tot_[i::N_cf_])
        y_tot_drone_tot_iteration.append(y_tot_[i::N_cf_])

    x_4drone_totiteration = []
    y_4drone_totiteration = []

    num_iter = len(x_tot_drone_tot_iteration[0]) / (N_mpc + 1)
    num_iter = int(num_iter)

    for drone in x_tot_drone_tot_iteration:
        for i in range(num_iter):
            x_4drone_totiteration.append(drone[i * (N_mpc + 1):(i + 1) * (N_mpc + 1)])

    for drone in y_tot_drone_tot_iteration:
        for i in range(num_iter):
            y_4drone_totiteration.append(drone[i * (N_mpc + 1):(i + 1) * (N_mpc + 1)])

    # Inizializzazione Traiettorie MPC - END

    # Crea la figura e gli assi
    fig, ax = plt.subplots()

    # PLOTTO OGGETTI STATICI

    # target
    tgt_x = local_mpc_target.desired_position.x
    tgt_y = local_mpc_target.desired_position.y

    # MODALITA' TARGET CENTER OF MASS
    # calcolo cm finale
    cm_fin_x, cm_fin_y = 0.0, 0.0
    for i in range(N_cf_):
        cm_fin_x += x_4drone[i][len(time_coords) - 1]
        cm_fin_y += y_4drone[i][len(time_coords) - 1]
    cm_fin_x /= N_cf_
    cm_fin_y /= N_cf_

    # Plotto traiettorie
    # for i in range(N_cf_):
    #     plt.plot(x_4drone[i], y_4drone[i], '-', lw=1, linestyle='dashed', color='black', alpha=0.3)

    # MODALITA' TARGET CENTER OF MASS
    # Plotto cm finale
    # plt.plot(cm_fin_x, cm_fin_y, 'bx', label='final_pos_cm')

    # MODALITA' TARGET CENTER OF MASS
    # Plotto target
    # plt.plot(tgt_x, tgt_y, 'rx', label='tgt_cm')
    plt.plot(tgt_x, tgt_y, 'rx', label='tgt')

    # MODALITA' TARGET CENTER OF MASS
    # Plotto target threshold
    gap_end_loop = 0.15
    tgt_ts = Circle((tgt_x, tgt_y), gap_end_loop, edgecolor='black', alpha=0.3, linestyle='dashed', facecolor='none',
                    label='tgt_threshold')
    plt.gca().add_patch(tgt_ts)

    # Plotto ostacoli
    for i in range(len(obs_array)):
        cx, cy, r = obs_array[i].x, obs_array[i].y, obs_array[i].r
        obs = Circle((cx, cy), r, edgecolor='black', facecolor='gray')
        plt.gca().add_patch(obs)
    # plt.gca().add_patch(Circle((0, 0), 0, edgecolor='black', facecolor='gray', label='obs'))  # etichetta

    # PLOTTO OGGETTI STATICI - END

    # +++++++++++++++++++++++++ ANIMATION +++++++++++++++++++++++++++++++++++++++

    # Inizializzo traiettorie
    lines = []
    for i in range(N_cf_):
        if i == 0:
            line, = ax.plot([], [], '-o', color='blue', lw=3, markersize=1,
                            label=f'trj_cf{i + 1}')  # creo istanza Line3D
        if i == 1:
            line, = ax.plot([], [], '-o', color='orange', lw=3, markersize=1,
                            label=f'trj_cf{i + 1}')  # creo istanza Line3D
        if i == 2:
            line, = ax.plot([], [], '-o', color='green', lw=3, markersize=1,
                            label=f'trj_cf{i + 1}')  # creo istanza Line3D
        if i == 3:
            line, = ax.plot([], [], '-o', color='red', lw=3, markersize=1,
                            label=f'trj_cf{i + 1}')  # creo istanza Line3D
        lines.append(line)

    lines_mpc = []
    for i in range(N_cf_):
        if i == 0:
            line, = ax.plot([], [], '-o', color='blue', lw=2, markersize=3, linestyle='dashed')
        if i == 1:
            line, = ax.plot([], [], '-o', color='orange', lw=2, markersize=3, linestyle='dashed')
        if i == 2:
            line, = ax.plot([], [], '-o', color='green', lw=2, markersize=3, linestyle='dashed')
        if i == 3:
            line, = ax.plot([], [], '-o', color='red', lw=2, markersize=3, linestyle='dashed')
        lines_mpc.append(line)

    lines_final_pos = []
    line, = ax.plot([], [], 'bx', label='final_pos_cm')
    lines_final_pos.append(line)

    # Inizializzo il testo del tempo
    time_text = ax.text(0.02, 0.95, '', transform=ax.transAxes)

    def save_frame(frame):
        fig_arg = "plot_trj2D_anim_MPC_t4d"
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        filename = f"{timestamp}_{fig_arg}_{frame:04d}.png"
        full_path = os.path.join(folder, filename)
        # Salva il frame corrente come immagine
        fig.savefig(full_path)

    # initialization function: plot the background of each frame
    def init():
        for ii in range(N_cf_):
            lines[ii].set_data([], [])

        for i in range(N_cf_):
            lines_mpc[i].set_data([], [])

        lines_final_pos[0].set_data([], [])

        return lines + lines_mpc + lines_final_pos + [time_text]

    # animation function: this is called sequentially
    def update(frame):
        for ii in range(N_cf_):
            # visualizza scia
            # scia = frame - 30
            # if scia < 0:
            #     scia = 0
            # lines[ii].set_data(x_4drone[ii][scia:frame], y_4drone[ii][scia:frame])
            lines[ii].set_data(x_4drone[ii][:frame], y_4drone[ii][:frame])
            time_text.set_text('Time: {:.2f} [s]'.format(time_coords[frame]))  # Aggiorna il testo del tempo
            for j in range(num_iter):
                if x_4drone_totiteration[ii * num_iter + j][0] == x_4drone[ii][frame] and \
                        y_4drone_totiteration[ii * num_iter + j][0] == y_4drone[ii][frame]:
                    lines_mpc[ii].set_data(x_4drone_totiteration[ii * num_iter + j],
                                           y_4drone_totiteration[ii * num_iter + j])
                    save_frame(frame)

        if frame == len(time_coords) - 10:  # Verifica se è l'ultimo frame
            lines_final_pos[0].set_data(cm_fin_x, cm_fin_y)
            save_frame(frame)

        return lines + lines_mpc + lines_final_pos + [time_text]

    # call the animator. blit=True means only re-draw the parts that have changed.
    anim = animation.FuncAnimation(fig, update, frames=len(time_coords), init_func=init,
                                   blit=True, interval=50)
    # Animation - END

    # Formattazione
    plt.xlabel('X [m]')
    plt.ylabel('Y [m]')
    plt.xlim(-0.5, 3.6)
    plt.ylim(-3.6, 1.2)
    plt.legend()
    plt.grid(True)
    plt.title("Trajectory - Animation")
    plt.legend(loc='upper right', bbox_to_anchor=(1.1, 1.15))

    # salvo animazione
    fig_arg = "plot_trj2D_anim_MPC.mp4"
    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    filename = f"{timestamp}_{fig_arg}"
    full_path = os.path.join(folder, filename)
    anim.save(full_path, writer='ffmpeg')

    # Mostra l'animazione
    plt.show()


def plot_trj2D_anim_MPC_t4d(N_cf_, obs_array, N_mpc, local_mpc_target, local_tgt4drone_array):
    # TODO: PER PRESENTAZIONE!

    filename_trj_ = '/home/matteo/catkin_ws/src/crazyCmd/scripts' \
                    '/Sperimentale/Swarm/MioPersonale/Preferiti' \
                    '/files_trj/trj.csv'

    filename_wopt_ = '/home/matteo/catkin_ws/src/crazyCmd/scripts' \
                     '/Sperimentale/Swarm/MioPersonale/Preferiti' \
                     '/files_trj/trj_wopt.csv'

    # Inizializzazione Traiettorie Sperimentali

    x_coords = []
    y_coords = []
    time_coords = []

    # Leggi il file CSV
    with open(filename_trj_, 'r') as csvfile:
        reader = csv.reader(csvfile, delimiter=',')
        for row in reader:
            time_coords.append(float(row[0]))
            x_coords.append(float(row[1]))
            y_coords.append(float(row[2]))

    time_coords = time_coords[0::N_cf_]

    x_4drone = []
    y_4drone = []

    for i in range(N_cf_):
        x_4drone.append(x_coords[i::N_cf_])
        y_4drone.append(y_coords[i::N_cf_])

    # MODALITA' TARGET 4 DRONE
    final_pos_x = []
    final_pos_y = []

    for i in range(N_cf_):
        final_pos_x.append(x_4drone[i][len(time_coords) - 1])
        final_pos_y.append(y_4drone[i][len(time_coords) - 1])

    # Inizializzazione Traiettorie Sperimentali - END

    # Inizializzazione Traiettorie MPC

    # Leggi il file CSV
    with open(filename_wopt_, 'r') as csvfile:
        reader = csv.reader(csvfile, delimiter=',')

        # Leggi le colonne dal file
        x_tot_ = []
        y_tot_ = []

        for riga in reader:
            x_tot_.append(float(riga[0]))
            y_tot_.append(float(riga[1]))

    # carico dati in lines
    x_tot_drone_tot_iteration = []
    y_tot_drone_tot_iteration = []

    for i in range(N_cf_):
        x_tot_drone_tot_iteration.append(x_tot_[i::N_cf_])
        y_tot_drone_tot_iteration.append(y_tot_[i::N_cf_])

    x_4drone_totiteration = []
    y_4drone_totiteration = []

    num_iter = len(x_tot_drone_tot_iteration[0]) / (N_mpc + 1)
    num_iter = int(num_iter)

    for drone in x_tot_drone_tot_iteration:
        for i in range(num_iter):
            x_4drone_totiteration.append(drone[i * (N_mpc + 1):(i + 1) * (N_mpc + 1)])

    for drone in y_tot_drone_tot_iteration:
        for i in range(num_iter):
            y_4drone_totiteration.append(drone[i * (N_mpc + 1):(i + 1) * (N_mpc + 1)])

    # Inizializzazione Traiettorie MPC - END

    # Crea la figura e gli assi
    fig, ax = plt.subplots()

    # PLOTTO OGGETTI STATICI

    # target
    tgt_x = local_mpc_target.desired_position.x
    tgt_y = local_mpc_target.desired_position.y

    # MODALITA' TARGET CENTER OF MASS
    # calcolo cm finale
    cm_fin_x, cm_fin_y = 0.0, 0.0
    for i in range(N_cf_):
        cm_fin_x += x_4drone[i][len(time_coords) - 1]
        cm_fin_y += y_4drone[i][len(time_coords) - 1]
    cm_fin_x /= N_cf_
    cm_fin_y /= N_cf_

    # MODALITA' TARGET 4 DRONE
    for i in range(N_cf_):
        plt.plot(local_tgt4drone_array[i].x, local_tgt4drone_array[i].y, 'x', label=f'tgt_cf{i + 1}')

    # MODALITA' TARGET 4 DRONE
    # Plotto target threshold
    gap_end_loop = 0.15
    for i in range(N_cf_):
        tgt_ts = Circle((local_tgt4drone_array[i].x, local_tgt4drone_array[i].y), gap_end_loop, edgecolor='black',
                        alpha=0.3, linestyle='dashed', facecolor='none')
        plt.gca().add_patch(tgt_ts)
    plt.gca().add_patch(Circle((0, 0), 0, edgecolor='black', alpha=0.3, linestyle='dashed', facecolor='none',
                               label='tgt_threshold'))  # etichetta una tantum

    # Plotto traiettorie
    # for i in range(N_cf_):
    #     plt.plot(x_4drone[i], y_4drone[i], '-', lw=1, linestyle='dashed', color='black', alpha=0.3)

    # Plotto ostacoli
    for i in range(len(obs_array)):
        cx, cy, r = obs_array[i].x, obs_array[i].y, obs_array[i].r
        obs = Circle((cx, cy), r, edgecolor='black', facecolor='gray')
        plt.gca().add_patch(obs)
    # plt.gca().add_patch(Circle((0, 0), 0, edgecolor='black', facecolor='gray', label='obs'))  # etichetta

    # PLOTTO OGGETTI STATICI - END

    # +++++++++++++++++++++++++ ANIMATION +++++++++++++++++++++++++++++++++++++++

    # Inizializzo traiettorie
    lines = []
    for i in range(N_cf_):
        if i == 0:
            line, = ax.plot([], [], '-o', color='blue', lw=3, markersize=1,
                            label=f'trj_cf{i + 1}')  # creo istanza Line3D
        if i == 1:
            line, = ax.plot([], [], '-o', color='orange', lw=3, markersize=1,
                            label=f'trj_cf{i + 1}')  # creo istanza Line3D
        if i == 2:
            line, = ax.plot([], [], '-o', color='green', lw=3, markersize=1,
                            label=f'trj_cf{i + 1}')  # creo istanza Line3D
        if i == 3:
            line, = ax.plot([], [], '-o', color='red', lw=3, markersize=1,
                            label=f'trj_cf{i + 1}')  # creo istanza Line3D
        lines.append(line)

    lines_mpc = []
    for i in range(N_cf_):
        if i == 0:
            line, = ax.plot([], [], '-o', color='blue', lw=2, markersize=3, linestyle='dashed')
        if i == 1:
            line, = ax.plot([], [], '-o', color='orange', lw=2, markersize=3, linestyle='dashed')
        if i == 2:
            line, = ax.plot([], [], '-o', color='green', lw=2, markersize=3, linestyle='dashed')
        if i == 3:
            line, = ax.plot([], [], '-o', color='red', lw=2, markersize=3, linestyle='dashed')
        lines_mpc.append(line)

    # Inizializzo il testo del tempo
    time_text = ax.text(0.02, 0.95, '', transform=ax.transAxes)

    def save_frame(frame):
        fig_arg = "plot_trj2D_anim_MPC_t4d"
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        filename = f"{timestamp}_{fig_arg}_{frame:04d}.png"
        full_path = os.path.join(folder, filename)
        # Salva il frame corrente come immagine
        fig.savefig(full_path)

    # initialization function: plot the background of each frame
    def init():
        for ii in range(N_cf_):
            lines[ii].set_data([], [])

        for i in range(N_cf_):
            lines_mpc[i].set_data([], [])

        return lines + lines_mpc + [time_text]

    # animation function: this is called sequentially
    def update(frame):

        for ii in range(N_cf_):
            # visualizza scia
            scia = frame - 30
            if scia < 0:
                scia = 0
            lines[ii].set_data(x_4drone[ii][scia:frame], y_4drone[ii][scia:frame])
            # lines[ii].set_data(x_4drone[ii][:frame], y_4drone[ii][:frame])
            time_text.set_text('Time: {:.2f} [s]'.format(time_coords[frame]))  # Aggiorna il testo del tempo
            for j in range(num_iter):
                if x_4drone_totiteration[ii * num_iter + j][0] == x_4drone[ii][frame] and \
                        y_4drone_totiteration[ii * num_iter + j][0] == y_4drone[ii][frame]:
                    lines_mpc[ii].set_data(x_4drone_totiteration[ii * num_iter + j],
                                           y_4drone_totiteration[ii * num_iter + j])
                    save_frame(frame)

        return lines + lines_mpc + [time_text]

    # call the animator. blit=True means only re-draw the parts that have changed.
    anim = animation.FuncAnimation(fig, update, frames=len(time_coords), init_func=init,
                                   blit=True, interval=50)
    # Animation - END

    # Formattazione
    plt.xlabel('X [m]')
    plt.ylabel('Y [m]')
    plt.xlim(-0.5, 3.0)
    plt.ylim(-3.0, 0.5)
    plt.legend()
    plt.grid(True)
    plt.title("Trajectory - Animation")
    plt.legend(loc='upper right', bbox_to_anchor=(1.1, 1.15))

    # salvo animazione
    fig_arg = "plot_trj2D_anim_MPC_t4d.mp4"
    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    filename = f"{timestamp}_{fig_arg}"
    full_path = os.path.join(folder, filename)
    anim.save(full_path, writer='ffmpeg')

    # Mostra l'animazione
    # plt.show()


def plot_trj2D_anim_xz_MPC_t4d(N_cf_, obs_array, N_mpc, local_mpc_target, local_tgt4drone_array):
    # TODO: PER PRESENTAZIONE!

    filename_trj_ = '/home/matteo/catkin_ws/src/crazyCmd/scripts' \
                    '/Sperimentale/Swarm/MioPersonale/Preferiti' \
                    '/files_trj/trj.csv'

    filename_wopt_ = '/home/matteo/catkin_ws/src/crazyCmd/scripts' \
                     '/Sperimentale/Swarm/MioPersonale/Preferiti' \
                     '/files_trj/trj_wopt.csv'

    # Inizializzazione Traiettorie Sperimentali

    x_coords = []
    z_coords = []
    time_coords = []

    # Leggi il file CSV
    with open(filename_trj_, 'r') as csvfile:
        reader = csv.reader(csvfile, delimiter=',')
        for row in reader:
            time_coords.append(float(row[0]))
            x_coords.append(float(row[1]))
            z_coords.append(float(row[3]))

    time_coords = time_coords[0::N_cf_]

    x_4drone = []
    z_4drone = []

    for i in range(N_cf_):
        x_4drone.append(x_coords[i::N_cf_])
        z_4drone.append(z_coords[i::N_cf_])

    # MODALITA' TARGET 4 DRONE
    final_pos_x = []
    final_pos_z = []

    for i in range(N_cf_):
        final_pos_x.append(x_4drone[i][len(time_coords) - 1])
        final_pos_z.append(z_4drone[i][len(time_coords) - 1])

    # Inizializzazione Traiettorie Sperimentali - END

    # Inizializzazione Traiettorie MPC

    # Leggi il file CSV
    with open(filename_wopt_, 'r') as csvfile:
        reader = csv.reader(csvfile, delimiter=',')

        # Leggi le colonne dal file
        x_tot_ = []
        z_tot_ = []

        for riga in reader:
            x_tot_.append(float(riga[0]))
            z_tot_.append(float(riga[2]))

    # carico dati in lines
    x_tot_drone_tot_iteration = []
    z_tot_drone_tot_iteration = []

    for i in range(N_cf_):
        x_tot_drone_tot_iteration.append(x_tot_[i::N_cf_])
        z_tot_drone_tot_iteration.append(z_tot_[i::N_cf_])

    x_4drone_totiteration = []
    z_4drone_totiteration = []

    num_iter = len(x_tot_drone_tot_iteration[0]) / (N_mpc + 1)
    num_iter = int(num_iter)

    for drone in x_tot_drone_tot_iteration:
        for i in range(num_iter):
            x_4drone_totiteration.append(drone[i * (N_mpc + 1):(i + 1) * (N_mpc + 1)])

    for drone in z_tot_drone_tot_iteration:
        for i in range(num_iter):
            z_4drone_totiteration.append(drone[i * (N_mpc + 1):(i + 1) * (N_mpc + 1)])

    # Inizializzazione Traiettorie MPC - END

    # Crea la figura e gli assi
    fig, ax = plt.subplots()

    # PLOTTO OGGETTI STATICI

    # target
    tgt_x = local_mpc_target.desired_position.x
    tgt_z = local_mpc_target.desired_position.z

    # MODALITA' TARGET CENTER OF MASS
    # calcolo cm finale
    cm_fin_x, cm_fin_z = 0.0, 0.0
    for i in range(N_cf_):
        cm_fin_x += x_4drone[i][len(time_coords) - 1]
        cm_fin_z += z_4drone[i][len(time_coords) - 1]
    cm_fin_x /= N_cf_
    cm_fin_z /= N_cf_

    # MODALITA' TARGET 4 DRONE
    for i in range(N_cf_):
        plt.plot(local_tgt4drone_array[i].x, local_tgt4drone_array[i].z, 'x', label=f'tgt_cf{i + 1}')

    # MODALITA' TARGET 4 DRONE
    # Plotto target threshold
    gap_end_loop = 0.15
    for i in range(N_cf_):
        tgt_ts = Circle((local_tgt4drone_array[i].x, local_tgt4drone_array[i].z), gap_end_loop, edgecolor='black',
                        alpha=0.3, linestyle='dashed', facecolor='none')
        plt.gca().add_patch(tgt_ts)
    plt.gca().add_patch(Circle((0, 0), 0, edgecolor='black', alpha=0.3, linestyle='dashed', facecolor='none',
                               label='tgt_threshold'))  # etichetta una tantum

    # Plotto traiettorie
    # for i in range(N_cf_):
    #     plt.plot(x_4drone[i], y_4drone[i], '-', lw=1, linestyle='dashed', color='black', alpha=0.3)

    # Plotto ostacoli
    for i in range(len(obs_array)):
        cx, cz, r = obs_array[i].x, obs_array[i].z, obs_array[i].r
        obs = Circle((cx, cz), r, edgecolor='black', facecolor='gray')
        plt.gca().add_patch(obs)
    # plt.gca().add_patch(Circle((0, 0), 0, edgecolor='black', facecolor='gray', label='obs'))  # etichetta

    # PLOTTO OGGETTI STATICI - END

    # +++++++++++++++++++++++++ ANIMATION +++++++++++++++++++++++++++++++++++++++

    # Inizializzo traiettorie
    lines = []
    for i in range(N_cf_):
        if i == 0:
            line, = ax.plot([], [], '-o', color='blue', lw=3, markersize=1,
                            label=f'trj_cf{i + 1}')  # creo istanza Line3D
        if i == 1:
            line, = ax.plot([], [], '-o', color='orange', lw=3, markersize=1,
                            label=f'trj_cf{i + 1}')  # creo istanza Line3D
        lines.append(line)

    lines_mpc = []
    for i in range(N_cf_):
        # for j in range(num_iter):
        line, = ax.plot([], [], '-o', lw=2, markersize=3, linestyle='dashed')  # creo istanza Line3D
        lines_mpc.append(line)

    # Inizializzo il testo del tempo
    time_text = ax.text(0.02, 0.95, '', transform=ax.transAxes)

    def save_frame(frame):
        fig_arg = "plot_trj2D_anim_xy_MPC_t4d"
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        filename = f"{timestamp}_{fig_arg}_{frame:04d}.png"
        full_path = os.path.join(folder, filename)
        # Salva il frame corrente come immagine
        fig.savefig(full_path)

    # initialization function: plot the background of each frame
    def init():
        for ii in range(N_cf_):
            lines[ii].set_data([], [])

        for i in range(N_cf_):
            lines_mpc[i].set_data([], [])

        return lines + lines_mpc + [time_text]

    # animation function: this is called sequentially
    def update(frame):

        for ii in range(N_cf_):
            # visualizza scia
            # scia = frame - 30
            # if scia < 0:
            #     scia = 0
            # lines[ii].set_data(x_4drone[ii][scia:frame], y_4drone[ii][scia:frame])
            lines[ii].set_data(x_4drone[ii][:frame], z_4drone[ii][:frame])
            time_text.set_text('Time: {:.2f} [s]'.format(time_coords[frame]))  # Aggiorna il testo del tempo
            for j in range(num_iter):
                if x_4drone_totiteration[ii * num_iter + j][0] == x_4drone[ii][frame] and \
                        z_4drone_totiteration[ii * num_iter + j][0] == z_4drone[ii][frame]:
                    lines_mpc[ii].set_data(x_4drone_totiteration[ii * num_iter + j],
                                           z_4drone_totiteration[ii * num_iter + j])
                    save_frame(frame)

        return lines + lines_mpc + [time_text]

    # call the animator. blit=True means only re-draw the parts that have changed.
    anim = animation.FuncAnimation(fig, update, frames=len(time_coords), init_func=init,
                                   blit=True, interval=50)
    # Animation - END

    # Formattazione
    plt.xlabel('X [m]')
    plt.ylabel('Z [m]')
    plt.xlim(-1.2, 3.6)
    plt.ylim(0.0, 3.0)
    plt.legend()
    plt.grid(True)
    plt.title("Trajectory X-Z - Animation")
    plt.legend()

    # salvo animazione
    fig_arg = "plot_trj2D_anim_xz_MPC_t4d.mp4"
    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    filename = f"{timestamp}_{fig_arg}"
    full_path = os.path.join(folder, filename)
    anim.save(full_path, writer='ffmpeg')

    # Mostra l'animazione
    # plt.show()
    # TODO: salvare animazione


def plot_cm_xyz(N_cf_, start_takeoff, finish_takeoff, start_landing, finish_landing, local_mpc_target):
    filename_trj_ = '/home/matteo/catkin_ws/src/crazyCmd/scripts' \
                    '/Sperimentale/Swarm/MioPersonale/Preferiti' \
                    '/files_trj/trj.csv'

    # Inizializza le liste per le coordinate x, y, z
    x_coords = []
    y_coords = []
    z_coords = []
    time_coords = []

    # Leggi il file CSV
    with open(filename_trj_, 'r') as csvfile:
        reader = csv.reader(csvfile, delimiter=',')
        for row in reader:
            time_coords.append(float(row[0]))
            x_coords.append(float(row[1]))
            y_coords.append(float(row[2]))
            z_coords.append(float(row[3]))

    # num_time_step = int(len(x_coords) / N_cf_)
    time_coords = time_coords[0::N_cf_]

    x_cm = []
    y_cm = []
    z_cm = []

    # calcolo centro di massa vs time
    for i in range(len(time_coords)):
        x_cm.append(sum(x_coords[i * N_cf_:(i + 1) * N_cf_]) / N_cf_)
        y_cm.append(sum(y_coords[i * N_cf_:(i + 1) * N_cf_]) / N_cf_)
        z_cm.append(sum(z_coords[i * N_cf_:(i + 1) * N_cf_]) / N_cf_)

    # Genera il grafico

    plt.figure()

    # plt.plot(time_coords, x_cm, 'r-o', lw=1, markersize=1, label='x_cm')
    # plt.plot(time_coords, y_cm, 'b-o', lw=1, markersize=1, label='y_cm')
    # plt.plot(time_coords, z_cm, 'g-o', lw=1, markersize=1, label='z_cm')
    plt.plot(time_coords, x_cm, 'r-o', lw=1, markersize=1, label='x_cm')
    plt.plot(time_coords, y_cm, 'b-o', lw=1, markersize=1, label='y_cm')
    plt.plot(time_coords, z_cm, 'g-o', lw=1, markersize=1, label='z_cm')

    # Aggiungi linee orizzontali tratteggiate per target
    x_tgt_cm = local_mpc_target.desired_position.x
    y_tgt_cm = local_mpc_target.desired_position.y
    z_tgt_cm = local_mpc_target.desired_position.z

    # plt.axhline(y=x_tgt_cm, color='r', linestyle='dashed', lw=1, label='x_tgt')
    # plt.axhline(y=y_tgt_cm, color='b', linestyle='dashed', lw=1, label='y_tgt')
    # plt.axhline(y=z_tgt_cm, color='g', linestyle='dashed', lw=1, label='z_tgt')
    plt.axhline(y=x_tgt_cm, color='r', linestyle='dashed', lw=1, label='x_tgt_cm')
    plt.axhline(y=y_tgt_cm, color='b', linestyle='dashed', lw=1, label='y_tgt_cm')
    plt.axhline(y=z_tgt_cm, color='g', linestyle='dashed', lw=1, label='z_tgtcm')

    # Aggiungi linee verticali per decollo/atterraggio
    plt.axvline(x=start_takeoff, color='black', lw=1)
    plt.axvline(x=finish_takeoff, color='black', lw=1)
    plt.axvline(x=start_landing, color='black', lw=1)
    plt.axvline(x=finish_landing, color='black', lw=1)

    # Colora l'area compresa tra le due linee verticali di grigio
    plt.axvspan(start_takeoff, finish_takeoff, color='red', alpha=0.3, label='takeoff/landing')
    plt.axvspan(start_landing, finish_landing, color='red', alpha=0.3)

    # Imposta le etichette degli assi
    plt.xlabel('Time [s]')
    # plt.ylabel('Position Center Of Mass [m]')
    plt.ylabel('Position [m]')

    # plt.title('Position in relation to the Target')
    plt.title('Position in relation to the Center of Mass Target')

    # Imposta legenda
    plt.legend()

    # Imposta il limite inferiore dell'ascissa a zero
    plt.xlim(start_takeoff, finish_landing)

    # salvo immagine
    fig_arg = "plot_cm_xyz"
    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    filename = f"{timestamp}_{fig_arg}"
    full_path = os.path.join(folder, filename)
    plt.savefig(full_path)

    plt.show()


def plot_distance_tgtcm(N_cf_, start_takeoff, finish_takeoff, start_landing, finish_landing, local_mpc_target):
    filename_trj_ = '/home/matteo/catkin_ws/src/crazyCmd/scripts' \
                    '/Sperimentale/Swarm/MioPersonale/Preferiti' \
                    '/files_trj/trj.csv'

    # Inizializza le liste per le coordinate x, y, z
    x_coords = []
    y_coords = []
    z_coords = []
    time_coords = []

    # Leggi il file CSV
    with open(filename_trj_, 'r') as csvfile:
        reader = csv.reader(csvfile, delimiter=',')
        for row in reader:
            time_coords.append(float(row[0]))
            x_coords.append(float(row[1]))
            y_coords.append(float(row[2]))
            z_coords.append(float(row[3]))

    # num_time_step = int(len(x_coords) / N_cf_)
    time_coords = time_coords[0::N_cf_]

    # calcolo centro di massa vs time

    x_cm = []
    y_cm = []
    z_cm = []

    for i in range(len(time_coords)):
        x_cm.append(sum(x_coords[i * N_cf_:(i + 1) * N_cf_]) / N_cf_)
        y_cm.append(sum(y_coords[i * N_cf_:(i + 1) * N_cf_]) / N_cf_)
        z_cm.append(sum(z_coords[i * N_cf_:(i + 1) * N_cf_]) / N_cf_)

    # acquisisco target tgt_cm
    x_tgt_cm = local_mpc_target.desired_position.x
    y_tgt_cm = local_mpc_target.desired_position.y
    z_tgt_cm = local_mpc_target.desired_position.z

    # calcolo distanza swarm_cm - tgt_cm
    distance = []

    for i in range(len(time_coords)):
        distance.append(math.sqrt((x_cm[i] - x_tgt_cm) ** 2 + (y_cm[i] - y_tgt_cm) ** 2 + (z_cm[i] - z_tgt_cm) ** 2))

    # Genera il grafico distanza swarm_cm - tgt_cm
    plt.plot(time_coords, distance, 'r-o', lw=2, markersize=1, label='d(cm,tgt_cm)')

    # calcolo distanza drone - tgt_cm

    distance_4drone_tot = [[] for i in range(N_cf_)]

    for i in range(N_cf_):
        distance_4drone_tot[i] = []

    for i in range(N_cf_):
        x_4drone = []
        y_4drone = []
        z_4drone = []

        x_4drone.append(x_coords[i::N_cf_])
        y_4drone.append(y_coords[i::N_cf_])
        z_4drone.append(z_coords[i::N_cf_])

        for ii in range(len(time_coords)):
            distance_4drone_tot[i].append(math.sqrt((x_4drone[0][ii] - x_tgt_cm) ** 2 +
                                                    (y_4drone[0][ii] - y_tgt_cm) ** 2 +
                                                    (z_4drone[0][ii] - z_tgt_cm) ** 2))
        # Genera il grafico distanza drone - tgt_cm
        plt.plot(time_coords, distance_4drone_tot[i], '-o', lw=1, markersize=1, label=f'd(cf{i + 1},tgt_cm)')

    # Aggiungi linee orizzontali tratteggiate per target
    plt.axhline(y=0, color='r', linestyle='dashed', lw=1, label='tgt')

    # Aggiungi linee verticali per decollo/atterraggio
    plt.axvline(x=start_takeoff, color='black', lw=1)
    plt.axvline(x=finish_takeoff, color='black', lw=1)
    plt.axvline(x=start_landing, color='black', lw=1)
    plt.axvline(x=finish_landing, color='black', lw=1)

    # Colora l'area compresa tra le due linee verticali di grigio
    plt.axvspan(start_takeoff, finish_takeoff, color='red', alpha=0.3, label='takeoff/landing')
    plt.axvspan(start_landing, finish_landing, color='red', alpha=0.3)

    # Imposta le etichette degli assi
    plt.xlabel('Time [s]')
    plt.ylabel('Distance [m]')

    # plt.title('Distance from Target')
    plt.title('Distance from Center of Mass Target')

    # Imposta legenda
    plt.legend()

    # Imposta il limite inferiore dell'ascissa a zero
    plt.xlim(start_takeoff, finish_landing)
    plt.ylim(-1)

    # salvo immagine
    fig_arg = "plot_distance_tgtcm"
    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    filename = f"{timestamp}_{fig_arg}"
    full_path = os.path.join(folder, filename)
    plt.savefig(full_path)

    plt.show()


def plot_distance_tgt4d(N_cf_, start_takeoff, finish_takeoff, start_landing, finish_landing, tgt4drone_array):
    filename_trj_ = '/home/matteo/catkin_ws/src/crazyCmd/scripts' \
                    '/Sperimentale/Swarm/MioPersonale/Preferiti' \
                    '/files_trj/trj.csv'

    # Inizializza le liste per le coordinate x, y, z
    x_coords = []
    y_coords = []
    z_coords = []
    time_coords = []

    # Leggi il file CSV
    with open(filename_trj_, 'r') as csvfile:
        reader = csv.reader(csvfile, delimiter=',')
        for row in reader:
            time_coords.append(float(row[0]))
            x_coords.append(float(row[1]))
            y_coords.append(float(row[2]))
            z_coords.append(float(row[3]))

    # num_time_step = int(len(x_coords) / N_cf_)
    time_coords = time_coords[0::N_cf_]

    x_4drone = []
    y_4drone = []
    z_4drone = []

    for i in range(N_cf_):
        x_4drone.append(x_coords[i::N_cf_])
        y_4drone.append(y_coords[i::N_cf_])
        z_4drone.append(z_coords[i::N_cf_])

    # calcolo distanza cfx - tgt_cfx

    distance_4drone = [[] for i in range(N_cf_)]

    for i in range(N_cf_):
        distance_4drone[i] = []

    plt.figure()

    for i in range(N_cf_):
        for ii in range(len(time_coords)):
            distance_4drone[i].append(math.sqrt((x_4drone[i][ii] - tgt4drone_array[i].x) ** 2 +
                                                (y_4drone[i][ii] - tgt4drone_array[i].y) ** 2 +
                                                (z_4drone[i][ii] - tgt4drone_array[i].z) ** 2))
        # Genera il grafico distanza drone - tgt_4d
        plt.plot(time_coords, distance_4drone[i], '-o', lw=1, markersize=1, label=f'd(cf{i + 1},tgt_cf{i + 1})')

    # Aggiungi linee orizzontali tratteggiate per target
    plt.axhline(y=0, color='r', linestyle='dashed', lw=1, label='tgt')

    # Aggiungi linee verticali per decollo/atterraggio
    plt.axvline(x=start_takeoff, color='black', lw=1)
    plt.axvline(x=finish_takeoff, color='black', lw=1)
    plt.axvline(x=start_landing, color='black', lw=1)
    plt.axvline(x=finish_landing, color='black', lw=1)

    # Colora l'area compresa tra le due linee verticali di grigio
    plt.axvspan(start_takeoff, finish_takeoff, color='red', alpha=0.3, label='takeoff/landing')
    plt.axvspan(start_landing, finish_landing, color='red', alpha=0.3)

    # Imposta le etichette degli assi
    plt.xlabel('Time [s]')
    plt.ylabel('Distance from Target [m]')

    # Imposta legenda
    plt.legend()

    # Imposta il limite inferiore dell'ascissa a zero
    plt.xlim(start_takeoff, finish_landing)
    plt.ylim(-1)

    plt.title('Distance of each Drone from Individual Target')

    # salvo immagine
    fig_arg = "plot_distance_tgt4d"
    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    filename = f"{timestamp}_{fig_arg}"
    full_path = os.path.join(folder, filename)
    plt.savefig(full_path)

    plt.show()


def plot_distance_ij(N_cf_, start_takeoff, finish_takeoff, start_landing, finish_landing, d_ref, d_emer, d_coll):
    filename_trj_ = '/home/matteo/catkin_ws/src/crazyCmd/scripts' \
                    '/Sperimentale/Swarm/MioPersonale/Preferiti' \
                    '/files_trj/trj.csv'

    # Inizializza le liste per le coordinate x, y, z
    x_coords = []
    y_coords = []
    z_coords = []
    time_coords = []

    # Leggi il file CSV
    with open(filename_trj_, 'r') as csvfile:
        reader = csv.reader(csvfile, delimiter=',')
        for row in reader:
            time_coords.append(float(row[0]))
            x_coords.append(float(row[1]))
            y_coords.append(float(row[2]))
            z_coords.append(float(row[3]))

    time_coords = time_coords[0::N_cf_]

    x_4drone = []
    y_4drone = []
    z_4drone = []

    for i in range(N_cf_):
        x_4drone.append(x_coords[i::N_cf_])
        y_4drone.append(y_coords[i::N_cf_])
        z_4drone.append(z_coords[i::N_cf_])

    distances = {}
    for i in range(N_cf_):
        for j in range(i + 1, N_cf_):
            distances[(i, j)] = [0.0] * len(time_coords)

    for i in range(N_cf_):
        for j in range(i + 1, N_cf_):
            for k in range(len(time_coords)):
                distance = math.sqrt(
                    (x_4drone[i][k] - x_4drone[j][k]) ** 2 + (y_4drone[i][k] - y_4drone[j][k]) ** 2 + (
                            z_4drone[i][k] - z_4drone[j][k]) ** 2)
                distances[(i, j)][k] = distance

    for i in range(N_cf_):
        for j in range(i + 1, N_cf_):
            plt.plot(time_coords, distances[(i, j)], '-o', lw=1, markersize=1, label=f'd(cf{i + 1},cf{j + 1})')

    # Aggiungi linee orizzontali tratteggiate per d_ref
    # plt.axhline(y=d_ref, color='green', linestyle='dashed', lw=1, label='d_ref')
    plt.axhline(y=d_emer, color='orange', linestyle='dashed', lw=1, label='d_safe')
    plt.axhline(y=d_coll, color='red', linestyle='dashed', lw=1, label='d_coll')

    # Aggiungi linee verticali per decollo/atterraggio
    plt.axvline(x=start_takeoff, color='black', lw=1)
    plt.axvline(x=finish_takeoff, color='black', lw=1)
    plt.axvline(x=start_landing, color='black', lw=1)
    plt.axvline(x=finish_landing, color='black', lw=1)

    # Colora l'area compresa tra le due linee verticali di grigio
    plt.axvspan(start_takeoff, finish_takeoff, color='red', alpha=0.3, label='takeoff/landing')
    plt.axvspan(start_landing, finish_landing, color='red', alpha=0.3)

    # Imposta le etichette degli assi
    plt.xlabel('Time [s]')
    plt.ylabel('Inter-Agent Distance [m]')

    # Imposta legenda
    plt.legend()

    plt.title('Distance Between Drones')

    # Imposta il limite inferiore dell'ascissa a zero
    plt.xlim(start_takeoff, finish_landing)

    # salvo immagine
    fig_arg = "plot_distance_ij"
    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    filename = f"{timestamp}_{fig_arg}"
    full_path = os.path.join(folder, filename)
    plt.savefig(full_path)

    plt.show()


def plot_distance_maxminavg(N_cf_, start_takeoff, finish_takeoff, start_landing, finish_landing, d_ref, d_emer, d_coll):
    filename_trj_ = '/home/matteo/catkin_ws/src/crazyCmd/scripts' \
                    '/Sperimentale/Swarm/MioPersonale/Preferiti' \
                    '/files_trj/trj.csv'

    # Inizializza le liste per le coordinate x, y, z
    x_coords = []
    y_coords = []
    z_coords = []
    time_coords = []

    # Leggi il file CSV
    with open(filename_trj_, 'r') as csvfile:
        reader = csv.reader(csvfile, delimiter=',')
        for row in reader:
            time_coords.append(float(row[0]))
            x_coords.append(float(row[1]))
            y_coords.append(float(row[2]))
            z_coords.append(float(row[3]))

    time_coords = time_coords[0::N_cf_]

    x_4drone = []
    y_4drone = []
    z_4drone = []

    for i in range(N_cf_):
        x_4drone.append(x_coords[i::N_cf_])
        y_4drone.append(y_coords[i::N_cf_])
        z_4drone.append(z_coords[i::N_cf_])

    distances = {}
    for i in range(N_cf_):
        for j in range(i + 1, N_cf_):
            distances[(i, j)] = [0.0] * len(time_coords)

    for i in range(N_cf_):
        for j in range(i + 1, N_cf_):
            for k in range(len(time_coords)):
                distance = math.sqrt(
                    (x_4drone[i][k] - x_4drone[j][k]) ** 2 + (y_4drone[i][k] - y_4drone[j][k]) ** 2 + (
                            z_4drone[i][k] - z_4drone[j][k]) ** 2)
                distances[(i, j)][k] = distance

    d_avg = [0.0] * len(time_coords)
    d_max = [-float('inf')] * len(time_coords)
    d_min = [float('inf')] * len(time_coords)

    for k in range(len(time_coords)):
        for i in range(N_cf_):
            for j in range(i + 1, N_cf_):
                d_avg[k] += distances[(i, j)][k]
                if distances[(i, j)][k] > d_max[k]:
                    d_max[k] = distances[(i, j)][k]
                if distances[(i, j)][k] < d_min[k]:
                    d_min[k] = distances[(i, j)][k]
        d_avg[k] /= len(distances)

    plt.plot(time_coords, d_avg, 'b-', lw=2, label='d_avg')
    plt.plot(time_coords, d_max, 'b-o', alpha=0.3, lw=1, markersize=1, label='d_max / d_min')
    plt.plot(time_coords, d_min, 'b-o', alpha=0.3, lw=1, markersize=1)
    plt.fill_between(time_coords, d_max, d_min, alpha=0.3)

    # Aggiungi linee orizzontali tratteggiate per d_ref
    plt.axhline(y=d_ref, color='green', linestyle='dashed', lw=1, label='d_ref')
    plt.axhline(y=d_emer, color='orange', linestyle='dashed', lw=1, label='d_emergency')
    plt.axhline(y=d_coll, color='red', linestyle='dashed', lw=1, label='d_collision')

    # Aggiungi linee verticali per decollo/atterraggio
    plt.axvline(x=start_takeoff, color='black', lw=1)
    plt.axvline(x=finish_takeoff, color='black', lw=1)
    plt.axvline(x=start_landing, color='black', lw=1)
    plt.axvline(x=finish_landing, color='black', lw=1)

    # Colora l'area compresa tra le due linee verticali di grigio
    plt.axvspan(start_takeoff, finish_takeoff, color='red', alpha=0.3, label='takeoff/landing')
    plt.axvspan(start_landing, finish_landing, color='red', alpha=0.3)

    # Imposta le etichette degli assi
    plt.xlabel('Time [s]')
    plt.ylabel('Inter-Agent Distance [m]')

    # Imposta legenda
    plt.legend()

    # Imposta il limite inferiore dell'ascissa a zero
    plt.xlim(start_takeoff, finish_landing)

    # salvo immagine
    fig_arg = "plot_distance_maxminavg"
    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    filename = f"{timestamp}_{fig_arg}"
    full_path = os.path.join(folder, filename)
    plt.savefig(full_path)

    plt.show()


def plot_distance_min(N_cf_, start_takeoff, finish_takeoff, start_landing, finish_landing, d_ref, d_emer, d_coll):
    filename_trj_ = '/home/matteo/catkin_ws/src/crazyCmd/scripts' \
                    '/Sperimentale/Swarm/MioPersonale/Preferiti' \
                    '/files_trj/trj.csv'

    # Inizializza le liste per le coordinate x, y, z
    x_coords = []
    y_coords = []
    z_coords = []
    time_coords = []

    # Leggi il file CSV
    with open(filename_trj_, 'r') as csvfile:
        reader = csv.reader(csvfile, delimiter=',')
        for row in reader:
            time_coords.append(float(row[0]))
            x_coords.append(float(row[1]))
            y_coords.append(float(row[2]))
            z_coords.append(float(row[3]))

    time_coords = time_coords[0::N_cf_]

    x_4drone = []
    y_4drone = []
    z_4drone = []

    for i in range(N_cf_):
        x_4drone.append(x_coords[i::N_cf_])
        y_4drone.append(y_coords[i::N_cf_])
        z_4drone.append(z_coords[i::N_cf_])

    distances = {}
    for i in range(N_cf_):
        for j in range(i + 1, N_cf_):
            distances[(i, j)] = [0.0] * len(time_coords)

    for i in range(N_cf_):
        for j in range(i + 1, N_cf_):
            for k in range(len(time_coords)):
                distance = math.sqrt(
                    (x_4drone[i][k] - x_4drone[j][k]) ** 2 + (y_4drone[i][k] - y_4drone[j][k]) ** 2 + (
                            z_4drone[i][k] - z_4drone[j][k]) ** 2)
                distances[(i, j)][k] = distance

    d_min = [float('inf')] * len(time_coords)

    for k in range(len(time_coords)):
        for i in range(N_cf_):
            for j in range(i + 1, N_cf_):
                if distances[(i, j)][k] < d_min[k]:
                    d_min[k] = distances[(i, j)][k]

    # plotto grafico
    plt.plot(time_coords, d_min, 'b-o', alpha=0.3, lw=2, markersize=1, label='d_min')

    # Aggiungi linee orizzontali tratteggiate per d_ref
    plt.axhline(y=d_ref, color='green', linestyle='dashed', lw=1, label='d_ref')
    plt.axhline(y=d_emer, color='orange', linestyle='dashed', lw=1, label='d_emergency')
    plt.axhline(y=d_coll, color='red', linestyle='dashed', lw=1, label='d_collision')

    y_min, y_max = plt.ylim()  # Ottieni i limiti correnti dell'asse y

    # Colora aree
    plt.axhspan(d_emer, y_max, color='green', alpha=0.3, label='OK')
    plt.axhspan(d_coll, d_emer, color='orange', alpha=0.3, label='emergency')
    plt.axhspan(-y_min, d_coll, color='red', alpha=0.3, label='collision')

    # Aggiungi linee verticali per decollo/atterraggio
    plt.axvline(x=start_takeoff, color='black', lw=1)
    plt.axvline(x=finish_takeoff, color='black', lw=1)
    plt.axvline(x=start_landing, color='black', lw=1)
    plt.axvline(x=finish_landing, color='black', lw=1)

    # Colora l'area compresa tra le due linee verticali di grigio
    plt.axvspan(start_takeoff, finish_takeoff, color='red', alpha=0.3, label='takeoff/landing')
    plt.axvspan(start_landing, finish_landing, color='red', alpha=0.3)

    # Imposta le etichette degli assi
    plt.xlabel('Time [s]')
    plt.ylabel('Inter-Agent Distance [m]')

    # Imposta legenda
    plt.legend()

    # Imposta limiti
    plt.xlim(start_takeoff, finish_landing)
    plt.ylim(y_min, y_max)

    # salvo immagine
    fig_arg = "plot_distance_min"
    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    filename = f"{timestamp}_{fig_arg}"
    full_path = os.path.join(folder, filename)
    plt.savefig(full_path)

    plt.show()


def plot_vel4drone(N_cf_, start_takeoff, finish_takeoff, start_landing, finish_landing, v_lim):
    filename_trj_ = '/home/matteo/catkin_ws/src/crazyCmd/scripts' \
                    '/Sperimentale/Swarm/MioPersonale/Preferiti' \
                    '/files_trj/trj.csv'

    # Inizializza le liste per le coordinate x, y, z
    vx_coords = []
    vy_coords = []
    vz_coords = []
    time_coords = []

    # Leggi il file CSV
    with open(filename_trj_, 'r') as csvfile:
        reader = csv.reader(csvfile, delimiter=',')
        for row in reader:
            time_coords.append(float(row[0]))
            vx_coords.append(float(row[4]))
            vy_coords.append(float(row[5]))
            vz_coords.append(float(row[6]))

    time_coords = time_coords[0::N_cf_]

    vx_4drone = []
    vy_4drone = []
    vz_4drone = []

    for i in range(N_cf_):
        vx_4drone.append(vx_coords[i::N_cf_])
        vy_4drone.append(vy_coords[i::N_cf_])
        vz_4drone.append(vz_coords[i::N_cf_])

    vtot_4drone = []

    for i in range(N_cf_):
        # for k in range(len(time_coords)):
        vtot_4drone.append([0.0] * len(time_coords))

    for i in range(N_cf_):
        for k in range(len(time_coords)):
            vtot_4drone[i][k] = math.sqrt(vx_4drone[i][k] ** 2 + vy_4drone[i][k] ** 2 + vz_4drone[i][k] ** 2)

    for i in range(N_cf_):
        plt.plot(time_coords, vtot_4drone[i], '-o', lw=1, markersize=3, label=f'v_cf{i + 1}')

    # v_ref = 0.35
    # v_lim = 0.20
    v_lim_tot = math.sqrt(v_lim ** 2 + v_lim ** 2 + v_lim ** 2)

    # Aggiungi linee orizzontali
    # plt.axhline(y=v_ref, color='green', linestyle='dashed', lw=1, label='v_ref')
    plt.axhline(y=v_lim_tot, color='red', linestyle='dashed', lw=1, label='v_lim')

    # Aggiungi linee verticali per decollo/atterraggio
    plt.axvline(x=start_takeoff, color='black', lw=1)
    plt.axvline(x=finish_takeoff, color='black', lw=1)
    plt.axvline(x=start_landing, color='black', lw=1)
    plt.axvline(x=finish_landing, color='black', lw=1)

    # Colora l'area compresa tra le due linee verticali di grigio
    plt.axvspan(start_takeoff, finish_takeoff, color='red', alpha=0.3, label='takeoff/landing')
    plt.axvspan(start_landing, finish_landing, color='red', alpha=0.3)

    # Imposta le etichette degli assi
    plt.xlabel('Time [s]')
    plt.ylabel('Velocity [m/s]')

    plt.title('Velocity')

    # Imposta legenda
    plt.legend()

    # Imposta il limite inferiore dell'ascissa a zero
    plt.xlim(start_takeoff, finish_landing)

    # salvo immagine
    fig_arg = "plot_vel4drone"
    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    filename = f"{timestamp}_{fig_arg}"
    full_path = os.path.join(folder, filename)
    plt.savefig(full_path)

    plt.show()


def plot_vel_maxminavg(N_cf_, start_takeoff, finish_takeoff, start_landing, finish_landing, v_lim):
    filename_trj_ = '/home/matteo/catkin_ws/src/crazyCmd/scripts' \
                    '/Sperimentale/Swarm/MioPersonale/Preferiti' \
                    '/files_trj/trj.csv'

    # Inizializza le liste per le coordinate x, y, z
    vx_coords = []
    vy_coords = []
    vz_coords = []
    time_coords = []

    # Leggi il file CSV
    with open(filename_trj_, 'r') as csvfile:
        reader = csv.reader(csvfile, delimiter=',')
        for row in reader:
            time_coords.append(float(row[0]))
            vx_coords.append(float(row[4]))
            vy_coords.append(float(row[5]))
            vz_coords.append(float(row[6]))

    time_coords = time_coords[0::N_cf_]

    vx_4drone = []
    vy_4drone = []
    vz_4drone = []

    for i in range(N_cf_):
        vx_4drone.append(vx_coords[i::N_cf_])
        vy_4drone.append(vy_coords[i::N_cf_])
        vz_4drone.append(vz_coords[i::N_cf_])

    vtot_4drone = []

    for i in range(N_cf_):
        #        for k in range(len(time_coords)):
        vtot_4drone.append([0.0] * len(time_coords))

    for i in range(N_cf_):
        for k in range(len(time_coords)):
            vtot_4drone[i][k] = math.sqrt(vx_4drone[i][k] ** 2 + vy_4drone[i][k] ** 2 + vz_4drone[i][k] ** 2)

    v_avg = [0.0] * len(time_coords)
    v_max = [-float('inf')] * len(time_coords)
    v_min = [float('inf')] * len(time_coords)

    for k in range(len(time_coords)):
        for i in range(N_cf_):
            v_avg[k] += vtot_4drone[i][k]
            if vtot_4drone[i][k] > v_max[k]:
                v_max[k] = vtot_4drone[i][k]
            if vtot_4drone[i][k] < v_min[k]:
                v_min[k] = vtot_4drone[i][k]
        v_avg[k] /= N_cf_

    plt.plot(time_coords, v_avg, 'b-', lw=2, label='v_avg')
    plt.plot(time_coords, v_max, 'b-o', alpha=0.3, lw=1, markersize=1, label='v_max / v_min')
    plt.plot(time_coords, v_min, 'b-o', alpha=0.3, lw=1, markersize=1)
    plt.fill_between(time_coords, v_max, v_min, alpha=0.3)

    v_ref = 0.0
    # v_lim = 0.20
    v_lim_tot = math.sqrt(v_lim ** 2 + v_lim ** 2 + v_lim ** 2)

    # Aggiungi linee orizzontali
    # plt.axhline(y=v_ref, color='green', linestyle='dashed', lw=1, label='v_ref')
    plt.axhline(y=v_lim_tot, color='red', linestyle='dashed', lw=1, label='v_lim')

    # Aggiungi linee verticali per decollo/atterraggio
    plt.axvline(x=start_takeoff, color='black', lw=1)
    plt.axvline(x=finish_takeoff, color='black', lw=1)
    plt.axvline(x=start_landing, color='black', lw=1)
    plt.axvline(x=finish_landing, color='black', lw=1)

    # Colora l'area compresa tra le due linee verticali di grigio
    plt.axvspan(start_takeoff, finish_takeoff, color='red', alpha=0.3, label='takeoff/landing')
    plt.axvspan(start_landing, finish_landing, color='red', alpha=0.3)

    # Imposta le etichette degli assi
    plt.xlabel('Time [s]')
    plt.ylabel('Velocity [m/s]')

    # Imposta legenda
    plt.legend()

    # Imposta il limite inferiore dell'ascissa a zero
    plt.xlim(start_takeoff, finish_landing)

    # salvo immagine
    fig_arg = "plot_vel_maxminavg"
    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    filename = f"{timestamp}_{fig_arg}"
    full_path = os.path.join(folder, filename)
    plt.savefig(full_path)

    plt.show()


def plot_v_MPC(N_cf_, start_takeoff, finish_takeoff, start_landing, finish_landing, v_lim):
    filename_V_MPC = '/home/matteo/catkin_ws/src/crazyCmd/scripts' \
                     '/Sperimentale/Swarm/MioPersonale/Preferiti' \
                     '/files_trj/V_MPC.csv'

    # Inizializza le liste per le coordinate x, y, z
    vx_coords = []
    vy_coords = []
    vz_coords = []

    # Leggi il file CSV
    with open(filename_V_MPC, 'r') as csvfile:
        reader = csv.reader(csvfile, delimiter=',')
        for row in reader:
            # time_coords.append(float(row[0]))
            vx_coords.append(float(row[0]))
            vy_coords.append(float(row[1]))
            vz_coords.append(float(row[2]))

    vx_4drone = []
    vy_4drone = []
    vz_4drone = []

    for i in range(N_cf_):
        vx_4drone.append(vx_coords[i::N_cf_])
        vy_4drone.append(vy_coords[i::N_cf_])
        vz_4drone.append(vz_coords[i::N_cf_])

    # time_coords = np.linspace(finish_takeoff, start_landing, len(vx_4drone[0]))  # approssimazione
    num_MPCiter = np.linspace(1, len(vx_4drone[0]), len(vx_4drone[0]))  # numero di iterazioni MPC

    vtot_4drone = []

    for i in range(N_cf_):
        #     for k in range(len(time_coords)):
        vtot_4drone.append([0.0] * len(num_MPCiter))

    for i in range(N_cf_):
        for k in range(len(num_MPCiter)):
            vtot_4drone[i][k] = math.sqrt(vx_4drone[i][k] ** 2 + vy_4drone[i][k] ** 2 + vz_4drone[i][k] ** 2)

    for i in range(N_cf_):
        plt.plot(num_MPCiter, vtot_4drone[i], '-o', lw=1, markersize=3, label=f'v_opt_MPC_cf{i + 1}')

    # v_ref = 0.35
    # v_lim = 0.20
    v_lim_tot = math.sqrt(v_lim ** 2 + v_lim ** 2 + v_lim ** 2)

    # Aggiungi linee orizzontali
    # plt.axhline(y=v_ref, color='green', linestyle='dashed', lw=1, label='v_ref')
    plt.axhline(y=v_lim_tot, color='red', linestyle='dashed', lw=1, label='v_lim')

    '''
    # Aggiungi linee verticali per decollo/atterraggio
    plt.axvline(x=start_takeoff, color='black', lw=1)
    plt.axvline(x=finish_takeoff, color='black', lw=1)
    plt.axvline(x=start_landing, color='black', lw=1)
    plt.axvline(x=finish_landing, color='black', lw=1)
    
    # Colora l'area compresa tra le due linee verticali di grigio
    plt.axvspan(start_takeoff, finish_takeoff, color='red', alpha=0.3, label='takeoff/landing')
    plt.axvspan(start_landing, finish_landing, color='red', alpha=0.3)
    '''

    # Imposta le etichette degli assi
    plt.xlabel('MPC Prediction [-]')
    plt.ylabel('Velocity [m/s]')

    # Imposta legenda
    plt.legend()

    plt.title('MPC Velocity (Theoretical)')

    # Imposta il limite inferiore dell'ascissa a zero
    # plt.xlim(start_takeoff, finish_landing)

    # salvo immagine
    fig_arg = "plot_v_MPC_4drone"
    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    filename = f"{timestamp}_{fig_arg}"
    full_path = os.path.join(folder, filename)
    plt.savefig(full_path)

    plt.show()


def plot_distance_obs(N_cf_, start_takeoff, finish_takeoff, start_landing, finish_landing, obs_array, r_drone, r_safe):
    filename_trj_ = '/home/matteo/catkin_ws/src/crazyCmd/scripts' \
                    '/Sperimentale/Swarm/MioPersonale/Preferiti' \
                    '/files_trj/trj.csv'

    # Inizializza le liste per le coordinate x, y, z
    x_coords = []
    y_coords = []
    z_coords = []
    time_coords = []

    # Leggi il file CSV
    with open(filename_trj_, 'r') as csvfile:
        reader = csv.reader(csvfile, delimiter=',')
        for row in reader:
            time_coords.append(float(row[0]))
            x_coords.append(float(row[1]))
            y_coords.append(float(row[2]))
            z_coords.append(float(row[3]))

    time_coords = time_coords[0::N_cf_]

    x_4drone = []
    y_4drone = []
    z_4drone = []

    for i in range(N_cf_):
        x_4drone.append(x_coords[i::N_cf_])
        y_4drone.append(y_coords[i::N_cf_])
        z_4drone.append(z_coords[i::N_cf_])

    distances = {}
    for i in range(N_cf_):
        for j in range(len(obs_array)):
            distances[(i, j)] = [0.0] * len(time_coords)

    for i in range(N_cf_):
        for j in range(len(obs_array)):
            for k in range(len(time_coords)):
                distance = math.sqrt(
                    (x_4drone[i][k] - obs_array[j].x) ** 2 + (y_4drone[i][k] - obs_array[j].y) ** 2 + (
                            z_4drone[i][k] - obs_array[j].z) ** 2)
                distances[(i, j)][k] = distance

    for i in range(N_cf_):
        for j in range(len(obs_array)):
            plt.plot(time_coords, distances[(i, j)], '-o', lw=1, markersize=1, label=f'd(cf{i + 1},obs{j + 1})')

    """
    d_min = []
    for j in range(len(obs_array)):
        d_min.append(obs_array[j].r + r_drone + r_safe)
        plt.axhline(y=d_min[j], linestyle='dashed', lw=1, label=f'd_min_obs{j + 1}')
    """

    radius_obs = obs_array[0].r  # LIMITAZIONE - TUTTI GLI OSTACOLI CON STESSO RAGGIO

    d_safe = r_drone + radius_obs + r_safe
    d_coll = r_drone + radius_obs
    plt.axhline(y=d_safe, linestyle='dashed', color='orange', lw=1, label='d_safe')
    plt.axhline(y=d_coll, linestyle='dashed', color='red', lw=1, label='d_coll')

    # Aggiungi linee verticali per decollo/atterraggio
    plt.axvline(x=start_takeoff, color='black', lw=1)
    plt.axvline(x=finish_takeoff, color='black', lw=1)
    plt.axvline(x=start_landing, color='black', lw=1)
    plt.axvline(x=finish_landing, color='black', lw=1)

    # Colora l'area compresa tra le due linee verticali di grigio
    plt.axvspan(start_takeoff, finish_takeoff, color='red', alpha=0.3, label='takeoff/landing')
    plt.axvspan(start_landing, finish_landing, color='red', alpha=0.3)

    # Imposta le etichette degli assi
    plt.xlabel('Time [s]')
    plt.ylabel('Distance from Obstacles [m]')

    # Imposta legenda
    plt.legend()

    plt.title('Distance from Obstacles')

    # Imposta il limite inferiore dell'ascissa a zero
    plt.xlim(start_takeoff, finish_landing)

    # salvo immagine
    fig_arg = "plot_distance_obs"
    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    filename = f"{timestamp}_{fig_arg}"
    full_path = os.path.join(folder, filename)
    plt.savefig(full_path)

    plt.show()


def plot_distance_closestobs(N_cf_, start_takeoff, finish_takeoff, start_landing, finish_landing, obs_array, r_drone,
                             r_safe):
    filename_trj_ = '/home/matteo/catkin_ws/src/crazyCmd/scripts' \
                    '/Sperimentale/Swarm/MioPersonale/Preferiti' \
                    '/files_trj/trj.csv'

    # Inizializza le liste per le coordinate x, y, z
    x_coords = []
    y_coords = []
    z_coords = []
    time_coords = []

    # Leggi il file CSV
    with open(filename_trj_, 'r') as csvfile:
        reader = csv.reader(csvfile, delimiter=',')
        for row in reader:
            time_coords.append(float(row[0]))
            x_coords.append(float(row[1]))
            y_coords.append(float(row[2]))
            z_coords.append(float(row[3]))

    time_coords = time_coords[0::N_cf_]

    x_4drone = []
    y_4drone = []
    z_4drone = []

    for i in range(N_cf_):
        x_4drone.append(x_coords[i::N_cf_])
        y_4drone.append(y_coords[i::N_cf_])
        z_4drone.append(z_coords[i::N_cf_])

    distances = {}
    for i in range(N_cf_):
        for j in range(len(obs_array)):
            distances[(i, j)] = [0.0] * len(time_coords)

    for i in range(N_cf_):
        for j in range(len(obs_array)):
            for k in range(len(time_coords)):
                distance = math.sqrt(
                    (x_4drone[i][k] - obs_array[j].x) ** 2 + (y_4drone[i][k] - obs_array[j].y) ** 2 + (
                            z_4drone[i][k] - obs_array[j].z) ** 2)
                distances[(i, j)][k] = distance

    # inizializzo distanza min, che contiene la distanza minima di ogni drone da qualsiasi ostacolo
    distances_min = [[float('inf')] * len(time_coords) for _ in range(N_cf_)]

    for k in range(len(time_coords)):
        for i in range(N_cf_):
            for j in range(len(obs_array)):
                if distances[(i, j)][k] < distances_min[i][k]:
                    distances_min[i][k] = distances[(i, j)][k]

    for i in range(N_cf_):
        plt.plot(time_coords, distances_min[i], '-o', lw=1, markersize=1, label=f'd(cf{i + 1},obs_closest)')

    obs_radius = obs_array[0].r  # LIMITAZIONE: USARE OSTACOLI CON MEDESIMO RAGGIO! *
    d_coll = obs_radius + r_drone
    plt.axhline(y=d_coll, color='red', linestyle='dashed', lw=1, label=f'd_coll')
    plt.axhline(y=d_coll + r_safe, color='orange', linestyle='dashed', lw=1, label=f'd_safe')

    """
    * per ovviare il problema: normalizzazione necessaria, vedi plot_distance_closestobs_normradius
    """

    # Aggiungi linee verticali per decollo/atterraggio
    plt.axvline(x=start_takeoff, color='black', lw=1)
    plt.axvline(x=finish_takeoff, color='black', lw=1)
    plt.axvline(x=start_landing, color='black', lw=1)
    plt.axvline(x=finish_landing, color='black', lw=1)

    # Colora l'area compresa tra le due linee verticali di grigio
    plt.axvspan(start_takeoff, finish_takeoff, color='red', alpha=0.3, label='takeoff/landing')
    plt.axvspan(start_landing, finish_landing, color='red', alpha=0.3)

    # Imposta le etichette degli assi
    plt.xlabel('Time [s]')
    plt.ylabel('Distance from Obstacles [m]')

    # Imposta legenda
    plt.legend()

    # Imposta il limite inferiore dell'ascissa a zero
    plt.xlim(start_takeoff, finish_landing)

    # salvo immagine
    fig_arg = "plot_distance_closestobs"
    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    filename = f"{timestamp}_{fig_arg}"
    full_path = os.path.join(folder, filename)
    plt.savefig(full_path)

    plt.show()


def plot_distance_closestobs_normradius(N_cf_, start_takeoff, finish_takeoff, start_landing, finish_landing, obs_array,
                                        r_drone, r_safe):
    filename_trj_ = '/home/matteo/catkin_ws/src/crazyCmd/scripts' \
                    '/Sperimentale/Swarm/MioPersonale/Preferiti' \
                    '/files_trj/trj.csv'

    # Inizializza le liste per le coordinate x, y, z
    x_coords = []
    y_coords = []
    z_coords = []
    time_coords = []

    # Leggi il file CSV
    with open(filename_trj_, 'r') as csvfile:
        reader = csv.reader(csvfile, delimiter=',')
        for row in reader:
            time_coords.append(float(row[0]))
            x_coords.append(float(row[1]))
            y_coords.append(float(row[2]))
            z_coords.append(float(row[3]))

    time_coords = time_coords[0::N_cf_]

    x_4drone = []
    y_4drone = []
    z_4drone = []

    for i in range(N_cf_):
        x_4drone.append(x_coords[i::N_cf_])
        y_4drone.append(y_coords[i::N_cf_])
        z_4drone.append(z_coords[i::N_cf_])

    distances = {}
    for i in range(N_cf_):
        for j in range(len(obs_array)):
            distances[(i, j)] = [0.0] * len(time_coords)

    for i in range(N_cf_):
        for j in range(len(obs_array)):
            for k in range(len(time_coords)):
                distance = math.sqrt(
                    (x_4drone[i][k] - obs_array[j].x) ** 2 + (y_4drone[i][k] - obs_array[j].y) ** 2 + (
                            z_4drone[i][k] - obs_array[j].z) ** 2)
                distance -= obs_array[j].r  # normalizzo in funzione del raggio
                distance -= r_drone  # normalizzo la dimensione del drone
                distances[(i, j)][k] = distance

    # inizializzo distanza min, che contiene la distanza minima di ogni drone da qualsiasi ostacolo
    distances_min = [[float('inf')] * len(time_coords) for _ in range(N_cf_)]

    for k in range(len(time_coords)):
        for i in range(N_cf_):
            for j in range(len(obs_array)):
                if distances[(i, j)][k] < distances_min[i][k]:
                    distances_min[i][k] = distances[(i, j)][k]

    for i in range(N_cf_):
        plt.plot(time_coords, distances_min[i], '-o', lw=1, markersize=1, label=f'd(cf{i + 1},obs_closest)')

    d_coll = 0
    d_safe = r_safe
    plt.axhline(y=d_safe, color='orange', linestyle='dashed', lw=1, label='d_safe')
    plt.axhline(y=d_coll, color='red', linestyle='dashed', lw=1, label='d_coll')

    # Aggiungi linee verticali per decollo/atterraggio
    plt.axvline(x=start_takeoff, color='black', lw=1)
    plt.axvline(x=finish_takeoff, color='black', lw=1)
    plt.axvline(x=start_landing, color='black', lw=1)
    plt.axvline(x=finish_landing, color='black', lw=1)

    # Colora l'area compresa tra le due linee verticali di grigio
    plt.axvspan(start_takeoff, finish_takeoff, color='red', alpha=0.3, label='takeoff/landing')
    plt.axvspan(start_landing, finish_landing, color='red', alpha=0.3)

    # Imposta le etichette degli assi
    plt.xlabel('Time [s]')
    plt.ylabel('Distance from Obstacles [m]')

    # Imposta legenda
    plt.legend()

    plt.title('Minimum Distance from the Closest Obstacle')

    # Imposta il limite inferiore dell'ascissa a zero
    plt.xlim(start_takeoff, finish_landing)

    # salvo immagine
    fig_arg = "plot_distance_closestobs_normradius"
    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    filename = f"{timestamp}_{fig_arg}"
    full_path = os.path.join(folder, filename)
    plt.savefig(full_path)

    plt.show()


def plot_estimation_vs_real(vettore_tgt_x, vettore_tgt_y):
    file_estimated = '/home/matteo/catkin_ws/src/crazyCmd/scripts' \
                     '/Sperimentale/Swarm/MioPersonale/Preferiti' \
                     '/files_trj/final_pos_estimated_addeddist0.csv'

    file_real = '/home/matteo/catkin_ws/src/crazyCmd/scripts' \
                '/Sperimentale/Swarm/MioPersonale/Preferiti' \
                '/files_trj/final_pos_real_addeddist0.csv'

    # Inizializzazione

    # vettore posizioni finali - ESTIMATED
    x_fin_estimated = []
    y_fin_estimated = []
    with open(file_estimated, 'r') as csvfile:
        reader = csv.reader(csvfile, delimiter=',')
        for row in reader:
            x_fin_estimated.append(float(row[0]))
            y_fin_estimated.append(float(row[1]))

    # vettore posizioni finali - REAL
    x_fin_real = []
    y_fin_real = []
    with open(file_real, 'r') as csvfile:
        reader = csv.reader(csvfile, delimiter=',')
        for row in reader:
            x_fin_real.append(float(row[0]))
            y_fin_real.append(float(row[1]))

    # numero di test eseguiti
    num_test_4distance = 3
    num_test_tot = int(len(x_fin_estimated) / num_test_4distance)

    # Inizializzazione - END

    # plotto start
    plt.plot(0, 0, 'o', color='black', label='start_pos')

    # plotto target
    gap_end_loop = 0.15
    for i in range(num_test_tot):
        plt.plot(vettore_tgt_x[i], vettore_tgt_y[i], 'ro', markersize=8)
        tgt_ts = Circle((vettore_tgt_x[i], vettore_tgt_y[i]), gap_end_loop, edgecolor='black', alpha=0.3,
                        linestyle='dashed', facecolor='none')
        plt.gca().add_patch(tgt_ts)

    # plotto posizione finale
    for i in range(num_test_tot * num_test_4distance):
        plt.plot(x_fin_real[i], y_fin_real[i], 'bx')
        plt.plot(x_fin_estimated[i], y_fin_estimated[i], 'bo')

    # aggiungo legenda una tantum
    plt.plot([], [], 'bx', label='final_pos_real')
    plt.plot([], [], 'bo', label='final_pos_estimated')
    plt.plot([], [], 'ro', label='tgt', markersize=8)
    plt.gca().add_patch(
        Circle((0, 0), 0, edgecolor='black', alpha=0.3, linestyle='dashed', facecolor='none', label='tgt_threshold'))

    # plotto segmento di riferimento
    x_start, y_start = 0.0, 0.0
    x_end, y_end = 3.0, -3.0

    plt.plot([x_start, x_end], [y_start, y_end], linestyle='--', color='black', linewidth=0.5)

    # +++++++++++++++++++++++++ FORMATTAZIONE +++++++++++++++++++++++++++++++++++++++

    plt.xlabel('X [m]')
    plt.ylabel('Y [m]')
    plt.xlim(-0.5, 4.0)
    plt.ylim(-3.6, 1.2)
    plt.legend()
    plt.grid(True)
    plt.title("Estimated State vs Real State")

    # salvo immagine
    fig_arg = "plot_estimation_vs_real"
    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    filename = f"{timestamp}_{fig_arg}"
    full_path = os.path.join(folder, filename)
    plt.savefig(full_path)

    plt.show()


def plot_estimation_vs_real_error(vettore_tgt_x, vettore_tgt_y):
    file_estimated = '/home/matteo/catkin_ws/src/crazyCmd/scripts' \
                     '/Sperimentale/Swarm/MioPersonale/Preferiti' \
                     '/files_trj/final_pos_estimated_addeddist0.csv'

    file_real = '/home/matteo/catkin_ws/src/crazyCmd/scripts' \
                '/Sperimentale/Swarm/MioPersonale/Preferiti' \
                '/files_trj/final_pos_real_addeddist0.csv'

    # Inizializzazione

    # vettore posizioni finali - ESTIMATED
    x_fin_estimated = []
    y_fin_estimated = []
    with open(file_estimated, 'r') as csvfile:
        reader = csv.reader(csvfile, delimiter=',')
        for row in reader:
            x_fin_estimated.append(float(row[0]))
            y_fin_estimated.append(float(row[1]))

    # vettore posizioni finali - REAL
    x_fin_real = []
    y_fin_real = []
    with open(file_real, 'r') as csvfile:
        reader = csv.reader(csvfile, delimiter=',')
        for row in reader:
            x_fin_real.append(float(row[0]))
            y_fin_real.append(float(row[1]))

    # Inizializzazione - END

    # +++++++++++++++++++++++++ CALCOLO ERRORE +++++++++++++++++++++++++++++++++++++++

    # calcolo errore
    errors = np.sqrt((np.array(x_fin_estimated) - np.array(x_fin_real)) ** 2 +
                     (np.array(y_fin_estimated) - np.array(y_fin_real)) ** 2)

    num_test_4distance = 3
    num_test_tot = int(len(errors) / num_test_4distance)

    # media errore
    err_average = []
    for i in range(num_test_tot):
        avg = sum(errors[i * num_test_4distance: (i + 1) * num_test_4distance]) / num_test_4distance
        err_average.append(avg)

    # +++++++++++++++++++++++++ CALCOLO DISTANZA +++++++++++++++++++++++++++++++++++++++

    # origine
    origine_x = 0.0
    origine_y = 0.0

    # calcolo distanza
    distances = np.sqrt((np.array(vettore_tgt_x) - origine_x) ** 2 + (np.array(vettore_tgt_y) - origine_y) ** 2)

    # +++++++++++++++++++++++++ PLOTTO +++++++++++++++++++++++++++++++++++++++

    plt.plot(distances, err_average, 'o', linestyle='--', color='black', linewidth=1, markersize=8)

    plt.xlabel('Distance [m]')
    plt.ylabel('Average Error [m]')
    plt.xlim(0.0)
    plt.ylim(0.0)
    plt.legend()
    plt.grid(True)
    plt.title("Average Error in State Estimation")

    # +++++++++++++++++++++++++ SALVO IMMAGINE +++++++++++++++++++++++++++++++++++++++

    fig_arg = "plot_estimation_vs_real_error"
    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    filename = f"{timestamp}_{fig_arg}"
    full_path = os.path.join(folder, filename)
    plt.savefig(full_path)

    plt.show()


def plot_timeMPCsolv3D():
    file_timeMPCsolv = '/home/matteo/catkin_ws/src/crazyCmd/scripts' \
                       '/Sperimentale/Swarm/MioPersonale/Preferiti' \
                       '/files_trj/t_solv_plot.csv'

    x = np.array([4, 8, 12])
    y = np.array([1, 2, 3, 4])
    z = np.zeros((len(y), len(x)))  # Inizializza l'array z con zeri

    timeMPCsolv_array = []

    with open(file_timeMPCsolv, 'r') as csvfile:
        reader = csv.reader(csvfile, delimiter=',')
        for row in reader:
            timeMPCsolv_array.append(float(row[0]))

    for i in range(len(y)):
        for j in range(len(x)):
            z[i][j] = timeMPCsolv_array[i * len(x) + j]

    # interpolazione
    xfine = np.linspace(min(x), max(x), 100)
    zfine = np.zeros((len(y), len(xfine)))
    for i in range(len(y)):
        f = interp1d(x, z[i], kind='quadratic')
        zfine[i] = f(xfine)  # Generazione di zfine interpolato da xfine
    # TODO: interpolazione sia su x che su y

    # plot
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')
    Xfine, Y = np.meshgrid(xfine, y)

    surf = ax.plot_surface(Xfine, Y, zfine, cmap='jet')

    # Plotta la superficie
    # X, Y = np.meshgrid(x, y)
    # surf = ax.plot_surface(X, Y, z, cmap='jet')
    # ax.scatter(X, Y, z, s=20, c='black', marker='o', alpha=0.7)

    # Personalizzazioni estetiche
    cbar = fig.colorbar(surf)
    cbar.mappable.set_clim(0, 6.5)  # Imposta i limiti inferiore e superiore
    cbar.set_ticks(np.linspace(0, 6.5, 10))  # Imposta i tick della colorbar

    # Plotta i dati
    # ax.scatter(Nmpc_array, Ncf_array, timeMPCsolv_array)

    ax.set_xticks(x)
    ax.set_yticks(y)
    ax.zaxis.set_major_formatter(FormatStrFormatter('%.1f'))

    # Aggiungi etichette agli assi
    ax.set_title('MPC solver computation time')
    ax.set_xlabel('Num. Steps [-]')
    ax.set_ylabel('Num. drones [-]')
    ax.set_zlabel('Computation Time [s]')

    # +++++++++++++++++++++++++ SALVO IMMAGINE +++++++++++++++++++++++++++++++++++++++

    fig_arg = "plot_timeMPCsolv3D"
    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    filename = f"{timestamp}_{fig_arg}"
    full_path = os.path.join(folder, filename)
    plt.savefig(full_path)

    # Mostra il grafico
    plt.show()


def plot_timeMPCsolv2D():
    file_timeMPCsolv = '/home/matteo/catkin_ws/src/crazyCmd/scripts' \
                       '/Sperimentale/Swarm/MioPersonale/Preferiti' \
                       '/files_trj/t_solv_plot.csv'

    N_cf_array = []
    N_mpc_array = []
    timeMPCsolv_array = []

    with open(file_timeMPCsolv, 'r') as csvfile:
        reader = csv.reader(csvfile, delimiter=',')
        for row in reader:
            timeMPCsolv_array.append(float(row[0]))
            N_cf_array.append(float(row[1]))
            N_mpc_array.append(float(row[2]))

    # Reshape degli array per ottenere 4 gruppi di 3 valori
    N_mpc_array_reshaped = np.reshape(N_mpc_array, (4, 3))
    timeMPCsolv_array_reshaped = np.reshape(timeMPCsolv_array, (4, 3))

    N_mpc_array_fine = np.linspace(min(N_mpc_array), max(N_mpc_array), 100)

    timeMPCsolv_array_reshaped_fine = np.zeros((4, len(N_mpc_array_fine)))

    for i in range(4):
        f = interp1d(N_mpc_array_reshaped[i], timeMPCsolv_array_reshaped[i], kind='quadratic')
        timeMPCsolv_array_reshaped_fine[i] = f(N_mpc_array_fine)  # Generazione di zfine interpolato da xfine

    colors = ['red', 'blue', 'green', 'orange']  # Lista dei colori desiderati

    # Plot delle linee
    for i in range(4):
        color = colors[i % len(colors)]
        # plt.plot(N_mpc_array_fine, timeMPCsolv_array_reshaped_fine[i], linestyle='dashed', color=color)
        plt.plot(N_mpc_array_reshaped[i], timeMPCsolv_array_reshaped[i], '-o', markersize=8,
                 label=f'Num. drones = {i + 1}', color=color, linestyle='dashed')

    # +++++++++++++++++++++++++ FORMATTAZIONE +++++++++++++++++++++++++++++++++++++++

    plt.xlabel('Num. Steps [-]')
    plt.ylabel('Computation Time [s]')
    plt.xlim(3.5)
    plt.ylim(0.0)
    plt.legend()
    plt.grid(True)
    plt.title('MPC solver computation time')

    # +++++++++++++++++++++++++ SALVO IMMAGINE +++++++++++++++++++++++++++++++++++++++

    fig_arg = "plot_timeMPCsolv2D"
    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    filename = f"{timestamp}_{fig_arg}"
    full_path = os.path.join(folder, filename)
    plt.savefig(full_path)

    # Mostra il grafico
    plt.show()
