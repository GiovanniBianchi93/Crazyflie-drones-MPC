

def plot_trj_wopt(N_cf_, obs_array_, N_mpc, filename_wopt_, local_mpc_target):

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
                                   label=f'cf{i + 1}_pred{j*5}')  # creo istanza Line3D
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
