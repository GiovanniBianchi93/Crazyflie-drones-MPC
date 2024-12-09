#! /usr/bin/env python3
from crazyCmd.scripts.Sperimentale.Swarm.MioPersonale.Preferiti import lib_plot_mv
from crazyCmd.scripts.Sperimentale.Swarm.MioPersonale.Preferiti import lib_mv
from crazyflie_messages.msg import Position, SphereData
from geometry_msgs.msg import Point

if __name__ == '__main__':

    local_mpc_target = Position()

    N_cf_ = 4  # <---

    N_mpc = 4  # <---

    local_mpc_target.desired_position.x = 2.5
    local_mpc_target.desired_position.y = -1.5
    local_mpc_target.desired_position.z = 0.5

    local_tgt4drone_array = []

    '''
    tgt4drone = []  # contiene i singoli target per ogni drone

    # inizializzo tgt4drone
    for i in range(N_cf_):
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

    for i in range(N_cf_):
        tgt4drone_single = Point()
        tgt4drone_single.x = tgt4drone[i]['x']
        tgt4drone_single.y = tgt4drone[i]['y']
        tgt4drone_single.z = tgt4drone[i]['z']
        local_tgt4drone_array.append(tgt4drone_single)

    local_mpc_target = lib_mv.calcolo_centro_massa_target(local_tgt4drone_array)
    '''

    # OSTACOLI
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
        obs += lib_mv.sphere_approximation(info_par[i]['x'], info_par[i]['y'], info_par[i]['z'],
                                           info_par[i]['a'], info_par[i]['b'], info_par[i]['c'])

    for i in range(len(obs)):
        obs_single = SphereData()
        obs_single.x = obs[i][0]
        obs_single.y = obs[i][1]
        obs_single.z = obs[i][2]
        obs_single.r = obs[i][3]
        obs_array.append(obs_single)

    # OSTACOLI - FINE

    # CONSEGNA TESI - NON MODIFICARE
    r_drone = 0.1  # <--- stessa d_ref di MPC
    r_safe = 0.3  # <--- stessa d_ref di MPC
    d_ref = 0.75  # <--- stessa d_ref di MPC
    v_lim = 0.2
    d_coll = r_drone * 2
    d_emer = d_coll + r_safe

    # IPOTIZZATI
    start_takeoff = 1.5  # provvisorio
    finish_takeoff = 5.6  # provvisorio
    start_landing = 37  # provvisorio
    finish_landing = 40  # provvisorio

    # FILE
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

    # lib_plot_mv.plot_trj(N_cf_, obs_array, local_mpc_target)
    # lib_plot_mv.plot_trj_wopt(N_cf_, obs_array, N_mpc, filename_trjMPC_iter0, local_mpc_target, local_tgt4drone_array)
    # lib_plot_mv.plot_trj_wopt(N_cf_, obs_array, N_mpc, filename_wopt, local_mpc_target, local_tgt4drone_array)
    # lib_plot_mv.plot_trj2D(N_cf_, obs_array, local_mpc_target)
    # lib_plot_mv.plot_trj2D_wopt(N_cf_, obs_array, N_mpc, filename_trjMPC_iter0, local_mpc_target)
    # lib_plot_mv.plot_trj2D_wopt(N_cf_, obs_array, N_mpc, filename_wopt, local_mpc_target)
    # lib_plot_mv.plot_trj2D_wopt_t4d(N_cf_, obs_array, N_mpc, filename_trjMPC_iter0, local_mpc_target, local_tgt4drone_array)
    # lib_plot_mv.plot_trj2D_wopt_t4d(N_cf_, obs_array, N_mpc, filename_wopt, local_mpc_target, local_tgt4drone_array)

    # ------------------------------------------------------------------------------------------------------------------
    #
    #       P L O T   T A R G E T
    #
    # ------------------------------------------------------------------------------------------------------------------

    # lib_plot_mv.plot_cm_xyz(N_cf_, start_takeoff, finish_takeoff, start_landing, finish_landing, local_mpc_target)
    # lib_plot_mv.plot_distance_tgtcm(N_cf_, start_takeoff, finish_takeoff, start_landing, finish_landing,
    #                                local_mpc_target)
    # lib_plot_mv.plot_distance_tgt4d(N_cf_, start_takeoff, finish_takeoff, start_landing, finish_landing,
    #                                         local_tgt4drone_array)

    # ------------------------------------------------------------------------------------------------------------------
    #
    #       P L O T   V E L O C I T Y
    #
    # ------------------------------------------------------------------------------------------------------------------

    # lib_plot_mv.plot_vel_maxminavg(N_cf_, start_takeoff, finish_takeoff, start_landing, finish_landing)
    # lib_plot_mv.plot_vel4drone(N_cf_, start_takeoff, finish_takeoff, start_landing, finish_landing, v_lim)
    # lib_plot_mv.plot_v_MPC(N_cf_, start_takeoff, finish_takeoff, start_landing, finish_landing, v_lim)

    # ------------------------------------------------------------------------------------------------------------------
    #
    #       P L O T   I N T E R - A G E N T   D I S T A N C E
    #
    # ------------------------------------------------------------------------------------------------------------------

    lib_plot_mv.plot_distance_ij(N_cf_, start_takeoff, finish_takeoff, start_landing, finish_landing, d_ref, d_emer,
                                 d_coll)
    # lib_plot_mv.plot_distance_maxminavg(N_cf_, start_takeoff, finish_takeoff, start_landing, finish_landing, d_ref, d_emer, d_coll)
    # lib_plot_mv.plot_distance_min(N_cf_, start_takeoff, finish_takeoff, start_landing, finish_landing, d_ref, d_emer, d_coll)

    # ------------------------------------------------------------------------------------------------------------------
    #
    #       P L O T   O B S T A C L E S
    #
    # ------------------------------------------------------------------------------------------------------------------

    # lib_plot_mv.plot_distance_obs(N_cf_, start_takeoff, finish_takeoff, start_landing, finish_landing, obs_array,
    #                              r_drone, r_safe)
    # lib_plot_mv.plot_distance_closestobs(N_cf_, start_takeoff, finish_takeoff, start_landing, finish_landing, obs_array,
    #                                   r_drone, r_safe)
    # lib_plot_mv.plot_distance_closestobs_normradius(N_cf_, start_takeoff, finish_takeoff, start_landing, finish_landing, obs_array,
    #                                           r_drone, r_safe)

    # ------------------------------------------------------------------------------------------------------------------
    #
    #       P L O T   T R A J E C T O R Y   A N I M A T I O N
    #
    # ------------------------------------------------------------------------------------------------------------------

    # lib_plot_mv.plot_trj2D_anim(N_cf_, obs_array, local_mpc_target)
    # lib_plot_mv.plot_trj2D_wopt_t4d(N_cf_, obs_array, N_mpc, filename_trjMPC_iter0, local_mpc_target, local_tgt4drone_array)

    # lib_plot_mv.plot_trj2D_anim_MPC(N_cf_, obs_array, N_mpc, local_mpc_target)
    # lib_plot_mv.plot_trj2D_anim_MPC_t4d(N_cf_, obs_array, N_mpc, local_mpc_target, local_tgt4drone_array)

    # lib_plot_mv.plot_trj2D_anim_xz_MPC_t4d(N_cf_, obs_array, N_mpc, local_mpc_target, local_tgt4drone_array)

    # ------------------------------------------------------------------------------------------------------------------
    #
    #       P O S I Z I O N E   F I N A L E   S T I M A T A   V S   R E A L E
    #
    # ------------------------------------------------------------------------------------------------------------------
    '''
    # vettore target
    vettore_tgt_x = [0.0, 0.5, 1.0, 1.5, 2.0, 2.5, 3.0]
    vettore_tgt_y = [0.0, -0.5, -1.0, -1.5, -2.0, -2.5, -3.0]

    lib_plot_mv.plot_estimation_vs_real(vettore_tgt_x, vettore_tgt_y)
    lib_plot_mv.plot_estimation_vs_real_error(vettore_tgt_x, vettore_tgt_y)

    # ------------------------------------------------------------------------------------------------------------------
    #
    #       P O S I Z I O N E   F I N A L E   S T I M A T A   V S   R E A L E
    #
    # ------------------------------------------------------------------------------------------------------------------

    lib_plot_mv.plot_timeMPCsolv3D()
    lib_plot_mv.plot_timeMPCsolv2D()
    '''
