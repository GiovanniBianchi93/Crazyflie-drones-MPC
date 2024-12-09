with open('./users_files/get_input_file', 'r') as file:

    lines = file.readlines()

    local_N_cf = int(lines[0].replace('number drones:', '').strip())
    cf_names = standardNameList(local_N_cf)
    uris = create_uris_list(cf_names)

    offset = [{'x': 0.0, 'y': 0.0, 'z': 0.0} for _ in range(local_N_cf)]
    offset_str = lines[1].replace('offset (x y z) [m] for each drone:', '').strip().split()

    if len(offset_str) != 3 * local_N_cf:
        print('errore - dati mancanti')
        sys.exit(1)

    for i in range(local_N_cf):
        offset[i] = {'x': float(offset_str[i * 3]), 'y': float(offset_str[i * 3 + 1]), 'z': float(offset_str[i * 3 + 2])}

    local_scelta_target = int(lines[2].replace('target modality (1/2):', '').strip())

    if local_scelta_target not in [1, 2]:
        print('errore - scelta non valida')
        sys.exit(1)

    mpc_target_str = lines[3].replace('target center of mass (x y z) [m]:', '').strip().split()

    if local_scelta_target == 1 and len(mpc_target_str) != 3:
        print('errore - dati mancanti')
        sys.exit(1)

    tgt4drone = [{'x': 0.0, 'y': 0.0, 'z': 0.0} for _ in range(local_N_cf)]
    tgt4drone_str = lines[4].replace('target (x y z) [m] for each drone:', '').strip().split()

    if local_scelta_target == 2 and len(tgt4drone_str) != 3 * local_N_cf:
        print('errore - dati mancanti')
        sys.exit(1)

    for i in range(local_N_cf):
        tgt4drone[i] = {'x': float(tgt4drone_str[i * 3]), 'y': float(tgt4drone_str[i * 3 + 1]), 'z': float(tgt4drone_str[i * 3 + 2])}

    num_obs = int(lines[5].replace('number parallelepiped obstacles:', '').strip())
    info_par = []

    for i in range(num_obs):
        info_par_str = lines[6 + i].replace('center of mass (x y z) [m] and dimensions (a b c) [m] for each obstacle:', '').strip().split()

        if len(info_par_str) != 6:
            print('errore - dati mancanti')
            sys.exit(1)

        info_par.append({
            'x': float(info_par_str[0]),
            'y': float(info_par_str[1]),
            'z': float(info_par_str[2]),
            'a': float(info_par_str[3]),
            'b': float(info_par_str[4]),
            'c': float(info_par_str[5])
        })

    # debug
    print('offset_str:', offset_str)
    print('mpc_target_str:', mpc_target_str)
    print('tgt4drone_str:', tgt4drone)
