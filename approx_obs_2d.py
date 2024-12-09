import math

import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle, Circle


def sphere_approximation(x_par, y_par, z_par, a, b, c):
    """
    --- PARALLELEPIPED TO SPHERES APPROXIMATION ---

    Restituisce un elenco di sfere che approssimano il parallelepipedo
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

    # print('Sph_res: ', sph_res)

    # traslo opportunamente: - a / 2 per centro di massa, + x_par per traslazione
    for sphere in sph_res:
        sphere[0] += x_par - a / 2
        sphere[1] += y_par - b / 2
        sphere[2] += z_par - c / 2

    # print('Sph_res_translated: ', sph_res)
    print('Result: ', sph_res)

    return sph_res


if __name__ == '__main__':

    info_par = []
    obs = []
    obs_array = []  # lista di variabili Point()

    for i in range(1):
        info_par.append({'x': 0.0, 'y': 0.0, 'z': 0.0, 'a': 0.0, 'b': 0.0, 'c': 0.0})  # inizializzo

    # obs1
    info_par[0]['x'] = 1.0
    info_par[0]['y'] = 1.0
    info_par[0]['z'] = 0.8
    info_par[0]['a'] = 0.5
    info_par[0]['b'] = 0.5
    info_par[0]['c'] = 1.6

    for i in range(1):
        obs += sphere_approximation(info_par[i]['x'], info_par[i]['y'], info_par[i]['z'],
                                    info_par[i]['a'], info_par[i]['b'], info_par[i]['c'])

    spheres = obs

    x = info_par[0]['x']
    y = info_par[0]['y']
    z = info_par[0]['z']
    a = info_par[0]['a']
    b = info_par[0]['b']
    c = info_par[0]['c']

    # CONSEGNA TESI
    a = a * 0.8
    b = b * 0.8

    # Creazione della figura e dei sottografi
    fig = plt.figure()

    # Sottografo per l'asse x-y
    ax_xy = fig.add_subplot(131)
    rectangle_xy = ax_xy.add_patch(Rectangle((x - a / 2, y - b / 2), a, b, color='blue', alpha=0.5))
    # for sphere in spheres[0]:
    xs, ys, zs, rs = spheres[0]
    circle_xy = Circle((xs, ys), rs, color='red', alpha=0.5)
    ax_xy.add_patch(circle_xy)
    ax_xy.set_xlim([0, 2.0])
    ax_xy.set_ylim([0, 2.0])
    ax_xy.set_xlabel('X')
    ax_xy.set_ylabel('Y')
    ax_xy.grid(True)
    ax_xy.set_title('XY View')


    # Sottografo per l'asse y-z
    ax_yz = fig.add_subplot(132)
    rectangle_yz = ax_yz.add_patch(Rectangle((y - b / 2, z - c / 2), b, c, color='blue', alpha=0.5))
    for sphere in spheres:
        xs, ys, zs, rs = sphere
        circle_yz = Circle((ys, zs), rs, color='red', alpha=0.5)
        ax_yz.add_patch(circle_yz)
    ax_yz.set_xlim([0, 2.0])
    ax_yz.set_ylim([0, 2.0])
    ax_yz.set_xlabel('Y')
    ax_yz.set_ylabel('Z')
    ax_yz.grid(True)
    ax_yz.set_title('YZ View')

    # Sottografo per l'asse x-z
    ax_xz = fig.add_subplot(133)
    rectangle_xz = ax_xz.add_patch(Rectangle((x - a / 2, z - c / 2), a, c, color='blue', alpha=0.5))
    for sphere in spheres:
        xs, ys, zs, rs = sphere
        circle_xz = Circle((xs, zs), rs, color='red', alpha=0.5)
        ax_xz.add_patch(circle_xz)
    ax_xz.set_xlim([0, 2.0])
    ax_xz.set_ylim([0, 2.0])
    ax_xz.set_xlabel('X')
    ax_xz.set_ylabel('Z')
    ax_xz.grid(True)
    ax_xz.set_title('XZ View')
    ax_xz.legend([rectangle_xz, circle_xz], ['Parallelepiped', 'Spheres'], loc='upper right', bbox_to_anchor=(1.2, 1.05))


    # Impostazione dei margini della figura
    fig.subplots_adjust(hspace=0.4, wspace=0.4)

    # Visualizzazione del grafico
    plt.show()

