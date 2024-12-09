#! /usr/bin/env python3
import math

import numpy as np
import matplotlib.pyplot as plt
from matplotlib.patches import Circle
from mpl_toolkits.mplot3d import axes3d
from mpl_toolkits.mplot3d.art3d import Poly3DCollection

import matplotlib.patches as mpatches


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

    print('\n------------- O B S T A C L E S   ( P A R A L L E L E P I P E D ) -------------')

    num_obs = 1  # <---

    info_par = []
    obs = []
    obs_array = []  # lista di variabili Point()

    for i in range(num_obs):
        info_par.append({'x': 0.0, 'y': 0.0, 'z': 0.0, 'a': 0.0, 'b': 0.0, 'c': 0.0})  # inizializzo

    # obs1
    info_par[0]['x'] = 1.0
    info_par[0]['y'] = 1.0
    info_par[0]['z'] = 0.8
    info_par[0]['a'] = 0.5
    info_par[0]['b'] = 0.5
    info_par[0]['c'] = 1.6
    '''
    # obs2
    info_par[1]['x'] = 1.6
    info_par[1]['y'] = - 2.4
    info_par[1]['z'] = 0.5
    info_par[1]['a'] = 0.8
    info_par[1]['b'] = 0.8
    info_par[1]['c'] = 0.8
    '''
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
        obs += sphere_approximation(info_par[i]['x'], info_par[i]['y'], info_par[i]['z'],
                                    info_par[i]['a'], info_par[i]['b'], info_par[i]['c'])

    spheres = obs

    # --- AGGIUNTO CHAT GPT ---

    x = info_par[0]['x']
    y = info_par[0]['y']
    z = info_par[0]['z']
    a = info_par[0]['a']
    b = info_par[0]['b']
    c = info_par[0]['c']

    '''
    vertices = np.array([
        [x-a/2, y-b/2, z-c/2],
        [x+a/2, y-b/2, z-c/2],
        [x+a/2, y+b/2, z-c/2],
        [x-a/2, y+b/2, z-c/2],
        [x-a/2, y-b/2, z+c/2],
        [x+a/2, y-b/2, z+c/2],
        [x+a/2, y+b/2, z+c/2],
        [x-a/2, y+b/2, z+c/2]
    ])
    '''

    # CONSEGNA TESI
    a = a * 0.8
    b = b * 0.8
    c = c

    vertices = np.array([
        [x - a / 2, y - b / 2, z - c / 2],
        [x + a / 2, y - b / 2, z - c / 2],
        [x + a / 2, y + b / 2, z - c / 2],
        [x - a / 2, y + b / 2, z - c / 2],
        [x - a / 2, y - b / 2, z + c / 2],
        [x + a / 2, y - b / 2, z + c / 2],
        [x + a / 2, y + b / 2, z + c / 2],
        [x - a / 2, y + b / 2, z + c / 2]
    ])

    # Definizione dei vertici del parallelepipedo
    faces = [
        [vertices[0], vertices[1], vertices[2], vertices[3]],
        [vertices[4], vertices[5], vertices[6], vertices[7]],
        [vertices[0], vertices[1], vertices[5], vertices[4]],
        [vertices[2], vertices[3], vertices[7], vertices[6]],
        [vertices[1], vertices[2], vertices[6], vertices[5]],
        [vertices[4], vertices[7], vertices[3], vertices[0]]
    ]

    # --- AGGIUNTO CHAT GPT - END ---

    # 3d figure
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')

    # drawing each sphere
    for sphere in spheres:
        x, y, z, r = sphere

        """
        u e v sono due griglie, tali che, per ogni punto (u[i,j], v[i,j]) sulla griglia, 
        si può ottenere una singola coordinata in longitudine e latitudine, cioè (theta, phi), dove:
        theta = u[i,j] , phi = v[i,j]
        
        Combinando tutti i valori di u e v, puoi coprire tutte le possibili coordinate sulla sfera.
        
        Esempio: se si ha una griglia u di dimensione 3x3 e una griglia v di dimensione 3x3, 
        si ottiene 9x9=81 coordinate possibili.
        """

        u, v = np.mgrid[0:2 * np.pi:2 * np.pi / 20,
               0:np.pi:np.pi / 10]  # u: angolo di longitudine, v: angolo di latitudine
        x = x + r * np.cos(u) * np.sin(v)
        y = y + r * np.sin(u) * np.sin(v)
        z = z + r * np.cos(v)
        # ax.scatter(x, y, z, color='black')
        ax.plot_surface(x, y, z, color='red')
        # ax.plot_wireframe(x, y, z, color='k', alpha=0.3)  # TODO: non funziona!

    ax.scatter([], [], [], color='black')

    # plot customization
    # ax.scatter(0, 0, 0, color='r')

    # AGGIUGNO PARALLELEPIPEDO
    ax.add_collection3d(Poly3DCollection(faces, facecolors='blue', edgecolors='black', alpha=0.2))

    legend_parallelepiped = mpatches.Patch(color='blue', alpha=0.2, label='Parallelepiped')
    legend_sphere = mpatches.Patch(color='red', label='Spheres approximation')
    # legend_origin = ax.scatter(0, 0, 0, color='r', label='Origin')

    ax.legend(handles=[legend_parallelepiped, legend_sphere])

    ax.set_xlim([0, 2.0])
    ax.set_ylim([0, 2.0])
    ax.set_zlim([0, 2.0])
    ax.set_xlabel('X')
    ax.set_ylabel('Y')
    ax.set_zlabel('Z')

    # showing plot
    plt.show()
