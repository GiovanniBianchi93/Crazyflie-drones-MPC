#! /usr/bin/env python3

"""
--- partosph: parallelepiped to spheres ---

Restituisce un elenco di sfere approssimative per il parallelepipedo
definito dalle misure a, b, c e con centro di massa definito in x_par, y_par, z_par.

Restituisce un elenco di (x, y, z, r), dove x, y, z sono le coordinate del centro della sfera
e r è il suo raggio.

Scegliere math.floor o math.ceil in base alle esigenze:

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

import math


# TODO: integrare file in MPC!

def sphere_approximation(x_par, y_par, z_par, a, b, c):
    # Calcola il raggio ottimo per approssimare le sfere
    max_r = min(a, b, c) / 2

    max_d = max_r * 2

    n_a = a / max_d
    n_b = b / max_d
    n_c = c / max_d

    n_a = math.floor(n_a)
    n_b = math.floor(n_b)
    n_c = math.floor(n_c)

    '''
    n_a = math.ceil(n_a)
    n_b = math.ceil(n_b)
    n_c = math.ceil(n_c)
    '''

    xs = [a / n_a * (0.5 + ii) for ii in range(n_a)]
    ys = [b / n_b * (0.5 + ii) for ii in range(n_b)]
    zs = [c / n_c * (0.5 + ii) for ii in range(n_c)]

    # Genera lista (di liste) con tutte le possibilità...
    sph_res = [[x, y, z, max_r] for x in xs for y in ys for z in zs]

    print('sph_res: ', sph_res)

    # traslo opportunamente: - a / 2 per centro di massa, + x_par per traslazione
    for sphere in sph_res:
        sphere[0] += x_par - a / 2
        sphere[1] += y_par - b / 2
        sphere[2] += z_par - c / 2

    print('sph_res with translations: ', sph_res)

    return sph_res


if __name__ == '__main__':
    info_par = {'x': 0.0, 'y': 0.0, 'z': 0.0, 'a': 0.0, 'b': 0.0, 'c': 0.0, }  # inizializzo

    print('\nEnter info parallelepiped: ')
    info_par['x'] = float(input("x:  "))
    info_par['y'] = float(input("y:  "))
    info_par['z'] = float(input("z:  "))
    info_par['a'] = float(input("a:  "))
    info_par['b'] = float(input("b:  "))
    info_par['c'] = float(input("c:  "))
    print(info_par, '\n')

    obs = []

    obs += sphere_approximation(info_par['x'], info_par['y'], info_par['z'],
                                info_par['a'], info_par['b'], info_par['c'])

    print('\nobs: ', obs)
    print('length obs: ', len(obs))
