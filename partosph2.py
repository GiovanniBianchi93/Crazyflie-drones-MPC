#! /usr/bin/env python3

import math

# TODO: il seguente codice funziona - MA: crea troppe sfere
#  che spesso si sovrappongono! - modificare migliorando codice...

def sphere_approximation(x_par, y_par, z_par, a, b, c, max_r):
    """
    Restituisce un elenco di sfere approssimative per il parallelepipedo definito dalle misure a, b, c.

    a: larghezza del parallelepipedo
    b: lunghezza del parallelepipedo
    c: altezza del parallelepipedo
    max_r: raggio massimo delle sfere approssimative

    Restituisce un elenco di tuple (x, y, z, r), dove x, y, z sono le coordinate del centro della sfera
    e r è il suo raggio.
    """

    # Calcola il volume del parallelepipedo
    vol = a * b * c

    # Calcola il raggio massimo delle sfere approssimative
    max_diam = math.sqrt(a ** 2 + b ** 2 + c ** 2)
    max_r = min(max_r, max_diam / 2)

    # Determina il numero di sfere approssimative necessarie
    n_spheres = math.ceil(vol * 4 / 3 * math.pi / (4 / 3 * math.pi * max_r ** 3))

    # Calcola le dimensioni approssimative di ogni sfera
    approx_r = (vol / n_spheres) ** (1 / 3)

    approx_r /= 2

    # Dividi il parallelepipedo in n_spheres cubi approssimativi
    xs = [a / n_spheres * (0.5 + ii) for ii in range(n_spheres)]
    ys = [b / n_spheres * (0.5 + ii) for ii in range(n_spheres)]
    zs = [c / n_spheres * (0.5 + ii) for ii in range(n_spheres)]

    # Genera lista (di liste) con tutte le possibilità...
    spheres_possibles = [[x, y, z, approx_r] for x in xs for y in ys for z in zs]

    # Rimuovi le sfere al di fuori del parallelepipedo # TODO: rivedere condizione...
    spheres_in_cube = []
    for sphere in spheres_possibles:
        x, y, z, radius = sphere[0], sphere[1], sphere[2], sphere[3]
        if (x - radius >= 0 and x + radius <= a
                and y - radius >= 0 and y + radius <= b
                and z - radius >= 0 and z + radius <= c):
            spheres_in_cube.append(sphere)

    # traslo opportunamente...  TODO: spiegare meglio
    for sphere in spheres_in_cube:
        sphere[0] += x_par - a / 2
        sphere[1] += y_par - b / 2
        sphere[2] += z_par - c / 2

    print('spheres_in_cube: ', spheres_in_cube)

    return spheres_in_cube


if __name__ == '__main__':

    num_parallelepipedi = int(input("inserire numero dei parallelepipedi: "))
    max_radius = float(input("inserire massimo raggio sfera approssimante: "))
    # TODO: aggiungere commento su massimo raggio -> len(obs) cresce di molto!

    obs = []

    for i in range(num_parallelepipedi):
        info_parallelepiped = {'x': 0.0, 'y': 0.0, 'z': 0.0, 'a': 0.0, 'b': 0.0, 'c': 0.0, }  # inizializzo
        print('\nEnter infos - parallelepiped %s: ' % (i + 1))
        info_parallelepiped['x'] = float(input("x:  "))
        info_parallelepiped['y'] = float(input("y:  "))
        info_parallelepiped['z'] = float(input("z:  "))
        info_parallelepiped['a'] = float(input("a:  "))
        info_parallelepiped['b'] = float(input("b:  "))
        info_parallelepiped['c'] = float(input("c:  "))
        print(info_parallelepiped)

        # TODO: ? obs da migliorare - ora solo una lista imprecisa /ok per fine MPC g... (o no?)
        obs += sphere_approximation(info_parallelepiped['x'],
                                    info_parallelepiped['y'],
                                    info_parallelepiped['z'],
                                    info_parallelepiped['a'],
                                    info_parallelepiped['b'],
                                    info_parallelepiped['c'],
                                    max_radius)

    print('\nobs: ', obs)
    print('length obs: ', len(obs))
