#! /usr/bin/env python3
import math


# TODO: il seguente codice funziona - MA: crea troppe sfere
#  che spesso si sovrappongono! - modificare migliorando codice...

def sphere_approximation(x_par, y_par, z_par, a, b, c):
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

    # Calcola il raggio ottimo per approssimare le sfere
    max_r = min(a, b, c) / 2
    # max_r = (a * b * c)**(1/3) / (3*(a**(-2/3) + b**(-2/3) + c**(-2/3)))**(1/3)
    # altra possibilità /? raggio di maxwell?

    k = 0.74

    n_spheres = int((a / max_r) * (b / max_r) * (c / max_r) * k)

    # Dividi il parallelepipedo in n_spheres cubi approssimativi
    xs = [a / n_spheres * (0.5 + ii) for ii in range(n_spheres)]
    ys = [b / n_spheres * (0.5 + ii) for ii in range(n_spheres)]
    zs = [c / n_spheres * (0.5 + ii) for ii in range(n_spheres)]

    # Genera lista (di liste) con tutte le possibilità...
    spheres_possibles = [[x, y, z, max_r] for x in xs for y in ys for z in zs]

    # Rimuovi le sfere al di fuori del parallelepipedo # TODO: rivedere condizione...
    spheres_in_cube = []
    for sphere in spheres_possibles:
        x, y, z, radius = sphere[0], sphere[1], sphere[2], sphere[3]
        if (x - radius >= 0 and x + radius <= a
                and y - radius >= 0 and y + radius <= b
                and z - radius >= 0 and z + radius <= c):
            spheres_in_cube.append(sphere)

    # traslo opportunamente: - a / 2 per centro di massa, + x_par per traslazione
    for sphere in spheres_in_cube:
        sphere[0] += x_par - a / 2
        sphere[1] += y_par - b / 2
        sphere[2] += z_par - c / 2

    print('spheres_in_cube: ', spheres_in_cube)

    return spheres_in_cube


if __name__ == '__main__':

    info_parallelepiped = {'x': 0.0, 'y': 0.0, 'z': 0.0, 'a': 0.0, 'b': 0.0, 'c': 0.0, }  # inizializzo
    print('\nEnter infos parallelepiped: ')
    info_parallelepiped['x'] = float(input("x:  "))
    info_parallelepiped['y'] = float(input("y:  "))
    info_parallelepiped['z'] = float(input("z:  "))
    info_parallelepiped['a'] = float(input("a:  "))
    info_parallelepiped['b'] = float(input("b:  "))
    info_parallelepiped['c'] = float(input("c:  "))
    print(info_parallelepiped)

    obs = []

    obs += sphere_approximation(info_parallelepiped['x'],
                                info_parallelepiped['y'],
                                info_parallelepiped['z'],
                                info_parallelepiped['a'],
                                info_parallelepiped['b'],
                                info_parallelepiped['c'])

    print('\nobs: ', obs)
    print('length obs: ', len(obs))
