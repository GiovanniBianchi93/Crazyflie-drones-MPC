#! /usr/bin/env python3

"""
programma per valutare se i segmenti di una linea spezzata attraversano una sfera - ok! funzionante e robusto!
TODO: da integrare in codice mpc_swarm_mv_backup24may.py per evitare ostacolo!
"""

from math import sqrt

import numpy as np
import matplotlib.pyplot as plt
import mpl_toolkits.mplot3d.axes3d as p3


def is_segment_intersecting_sphere(segment_start, segment_end, sphere_center, sphere_radius):
    # Calcola il vettore direzione del segmento
    segment_dir = segment_end - segment_start

    # Calcola la distanza tra il centro della sfera e l'inizio del segmento
    start_to_center = sphere_center - segment_start

    # Calcola la proiezione della distanza sul vettore direzione del segmento
    projection = np.dot(start_to_center, segment_dir) / np.dot(segment_dir, segment_dir)

    # Calcola il punto pi\ù vicino sul segmento al centro della sfera
    closest_point = segment_start + np.clip(projection, 0, 1) * segment_dir

    # Verifica se il punto più vicino è all'interno della sfera
    distance = np.linalg.norm(closest_point - sphere_center)
    if distance <= sphere_radius:
        return True
    else:
        return False


# Definizione dei segmenti della linea spezzata
segments = np.array([[-0.75, -0.75, 0.5], [0.0, 0.25, 0.25]])

# Centro e raggio della sfera
sphere_center = np.array([0, 0, 0])
sphere_radius = 0.5

# Verifica se i segmenti attraversano la sfera
for i in range(len(segments) - 1):
    start = segments[i]
    end = segments[i + 1]
    if is_segment_intersecting_sphere(start, end, sphere_center, sphere_radius):
        print("Il segmento", i + 1, "attraversa la sfera.")
    else:
        print("Il segmento", i + 1, "non attraversa la sfera.")

# Visualizzazione dei segmenti e della sfera
fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')
ax.plot(segments[:, 0], segments[:, 1], segments[:, 2], 'black', label='Trajectory')
ax.scatter(sphere_center[0], sphere_center[1], sphere_center[2], color='r', label='Center of the sphere')
u = np.linspace(0, 2 * np.pi, 100)
v = np.linspace(0, np.pi, 100)
x = sphere_radius * np.outer(np.cos(u), np.sin(v)) + sphere_center[0]
y = sphere_radius * np.outer(np.sin(u), np.sin(v)) + sphere_center[1]
z = sphere_radius * np.outer(np.ones(np.size(u)), np.cos(v)) + sphere_center[2]
ax.plot_surface(x, y, z, color='r', alpha=0.5)

offset_axtext = 0.1

for i, point in enumerate(segments):
    ax.scatter(point[0], point[1], point[2], color='black')
    if i == 0:
        ax.text(point[0] + offset_axtext, point[1] + offset_axtext, point[2] + offset_axtext, 'x(k)', color='black', fontsize=12)
    else:
        ax.text(point[0] + offset_axtext, point[1] + offset_axtext, point[2] + offset_axtext, f'x(k+{i})', color='black', fontsize=12)


ax.set_xlabel('X')
ax.set_ylabel('Y')
ax.set_zlabel('Z')
ax.set_xlim([-1, 1])
ax.set_ylim([-1, 1])
ax.set_zlim([-1, 1])

# Adding a legend
ax.legend()

plt.show()
