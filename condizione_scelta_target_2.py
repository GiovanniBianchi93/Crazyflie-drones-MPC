#! /usr/bin/env python3

"""
Supponiamo di avere un array A di n variabili x y z;
considerato un secondo array B sempre di n variabili x, y, z,
vorrei riordinare tale array B, ottenendo un array C di n variabili x y z tale che
la distanza dei punti contenuti in C[i] e A[i] sia la minima possibile.
"""

###################################################################

#      O P Z I O N E   1

###################################################################
'''
import math

import numpy as np
from scipy.spatial.distance import cdist


def nearest_neighbour_indexing(a, b):
    c = np.zeros_like(a)
    for i in range(len(a)):
        distances = cdist(a[i, :][np.newaxis, :], b)
        nearest_idx = np.argmin(distances)
        c[i, :] = b[nearest_idx, :]
        b = np.delete(b, nearest_idx, axis=0)
    return c


# definizione di n droni e n target positions come liste di tuple
A = [(1, 1, 1), (2, 2, 2), (3, 3, 3)]
B = [(3.1, 3.1, 3.1), (0.3, 0.3, 0.3), (1.7, 1.7, 1.7)]

C = nearest_neighbour_indexing(A, B)

print(C)
'''
###################################################################

#      O P Z I O N E   2

###################################################################

import numpy as np

def nearest_neighbor(A, B):
    n = len(B)
    dist = np.zeros(n)
    order = np.zeros(n, dtype=int)
    for i in range(n):
        dist[i] = np.linalg.norm(A - B[i])
        order[0] = np.argmin(dist)
    for i in range(1, n):
        dist[order[i - 1]] = np.inf
        order[i] = np.argmin(dist)
    return B[order]


A = np.array([0, 0, 0])
B = np.array([[9, 9, 9], [8, 8, 8], [7, 7, 7], [3, 3, 3], [1, 1, 1]])
B_sorted = nearest_neighbor(A, B)
print(B_sorted)
