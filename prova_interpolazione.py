#! /usr/bin/env python3

import numpy as np
from scipy.interpolate import interp1d

x = np.array([1, 2, 3, 4])  # Vettore di 4 elementi
z = np.array([0.185609579086304, 0.273277997970581, 0.476153612136841, 0.380399703979492])  # Vettore di 4 elementi

# Creazione della funzione interpolante
f = interp1d(x, z, kind='cubic')

# Generazione di zfine interpolato da xfine
xfine = np.linspace(min(x), max(x), 10)
zfine = f(xfine)

print(zfine)
