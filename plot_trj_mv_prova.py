#! /usr/bin/env python3

"""
Tentativo per avere plot in modulo opportuno - RuntimeError: main thread is not in main loop
soluzione: provare a utilizzare librerie come PyQt o PySide per risolvere il problema (?)

---

RuntimeError: main thread is not in main loop

Questo errore si verifica quando si tenta di creare una finestra di grafica
in un thread separato dal thread principale.

Per risolvere questo problema, puoi provare a creare una finestra di grafica
nel thread principale e passare l'oggetto della finestra di grafica al thread separato.
In questo modo, il thread separato può accedere alla finestra di grafica creata
dal thread principale senza causare errori.

tbn: se si vuole inserire plot_trj in un nuovo modulo da importare,
al fine di avere codice più pulito, si potrebbe provare a utilizzare
librerie come PyQt o PySide per creare finestre di grafica in thread separati.
Queste librerie gestiscono automaticamente la creazione della finestra
di grafica nel thread appropriato. (da provare(?))
"""

import threading

import numpy as np

from matplotlib import pyplot as plt, animation
import mpl_toolkits.mplot3d.axes3d as p3


def plot_trj(local_N_cf):
    x_plot = [0.0 for i in range(local_N_cf)]
    y_plot = [0.0 for i in range(local_N_cf)]
    z_plot = [0.0 for i in range(local_N_cf)]
    # TODO: provvisorio -> global in int main per utilizzo in pace_sub_callback

    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')

    # Set plot properties
    ax.set_xlabel('X')
    ax.set_ylabel('Y')
    ax.set_zlabel('Z')
    ax.set_title('3D Trajectory Animation')

    lines = []
    for i in range(local_N_cf):
        line, = ax.plot3D([], [], [], lw=1, label=f'cf {i + 1}')  # creo istanza Line3D
        lines.append(line)

    # initialization function: plot the background of each frame
    def init():
        for ii in range(local_N_cf):
            lines[ii].set_data_3d([], [], [])
        return lines

    # animation function: this is called sequentially
    def animate(frame_i):
        for ii in range(local_N_cf):
            x_plot[ii] = np.random.random()  # TODO: provvisorio
            y_plot[ii] = np.random.random()
            z_plot[ii] = np.random.random()
            lines[ii].set_data_3d(np.append(lines[ii]._verts3d[0], x_plot[ii]),
                                  np.append(lines[ii]._verts3d[1], y_plot[ii]),
                                  np.append(lines[ii]._verts3d[2], z_plot[ii]))
        return lines

    # call the animator.  blit=True means only re-draw the parts that have changed.
    anim = animation.FuncAnimation(fig, animate, init_func=init,
                                   frames=None, interval=50, blit=True)
    ax.legend()
    plt.show()

    """
    Per salvare l'animazione, seguire le istruzioni di seguito:
    
    save the animation as an mp4.  This requires ffmpeg or mencoder to be
    installed.  The extra_args ensure that the x264 codec is used, so that
    the video can be embedded in html5.  You may need to adjust this for
    your system: for more information, see
    http://matplotlib.sourceforge.net/api/animation_api.html
    anim.save('basic_animation.mp4', fps=30, extra_args=['-vcodec', 'libx264'])
    """
