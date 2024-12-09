#! /usr/bin/env python3

"""
Matplotlib Animation Example - Random 3D Animation
"""
import random
import time

import numpy as np
from matplotlib import pyplot as plt
from matplotlib import animation
import mpl_toolkits.mplot3d.axes3d as p3

# initialization function: plot the background of each frame
def init():
    line.set_data_3d([], [], [])  # inizializzo
    return line,


# animation function.  This is called sequentially
def animate(ii):
    x = np.random.random()
    y = np.random.random()
    z = np.random.random()
    line.set_data_3d(np.append(line._verts3d[0], x),
                     np.append(line._verts3d[1], y),
                     np.append(line._verts3d[2], z))
    return line,


if __name__ == '__main__':
    # First set up the figure, the axis, and the plot element we want to animate
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')

    # Set plot properties
    ax.set_xlim3d([0, 1.0])
    ax.set_xlabel('X')
    ax.set_ylim3d([0, 1.0])
    ax.set_ylabel('Y')
    ax.set_zlim3d([0, 1.0])
    ax.set_zlabel('Z')
    ax.set_title('3D Random Animation')

    line, = ax.plot3D([], [], [], lw=1)  # creo istanza Line3D

    # call the animator.  blit=True means only re-draw the parts that have changed.
    anim = animation.FuncAnimation(fig, animate, init_func=init,
                                   frames=200, interval=20, blit=True)

    '''
    # save the animation as an mp4.  This requires ffmpeg or mencoder to be
    # installed.  The extra_args ensure that the x264 codec is used, so that
    # the video can be embedded in html5.  You may need to adjust this for
    # your system: for more information, see
    # http://matplotlib.sourceforge.net/api/animation_api.html
    # anim.save('basic_animation.mp4', fps=30, extra_args=['-vcodec', 'libx264'])
    '''

    plt.show()
