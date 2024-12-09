#! /usr/bin/env python3

import time
import os

# ROS MODULES
import rospy

# CUSTOM MODULES
from crazyflie_drone.CrazyDrone import CrazyDrone

#from crazyflie_drone.CrazyDrone4Swarm import CrazyDrone4Swarm

from crazy_common_py.dataTypes import Vector3

from cflib.positioning.motion_commander import MotionCommander

# Function called when the node is shutdown, in order to perform exiting operations:
# def exiting_hook():
#     global drone
#     drone.exit_operations()

# def compute_address(name):  

    #num_ID = int(name[2:]) - 1
    
    # print('num_ID is: ', num_ID)
    # print('radio address is: //0/80/2M/E7E7E7E7E'+ hex(num_ID)[-1])

    #return 'radio://0/80/2M/E7E7E7E7E' + hex(num_ID)[-1]

if __name__ == '__main__':

    #created folder and file for debug purposes - general
    filepath = os.path.join('/home/matteo/catkin_ws/src/my_debug_print', 'my_file.txt')
    if not os.path.exists('/home/matteo/catkin_ws/src/my_debug_print'):
        os.makedirs('/home/matteo/catkin_ws/src/my_debug_print')
    f = open(filepath, "w")
    f.close()

    #debug - proportional control
    filepath1 = os.path.join('/home/matteo/catkin_ws/src/my_debug_print', 'my_file_P_control.txt')
    if not os.path.exists('/home/matteo/catkin_ws/src/my_debug_print'):
        os.makedirs('/home/matteo/catkin_ws/src/my_debug_print')
    f1 = open(filepath1, "w")
    f1.close()

    # Node initialization:
    rospy.init_node('crazyflie_real_node', log_level=rospy.DEBUG)

    # Extracting rosparam informations (from launch file):
    crazyflie_name = rospy.get_param('crazyflie_real_node/name', 'cf1')
    initial_pos = rospy.get_param('crazyflie_real_node/initial_position', [0.0, 0.0, 0.0])

    # Creating CrazyDrone4Swarm instance (to control a single drone in a swarm):
    # drone = CrazyDrone4Swarm(crazyflie_name, compute_address(crazyflie_name), Vector3(initial_pos[0], initial_pos[1], initial_pos[2]))
    
    # Creating CrazyDrone instance
    #drone = CrazyDrone(crazyflie_name, compute_address(crazyflie_name), Vector3(initial_pos[0], initial_pos[1], initial_pos[2]))
    drone = CrazyDrone(crazyflie_name, 'radio://0/80/2M/E7E7E7E7E0', Vector3(initial_pos[0], initial_pos[1], initial_pos[2]))

    #time.sleep(3) #? - perche 10sec? - necessari per creazione corretta istanza?

    #aggiungere varie azioni 8ad esempio: decollo,... - publication,...)



    ##prova
    #drone.__mc.take_off(height=0.5)
    #time.sleep(3)
    #drone.__mc.land()

    rospy.spin()
    # rospy.on_shutdown(exiting_hook)
