#! /usr/bin/env python3
# ROS MODULES
import rospy
import time

# CUSTOM MODULES
from crazy_common_py.dataTypes import Vector3
from crazyflie_drone.CrazyDrone4Swarm import CrazyDrone4Swarm

from crazyflie_drone.CrazyDrone import CrazyDrone

# Function called when the node is shutdown, in order to perform exiting operations:
# def exiting_hook():
#     global drone
#     drone.exit_operations()

# def compute_address(name):

    #num_ID = int(name[2:]) - 1
    
    # print('num_ID is: ', num_ID)
    # print('radio address is: //0/80/2M/E7E7E7E7E'+ hex(num_ID)[-1])
    
    #return 'radio://0/80/2M/E7E7E7E7E' + hex(num_ID)[-1]
    
    #return 'radio://0/80/2M/E7E7E7E7E2'

    # print('radio address is: //0/80/2M/E7E7E7E7E0')
    # return 'radio://0/80/2M/E7E7E7E7E0'

if __name__ == '__main__':
    # Node initialization:
    rospy.init_node('crazyflie_real_node', log_level=rospy.DEBUG)

    # Extracting rosparam informations (to understand the name and spawn position):
    # crazyflie_name = rospy.get_param('crazyflie_real_node/name')
    # initial_pos = rospy.get_param('crazyflie_real_node/initial_position')

    # Creating CrazyDrone4Swarm instance (to control a single drone in a swarm):
    # drone = CrazyDrone4Swarm(crazyflie_name, compute_address(crazyflie_name), Vector3(initial_pos[0], initial_pos[1], initial_pos[2]))
    
    #drone = CrazyDrone(crazyflie_name, compute_address(crazyflie_name), Vector3(initial_pos[0], initial_pos[1], initial_pos[2]))
    # drone = CrazyDrone(crazyflie_name, 'radio://0/80/2M/E7E7E7E7E7', Vector3(initial_pos[0], initial_pos[1], initial_pos[2]))
    drone = CrazyDrone('cf1', 'radio://0/80/2M/E7E7E7E7E0', Vector3(0, 0, 0))

    #time.sleep(10)

    '''# CrazyDrone4Swarm instance:
    drone = CrazyDrone4Swarm('cf1', 'radio://0/80/2M/E7E7E7E7E7', Vector3(0, 0, 0))'''

    '''
    radio://USB_DONGLE_NUMBER/RADIO_CHANNEL/RADIO_SPEED(kbit/s)/CRAZYFLIE_ID
    '''

    rospy.spin()

    # rospy.on_shutdown(exiting_hook)
