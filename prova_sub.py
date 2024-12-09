#! /usr/bin/env python3

from crazy_common_py.common_functions import standardNameList

import time

import rospy
from crazyflie_messages.msg import Position, SphereData

from geometry_msgs.msg import Point
from std_msgs.msg import Int32

###########################################################################

#                S U B S C R I B E R     C A L L B A C K S

###########################################################################

def get_N_cf(msg):
    global N_cf
    N_cf = msg


def get_scelta_target(msg):
    global scelta_target
    scelta_target = msg


def get_mpc_target(msg):
    global mpc_target
    mpc_target = msg


def get_tgt4drone(msg):
    global tgt4drone_array
    tgt4drone_array.append(msg)


def get_obs(msg):
    global obs_array
    obs_array.append(msg)


def check_messages_received():
    global all_messages_received

    if N_cf.data != 0 and scelta_target.data == 1 and len(obs_array) != 0:
        if mpc_target.desired_position.x != 0.0 or mpc_target.desired_position.y != 0.0 or mpc_target.desired_position.z != 0.0:
            all_messages_received = True

    if N_cf.data != 0 and scelta_target.data == 2 and len(obs_array) != 0:
        if len(tgt4drone_array) != 0:
            all_messages_received = True


##################################################################

#                            M A I N

##################################################################

if __name__ == '__main__':
    rospy.init_node('node_mpc_swarm_mv', log_level=rospy.DEBUG)

    ##############################################################

    #               S U B S C R I B E R S   S E T U P

    ##############################################################

    all_messages_received = False

    N_cf_sub = rospy.Subscriber('N_cf_topic', Int32, callback=get_N_cf)
    N_cf = Int32()

    scelta_target_sub = rospy.Subscriber('scelta_target_topic', Int32, callback=get_scelta_target)
    scelta_target = Int32()

    mpc_target_sub = rospy.Subscriber('/swarm/mpc_target', Position, callback=get_mpc_target)
    mpc_target = Position()

    tgt4drone_sub = rospy.Subscriber('/tgt4drone_topic', Point, callback=get_tgt4drone)
    tgt4drone_array = []

    obs_sub = rospy.Subscriber('/obs_topic', SphereData, callback=get_obs)
    obs_array = []

    while not all_messages_received:
        check_messages_received()
        time.sleep(0.1)

    print('tutti i messaggi sono stati ricevuti correttamente!\n')

    print('messaggi ricevuti: \n\n')

    print("N_cf = ", N_cf.data, '\n')
    print("scelta target = ", scelta_target.data, '\n')
    print("mpc_target = ", mpc_target, '\n')
    print("tgt4drone_array = ", tgt4drone_array, '\n')
    print("ostacoli = ", obs_array, '\n')

    print('\nfine')
