#! /usr/bin/env python
import os
import sys

import rospy
from std_msgs.msg import Empty


def my_exit():
    print('\nspengo ROS')  # debug
    print('spengo terminale')  # debug

    rospy.is_shutdown()
    sys.exit(1)  # TODO: non funziona! (???!)

    # alternativa: os._exit(1)  # non funziona!


def swarm_land_act_callback(msg):
    print('about to land...')
    rospy.sleep(2)

    my_exit()


if __name__ == '__main__':
    rospy.init_node('prova_sub', log_level=rospy.DEBUG)

    land_sub = rospy.Subscriber('/land_topic', Empty, swarm_land_act_callback, queue_size=1)

    rospy.spin()
