#! /usr/bin/env python

import sys

import rospy
from std_msgs.msg import Empty


def target_reached():
    print('\n--------- T A R G E T   R E A C H E D --------')

    land_pub.publish(land_trigger)
    rospy.sleep(2)

    # termino programma
    rospy.is_shutdown()
    sys.exit(1)


if __name__ == '__main__':
    rospy.init_node('prova_pub', log_level=rospy.DEBUG)

    land_pub = rospy.Publisher('/land_topic', Empty, queue_size=1)
    land_trigger = Empty()

    rospy.sleep(5)

    print('chiamo target_reached...')
    target_reached()
