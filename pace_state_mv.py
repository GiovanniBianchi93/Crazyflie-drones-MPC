#! /usr/bin/env python

# pace that triggers the publication of the state

import rospy
from std_msgs.msg import Empty

if __name__ == '__main__':

    rospy.init_node('Clock_Node')

    pub = rospy.Publisher('/pace_state_topic', Empty, queue_size=1)

    pace_trigger = 10   # pace [Hz]
    # TODO: probabilmente non necessaria così alta,
    #  ma importante per assicurarsi che la pubblicazione dello stato avvenga appena possibile

    rate = rospy.Rate(pace_trigger)
    pace = Empty()

    print('Triggering state publication...')
    print('Pace [Hz] = ', pace_trigger)

    while not rospy.is_shutdown():
        pub.publish(pace)
        rate.sleep()

    print('Triggering state publication: done')
