#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Point
from std_msgs.msg import Float32


def target():
    mpc_target_pub = rospy.Publisher('/swarm/mpc_target', Point, queue_size=1)
    mpc_target = Point()

    # Acquisisce mpc_target da terminale
    mpc_target.x = float(input("mpc_target.x : "))
    mpc_target.y = float(input("mpc_target.y: "))
    mpc_target.z = float(input("mpc_target.z: "))

    # Pubblica mpc_target
    mpc_target_pub.publish(mpc_target)

def offset():
    offset_swarm = [Point(x=1.0, y=2.0, z=3.0), Point(x=2.0, y=3.0, z=4.0)]  # sostituisci con i valori desiderati

    # offset_pub = rospy.Publisher('/swarm/offset', Point[], queue_size=1)

    # Acquisisce 3 numeri float da terminale
    numero1 = float(input("Inserisci il primo numero: "))
    numero2 = float(input("Inserisci il secondo numero: "))
    numero3 = float(input("Inserisci il terzo numero: "))

    # Pubblica i numeri sul topic 'numeri'
    rospy.loginfo("Sto pubblicando i numeri sul topic 'numeri'")
    # offset_pub.publish(Float32(numero1))
    # offset_pub.publish(Float32(numero2))
    # offset_pub.publish(Float32(numero3))

if __name__ == '__main__':

    rospy.init_node('input_MPC')

    target()
    offset()

    rospy.is_shutdown()
