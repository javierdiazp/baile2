#! /usr/bixn/env python
# -*- coding: utf-8 -*-

import sys, signal

import roslib; roslib.load_manifest('baile2')
import rospy
from sensor_msgs.msg import JointState

# Brings in the SimpleActionClient
import actionlib

# Brings in the messages used by the Baile action, including the
# goal message and the result message.
import baile2.msg

# Constantes de estado
MOV_FORWARD = 1
TURNING = 2

# Creates the SimpleActionClient, passing the type of the action
# (BaileAction) to the constructor.
client = actionlib.SimpleActionClient('baile', baile2.msg.BaileAction)

# Para cerrar el programa con Control-C
signal.signal(signal.SIGINT, lambda s, f: sys.exit(0))

if __name__ == '__main__':
    try:
        # Initializes a rospy node so that the SimpleActionClient can
        # publish and subscribe over ROS.
        rospy.init_node('baile_client')
        while True:
            # Que avance
            client.wait_for_server()
            goal = baile2.msg.BaileGoal(order=MOV_FORWARD)
            client.send_goal(goal)
            client.wait_for_result()
            
            # Que gire
            client.wait_for_server()
            goal = baile2.msg.BaileGoal(order=TURNING)
            client.send_goal(goal)
            client.wait_for_result()

    except rospy.ROSInterruptException:
        print "program interrupted before completion"
