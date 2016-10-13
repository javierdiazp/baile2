#! /usr/bixn/env python
# -*- coding: utf-8 -*-

import roslib; roslib.load_manifest('baile2')
import rospy
from sensor_msgs.msg import JointState

# Brings in the SimpleActionClient
import actionlib

# Brings in the messages used by the Baile action, including the
# goal message and the result message.
import baile2.msg

# Constantes de estado
INIT_LOOP = 0
MOV_FORWARD = 1
TURNING = 2

# Estado del robot
state = INIT_LOOP

# Posicion inicial de cada rueda [izq, der]
iPos = [0, 0]

# Creates the SimpleActionClient, passing the type of the action
# (BaileAction) to the constructor.
client = actionlib.SimpleActionClient('baile', baile2.msg.BaileAction)

# Retorna True si el robot se está moviendo (con margen de error eps). False si no.
def isMoving(vel):
    eps = 0.001
    return abs(vel[0]) > eps or abs(vel[1]) > eps

# Funcion que se ejecutra cada vez que llega informacion del robot.
def callback(data):
    global iPos
    global state
    
    # Distancia que debe avanzar el robot antes de frenar.
    # Forma un lado del triangulo.
    distLin = 27
    
    # Distancia que avanzan las ruedas (en sentido contrario) para girar al robot.
    # Forma un vértice del triángulo.
    distAng = 7
    
    order = -1
    
    # Waits until the action server has started up and started
    # listening for goals.
    client.wait_for_server()

    if state == INIT_LOOP:
        iPos = data.position
        state = MOV_FORWARD
    
    if state == MOV_FORWARD:
        if data.position[0] - iPos[0] < distLin or data.position[1] - iPos[1] < distLin:
            order = MOV_FORWARD
        elif not isMoving(data.velocity):
            iPos = data.position 
            state = TURNING
    
    if state == TURNING:
        if data.position[0] - iPos[0] < distAng or data.position[1] - iPos[1] > 0-distAng:
            order = TURNING
        elif not isMoving(data.velocity):
            state = INIT_LOOP

    # Creates a goal to send to the action server.
    goal = baile2.msg.BaileGoal(order=order)

    # Sends the goal to the action server.
    client.send_goal(goal)

if __name__ == '__main__':
    try:
        # Initializes a rospy node so that the SimpleActionClient can
        # publish and subscribe over ROS.
        rospy.init_node('baile_client')
        rospy.Subscriber("/joint_states", JointState , callback)
        rospy.spin()
    except rospy.ROSInterruptException:
        print "program interrupted before completion"
