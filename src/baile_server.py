#! /usr/bin/env python
# -*- coding: utf-8 -*-

import roslib; roslib.load_manifest('baile2')
import rospy

from geometry_msgs.msg import Twist, Vector3
from sensor_msgs.msg import JointState

import actionlib

import baile2.msg

# Constantes de estado
MOV_FORWARD = 1
TURNING = 2

# True si el robot se está moviendo (con margen de error eps). False si no.
isMoving = False

# Posicion actual de cada rueda [izq, der]
curPos = [0, 0]

class BaileAction(object):
  # create messages that are used to publish feedback/result
  _feedback = baile2.msg.BaileFeedback()
  _result   = baile2.msg.BaileResult()

  def __init__(self, name):
    # Topico donde se publican los comando para que el robot se mueva.
    self.pub = rospy.Publisher("/cmd_vel_mux/input/teleop", Twist, queue_size=10)
    
    self._action_name = name
    self._as = actionlib.SimpleActionServer(self._action_name, baile2.msg.BaileAction, execute_cb=self.execute_cb, auto_start = False)
    self._as.start()
    
  def execute_cb(self, goal):
    self._feedback.sequence = []
    
    # Distancia que debe avanzar el robot antes de frenar.
    # Forma un lado del triangulo.
    distLin = 27
    
    # Distancia que avanzan las ruedas (en sentido contrario) para girar al robot.
    # Forma un vértice del triángulo.
    distAng = 7
    
    # Posicion inicial de cada rueda [izq, der]
    iPos = curPos
    
    if goal.order == MOV_FORWARD:
        while curPos[0] - iPos[0] < distLin or curPos[1] - iPos[1] < distLin:
            self.pub.publish(Vector3(0.3, 0, 0), Vector3(0, 0, 0))
    elif goal.order == TURNING:
        while curPos[0] - iPos[0] < distAng or curPos[1] - iPos[1] > 0-distAng:
            self.pub.publish(Vector3(0, 0, 0), Vector3(0, 0, -0.3))
    
    while isMoving:
        self.pub.publish(Vector3(0, 0, 0), Vector3(0, 0, 0))
    
    self._result.sequence = self._feedback.sequence
    self._as.set_succeeded(self._result)

def callback(data):
    global isMoving
    global curPos

    # Margen de error
    eps = 0.001
    
    isMoving = abs(data.velocity[0]) > eps or abs(data.velocity[1]) > eps
    curPos = data.position
      
if __name__ == '__main__':
  rospy.init_node('baile')
  BaileAction(rospy.get_name())
  rospy.Subscriber("/joint_states", JointState , callback)
  rospy.spin()
