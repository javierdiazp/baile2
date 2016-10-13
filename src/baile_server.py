#! /usr/bin/env python
# -*- coding: utf-8 -*-

import roslib; roslib.load_manifest('baile2')
import rospy

from geometry_msgs.msg import Twist, Vector3
from sensor_msgs.msg import JointState

import actionlib

import baile2.msg

# Constantes de estado
INIT_LOOP = 0
MOV_FORWARD = 1
TURNING = 2

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
    
    vLin = 0
    vAng = 0
    
    # start executing the action
    # Posicion inicial de cada rueda [izq, der]
    if goal.order == MOV_FORWARD:
        vLin = 0.3
    elif goal.order == TURNING:
        vAng = -0.3
        
    self.pub.publish(Vector3(vLin, 0, 0), Vector3(0, 0, vAng))
      
    self._result.sequence = self._feedback.sequence
    self._as.set_succeeded(self._result)
      
if __name__ == '__main__':
  rospy.init_node('baile')
  BaileAction(rospy.get_name())
  rospy.spin()
