#!/usr/bin/env python3

import math
import rospy
from duckietown_msgs import FSMState, Twist2DStamped

class Node:
	def __init__(self):
		
		rospy.Subscriber('fsm_node/node', FSMState, self.talk)
		self.pub = rospy.Publisher('car_cmd_switch_node/car_cmd', Twist2DStamped, queue_size=10)
    
    self.moveCMD = Twist2DStamped()
    self.moveCMD.v = 8.3
    self.moveCMD.omega = 0
    
    self.stopCMD = Twist2DStamped()
    self.stopCMD.v = 0
    self.stopCMD.omega = 0
    
    self.flag = False
    
	def talk(self, msg)
    
    if msg.state == "LANE_FOLLOWING" and self.flag == False
      self.flag = true
      for i in range(102) #for 102 ticks
      
if __name__ == '__main__':
  rospy.init_node('hw6Node', anonymous=True)
	Node()
	rospy.spin()
      
        
  
