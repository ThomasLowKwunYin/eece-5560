#!/usr/bin/env python3

import rospy
import numpy as np
from std_msgs.msg import Float32
from duckietown_msgs.msg import Twist2DStamped, FSMState, LanePose
import lab4Control as PID

class Node:
	def __init__(self):
		self.pub = rospy.Publisher('lane_controller_node/car_cmd', Twist2DStamped, queue_size=10)
		rospy.Subscriber('fsm_node/mode', FSMState, self.state)
		rospy.Subscriber('lane_filter_node/lane_pose', LanePose, self.control)
		
		dt = .01
		self.flag = False
		self.trim = .8
		
		PositionPIDParam = [2,0,0]
		self.PositiontErrorMax = 0.15
		self. PositiontErrorMin = -.12
		AnglePIDParam = [2,0,0]
		self.AngleErrorMax = 0.15
		self.AngleErrorMin = -.12
		
		self.PositionPID = PID.PID(PositionPIDParam, dt)
		self.AnglePID = PID.PID(AnglePIDParam, dt)		
		self.signPos = None
		
	def control(self, state):
        self.state = state.state
        
	def talk(self,msg)
		Vector = Twisted2DStamped()
		if self,state == "LANE_FOLLOWING":
			if self.flag == False:
				rospy.logwarn("Lane Following")
				self.flag = True
			if len(msg.detections) != 0:
				z = self.sign_Pos[0]
                x = self.sign_Pos[1]
			
			
