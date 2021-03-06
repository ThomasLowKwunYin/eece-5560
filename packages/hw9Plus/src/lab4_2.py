#!/usr/bin/env python3

import rospy
import numpy as np
from std_msgs.msg import Float32
from duckietown_msgs.msg import Twist2DStamped, FSMState, LanePose
import lab4Control as PID

class Node:
	def __init__(self):
		self.pub = rospy.Publisher('lane_controller_node/car_cmd', Twist2DStamped, queue_size=10)
		rospy.Subscriber('fsm_node/mode', FSMState, self.control)
		rospy.Subscriber('lane_filter_node/lane_pose', LanePose, self.talk)
		
		dt = .01
		self.flag = False
		self.trim = .8
		
		PositionPIDParam = [2,0,0]
		self.PositiontErrorMax = 0.15
		self. PositiontErrorMin = -.12
		AnglePIDParam = [1,0,0]
		self.AngleErrorMax = 0.15
		self.AngleErrorMin = -.12
		
		self.PositionPID = PID.PID(PositionPIDParam, dt)
		self.AnglePID = PID.PID(AnglePIDParam, dt)		
		self.signPos = None
		
	def control(self, state):
		self.state = state.state

	def talk(self,msg):
		Vector = Twist2DStamped() #drive vector
		rospy.logwarn("start")
		if self.state == "LANE_FOLLOWING":
			rospy.logwarn(self.state)
			self.flag = True
			#turning adjectments
			PosError = 0 - msg.d
			AngError = 0 - msg.phi
			rospy.logwarn(f"PosError:{PosError}, AngError:{AngError}")
			if PosError > self.PositiontErrorMax:
				PosError = self.PositiontErrorMax
			elif PosError< self.PositiontErrorMin:
				PosError = self.PositiontErrorMin
				
			error = self.PositionPID.talk(PosError) + self.AnglePID.talk(AngError)
			vError = abs(0.4*(error/(1-error)))
			rospy.logwarn(f"Vector Error:{str(vError)} ")
			if vError <.4:
				Vector.v = .4-vError
			else:
				Vector.v = 0
				error = error*3
			rospy.logwarn(f"Error:{error} ")
			Vector.omega  = self.trim + error
			self.pub.publish(Vector)
			
		else:
			if self.state != "LANE_FOLLOWING":
				rospy.logwarn(self.state)
				self.flag = False
			Vector.v = 0
			Vector.omega = 0
			self.pub.publish(Vector)
		#rospy.logwarn(f"Vector:{Vector} ")

if __name__ == "__main__":
	rospy.init_node("lab4", anonymous=True)
	Node()
	rospy.spin()
