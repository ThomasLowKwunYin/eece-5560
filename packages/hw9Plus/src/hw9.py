#!/usr/bin/env python3

import rospy
from std_msgs.msg import Float32
import PIDControllerClass as PID

class node:
	def __init__(self):
	
		= rospy.Subscriber("/error", Float32, self.control)
		self.pub = rospy.Publisher("/control_input", Float32, queue_size=10)
		
		rospy.set_param("controller_ready", "true") 
        	rospy.set_param("graph_ready", "true") 
        	
        	kp = 0
        	ki = 0
        	kd = 0
        	
        	self.K = [kp, ki, kd]
		self.dt = 0.01
		self.PIDClass = PID.PID(self.K, self.dt)
		
	def control(self, error):
		rospy.loginfo(error.data)
		sumPID = self.PIDClass.run(error.data)
		self.pub.publish(sumPID)
		
if __name__ == "__main__":
	rospy.init_node("hw9", anonymous=True)
	node()
	rospy.spin()
