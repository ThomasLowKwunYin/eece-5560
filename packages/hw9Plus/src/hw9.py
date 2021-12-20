#!/usr/bin/env python3

import rospy
from std_msgs.msg import Float32

class node:
	def __init__(self):
		rospy.Subscriber("/error", Float32, self.talk)
		self.pub = rospy.Publisher("/control_input", Float32, queue_size=10)

		rospy.set_param("controller_ready", "true") 
		rospy.set_param("graph_ready", "true") 

		self.kp = 0.47
		self.ki = 0
		self.kd = 0
		self.error = 0
		self.prevError = 0
		self.errorDelta = 0
		self.dt = 0.01
		self.timeInit = rospy.get_time()
	def talk(self, msg):
		rospy.logwarn(msg.data)
		time = rospy.get_time()
		self.error += msg.data*(time-self.timeInit)
		self.errorDelta = (self.error-self.prevError)/(time-self.timeInit)
		self.timeInit = time
		
		self.prevError = msg.data
		output = (self.kp*msg.data)+(self.ki*self.error)+(self.kd*self.errorDelta)
		self.pub.publish(output)
		
if __name__ == "__main__":
	rospy.init_node("hw9", anonymous=True)
	node()
	rospy.spin()
