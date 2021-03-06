#!/usr/bin/env python3

import rospy
import time
from duckietown_msgs.msg import FSMState, Twist2DStamped

class Node:
	def __init__(self):
		rospy.Subscriber('fsm_node/mode', FSMState, self.talk)
		self.pub = rospy.Publisher('car_cmd_switch_node/cmd', Twist2DStamped, queue_size=10)
    
		self.moveCMD = Twist2DStamped()
		self.moveCMD.v = .7
		self.moveCMD.omega = 2

		self.stopCMD = Twist2DStamped()
		self.stopCMD.v = 0
		self.stopCMD.omega = 0

		self.flag = False
	def talk(self, msg):
		startTime = time.time()
		if msg.state == "LANE_FOLLOWING" and self.flag == False:
			rospy.logwarn(f"{msg.state}")
			self.flag = True
			for i in range(170):
				curTime = time.time()-startTime
				rospy.logwarn(f"{curTime}: move")
				self.pub.publish(self.moveCMD)
			rospy.logwarn(f"{curTime}: stop")
			self.pub.publish(self.stopCMD)
		else:
			rospy.logwarn(f"{msg.state}")
			if msg.state != "LANE_FOLLOWING":
				self.flag = False
			self.pub.publish(self.stopCMD)

if __name__ == '__main__':
	rospy.init_node('Lab2_2', anonymous=True)
	Node()
	rospy.spin()
