#!/usr/bin/env python3

import rospy
import time
from duckietown_msgs.msg import FSMState, Twist2DStamped

class Node:
	def __init__(self):
		rospy.Subscriber('fsm_node/mode', FSMState, self.talk)
		self.pub = rospy.Publisher('car_cmd_switch_node/cmd', Twist2DStamped, queue_size=10)
    
		self.moveCMD = Twist2DStamped()
		self.moveCMD.v = .41
		self.moveCMD.omega = 0
    
    		self.turnCMD = Twist2DStamped()
		self.turnCMD.v = 0
		self.turnCMD.omega = 2.1

		self.stopCMD = Twist2DStamped()
		self.stopCMD.v = 0
		self.stopCMD.omega = 0
    
    		self.flag = False
    
	def talk(self, msg):
		startTime = time.time()
		
		if msg.state == "LANE_FOLLOWING" and self.flag == False:
			rospy.logwarn(f"{msg.state}")
			self.flag = True
			for a in range(4):
				curTime = time.time()-startTime
				rospy.logwarn(f"{curTime}: cycle {a}")
		      		for i in range(102): #for 102 ticks, apporximately 3 sec
					curTime = time.time()-startTIme
					rospy.logwarn(f"{curTime}: move")
					self.pub.publish(self.moveCMD)
				for j in range(10):
					curTime = time.time()-startTIme
					rospy.logwarn(f"{curTime}: stop")
					self.pub.publish(self.stopCMD)
				for k in range(15):
					curTime = time.time()-startTIme
					rospy.logwarn(f"{curTime}: turn")
					self.pub.publish(self.turnCMD)
				for j in range(10):
					curTime = time.time()-startTIme
					rospy.logwarn(f"{curTime}: stop")
					self.pub.publish(self.stopCMD)
				rospy.logwarn(f"{curTime}: stop")
			curTime = time.time()-startTIme
			rospy.logwarn(f"{curTime}: end")
		else:
			rospy.logwarn(f"{msg.state}")
			if msg.state != "LANE_FOLLOWING":
				self.flag = False
			self.pub.publish(self.stopCMD)
				
      
if __name__ == '__main__':
	rospy.init_node('Lab2_3', anonymous=True)
	Node()
	rospy.spin()
