#!/usr/bin/env python3

import rospy
import time
from duckietown_msgs.msg import FSMState, Twist2DStamped

class Node:
	def __init__(self):
		rospy.Subscriber('fsm_node/mode', FSMState, self.talk)
		self.pub = rospy.Publisher('car_cmd_switch_node/car_cmd', Twist2DStamped, queue_size=10)
    
		self.moveCMD = Twist2DStamped()
		self.moveCMD.v = .41
		self.moveCMD.omega = 0

		self.stopCMD = Twist2DStamped()
		self.stopCMD.v = 0
		self.stopCMD.omega = 0
		self.flag = False
		rospy.logwarn(f"Lab2_1.py ready")
    
	def talk(self, msg):
		startTime = time.time()
		rospy.logwarn(f"{startTime}")
		rospy.logwarn(f"state:{msg}")
		if msg.state == "LANE_FOLLOWING" and self.flag == False:
			self.flag = True
			rospy.logwarn(f"flag:{self.flag}")
			for i in range(102):
	      		#for 102 ticks, apporximately 3 sec
				curTIme = time.time()-startTIme
				rospy.logwarn(f"{i}: move")
				self.pub.publish(self.moveCMD)
			rospy.logwarn(f"{curTIme}: stop")
			self.pub.publish(self.stopCMD)
		else:
			if msg.state != "LANE_FOLLOWING":
				self.flag = False
			self.pub.publish(self.stopCMD)
				
      
if __name__ == '__main__':
	rospy.init_node('Lab2_1', anonymous=True)
	Node()
	rospy.spin()
      
        
  
