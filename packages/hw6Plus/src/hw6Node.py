#!/usr/bin/env python3

import math
import rospy
import numpy as np
from odometry_hw.msg import Pose2D
from odometry_hw.msg import  DistWheel

class Node:
	def __init__(self):
		self.pub_msg = Pose2D()
		self.pub_msg.x = 0
		self.pub_msg.y = 0
		self.pub_msg.theta = 0
		rospy.Subscriber('/dist_wheel', DistWheel, self.talk)
		self.pub = rospy.Publisher('/pose', Pose2D, queue_size=10)

	
	def talk(self,msg):
		ds = (msg.dist_wheel_left + msg.dist_wheel_right)/2		
		dTheta=(msg.dist_wheel_right-msg.dist_wheel_left)/0.1
		dx =ds*math.cos(self.pub_msg.theta+dTheta/2)
		dy =ds*math.sin  (self.pub_msg.theta+dTheta/2)
		self.pub_msg.x = self.pub_msg.x + dx
		self.pub_msg.y = self.pub_msg.y + dy
		self.pub_msg.theta = self.pub_msg.theta+dTheta
		self.pub.publish(self.pub_msg)

if __name__ == '__main__':
	rospy.init_node('hw6Node', anonymous=True)
	Node()
	rospy.spin()
