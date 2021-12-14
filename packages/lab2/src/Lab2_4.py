#!/usr/bin/env python3

import math
import rospy
import numpy as np
import time
from duckietown_msgs.msg import WheelsCmdStamped
from odometry_hw.msg import  Pose2D


class Node:
	def __init__(self):
		self.pub_msg = Pose2D()
		self.pub_msg.x = 0
		self.pub_msg.y = 0
		self.pub_msg.theta = 0
		rospy.Subscriber('wheels_driver_node/wheels_cmd', WheelsCmdStamped, self.talk)
		self.pub = rospy.Publisher('pose', Pose2D, queue_size=10)
		self.startTime = time.time()

	
	def talk(self,msg):
		curTime = time.time()-self.startTime
		leftWheelPos = msg.vel_left*curTime
		rightWheelPos = msg.vel_right*curTime
		ds = (leftWheelPos + rightWheelPos)/2		
		dTheta=(rightWheelPos-leftWheelPos)/0.1
		dx =ds*math.cos(self.pub_msg.theta+dTheta/2)
		dy =ds*math.sin  (self.pub_msg.theta+dTheta/2)
		self.pub_msg.x = self.pub_msg.x + dx
		self.pub_msg.y = self.pub_msg.y + dy
		self.pub_msg.theta = self.pub_msg.theta+dTheta
		rospy.logwarn(f"location:{self.pub_msg}")
		self.pub.publish(self.pub_msg)
		self.startTime= curTime

if __name__ == '__main__':
	rospy.init_node('Lab2_4', anonymous=True)
	Node()
	rospy.spin()
