#!/usr/bin/env python3

import rospy
from hw5Plus.msg import Vector2D

class Node:
	def __init__(self):
		self.converted = 0
		self.robotFrame= Vector2D()
		self.worldFrame= Vector2D()
		rospy.Subscriber('duckietown_msgs/Vector2D ', Vector2D, self.talk)
		self.pub1 = rospy.Publisher('Robot_Frame', Vector2D, queue_size=10)
		self.pub2 = rospy.Publisher('World_Frame', Vector2D, queue_size=10)

	
	def talk(self,msg):
	
		self.robotFrame.xvalue = -1*self.msg.xvalue-2
		self.robotFrame.yvalue = -1*self.msg.yvalue
		
		self.worldFrame.xvalue = 0.707*self.msg.xvalue+2
		self.worldFrame.yvalue = 0.707*self.msg.yvalue+7
		
		self.pub1.publish(self.robotFrame)
		self.pub2.publish(self.worldFrame)


if __name__ == '__main__':
	rospy.init_node('hw5Node', anonymous=True)
	Node()
	rospy.spin()
