#!/usr/bin/env python3

import rospy
from std_msgs.msg import Float32
from mystery_package.msg import UnitsLabelled

class Node:
	def __init__(self):
		self.converted = 0
		self.pub_msg = UnitsLabelled()
		self.pub_msg.units = "feets"
		rospy.Subscriber('/mystery/output2', UnitsLabelled, self.talk)
		self.pub = rospy.Publisher('Somewhere', UnitsLabelled, queue_size=10)

	
	def talk(self,msg):
		self.converted = msg.value*3.28
		self.pub_msg.value = self.converted
		self.pub.publish(self.pub_msg)

if __name__ == '__main__':
	rospy.init_node('hw3Node', anonymous=True)
	Node()
	rospy.spin()