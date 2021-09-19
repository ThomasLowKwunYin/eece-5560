#!/usr/bin/env python3

import rospy
from std_msgs.msg import Float32
from mystery_package.msg import UnitsLabelled

class Node:
	def __init__(self):
		rospy.Subscriber("/mystery/output2", UnitsLabelled, self.callback)
		self.pub = rospy.Publisher('/mystery/output2', Float32, queue_size=10)
		self.pub_msg = UnitsLabelled()
		self.pub_msg.units = "feets"
	
	def callback(self,msg):
		self.pub_msg.value = msg.data.value*3.2
		rospy.pub(self.pub_msg)

if __name__ == '__main__':
	rospy.init_node('listener', anonymous = True)
	Node()
	rospy.spin()
