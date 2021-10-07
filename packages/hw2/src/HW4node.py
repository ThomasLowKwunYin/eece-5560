#!/usr/bin/env python3

import rospy
from std_msgs.msg import Float32
from mystery_package.msg import UnitsLabelled

class Node:
	def __init__(self):
		self.pub_msg = UnitsLabelled()
		rospy.Subscriber('/mystery/output2', UnitsLabelled, self.talk)
		self.pub = rospy.Publisher('Darkness', UnitsLabelled, queue_size=10)

	def talk(self,msg):
		
		if rospy.has_param("unit_param"):
			self.foo = rospy.get_param("unit_param")
		else:
			self.foo = "default"
		
		if self.foo is "Feets"
			self.pub_msg.value = msg.value*3.28
			self.pub_msg.units = "Feets"
		elif self.foo is "Meters":
			self.pub_msg = msg.value
			self.pub_msg.units = "Meters"
		else:
			self.pub_msg.value = msg.value*1.7018
			self.pub_msg.units = "Smoots"
		
		self.pub.publish(self.pub_msg)

if __name__ == '__main__':
	rospy.init_node('hw4Node', anonymous=True)
	Node()
	rospy.spin()
