#!/usr/bin/env python3

import rospy
from std_msgs.msg import FLoat32

class Listener
	def __init__(self):
		rospy.Subscriber("chatter", Float32, self.callback)
	
	def callback(self,msg):
		rospy.loginfo(f"{rospy.get_caller_id()}+I heard {msg.data}")

if __name__ = '__main__'
	rospy.int_node('listener', anonymous = True)
	Listener()
	rospy.spin()
