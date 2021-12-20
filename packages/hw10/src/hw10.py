#!/usr/bin/env python3

import rospy
import example_service.srv
import example_action_server.msg
import actionlib

class FibonacciAction(object):
	_feedback = example_action_server.msg.FibonacciFeedback()
	_result = example_action_server.msg.FibonacciResult()
	
	def _init_(self,name):
		self._action_name=name
		self._as = actionlib.SimpleActionServer(self._action_name, example_action_server.msg.FibonacciAction, execute_cb = self.execute_cb, auto_start = False)
		self._as.start()
		
	def execute_cb(self,goal):
		
if __name__ == '__main__'
	rospy.init_node('hw10')
	server = FibonacciAction(rospy.get_name())
	rospy.spin()
