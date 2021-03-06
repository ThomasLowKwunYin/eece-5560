#!/usr/bin/env python3

import rospy
import example_action_server.msg
from example_service.srv import *
import actionlib
class HW10:
	def fiboserv(self, i):
		rospy.wait_for_service('/calc_fibonacci')
		try:
			fibonacci=rospy.ServiceProxy('/calc_fibonacci', Fibonacci)
			start_time = rospy.get_time()
			fib1 = fibonacci(i)
			rospy.logwarn(f"case:{i}, received at {rospy.get_time()-start_time}sec", )
		except rospy.ServiceException as error:
			rospy.logwarn(f"Service call failed" )
		
	def fiboact(self, i):
		client = actionlib.SimpleActionClient('fibonacci', example_action_server.msg.FibonacciAction)
		client.wait_for_server()
		goal = example_action_server.msg.FibonacciGoal(order=i)
		startTime = rospy.get_time()
		client.send_goal(goal)
		rospy.logwarn(f" case:{i}, Request returned time: {rospy.get_time()-startTime}sec")
		startTime = rospy.get_time()
		rospy.logwarn(f"case:{i}, Answer returned time: {rospy.get_time()-startTime}sec")
		client.wait_for_result()
		return client.get_result()
if __name__ == '__main__':
	rospy.init_node('hw10')
	a = HW10()
	a.fiboserv(3)
	a.fiboact(3)
	a.fiboserv(15)
	a.fiboact(15)
