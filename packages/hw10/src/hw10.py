#!/usr/bin/env python3

import rospy
import example_action_server.msg
import actionlib

def HW10():
	client = actionlib.SimpleActionClient('fibonacci' example_action_server.msg.FibonacciAction)
	client.wait_for_server()
	goal = example_action_server.msg.FibonacciGoal(order=10)
	client.send_goal(goal)
	client.wait_for_result()
	return client.get_result()
	
if __name__ == '__main__'
	try:
		rospy.init_node('hw10')
		result = fibonacci_client()
		print("Result:",','.join([str(n) for n in result.sequence]))
	except rospy.ROSInterruptException:
		print("program interrupted before completion", file = sys.stderr)
