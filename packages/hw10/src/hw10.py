#!/usr/bin/env python3

import rospy
import example_action_server.msg
import example_service.srv
import actionlib
class HW10:
	def fiboserv(self, i):
        rospy.wait_for_service('/calc_fibonacci')
        try:
            fib=rospy.ServiceProxy('/calc_fibonacci', Fibonacci)
            start_time = rospy.get_time()
            fib1 = fib(x)
            rospy.loginfo(f"received at {rospy.get_time()-start_time}sec", )
        except rospy.ServiceException as error:
            rospy.loginfo(f"Service call failed: {%error}" )
		
		
		
	def fiboact(self, i):
		client = actionlib.SimpleActionClient('fibonacci' example_action_server.msg.FibonacciAction)
		client.wait_for_server()
		goal = example_action_server.msg.FibonacciGoal(order=i)
		startTime = rospy.get_time()
		client.send_goal(goal)
		rospy.loginfo(f"Request returned time: {rospy.get_time()-startTime}sec")
		startTime = rospy.get_time()
		rospy.loginfo(f"Answer returned time: {rospy.get_time()-startTime}sec")
		client.wait_for_result()
		
		return client.get_result()
	
if __name__ == '__main__'
	rospy.init_node('hw10')
	node = HW10
	node.fiboserv(3)
	node.fiboact(3)
	node.fiboserv(15)
	node.fiboact(15)
