#!/usr/bin/env python3

import rospy
import numpy as np
from std_msgs.msg import Float32
from duckietown_msgs.msg import AprilTagDetectionArray, Twist2DStamped, FSMState
import lab4Control as PID

class lab4:
	def __init__(self):
		
		self.pub = rospy.Publisher('lane_controller_node/car_cmd', Twist2DStamped, queue_size=10)
        rospy.Subscriber("apriltag_detector_node/detections", AprilTagDetectionArray, self.detected)
        rospy.Subscriber('fsm_node/mode', FSMState, self.control)
	
		self.flag = False
		self.detect = 0
		self.noDetect = 0
		self.state = None
		
		TurnPIDParam = [1,0,0]
		self.errorMargin = 0.1
		DistancePIDParam = [0.5,0,0]
		dt = 0.01
		
		self.TurnPID = PID.PID(TurnPIDParam, dt)
		self.DistancePID = PID.PID(DistancePIDParam, dt)
		self.signPos = None

	def control(self, state):
        self.state = state.state
	def tracking(self,msg):
		vector = Twist2DStamped()
		#check for error for PID controller 
		if len(msg.detections) != 0:
			if self.detect == 0:
				self.detect = 0.8
				self.noDetect = 0
			z = msg.detections[0].transform.translation.z
			x = msg.detections[0].transform.translation.x
			self.signPos = [z, x]
		else:
			if self.noDetect == 0:
				rospy.logwar("No signal")
				self.detect = 0
				self.noDetect = 1
				
		#Tracking signs
		if self.state == "LANE_FOLLOWING":
			if self.flag == False:
				z = self.signPos[0]
				x = self.signPos[1]
				theta = np.arctan(x/z)
				turnCorrection = self.TurnPID.talk(theta)
				if theta > self.errorMargin or theta < -self.errorMargin:
					vector.omega = turnCorrection*(-0.5)
				else:
					vector.omega = 0
				
				DistCorrection = self.DistancePID.talk(z-.1)
				if z> .15:
					vector.v = DistCorrection*0.4
					vector.omega = vector.omega*2
				else:
					vector.v = 0
				
				rospy.loginfo(f"vector: {vector}")		
				self.pub.publish(vector)
			else:
				turnCorrection = self.TurnPID.talk(0)
				DistCorrection = self.DistancePID.talk(0)
				vector.v = 0
				vector.omega = 0
				self.pub.publish(vector)
		else:
			if msg.state != "LANE_FOLLOWING":
				self.flag = False
			vector.v = 0
			vector.omega = 0
			self.pub.publish(vector)
			
if __name__ == "__main__":
    rospy.init_node("lab4", anonymous=True)
    lab4()
    rospy.spin()
			

