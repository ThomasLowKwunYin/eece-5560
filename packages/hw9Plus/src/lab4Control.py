#!/usr/bin/env python3

import rospy

class PID:
	def __init__(self,pid,t):
		self.kp = pid[0]
		self.ki = pid[1]
		self.kd = pid[2]
		self.dt = t

		self.prevError = 0
		self.errorIntegral = None
		self.errorDelta = None
       
	def talk(self,error):
		if errorIntegral == None:
    		self.errorDelta = 0
		self.errorIntegral = 0
		else:
			self.errorDelta = (error-self.prevError)/self.dt
		self.errorIntegral += error		
		output = (self.kp*error)+(self.ki*error)+(self.kd*self.errorDelta)
        self.prevError = error
        return(output)
        
if __name__ == "__main__":
    rospy.init_node("lab4Control", anonymous=True)
    rospy.spin()
