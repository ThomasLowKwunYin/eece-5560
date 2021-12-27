#!/usr/bin/env python3

import rospy

class PID:
	def __init__(self,p,i,d,t):
		self.kp = p
		self.ki = i
		self.kd = d
		self.pt = t

		self.prevError = 0
		self.errorIntegral = 0
		self.prevError = 0
       
    def run(self,error,t):
    	dt = t - self.pt
    	if not dt:
    		return 0

		self.errorIntegral += error*dt
		self.errorDelta = (self.error-self.prevError)/dt
		self.timeInit = time
		
		output = (self.kp*error)+(self.ki*error)+(self.kd*self.errorDelta)
        self.prevError = error
		self.pt = t
        return(output)
        
if __name__ == "__main__":
    rospy.init_node("lab4Control", anonymous=True)
    rospy.spin()
