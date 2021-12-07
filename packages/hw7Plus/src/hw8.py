#!/usr/bin/env python3

import rospy
import cv2
import numpy as np
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

class node:
	def __init__(self): #Subscribes to the cropped image from Homework 7
		rospy.Subscriber("/image_cropped", Image, self.crop)
		#Subscribes to the white and yellow filtered images from Homework 7
		rospy.Subscriber("/image_white", Image, self.white) 
		rospy.Subscriber("/image_yellow", Image, self.yellow)

		self.bridge = CvBridge()

		self.edges = rospy.Publisher("/image_edges", Image, queue_size=10)
		self.white = rospy.Publisher("/image_lines_white", Image, queue_size=10)
		self.yellow = rospy.Publisher("/image_lines_yellow", Image, queue_size=10)
		
		self.yellowImg = None
		self.whiteImg = None
	

	def yellow(self, msg):
        	self.yellowImg = self.bridge.imgmsg_to_cv2(msg, "mono8")

	def white(self, msg):
		self.whiteImg = self.bridge.imgmsg_to_cv2(msg,"mono8")
		
	def output_lines(self, original_image, lines):
		output = np.copy(original_image)
		if lines is not None:
			for i in range(len(lines)):
				l = lines[i][0]
				cv2.line(output, (l[0],l[1]), (l[2],l[3]), (255,0,0), 2, cv2.LINE_AA)
				cv2.circle(output, (l[0],l[1]), 2, (0,255,0))
				cv2.circle(output, (l[2],l[3]), 2, (0,0,255))
		return output
		
	def crop(self, msg):
		
		cropImg= self.bridge.imgmsg_to_cv2(msg, "bgr8")
		
		cannyImg = cv2.Canny(cropImg, 100, 255)
		self.edges.publish(self.bridge.cv2_to_imgmsg(cannyImg, "8UC1"))
		yellowDilate = cv2.dilate(self.yellowImg, cv2.getStructuringElement(cv2.MORPH_ELLIPSE,(8,7)))
		
		yellowBitwise = cv2.bitwise_and(cannyImg, yellowDilate)
		whiteBitwise = cv2.bitwise_and(cannyImg, self.whiteImg)

		
		yellowHough = cv2.HoughLinesP(yellowBitwise, 1, np.pi/180, 5, np.array([]), 5, 5)
		whiteHough = cv2.HoughLinesP(whiteBitwise, 1, np.pi/180, 10, np.array([]), 10, 250)
		
		yellowOut = self.output_lines(cropImg, yellowHough)
		whiteOut = self.output_lines(cropImg, whiteHough)
		
		
		yellowOutImg = self.bridge.cv2_to_imgmsg(yellowOut, "bgr8")
		whiteOutImg = self.bridge.cv2_to_imgmsg(whiteOut, "bgr8")

		self.yellow.publish(yellowOutImg)
		self.white.publish(whiteOutImg)
		
	

if __name__ == "__main__":
	rospy.init_node("node", anonymous=True)
	node()
	rospy.spin()


