#!/usr/bin/env python3

import rospy
import cv2
from numpy import np
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
		self.both = rospy.Publisher("/image_lines_all", Image, queue_size=10)

		self.yellowImg = None
		self.whiteImg = None

		ts = message_filters.TimeSynchronizer([cropped, yellow_im, white_im], 10)
		ts.registerCallback(self.callback)

	def yellow(self, msg):
        	self.yellowImg = cv2.cvtColor(msg, cv2.COLOR_BGR2GRAY)

	def white(self, msg):
		self.whiteImg = cv2.cvtColor(msg, cv2.COLOR_BGR2GRAY)
	def crop(self, msg)
		edgedImg= self.bridge.imgmsg_to_cv2(msg, "bgr8")
        	lineImg = cv2.Canny(edgedImg, 0, 255)
		self.edges.publish(self.bridge.cv2_to_imgmsg(lineImg, "8UC1"))
		
		yellowOut = cv2.bitwise_and(lineImg, lineImg, mask = self.yellowImg)
		whiteOut = cv2.bitwise_and(lineImg, lineImg, mask = self.whiteImg)
		self.yellow.publish(self.bridge.cv2_to_imgmsg(yellowOut, "8UC1"))
		self.white.publish(self.bridge.cv2_to_imgmsg(whiteOut, "8UC1"))
		
		yellowHough = cv2.HoughLinesP(yellow_lanes, 1, np.pi / 180, 5, minLineLength=5, maxLineGap=5)
		whiteHough = cv2.HoughLinesP(white_lanes, 1, np.pi / 180, 5, minLineLength=5, maxLineGap=5)
		
		yellowMarker = self.bridge.imgmsg_to_cv2(msg, "bgr8")
		for line in yellowHough:
			x1, x2, y1, y2 = line[0]
			cv2.line(yellowMarker, (x1, y1), (x2, y2), (255, 0, 0), 2)
			cv2.circle(yellowMarker, (x1, y1), 2, (0, 255, 255), 2)
			cv2.circle(yellowMarker, (x2, y2), 2, (0, 0, 255), 2)
		yellowFilter = self.bridge.cv2_to_imgmsg(yellowMarker, "bgr8")
		
		whiteMarker = self.bridge.imgmsg_to_cv2(msg, "bgr8")
		for line in whiteHough:
			x1, x2, y1, y2 = line[0]
			cv2.line(whiteMarker, (x1, y1), (x2, y2), (0, 255, 0), 2)
			cv2.circle(whiteMarker, (x1, y1), 2, (0, 0, 255), 2)
			cv2.circle(whiteMarker, (x2, y2), 2, (255, 0, 255), 2)
		whiteFilter = self.bridge.cv2_to_imgmsg(whiteMarker, "bgr8")
            
		for line in yellowHough:
			x1, x2, y1, y2 = line[0]
			cv2.line(whiteMarker, (x1, y1), (x2, y2), (0, 0, 255), 2)
			cv2.circle(whiteMarker, (x1, y1), 2, (0, 255, 0), 2)
			cv2.circle(whiteMarker, (x2, y2), 2, (255, 255, 0), 2)
		allFilter = self.bridge.cv2_to_imgmsg(whiteMarker, "bgr8")

		self.yellow.publish(whiteMarker)
		self.white.publish(whiteFilter)
		self.both.publish(allFilter)
if __name__ == "__main__":
	rospy.init_node("node", anonymous=True)
	node()
	rospy.spin()


