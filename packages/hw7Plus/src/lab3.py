#!/usr/bin/env python3

import rospy
import cv2
import numpy as np
from sensor_msgs.msg import Image, CompressedImage
from cv_bridge import CvBridge
from duckietown_msgs.msg import SegmentList

class Node:
	def __init__(self):
		self.sub = rospy.Subscriber("camera_node/image/compressed", CompressedImage, self.talk, queue_size=1, buff_size=2**24)
		
		self.overlay = rospy.Publisher("/img_overlay", Image, queue_size=10)
		self.test = rospy.Publisher("/img_test", Image, queue_size=10)
		self.lines = rospy.Publisher("line_detector_node/segment_list", SegmentList, queue_size=10)
		
		self.bridge = CvBridge()
		self.kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (3,3))
		
		
	def talk(self, msg):
		segment = Segment()
		segmentList = SegmentList()
		
		incomingImg = self.bridge.compressed_imgmsg_to_cv2(msg,"bgr8") # ROS camera image to cv2
		
		image_Size  = (160, 120)
		offset = 40
		new_image = cv2.resize(cv_img, image_Size, interpolation=cv2.INTER_NEAREST)
		cropped_image = new_image[offset:,:] 
		hsvImg = cv2.cvtColor(cropped_image, cv2.COLOR_BGR2HSV)
		#mask
		white  = cv2.inRange(hsvImg, (0,0,180),(255,40,255))		
		yellow = cv2.inRange(hsvImg, (25,70,150),(40,255,255))
		#erode
		white  = cv2.erode(white,self.kernel)
		yellow = cv2.erode(yellow,self.kernel)
		#dilate
		white  = cv2.dilate(white,self.kernel)
		yellow = cv2.dilate(yellow,self.kernel)
		
		#color only
		white  = cv2.bitwise_and(cropped_image, cropped_image, mask=white)		
		yellow = cv2.bitwise_and(cropped_image, cropped_image, mask=yellow)
		
		#crop, canny edge detection and extra dilate
		croppedEdges = cv2.Canny(cropped, 10, 255)
		white = cv2.dilate(white,self.kernel)
		whiteEdges = np.array(white)
		yellow = cv2.dilate(yellow,self.kernel)
		yellowEdges = np.array(yellow)

		#overlay
		whiteOverlay  = cv2.bitwise_and(self.whiteEdges,  self.whiteEdges,  mask=self.cropped_edges)
		yellowOverlay = cv2.bitwise_and(self.yellowEdges, self.yellowEdges, mask=self.cropped_edges)
		
		#edge to rgb
		Whitergb = cv2.cvtColor(whiteOverlay,  cv2.COLOR_HSV2RGB)
		Yellowrgb = cv2.cvtColor(yellowOverlay,  cv2.COLOR_HSV2RGB)
		
		#rgb to grey
		whiteGrey = cv2.cvtColor(Whitergb,  cv2.COLOR_HSV2RGB)
		yellowGrey = cv2.cvtColor(Yellowrgb,  cv2.COLOR_HSV2RGB)
		
		#Hough Transform
		whiteHough  = cv2.HoughLinesP(whiteGrey, rho=1, theta=np.pi/180, threshold=7, minLineLength=10, maxLineGap=5)
		yellowHough = cv2.HoughLinesP(yellowGrey, rho=1, theta=np.pi/180, threshold=7, minLineLength=10, maxLineGap=5)
		arr_cutoff = np.array([0, offset, 0, offset])
		arr_ratio  = np.array([1. / image_Size[0], 1. / image_Size[1], 1. / image_Size[0], 1. / image_Size[1]])
		
		#Normalize and create segment
		if whiteHough is not None:
			whiteNormalized =  (whiteHough  + arr_cutoff) * arr_ratio
			for line in whiteNormalized:
				for x1,y1,x2,y2 in line:
					segment.color = 0
					segment.pixels_normalized[0].x = whiteNormalized[0]
					segment.pixels_normalized[0].y = whiteNormalized[1]
					segment.pixels_normalized[1].x = whiteNormalized[2]
					segment.pixels_normalized[1].y = whiteNormalized[3]
					segmentList.segments.append(segment)
					
		if yellowHough is not None:
			yellowNormalized = (yellowHough + arr_cutoff) * arr_ratio
			for line in yellowNormalized:
				for x1,y1,x2,y2 in line:
					segment.color = 1
					segment.pixels_normalized[0].x = yellowNormalized[0]
					segment.pixels_normalized[0].y = yellowNormalized[1]
					segment.pixels_normalized[1].x = yellowNormalized[2]
					segment.pixels_normalized[1].y = yellowNormalized[3]
					segmentList.segments.append(segment)
					
		self.line.publish(segmentList)
		#stackingImg
		whiteOut = self.output_lines_white(cropped_image, whiteHough)
		whiteYellowOut = self.output_lines_both(whiteOut, yellowHough)
		
		#convert to ros img then pub
		self.overlay.publish(self.bridge.cv2_to_imgmsg(whiteYellowOut,"bgr8"))
		
		#test output
		rosTested = self.bridge.cv2_to_imgmsg(whiteOverlay,"bgr8")
		self.testing.publish(whiteOverlay)

	def output_lines_white(self, original_image, lines):
		output = np.copy(original_image)
		if lines is not None:
			for i in range(len(lines)):
				l = lines[i][0]
				cv2.line(output, (l[0],l[1]), (l[2],l[3]), (255,255,255), 2, cv2.LINE_AA)
				cv2.circle(output, (l[0],l[1]), 2, (0,255,0))
				cv2.circle(output, (l[2],l[3]), 2, (0,0,255))
		return output
	def output_lines_both(self, original_image, lines):
		output = np.copy(original_image)
		if lines is not None:
			for i in range(len(lines)):
				l = lines[i][0]
				cv2.line(output, (l[0],l[1]), (l[2],l[3]), (255,255,0), 2, cv2.LINE_AA)
				cv2.circle(output, (l[0],l[1]), 2, (0,255,0))
				cv2.circle(output, (l[2],l[3]), 2, (0,0,255))
		return output
	

if __name__ == "__main__":
	rospy.init_node("node", anonymous=True)
	Node()
	rospy.spin()
