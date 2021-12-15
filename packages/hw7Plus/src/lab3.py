#!/usr/bin/env python3

import rospy
import cv2
import numpy as np
from sensor_msgs.msg import Image, CompressedImage
from cv_bridge import CvBridge
from duckietown_msgs.msg import SegmentList

class Node:
	def __init(self):
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
		self.cropped_edges = cv2.Canny(cropped, 10, 255)
		white = cv2.dilate(white,self.kernel)
		self.white_edges = np.array(white)
		yellow = cv2.dilate(yellow,self.kernel)
		self.yellow_edges = np.array(yellow)
	
	def output_lines(self, original_image, lines):
		output = np.copy(original_image)
		if lines is not None:
	    		for i in range(len(lines)):
				l = lines[i][0]
				cv2.line(output, (l[0],l[1]), (l[2],l[3]), (255,0,0), 2, cv2.LINE_AA)
				cv2.circle(output, (l[0],l[1]), 2, (0,255,0))
				cv2.circle(output, (l[2],l[3]), 2, (0,0,255))
		return output
		
