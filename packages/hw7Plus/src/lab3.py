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
		new_image = cv2.resize(cv_img, imgSize, interpolation=cv2.INTER_NEAREST)
		cropped_image = new_image[offset:,:] 
		hsvImg = cv2.cvtColor(cropped, cv2.COLOR_BGR2HSV)
		# mask
		white  = cv2.inRange(hsvImg, (0,0,180),(255,40,255))		
		yellow = cv2.inRange(hsvImg, (25,70,150),(40,255,255))
		
		white  = cv2.erode(white,self.kernel)				# erode
		
		white  = cv2.dilate(white,self.kernel)			# dilate
		white  = cv2.bitwise_and(cropped, cropped, mask=white)	# crop image with white only
		
			# Yellow Mask
		yellow = cv2.erode(yellow,self.kernel)			# Eroded white mask
		yellow = cv2.dilate(yellow,self.kernel)			# Dilated white mask
		yellow = cv2.bitwise_and(cropped, cropped, mask=yellow)
