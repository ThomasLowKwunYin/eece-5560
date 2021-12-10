#!/usr/bin/env python3

import rospy
import cv2
import numpy as np
from sensor_msgs.msg import Image, CompressedImage
from cv_bridge import CvBridge
from duckietown_msgs.msg import SegmentList

class Node:
	def __init(self):
		self.sub = rospy.Subscriber("camera_node/image/compressed", CompressedImage, self.filter, queue_size=1, buff_size=2**24)
		
		self.overlay = rospy.Publisher("/img_overlay", Image, queue_size=10)
		self.test = rospy.Publisher("/img_test", Image, queue_size=10)
		self.lines = rospy.Publisher("line_detector_node/segment_list", SegmentList, queue_size=10)
		
		self.bridge = CvBridge()
		self.kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (3,3))
	def filter(self, msg):
		incomingImg = self.bridge.compressed_imgmsg_to_cv2(msg,"bgr8") 
