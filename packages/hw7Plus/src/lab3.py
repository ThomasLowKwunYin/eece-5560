#!/usr/bin/env python3

import rospy
import cv2
import numpy as np
from sensor_msgs.msg import Image, CompressedImage
from cv_bridge import CvBridge
from duckietown_msgs.msg import SegmentList

class Node:
	def __init(self):
		self.overlay = rospy.Publisher("/img_overlay", Image, queue_size=10)
        	self.testing = rospy.Publisher("/img_test", Image, queue_size=10)
