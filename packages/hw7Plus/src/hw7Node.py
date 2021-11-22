#!/usr/bin/env python3

import rospy
import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

class node:
    def __init__(self):
        self.crop = rospy.Publisher("/image_cropped", Image, queue_size=10)
        self.whiteMask = rospy.Publisher("/image_white", Image, queue_size=10)
        self.yellowMask = rospy.Publisher("/image_yellow", Image, queue_size=10)
        
        rospy.Subscriber("/image", Image, self.filter)
        
        self.erosionKernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (3,3))
        self.dilationKernel= cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (3,3))
        
        self.bridge = CvBridge()
        
    def talk(self, msg):
    	cv_img = self.bridge.imgmsg_to_cv2(msg,"bgr8")
    	imgAxisY = len(cv_img)
    	imgYHalf = int(imgAxisY/2)
    	imgAxisX = len(cv_img[0])
    	cropped = cv_img[int(imgAxisY/2):imgAxisY, 0:imgAxisX]
    	
    	whiteFilter  = cv2.inRange(hsv_img, (0,0,0),(180,25,255))
        yellowFilter = cv2.inRange(hsv_img, (30,100,150),(40,255,255))
        
	crop = self.bridge.cv2_to_imgmsg(cropped,"bgr8")
	whiteMask = self.bridge.cv2_to_imgmsg(whiteFilter, "mono8")
	yellowMask = self.bridge.cv2_to_imgmsg(yellowFilter , "mono8")
        
	self.pub_cropped.publish(crop)
	self.pub_white.publish(whiteMask)
	self.pub_yellow.publish(yellowMask)
    	
if __name__ == "__main__":
    rospy.init_node("node", anonymous=True)
    ImageFilter()
    rospy.spin()
