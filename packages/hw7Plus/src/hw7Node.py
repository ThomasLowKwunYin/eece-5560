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
    	cropped = cv_img[imgYHalf:imgAxisY, 0:imgAxisX]
    	
    	white  = cv2.inRange(hsv_img, (0,0,0),(180,25,255))
        white  = cv2.erode(white,self.ero_k)
        white  = cv2.dilate(white,self.dil_k)
        white  = cv2.bitwise_and(cropped, cropped, mask=white)
        yellow = cv2.inRange(hsv_img, (30,100,150),(40,255,255))
        yellow = cv2.erode(yellow,self.ero_k)
        yellow = cv2.dilate(yellow,self.dil_k)
        yellow = cv2.bitwise_and(cropped, cropped, mask=yellow)
    	
if __name__ == "__main__":
    rospy.init_node("node", anonymous=True)
    ImageFilter()
    rospy.spin()
