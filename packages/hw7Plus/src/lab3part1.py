#!/usr/bin/env python3

import rospy
import cv2
import numpy as np
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from sensor_msgs.msg import CompressedImage
from duckietown_msgs.msg import SegmentList, Segment


class ImageFilter:
    def __init__(self):
        self.overlay = rospy.Publisher("/lab3_img_overlay", Image, queue_size=10)
        self.testing = rospy.Publisher("/img_test", Image, queue_size=10)
        self.lineSegments = rospy.Publisher("line_detector_node/segment_list", SegmentList, queue_size=10)
        self.bridge = CvBridge()
        self.kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (3,3))		# Kernel
        self.sub = rospy.Subscriber("camera_node/image/compressed", CompressedImage, self.filter, queue_size=1, buff_size=2**24)
        
    def filter(self, msg):
        segment = Segment()
        segmentList = SegmentList()
        
    	# Crop incoming message
        cv_img = self.bridge.compressed_imgmsg_to_cv2(msg,"bgr8") 	#convert ROS img to cv2
        image_size = (160, 120)
        offset = 40
        resized_img = cv2.resize(cv_img, image_size, interpolation=cv2.INTER_NEAREST)
        cropped = resized_img[offset:,:]       
        hsv_img = cv2.cvtColor(cropped, cv2.COLOR_BGR2HSV)
        
        # Find white and yellow colors and erode, dilate, and apply to a mask
        white  = cv2.inRange(hsv_img, (0,0,180),(255,40,255))	# White mask
        white  = cv2.erode(white,self.kernel)				# Eroded white mask
        white  = cv2.dilate(white,self.kernel)			# Dilated white mask
        white  = cv2.bitwise_and(cropped, cropped, mask=white)	# Cropped Image with only white
        yellow = cv2.inRange(hsv_img, (25,70,150),(40,255,255))	# Yellow Mask
        yellow = cv2.erode(yellow,self.kernel)			# Eroded white mask
        yellow = cv2.dilate(yellow,self.kernel)			# Dilated white mask
        yellow = cv2.bitwise_and(cropped, cropped, mask=yellow)	# Cropped image with only yellow
                
        # Canny Edge detection and extra dilate
        self.cropped_edges = cv2.Canny(cropped, 10, 255)
        white = cv2.dilate(white,self.kernel)
        self.white_edges = np.array(white)
        yellow = cv2.dilate(yellow,self.kernel)
        self.yellow_edges = np.array(yellow)
        
        # Overlaying edges to create only white and yellow edges
        white_edges  = cv2.bitwise_and(self.white_edges,  self.white_edges,  mask=self.cropped_edges)
        yellow_edges = cv2.bitwise_and(self.yellow_edges, self.yellow_edges, mask=self.cropped_edges)
            
        # Convert edges to rgb then to grey for hough xform
        rgb_w  = cv2.cvtColor(white_edges,  cv2.COLOR_HSV2RGB)	
        grey_w = cv2.cvtColor(rgb_w, cv2.COLOR_RGB2GRAY)
        rgb_y  = cv2.cvtColor(yellow_edges, cv2.COLOR_HSV2RGB)	
        grey_y = cv2.cvtColor(rgb_y, cv2.COLOR_RGB2GRAY)	
        
        # Performs Hough Transform to get lines
        arr_cutoff = np.array([0, offset, 0, offset])
        arr_ratio  = np.array([1. / image_size[0], 1. / image_size[1], 1. / image_size[0], 1. / image_size[1]])
        white_lines  = cv2.HoughLinesP(grey_w, rho=1, theta=np.pi/180, threshold=7, minLineLength=10, maxLineGap=5)
        yellow_lines = cv2.HoughLinesP(grey_y, rho=1, theta=np.pi/180, threshold=7, minLineLength=10, maxLineGap=5)
        #rospy.loginfo(white_lines)
        
        # Normalize and create segment lists
        if white_lines is not None:
            line_normalized_white =  (white_lines  + arr_cutoff) * arr_ratio
            for line in line_normalized_white:
                for x1,y1,x2,y2 in line:
                    segment.color = 0		# Segment is a White line
                    segment.pixels_normalized[0].x = x1
                    segment.pixels_normalized[0].y = y1
                    segment.pixels_normalized[1].x = x2
                    segment.pixels_normalized[1].y = y2
                    rospy.loginfo(segmentList)
                    segmentList.segments.append(segment)
        if yellow_lines is not None:
            line_normalized_yellow = (yellow_lines + arr_cutoff) * arr_ratio
            for line in line_normalized_yellow:
                for x1,y1,x2,y2 in line:
                    segment.color = 1		# Segment is a White line
                    segment.pixels_normalized[0].x = line_normalized_yellow[0]
                    segment.pixels_normalized[0].y = line_normalized_yellow[1]
                    segment.pixels_normalized[1].x = line_normalized_yellow[2]
                    segment.pixels_normalized[1].y = line_normalized_yellow[3]
                    segmentList.segments.append(segment)
                    
        
        # Publish lines to segment_list
        self.lineSegments.publish(segmentList)
            
        # Draws lines over cropped image
        white_line_img = self.output_lines_white(cropped, white_lines)
        both_lines_img = self.output_lines_yellow(white_line_img, yellow_lines)
        
        # Convert to ROS img and Publish
        ROS_lines_B = self.bridge.cv2_to_imgmsg(both_lines_img,"bgr8")
        self.overlay.publish(ROS_lines_B)
        
        # Testing output
        test = white_edges			# Change this to test an image
        ros_tested_img = self.bridge.cv2_to_imgmsg(test,"bgr8")
        self.test.publish(ros_tested_img)
        
                
    def output_lines_white(self, original_image, lines):
        output = np.copy(original_image)
        if lines is not None:
            for i in range(len(lines)):
                l = lines[i][0]
                cv2.line(output, (l[0],l[1]), (l[2],l[3]), (255,255,255), 2, cv2.LINE_AA)
                cv2.circle(output, (l[0],l[1]), 2, (0,255,0))
                cv2.circle(output, (l[2],l[3]), 2, (0,0,255))
        return output
    
    def output_lines_yellow(self, original_image, lines):
        output = np.copy(original_image)
        if lines is not None:
            for i in range(len(lines)):
                l = lines[i][0]
                cv2.line(output, (l[0],l[1]), (l[2],l[3]), (0,255,255), 2, cv2.LINE_AA)
                cv2.circle(output, (l[0],l[1]), 2, (0,255,0))
                cv2.circle(output, (l[2],l[3]), 2, (0,0,255))
        return output
        
        
        
if __name__ == "__main__":
    rospy.init_node("image_filter", anonymous=True)
    ImageFilter()
    rospy.spin()
