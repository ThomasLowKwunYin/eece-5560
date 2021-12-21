#!/usr/bin/env python3

import rospy
from cv_bridge import CvBridge
import cv2
from sensor_msgs.msg import Image
from sensor_msgs.msg import CompressedImage
import numpy as np
from duckietown_msgs.msg import Segment
from duckietown_msgs.msg import SegmentList
from std_msgs.msg import Header
from duckietown_msgs.msg import Vector2D

class Lab3Part1Node():

    def __init__(self):
        self.bridge = CvBridge()
        rospy.Subscriber("/ritzquacker/camera_node/image/compressed", CompressedImage, self.callback, queue_size=1, buff_size=2**24)
        self.pub = rospy.Publisher ("/part1", Image, queue_size=10)
        #self.pub_test = rospy.Publisher ("/test", Image, queue_size=10)
        self.pub_part2 = rospy.Publisher ("/ritzquacker/line_detector_node/segment_list", SegmentList, queue_size=10)

    def output_lines(self, original_image, lines):
        output = np.copy(original_image)
        if lines is not None:
            for i in range(len(lines)):
                l = lines[i][0]
                cv2.line(output, (l[0],l[1]), (l[2],l[3]), (255,0,0), 2, cv2.LINE_AA)
                cv2.circle(output, (l[0],l[1]), 2, (0,255,0))
                cv2.circle(output, (l[2],l[3]), 2, (0,0,255))
        return output

    def part2_output(self, lines, color):
        newSeg = []
        for i in range(len(lines)):
            temp = Segment()
            l = lines[i][0]
            temp.color = color
            temp.pixels_normalized[0].x=l[0]
            temp.pixels_normalized[0].y=l[1]
            temp.pixels_normalized[1].x=l[2]
            temp.pixels_normalized[1].y=l[3]
            newSeg.append(temp)
        return newSeg

    def callback(self, msg):
        yellow_ground = Segment()
        ground_project = SegmentList()
        header = Header()
        header.stamp = rospy.Time.now()
        kernel= cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (3,3)) 
        cv_img = self.bridge.compressed_imgmsg_to_cv2(msg, "bgr8")
        image_size = (160, 120)
        offset = 40
        new_image = cv2.resize(cv_img, image_size, interpolation=cv2.INTER_NEAREST)
        cv_cropped = new_image[offset:, :]
        hsv_cropped= cv2.cvtColor(cv_cropped, cv2.COLOR_BGR2HSV)
        hsv_white = cv2.inRange(hsv_cropped,(0,0,0), (255,20,255))
        for i in range (0,1):
            hsv_white = cv2.erode(hsv_white, kernel)
        for i in range (0,2):
            hsv_white = cv2.dilate(hsv_white, kernel)
        hsv_yellow = cv2.inRange(hsv_cropped,(20,80,120), (45,255,255))
        for i in range (0,1):
            hsv_yellow = cv2.erode(hsv_yellow, kernel)
        for i in range (0,2):
            hsv_yellow = cv2.dilate(hsv_yellow, kernel)
        cv_canny = cv2.Canny(cv_cropped, 0, 500)
        white_edge = cv2.bitwise_and(hsv_white, hsv_white, mask=cv_canny)
        white_hough =cv2.HoughLinesP(white_edge, 1, np.pi/180, 10, 5, 2)
        cv_cropped = self.output_lines(cv_cropped, white_hough)
        yellow_edge = cv2.bitwise_and(hsv_yellow, hsv_yellow, mask=cv_canny)
        yellow_hough =cv2.HoughLinesP(yellow_edge, 1, np.pi/180, 1, 1, 2)
        cv_cropped = self.output_lines(cv_cropped, yellow_hough)
        output_image = self.bridge.cv2_to_imgmsg(cv_cropped, "bgr8")
        test_image = self.bridge.cv2_to_imgmsg(hsv_yellow, "mono8")
        self.pub.publish(output_image)
        arr_cutoff = np.array([0, offset, 0, offset])
        arr_ratio = np.array([1. / image_size[0], 1. / image_size[1], 1. / image_size[0], 1. / image_size[1]])
        if white_hough is not None:
           white_normalized = (white_hough+arr_cutoff) * arr_ratio
           ground_project.segments.extend(self.part2_output(white_normalized, 0))
        if yellow_hough is not None:
           yellow_normalized = (yellow_hough+arr_cutoff) * arr_ratio
           ground_project.segments.extend(self.part2_output(yellow_normalized, 1))
        ground_project.header = header
        self.pub_part2.publish(ground_project)
        
	       
if __name__=="__main__":
    rospy.init_node('hw7node', anonymous=True)
    Lab3Part1Node()
    rospy.spin()







#[x1, y1, x2, y2]


