#!/usr/bin/env python3

#Python Libs
import sys, time

#numpy
import numpy as np

#OpenCV
import cv2
from cv_bridge import CvBridge

#ROS Libraries
import rospy
import roslib

#ROS Message Types
from sensor_msgs.msg import CompressedImage

class Lane_Detector:
    def __init__(self):
        self.cv_bridge = CvBridge()

        #### REMEMBER TO CHANGE THE TOPIC NAME! #####        
        self.image_sub = rospy.Subscriber('/akandb/camera_node/image/compressed', CompressedImage, self.image_callback, queue_size=1)
        #############################################

        rospy.init_node("my_lane_detector")

        self.mode = 'both'

    def image_callback(self, msg):
        rospy.loginfo("image_callback: both")

        # Convert to opencv image 
        img = self.cv_bridge.compressed_imgmsg_to_cv2(msg, "bgr8")

        #### YOUR CODE GOES HERE ####

        # Crop the image
        img_out = cv2.flip(img, 0)
        img_out = img_out[0:350, 0:640]

        # Now apply processing ONLY to the cropped image
        hsv_img = cv2.cvtColor(img_out, cv2.COLOR_BGR2HSV)

        # White mask
        lower_white = np.array([0, 0, 200])
        upper_white = np.array([180, 25, 255])
        white_mask = cv2.inRange(hsv_img, lower_white, upper_white)
        white_result = cv2.bitwise_and(img_out, img_out, mask=white_mask)

        # Yellow mask
        lower_yellow = np.array([15, 80, 80])
        upper_yellow = np.array([30, 255, 255])
        yellow_mask = cv2.inRange(hsv_img, lower_yellow, upper_yellow)
        yellow_result = cv2.bitwise_and(img_out, img_out, mask=yellow_mask)

        # Edge detection
        white_edges = cv2.Canny(white_result, 50, 150)
        yellow_edges = cv2.Canny(yellow_result, 10, 40)

        # Hough transform
        lines_white = cv2.HoughLinesP(white_edges, 1, np.pi / 180, threshold=50, minLineLength=50, maxLineGap=10)
        lines_yellow = cv2.HoughLinesP(yellow_edges, 1, np.pi / 180, threshold=50, minLineLength=50, maxLineGap=10)

        # Output image to draw on
        output_img = np.copy(img_out)

        # Switch between modes: 'white', 'yellow', or 'both' for displaying different filters
        if self.mode == 'white':
            # Show only white-filtered image
            output_img = white_result

        elif self.mode == 'yellow':
            # Show only yellow-filtered image
            output_img = yellow_result

        elif self.mode == 'both':
            # Draw lines on the white and yellow filtered images
            if lines_white is not None:
                for line in lines_white:
                    x1, y1, x2, y2 = line[0]
                    cv2.line(output_img, (x1, y1), (x2, y2), (255, 255, 255), 2)

            if lines_yellow is not None:
                for line in lines_yellow:
                    x1, y1, x2, y2 = line[0]
                    cv2.line(output_img, (x1, y1), (x2, y2), (0, 255, 255), 2)

        #############################

        # Show image in a window
        cv2.imshow('img_out', output_img)
        cv2.waitKey(1)

    def run(self):
    	rospy.spin() # Spin forever but listen to message callbacks

if __name__ == "__main__":
    try:
        lane_detector_instance = Lane_Detector()
        lane_detector_instance.run()
    except rospy.ROSInterruptException:
        pass
    
    
