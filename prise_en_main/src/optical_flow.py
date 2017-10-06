#!/usr/bin/env python
"""OpenCV feature detectors with ros CompressedImage Topics in python.

This example subscribes to a ros topic containing sensor_msgs 
CompressedImage. It converts the CompressedImage into a numpy.ndarray, 
then detects and marks features in that image. It finally displays 
and publishes the new image - again as CompressedImage topic.
"""
__author__ =  'Simon Haller <simon.haller at uibk.ac.at>'
__version__=  '0.1'
__license__ = 'BSD'

# Python libs
import sys, time

# numpy
import numpy as np

# OpenCV
import cv2
from cv_bridge import CvBridge, CvBridgeError

# Ros libraries
import roslib
import rospy
# ROS Image message
from sensor_msgs.msg import Image

VERBOSE=False


class optical_flow:

    def __init__(self):
        '''Initialize ros subscriber'''
        self.bridge = CvBridge()
	self.img1 = None
	self.img2 = None

        # subscribed Topic
        self.subscriber = rospy.Subscriber("/bebop/image_raw", Image, self.callback, queue_size=1)
        if VERBOSE :
            print "subscribed to /bebop/image_raw"


    def callback(self, msg):
        '''Callback function of subscribed topic. 
        Here images get converted and features detected'''
        if VERBOSE :
            print 'received image of type: "%s"' % msg.format

        #### direct conversion to CV2 ####
        try:
	    # Convert your ROS Image message to OpenCV2
	    cv2_img = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            hsv = np.zeros_like(cv2_img)
            hsv[...,1] = 255

            cv2_img_gray = cv2.cvtColor(cv2_img, cv2.COLOR_BGR2GRAY)
        except:
            print("Error CV2")

        if self.img2 is not None:
	    self.img2 = cv2_img_gray
            flow = cv2.calcOpticalFlowFarneback(self.img1,self.img2, None, 0.5, 3, 15, 3, 5, 1.2, 0)
            mag, ang = cv2.cartToPolar(flow[...,0], flow[...,1])
            hsv[...,0] = ang*180/np.pi/2
            hsv[...,2] = cv2.normalize(mag,None,0,255,cv2.NORM_MINMAX)
            rgb = cv2.cvtColor(hsv,cv2.COLOR_HSV2BGR)
            
            img = cv2.add(cv2_img,rgb)
            cv2.imshow('Video',img)
            #cv2.imshow('Optical Flow', rgb)
	else:
            self.img2=cv2_img_gray

        self.img1 = self.img2
        cv2.waitKey(30)


def main(args):
    '''Initializes and cleanup ros node'''
    of = optical_flow()
    rospy.init_node('optical_flow', anonymous=True)
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print "Shutting down ROS Image feature detector module"
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main(sys.argv)
