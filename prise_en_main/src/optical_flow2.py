#!/usr/bin/env python

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

# params for ShiTomasi corner detection
feature_params = dict( maxCorners = 100,
                       qualityLevel = 0.7,
                       minDistance = 7,
                       blockSize = 7 )

# Parameters for lucas kanade optical flow
lk_params = dict( winSize  = (15,15),
                  maxLevel = 2,
                  criteria = (cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_COUNT, 10, 0.03))

# Create some random colors
color = np.random.randint(0,255,(100,3))

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

            cv2_img_gray = cv2.cvtColor(cv2_img, cv2.COLOR_BGR2GRAY)
        except:
            print("Error CV2")

        if self.img2 is not None:
            p0 = cv2.goodFeaturesToTrack(self.img1, mask = None, **feature_params)
            # Create a mask image for drawing purposes
            mask = np.zeros_like(self.img1)
	    self.img2 = cv2_img_gray
            p1, st, err = cv2.calcOpticalFlowPyrLK(self.img1, self.img2, p0, None, **lk_params)

            # Select good points
            good_new = p1[st==1]
            good_old = p0[st==1]

            # draw the tracks
            for i,(new,old) in enumerate(zip(good_new,good_old)):
                a,b = new.ravel()
                c,d = old.ravel()
                mask = cv2.line(mask, (a,b),(c,d), color[i].tolist(), 2)
                self.img2 = cv2.circle(self.img2,(a,b),5,color[i].tolist(),-1)
            img = cv2.add(self.img2,mask)
            
            cv2.imshow('frame',img)
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
