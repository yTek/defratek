#!/usr/bin/env python
import rospy
# ROS tf
import tf

import time

if __name__ == '__main__':

	listener = tf.TransformListener()

	rospy.init_node('bebop_tf_listener')

	rate = rospy.Rate(3.0)

	(trans,rot)=listener.lookupTransform('base_link','camera_base_link',rospy.Time(0))

	while 1:
		try:
			(trans,rot)=listener.lookupTransform('base_link','camera_base_link',rospy.Time(0))
			time.sleep(1)

		except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
			
			continue

		print("trans: ",trans)
		print("rot: ", rot)
