#!/usr/bin/env python
import rospy
# ROS tf
import tf

if __name__ == '__main__':
	rospy.init_node('bebop_tf_listener')
	
	listener = tf.TransformListener()
	
	rate = rospy.Rate(10.0)
	while not rospy.is_shutdown():
		try:
			(trans,rot)=listener.lookupTransform('odom','base_link',rospy.Time(0))

		except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):

			continue

		print("trans: ",transf)
		print("rot: ", rot)
