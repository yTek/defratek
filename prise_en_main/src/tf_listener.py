#!/usr/bin/env python
import rospy
# ROS tf
import tf

if __name__ == '__main__':

	pub = rospy.Publisher('/bebop/cmd_vel', Twist, queue_size = 1)
	pubTakeoff = rospy.Publisher('/bebop/takeoff', Empty, queue_size = 1)
	pubLand = rospy.Publisher('/bebop/land', Empty, queue_size = 1)

	listener = tf.TransformListener()

	rospy.init_node('bebop_tf_listener')
	
	print("Take off")
	pubTakeoff.publish()

	rate = rospy.Rate(3.0)

	(trans,rot)=listener.lookupTransform('odom','base_link',rospy.Time(0))

	while trans[1]<2:
		try:
			(trans,rot)=listener.lookupTransform('odom','base_link',rospy.Time(0))
			twist = Twist()
			twist.linear.y = 0.5
			print("twist: ", twist)				
			pub.publish(twist)
			
			

		except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):

			
			continue

		print("trans: ",trans)
		print("rot: ", rot)

	twist = Twist()
	twist.linear.x = 0.0; twist.linear.y = 0.0; twist.linear.z = 0.0
	twist.angular.x = 0.0; twist.angular.y = 0.0; twist.angular.z = 0.0
	pub.publish(twist)
	pubLand.publish()
