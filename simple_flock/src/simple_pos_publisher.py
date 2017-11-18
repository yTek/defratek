#!/usr/bin/env python
import roslib;
import rospy

from geometry_msgs.msg import Twist, Point
from std_msgs.msg import Empty
import sys, select, termios, tty, time

msg= """Test 
	test? 
	"""
mvtBindings = {
		'z':(0.5,0,0,0),
		's':(-0.5,0,0,0),
		'q':(0,0.5,0,0),
		'd':(0,-0.5,0,0),
		'j':(0,0,0,0.5),
		'l':(0,0,0,-0.5),
		'i':(0,0,1.0,0),
		'k':(0,0,-1.0,0),
	       }

if __name__=="__main__":

	pubPos = rospy.Publisher('/bebop1_Pos', Point, queue_size = 1)
	rospy.init_node('publisher_test', anonymous= True)

	p=Point()
	p.x=2.50
	p.y=0.0
	p.z=0.0

	print("Point: "+str(p))	

	while 1:
		time.sleep(1)
		pubPos.publish(p)
		print("Point: "+str(p))	

