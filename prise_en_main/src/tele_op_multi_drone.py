#!/usr/bin/env python
import roslib;
import rospy

from geometry_msgs.msg import Twist
from std_msgs.msg import Empty
import sys, select, termios, tty

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


def getKey():
	tty.setraw(sys.stdin.fileno())
	select.select([sys.stdin], [], [], 0)
	key = sys.stdin.read(1)
	termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
	return key


if __name__=="__main__":
    	settings = termios.tcgetattr(sys.stdin)
	
	pub = rospy.Publisher('/bebop1/cmd_vel', Twist, queue_size = 1)
	pubTakeoff = rospy.Publisher('/bebop1/takeoff', Empty, queue_size = 1)
	pubLand = rospy.Publisher('/bebop1/land', Empty, queue_size = 1)

	pub2 = rospy.Publisher('/bebop2/cmd_vel', Twist, queue_size = 1)
	pubTakeoff2 = rospy.Publisher('/bebop2/takeoff', Empty, queue_size = 1)
	pubLand2 = rospy.Publisher('/bebop2/land', Empty, queue_size = 1)

	rospy.init_node('teleop_test', anonymous= True)
	
	start = raw_input("Take off? ")
	if start == "yes":
		
		print("Take off")
		pubTakeoff.publish()
		pubTakeoff2.publish()
		
		x = 0.0
		y = 0.0
		z = 0.0
		th = 0.0
		status = 0.0

		try:
			print msg
			while(1):
				key = getKey()
				if key in mvtBindings.keys():
					print(key)
					x = mvtBindings[key][0]
					y = mvtBindings[key][1]
					z = mvtBindings[key][2]
					th = mvtBindings[key][3]
					if (status == 14):
						print msg
					status = (status + 1) % 15
				else:
					x = 0.0
					y = 0.0
					z = 0.0
					th = 0.0
					if (key == '\x03'):
						break

				twist = Twist()
				twist.linear.x = x; twist.linear.y = y; twist.linear.z = z;
				twist.angular.x = 0.0; twist.angular.y = 0.0; twist.angular.z = th
				print(twist)				
				pub.publish(twist)
				pub2.publish(twist)

		except:
			print "Error! Exception!"

		finally:
			twist = Twist()
			twist.linear.x = 0.0; twist.linear.y = 0.0; twist.linear.z = 0.0
			twist.angular.x = 0.0; twist.angular.y = 0.0; twist.angular.z = 0.0
			pub.publish(twist)
			pub2.publish(twist)
			
			pubLand.publish()
			pubLand2.publish()
			print("Land!")	    		
			termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
	else:
		print("No start")

