#!/usr/bin/env python
import roslib;
import rospy

from geometry_msgs.msg import Twist, Point
from std_msgs.msg import Empty
from nav_msgs.msg import Odometry
import sys, select, termios, tty

msg= """Test 
	test? 
	"""
mvtBindings = {
		'z':(0.2,0,0,0),
		's':(-0.2,0,0,0),
		'q':(0,0.2,0,0),
		'd':(0,-0.2,0,0),
		'j':(0,0,0,0.2),
		'l':(0,0,0,-0.2),
		'i':(0,0,0.2,0),
		'k':(0,0,-0.2,0),
	       }

currentPosition=(0.0,0.0,0.0)

OdomlastPosition=()

def getKey():
	tty.setraw(sys.stdin.fileno())
	select.select([sys.stdin], [], [], 0)
	key = sys.stdin.read(1)
	termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
	return key

def odometry_callback(msg):
	global currentPosition
	global OdomlastPosition

	#Inverse X so that +X is aiming where the bebop camera aims
	#Y pos is + when going left to where the camera aims
	#Z is + when drones goes up
	#Be careful with calibration
	odomOnX= msg.pose.pose.position.x
	odomOnY= msg.pose.pose.position.y
	odomOnZ=msg.pose.pose.position.z

	#Getting the delta corresponding to the movement 
	if OdomlastPosition:
		posX=odomOnX-OdomlastPosition[0]
		posY=odomOnY-OdomlastPosition[1]
		posZ=odomOnZ-OdomlastPosition[2]
		currentPosition=(currentPosition[0]+posX,currentPosition[1]+posY,currentPosition[2]+posZ)

	p=Point()
	print(currentPosition)
	p.x=currentPosition[0]
	p.y=currentPosition[1]
	p.z=currentPosition[2]

	pubPos.publish(p)

	OdomlastPosition=(odomOnX,odomOnY,odomOnZ)


if __name__=="__main__":
    	settings = termios.tcgetattr(sys.stdin)
	
	pub = rospy.Publisher('/bebop1/cmd_vel', Twist, queue_size = 1)
	pubTakeoff = rospy.Publisher('/bebop1/takeoff', Empty, queue_size = 1)
	pubLand = rospy.Publisher('/bebop1/land', Empty, queue_size = 1)
	
	pubPos = rospy.Publisher('/bebop1_Pos', Point, queue_size = 1)
	rospy.init_node('teleop_test', anonymous= True)
	
	start = raw_input("Take off? ")
	if start == "yes":
		
		print("Take off")
		pubTakeoff.publish()

		sub_odom = rospy.Subscriber("/bebop1/odom", Odometry, odometry_callback)
		
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

		except:
			print "Error! Exception!"

		finally:
			twist = Twist()
			twist.linear.x = 0.0; twist.linear.y = 0.0; twist.linear.z = 0.0
			twist.angular.x = 0.0; twist.angular.y = 0.0; twist.angular.z = 0.0
			pub.publish(twist)
			pubLand.publish()
			print("Land!")	    		
			termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
	else:
		print("No start")

