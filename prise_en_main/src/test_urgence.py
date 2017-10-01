#!/usr/bin/env python
import roslib;
import rospy

from geometry_msgs.msg import Twist
from std_msgs.msg import Empty
import sys, select, termios, tty, time

msg= """Mode Manuel 
	Commande:
		z : avancer
		q : translation a gauche
		s : reculer
		d : translation a droite

		i : monter
		k : descendre

		j : rotation trigonometrique
		l : rotation horaire
	"""

mvtBindings = {
		'z':(1.0,0,0,0),
		's':(-1.0,0,0,0),
		'q':(0,1.0,0,0),
		'd':(0,-1.0,0,0),
		'j':(0,0,0,1.0),
		'l':(0,0,0,-1.0),
		'i':(0,0,1.0,0),
		'k':(0,0,-1.0,0),
	       }


def autopilot():
	#Only turn around the Z axis
	twist = Twist()
	twist.linear.x = 0.0; twist.linear.y = 0.0; twist.linear.z = 0.0
	twist.angular.x = 0.0; twist.angular.y = 0.0; twist.angular.z = 1.0
	while (1):	
		print "autopilot"
		pub.publish(twist)

def getKey():
	tty.setraw(sys.stdin.fileno())
	select.select([sys.stdin], [], [], 0)
	key = sys.stdin.read(1)
	termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
	return key


if __name__=="__main__":
    	settings = termios.tcgetattr(sys.stdin)
	
	pub = rospy.Publisher('/bebop/cmd_vel', Twist, queue_size = 1)
	pubTakeoff = rospy.Publisher('/bebop/takeoff', Empty, queue_size = 1)
	pubLand = rospy.Publisher('/bebop/land', Empty, queue_size = 1)
	rospy.init_node('teleop_test_urgence', anonymous= True, disable_signals=True)
	
	start = raw_input("Take off? ")
	if start == "yes":
		
		print("Take off")
		pubTakeoff.publish()
		
		x = 0.0
		y = 0.0
		z = 0.0
		th = 0.0
		status = 0.0

		try:
			time.sleep(5)
			autopilot()

		except KeyboardInterrupt:
			try:
				print "SIGNAL!!! \n MODE MANUEL ENCLENCHE"
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
				print ("Error! Exit")
			
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

