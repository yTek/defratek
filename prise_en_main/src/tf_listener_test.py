#!/usr/bin/env python
import roslib;
import rospy

from geometry_msgs.msg import Twist
from std_msgs.msg import Empty
import sys, select, termios, tty, time
import tf

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

	#Rate to send data is 3Hz
	rate = rospy.Rate(3.0)

	(trans,rot)=listener.lookupTransform('odom','base_link',rospy.Time(0))
	
	#Allowed error on position
	epsilon=0.5
	
	confirm='n'
	
	while confirm == 'n' or confirm =='no'
		print("Enter a [x;y;z] coordinate:\n")
		
		pointX = raw_input("X: ? (enter stay to 'stay' on current coordinate)\n")
		pointY = raw_input("Y: ? (enter stay to 'stay' on current coordinate)\n")
		pointZ = raw_input("Z: ? (enter stay to 'stay' on current coordinate)\n")
		
		#If nothing entered on X stay at current position
		if not isinstance(pointX, float):
			pointX = trans[0]
			print("Will stay on current coordinate on X.\nIf you don't want to stay on position make sure you entered a float.")
		
		#If nothing entered on Y stay at current position
		if not isinstance(pointY, float):
			pointY = trans[1]
			print("Will stay on current coordinate on Y.\nIf you don't want to stay on position make sure you entered a float.")
			
		#If nothing entered on Z stay at current position
		if not isinstance(pointZ, float):
			pointY = trans[2]
			print("Will stay on current coordinate on Z.\nIf you don't want to stay on position make sure you entered a float.")
			
		confirm = raw_input("Do you want to go to [",pointX,",",pointY,",",pointY,"] (y/n)? (q to quit) \n")
	
	if confirm == "yes" || confirm == "y":
		
		#boolean for test position
		onX = False
		onY = False
		onZ = False
		
		while (onX and onY and onZ)==False:
			try:
				
				(trans,rot)=listener.lookupTransform('odom','base_link',rospy.Time(0))
				
				twist = Twist()
				
				#init twist X Y Z
				twist.linear.x = 0.0; twist.linear.y = 0.0; twist.linear.z = 0.0
				twist.angular.x = 0.0; twist.angular.y = 0.0; twist.angular.z = 0.0
				
				#Movement condition on X
				if trans[0] < pointX-epsilon:
				twist.linear.x = 0.5
				
				elif trans[0] > pointX+epsilon:
				twist.linear.x = -0.5
				
				else
				onX=True
				
				#Movement condition on Y
				if trans[1] < pointY-epsilon:
				twist.linear.y = 0.5
				
				elif trans[1] > pointY+epsilon:
				twist.linear.y = -0.5
				
				else :
				onY=True
				
				#Movement condition on Z
				if trans[2] < pointZ-epsilon :
				twist.linear.z = 0.5
				
				if trans[2] > pointZ+epsilon :
				twist.linear.z = -0.5
				
				else :
				onZ=False
				
				
				print("twist: ", twist)				
				pub.publish(twist)
				
				

			except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):

				
				continue

			print("trans: ",trans)
			print("rot: ", rot)
		
		(trans,rot)=listener.lookupTransform('odom','base_link',rospy.Time(0))
		print("Arrived at target: [",trans[0],",",trans[1],",",trans[2],"] !")

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
	listener = tf.TransformListener()

	rospy.init_node('bebop_tf_listener', anonymous= True, disable_signals=True)
	
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

