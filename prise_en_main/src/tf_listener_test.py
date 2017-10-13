#!/usr/bin/env python
import roslib;
import rospy

from geometry_msgs.msg import Twist
from std_msgs.msg import Empty
import sys, select, termios, tty, time
import tf
from math import exp

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


def goToPoint(pointX, pointY, pointZ):
	#boolean for test position
	onX = False
	onY = False
	onZ = False
		
	while (onX and onY and onZ) is False:
		try:
			#boolean for test position
			onX = False
			onY = False
			onZ = False

			Xcoeff=1-exp(trans[0]-pointX)
			Ycoeff=1-exp(trans[1]-pointY)
			Zcoeff=1-exp(trans[2]-pointZ)

			print(onX," --- ", onY," ---- ", onZ) 
			(trans,rot)=listener.lookupTransform('odom','base_link',rospy.Time(0))
			print(Xcoeff," --- ", Ycoeff," ---- ", Zcoeff) 
				
			twist = Twist()
				
			#init twist X Y Z
			twist.linear.x = 0.0; twist.linear.y = 0.0; twist.linear.z = 0.0
			twist.angular.x = 0.0; twist.angular.y = 0.0; twist.angular.z = 0.0
				
			#Movement condition on X
			if trans[0] < pointX-epsilon:
				twist.linear.x = 0.3
				print("x=",twist.linear.x,"\n")
				
			elif trans[0] > pointX+epsilon:
				twist.linear.x = -0.3
				print("x=",twist.linear.x,"\n")
				
			else:
				onX=True
				print("x=",twist.linear.x,"\n")
				
			#Movement condition on Y
			if trans[1] < pointY-epsilon:
				twist.linear.y = 0.3
				print("y=",twist.linear.y,"\n")
				
			elif trans[1] > pointY+epsilon:
				twist.linear.y = -0.3
				print("y=",twist.linear.y,"\n")
				
			else :
				onY=True
				print("y=",twist.linear.y,"\n")
				
			#Movement condition on Z
			if trans[2] < pointZ-epsilon :
				twist.linear.z = 0.3
				print("z=",twist.linear.z,"\n")
				
			if trans[2] > pointZ+epsilon :
				twist.linear.z = -0.3
				print("z=",twist.linear.z,"\n")
				
			else :
				onZ=True
				print("z=",twist.linear.z,"\n")
				
				
			print("twist: ", twist)				
			pub.publish(twist)
			print(onX," --- ", onY," ---- ", onZ)

		except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):

				
			continue

		print("trans odom: ",trans)
		print("trans camera: ",transCam)
		print("rot: ", rot)
		
	(trans,rot)=listener.lookupTransform('odom','base_link',rospy.Time(0))
	return (trans,rot)

def autopilot():

	#Rate to send data is 3Hz
	rate = rospy.Rate(3.0)

	(trans,rot)=listener.lookupTransform('odom','base_link',rospy.Time(0))
	(transCam,rotCam)=listenerCam.lookupTransform('base_link','camera_base_link',rospy.Time(0))

    
	#Allowed error on position
	epsilon=0.3
	
	confirm="n"
	
	while confirm == "n" or confirm =="no":
		print("Enter a [x;y;z] coordinate:\n")
		
		pointX = float(raw_input("X: ? (enter stay to 'stay' on current coordinate)\n"))
		pointY = float(raw_input("Y: ? (enter stay to 'stay' on current coordinate)\n"))
		pointZ = float(raw_input("Z: ? (enter stay to 'stay' on current coordinate)\n"))
		"""
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
			pointZ = 2
			print("Will stay on current coordinate on Z.\nIf you don't want to stay on position make sure you entered a float.")
		"""
		if pointZ<2.0:
			pointZ = 1
			print("Security Z must be > 1 m")
			
		print("The point you want to go is [",pointX,",",pointY,",",pointZ,"]")
		confirm = raw_input("Confirm the destination ? yes (y) or no (n). q to leave")
	
	if confirm == "yes" or confirm == "y":
		
		goToPoint(pointX, pointY, pointZ)

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
	listenerCam= tf.TransformListener()

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
			time.sleep(3)
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

