#!/usr/bin/env python
import roslib;
import rospy

from geometry_msgs.msg import Twist
from std_msgs.msg import Empty
import sys, select, termios, tty, time
import tf
from math import exp
import numpy as np
import matplotlib.pyplot as plt
from copy import copy

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
	
	#outFile=open("controlOutDrone.txt")
	

	#Allowed error on position
	epsilon=0.3
	
	abscisseValues=[]
	coeffValues=[]	

	(trans,rot)=listener.lookupTransform('odom','base_link',rospy.Time(0))

	#boolean for test position
	onX = False
	onY = False
	onZ = False
		
	while (onX and onY and onZ) is False:
		lastTrans=copy(trans)
		lastRot=copy(rot)
		(trans,rot)=listener.lookupTransform('odom','base_link',rospy.Time(0))
		#print(lastTrans," ----  ---- ", trans)	
		if lastTrans[0]!=trans[0] or lastTrans[1]!=trans[1] or lastTrans[2]!=trans[2]:
			try:
				#boolean for test position
				onX = False
				onY = False
				onZ = False
			
				#Speed variation: 1-exp(-x)
				Xcoeff=1-exp(-abs(trans[0]-pointX)/3)
				Ycoeff=1-exp(-abs(trans[1]-pointY)/3)
				Zcoeff=1-exp(-abs(trans[2]-pointZ)/3)

				#Speed variation: Linear function
				"""if pointX-trans[0]<3:
					Xcoeff=abs((pointX-trans[0])/3)
				else:
					Xcoeff=1.0

				if pointY-trans[1]<3:				
					Ycoeff=abs((pointY-trans[1])/3)
				else:
					Ycoeff=1.0

				if pointZ-trans[2]<3:
					Zcoeff=abs((pointZ-trans[2])/3)
				else:
					Zcoeff=1.0"""

				#Speed variation: Linear function
				#a(atan(bx+c)+Pi/2)+d
				"""a=0.4
				b=2.0
				c=-2.6
				d=-0.15
				if abs(pointX-trans[0])<3:
					Xcoeff=a*(np.arctan(b*(abs(pointX-trans[0]))+c)+np.pi/2)+d
				else:
					Xcoeff=1.0

				if abs(pointY-trans[1])<3:				
					Ycoeff=a*(np.arctan(b*abs(pointY-trans[1])+c)+np.pi/2)+d
				else:
					Ycoeff=1.0

				if abs(pointZ-trans[2])<3:
					Zcoeff=a*(np.arctan(b*abs(pointZ-trans[2])+c)+np.pi/2)+d
				else:
					Zcoeff=1.0"""
				
				#Divided values by 2 for security
				Xcoeff=Xcoeff/2
				Ycoeff=Ycoeff/2
				Zcoeff=Zcoeff/2

				print(onX," --- ", onY," ---- ", onZ) 
	 
				print(Xcoeff," --- ", Ycoeff," ---- ", Zcoeff) 
				#outFile.write(onX," --- ", onY," ---- ", onZ,"\n")

				twist = Twist()
				
				#init twist X Y Z
				twist.linear.x = 0.0; twist.linear.y = 0.0; twist.linear.z = 0.0
				twist.angular.x = 0.0; twist.angular.y = 0.0; twist.angular.z = 0.0
				
				#Movement condition on X
				if trans[0] < pointX-epsilon:
					twist.linear.x = Xcoeff
					print("x=",twist.linear.x,"\n")
				
				elif trans[0] > pointX+epsilon:
					twist.linear.x = -Xcoeff
					print("x=",twist.linear.x,"\n")
				
				else:
					onX=True
					print("x=",twist.linear.x,"\n")

				#outFile.write("x=",twist.linear.x,"\n")

				#Movement condition on Y
				if trans[1] < pointY-epsilon:
					twist.linear.y = Ycoeff
					print("y=",twist.linear.y,"\n")
				
				elif trans[1] > pointY+epsilon:
					twist.linear.y = -Ycoeff
					print("y=",twist.linear.y,"\n")
				
				else :
					onY=True
					print("y=",twist.linear.y,"\n")
			
				#outFile.write("y=",twist.linear.y,"\n")

				#Movement condition on Z
				if trans[2] < pointZ-epsilon :
					twist.linear.z = Zcoeff
					print("z=",twist.linear.z,"\n")
				
				if trans[2] > pointZ+epsilon :
					twist.linear.z = -Zcoeff
					print("z=",twist.linear.z,"\n")
				
				else :
					onZ=True
					print("z=",twist.linear.z,"\n")

				#outFile.write("z=",twist.linear.z,"\n")
				
				
				print("twist: ", twist)
				#outFile.write("twist: ", twist,"\n")			
				pub.publish(twist)
				print(onX," --- ", onY," ---- ", onZ)
				#outFile.write(onX," --- ", onY," ---- ", onZ,"\n")

			except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):

				
				continue

			print("trans odom: ",trans)
			#outFile.write("trans odom: ",trans,"\n")
			print("rot: ", rot)
		
			#Remembering X values to plot curve
			abscisseValues.append(pointX-trans[0])
			coeffValues.append(Xcoeff)
		#else:
			#print("no command sent")
		
	(trans,rot)=listener.lookupTransform('odom','base_link',rospy.Time(0))
	#outFile.close()
	return (trans,rot,coeffValues,abscisseValues)


def askPointToUser():
	confirm="n"
	
	while confirm == "n" or confirm =="no":
		print("Enter a [x;y;z] coordinate:\n")
		
		pointX = float(raw_input("X: ? \n"))
		pointY = float(raw_input("Y: ? \n"))
		pointZ = float(raw_input("Z: ? \n"))
		if pointZ<2.0:
			pointZ = 1
			print("Security Z must be > 1 m")
			
		print("The point you want to go is [",pointX,",",pointY,",",pointZ,"]")
		confirm = raw_input("Confirm the destination ? yes (y) or no (n). q to leave")
	
	if confirm == "yes" or confirm == "y":
		
		return (pointX, pointY, pointZ)


def autopilot():

	#Rate to send data is 3Hz
	rate = rospy.Rate(2.0)
	
	(trans,rot)=listener.lookupTransform('odom','base_link',rospy.Time(0))
	(transCam,rotCam)=listenerCam.lookupTransform('base_link','camera_base_link',rospy.Time(0))
	
	#User enter input using keyboard
	#points=askPointToUser()
	#pointX=points[0]
	#pointY=points[1]
	#pointZ=points[2]
	
	#Point values set in the code directly

	pointX=4.0
	pointY=0.0
	pointZ=1.0
	print("From python code: The point you want to go is [",pointX,",",pointY,",",pointZ,"]")
	print("Use CTRL+C to take control of the drone")
	time.sleep(2)
	res=goToPoint(pointX, pointY, pointZ)

	print("Arrived at target: [",res[0][0],",",res[0][1],",",res[0][2],"] !")
	plt.plot(res[3], res[2])


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
			plt.show()   		
			termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
	else:
		print("No start")

