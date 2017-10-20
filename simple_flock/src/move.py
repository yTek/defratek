#!/usr/bin/env python
import roslib;
import rospy

from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from std_msgs.msg import Empty
import sys, select, termios, tty, time
import tf
from math import exp,sqrt
import numpy as np
import time 
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

#Autmod=True => bebop follow script
autoMod=True

#Point variable
currentProsition=[]
objectifPointList=[]
objectifPoint=[0.0,0.0,1.0]

#List to plot curve
abscisseValues=[]
timeValues=[]
coeffValues=[]	

def getSimpleControl():

	#get direction (+ or -)
	dirX= float (np.sign(objectifPoint[0]-currentProsition.x))
	dirY= float (np.sign(objectifPoint[1]-currentProsition.y))
	dirZ= float (np.sign(objectifPoint[2]-currentProsition.z))

	#Speed variation: 1-exp(-x)
	Xcoeff= 1-exp(-abs(currentProsition.x-objectifPoint[0])/3)
	Ycoeff= 1-exp(-abs(currentProsition.y-objectifPoint[1])/3)
	Zcoeff= 1-exp(-abs(currentProsition.z-objectifPoint[2])/3)

	#Speed variation: Linear function
	"""if abs(objectifPoint[0]-currentProsition[0])<3:
		Xcoeff=abs((objectifPoint[0]-currentProsition[0])/3)
	else:
		Xcoeff=1.0

	if abs(objectifPoint[1]-currentProsition[1])<3:				
		Ycoeff=abs((objectifPoint[1]-currentProsition[1])/3)
	else:
		Ycoeff=1.0

	if abs(objectifPoint[2]-currentProsition[2])<3:
		Zcoeff=abs((objectifPoint[2]-currentProsition[2])/3)
	else:
		Zcoeff=1.0"""

	#Speed variation: Linear function
	#a(atan(bx+c)+Pi/2)+d
	"""a=0.4
	b=2.0
	c=-2.6
	d=-0.15
	if abs(objectifPoint[0]-currentProsition[0])<3:
		Xcoeff=a*(np.arctan(b*(abs(objectifPoint[0]-currentProsition[0]))+c)+np.pi/2)+d
	else:
		Xcoeff=1.0

	if abs(objectifPoint[1]-currentProsition[1])<3:				
		Ycoeff=a*(np.arctan(b*abs(objectifPoint[1]-currentProsition[1])+c)+np.pi/2)+d
	else:
		Ycoeff=1.0

	if abs(objectifPoint[1]-currentProsition[2])<3:	
		Zcoeff=a*(np.arctan(b*abs(objectifPoint[2]-currentProsition[2])+c)+np.pi/2)+d
	else:
		Zcoeff=1.0"""

	return [dirX*Xcoeff,dirY*Ycoeff,dirZ*Zcoeff]


def odometry_callback(msg):

	#Allowed error on position
	epsilon=0.3
	
	#Updating current Postion
	global currentProsition
	currentProsition=msg.pose.pose.position
	print(currentProsition)
	global objectifPoint
	print("Pos --- X: ", objectifPoint[0]," | Y: ", objectifPoint[1]," | Z: ", objectifPoint[2])
	
	#Automod
	if autoMod==True:
		twist = Twist()
		#init twist X Y Z
		twist.linear.x = 0.0; twist.linear.y = 0.0; twist.linear.z = 0.0
		twist.angular.x = 0.0; twist.angular.y = 0.0; twist.angular.z = 0.0

		cmd=getSimpleControl()

		#Test if in point radius
		if sqrt( (currentProsition.x-objectifPoint[0])**2 + (currentProsition.y-objectifPoint[1])**2 +(currentProsition.z-objectifPoint[2])**2 ) > epsilon:
			twist.linear.x = cmd[0]; twist.linear.y = cmd[1]; twist.linear.z = cmd[2]
			#List to plot curve
			abscisseValues.append(currentProsition.x)
			coeffValues.append(cmd[0])	
			pub.publish(twist)

		elif objectifPointList:
			print("Arrived at ", objectifPoint)
			objectifPoint=objectifPointList.pop()
			print("New objectif is ", objectifPoint)

		elif not objectifPointList and objectifPoint!=[0.0,0.0,1.0]:
			pubLand.publish()
			print("Land!")
			print("Landed at : [",currentProsition.x,",",currentProsition.y,",",currentProsition.z,"] !")
			#plt.plot(abscisseValues, coeffValues)
			#plt.show()   		

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
	
	#User enter input using keyboard
	#points=askPointToUser()
	#pointX=points[0]
	#pointY=points[1]
	#pointZ=points[2]
	
	#Point values set in the code directly
	time.sleep(5)
	objectifPointList.append([4.0,0.0,1.0])
	objectifPointList.append([2.0,0.0,1.0])
	print("New objectif is: ", [2.0,0.0,1.0])

	rospy.spin()


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

	rospy.Subscriber("/bebop/odom", Odometry, odometry_callback)
	#listener = tf.TransformListener()
	#listenerCam= tf.TransformListener()

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
				print ("SIGNAL!!! \n MODE MANUEL ENCLENCHE")
				print (msg)
				while(1):
					key = getKey()
					if key in mvtBindings.keys():
						print(key)
						x = mvtBindings[key][0]
						y = mvtBindings[key][1]
						z = mvtBindings[key][2]
						th = mvtBindings[key][3]
						if (status == 14):
							print (msg)
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

