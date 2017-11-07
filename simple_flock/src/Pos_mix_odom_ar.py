#!/usr/bin/env python
import roslib
import rospy

from geometry_msgs.msg import Twist,Pose, Point
from nav_msgs.msg import Odometry
from std_msgs.msg import Empty
from ar_track_alvar_msgs.msg import AlvarMarkers

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

ID_MARKER = 4

currentPosition=(0.0,0.0,0.0)
OdomcurrentPosition=()
OdomlastPosition=()

safe_dist = [1.0,0.0,0.0]

def PIDController():
	
	m = 1 #Number of meter around the point at which the drone start to slow

	if abs(objectifPoint[0]-currentPosition[0])<m:
		Xcoeff=abs((objectifPoint[0]-currentPosition[0])/m)
	else:
		Xcoeff=1.0
	if abs(objectifPoint[1]-currentPosition[1])<m:				
		Ycoeff=abs((objectifPoint[1]-currentPosition[1])/m)
	else:
		Ycoeff=1.0
	if abs(objectifPoint[2]-currentPosition[2])<m:
		Zcoeff=abs((objectifPoint[2]-currentPosition[2])/m)
	else:
		Zcoeff=1.0

	Xcoeff/=2
	Ycoeff/=2
	Zcoeff/=2

	return (Xcoeff,Ycoeff,Zcoeff)

"""Currently updates with the difference the distance between Alvar and drone
But needs to be modify by updating with the know coordonates of Alvar + distance between
drone and alvar"""
def alvar_callback(msg):
	global currentPosition

	for markers in msg.markers:
		if ID_MARKER is markers.id:
			print("AR identifie")
			currentPosition=(markers.pose.pose.position.x, markers.pose.pose.position.y,markers.pose.pose.position.z)
			#pubPos.publish(markers.pose.pose.position)
			print("Updating pos: ",currentPosition)


""" This function only compute the difference in the odometry 
current and last positions and had it to the position"""
def odometry_callback(msg):

	pos=Point()

	global currentPosition
	global OdomlastPosition
	"""ADDED on nov 5th To be tested"""

	if OdomlastPosition:
		posX=msg.pose.pose.position.x-OdomlastPosition[0]
		posY=msg.pose.pose.position.y-OdomlastPosition[1]
		posZ=msg.pose.pose.position.z-OdomlastPosition[2]
		currentPosition=(currentPosition[0]+posX,currentPosition[1]+posY,currentPosition[2]+posZ)

	OdomlastPosition=(msg.pose.pose.position.x,msg.pose.pose.position.y,msg.pose.pose.position.z)

	pos.x=currentPosition[0]
	pos.y=currentPosition[1]
	pos.z=currentPosition[2]
	#pubPos.publish(pos)

	print("pos: ",currentPosition)


def getKey():
	tty.setraw(sys.stdin.fileno())
	select.select([sys.stdin], [], [], 0)
	key = sys.stdin.read(1)
	termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
	return key


if __name__=="__main__":

	if len(sys.argv) == 1:
		name="bebop"

	else:
		name=str(argv[1])

	settings = termios.tcgetattr(sys.stdin)
	
	pub = rospy.Publisher('/bebop/cmd_vel', Twist, queue_size = 1)
	pubTakeoff = rospy.Publisher('/bebop/takeoff', Empty, queue_size = 1)
	pubLand = rospy.Publisher('/bebop/land', Empty, queue_size = 1)
	pubPos = rospy.Publisher(name+'_Pos', Point, queue_size = 1)	
	rospy.init_node('follow_ar', anonymous= True, disable_signals=True)
	sub = rospy.Subscriber("/ar_pose_marker", AlvarMarkers, alvar_callback)
	rospy.Subscriber("/bebop/odom", Odometry, odometry_callback)
	time.sleep(3)
	pubTakeoff.publish()
	rospy.spin()
