#!/usr/bin/env python
import roslib;
import rospy

from geometry_msgs.msg import Twist,Pose, Point
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

ar_pos = []
currentProsition=()
lastCurrentProsition=()

safe_dist = [1.0,0.0,0.0]

def PIDController():
	
	m = 1 #Number of meter around the point at which the drone start to slow

	if abs(objectifPoint[0]-currentProsition[0])<m:
		Xcoeff=abs((objectifPoint[0]-currentProsition[0])/m)
	else:
		Xcoeff=1.0
	if abs(objectifPoint[1]-currentProsition[1])<m:				
		Ycoeff=abs((objectifPoint[1]-currentProsition[1])/m)
	else:
		Ycoeff=1.0
	if abs(objectifPoint[2]-currentProsition[2])<m:
		Zcoeff=abs((objectifPoint[2]-currentProsition[2])/m)
	else:
		Zcoeff=1.0

	Xcoeff/=2
	Ycoeff/=2
	Zcoeff/=2

	return (Xcoeff,Ycoeff,Zcoeff)

def callback(msg):

	epsilon=0.5

	lastAr_pos=ar_pos
	for markers in msg.markers:
		if ID_MARKER is markers.id:
			print("AR identifie") 
			ar_pos.append((markers.pose.pose.position.x, markers.pose.pose.position.y,markers.pose.pose.position.z))

	if len(ar_pos)>5
		for i in ar_pos:
			avgX+=ar_pos[0]
			avgY+=ar_pos[1]
			avgZ+=ar_pos[2]

		currentProsition=(avgX/len(ar_pos),avgY/len(ar_pos),avgZ/len(ar_pos))
		print("Updating pos: ",currentProsition)

def odometry_callback(msg):
	lastCurrentProsition=currentProsition
	currentProsition=msg.pose.pose.position
	print("Odom pos: ",currentProsition)
	
	

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

	rospy.init_node('follow_ar', anonymous= True, disable_signals=True)
	sub = rospy.Subscriber("/ar_pose_marker", AlvarMarkers, callback)
	rospy.Subscriber("/bebop/odom", Odometry, odometry_callback)