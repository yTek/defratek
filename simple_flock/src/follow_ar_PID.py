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

		j : rotation sens trigonometrique
		l : rotation sens horaire
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

ar_pos = None
lastAr_pos = None

safe_dist = [1.0,0.0,0.0]

def PIDController(pos,lastPos):
	
	m = 1 #Number of meter around the point at which the drone start to slow

	if abs(pos[0]-lastPos[0])<m:
		Xcoeff=abs((pos[0]-lastPos[0])/m)
	else:
		Xcoeff=1.0
	if abs(pos[1]-lastPos[1])<m:				
		Ycoeff=abs((pos[1]-lastPos[1])/m)
	else:
		Ycoeff=1.0
	if abs(pos[2]-lastPos[2])<m:
		Zcoeff=abs((pos[2]-lastPos[2])/m)
	else:
		Zcoeff=1.0

	Xcoeff/=2
	Ycoeff/=2
	Zcoeff/=2

	return (Xcoeff,Ycoeff,Zcoeff)

def callback(msg):
	global lastAr_pos
	global ar_pos

	for markers in msg.markers:
		if ID_MARKER is markers.id:
			print("AR identifie") 
			ar_pos = [markers.pose.pose.position.x, markers.pose.pose.position.y,markers.pose.pose.position.z]
	
	#Rate to send data is 10Hz
	rate = rospy.Rate(3.0)

	#Allowed error on position
	epsilon=[0.2,0.2,0.1]
	
	twist = Twist()
	#init twist X Y Z
	twist.linear.x = 0.0; twist.linear.y = 0.0; twist.linear.z = 0.0
	twist.angular.x = 0.0; twist.angular.y = 0.0; twist.angular.z = 0.0

	if lastAr_pos != ar_pos and (lastAr_pos is not None or ar_pos is not None):
		try:
			print("ar_pos:", ar_pos)
			
			#Get PID control
			control=PIDController(ar_pos,lastAr_pos)

			#Movement condition on X
			if ar_pos[0] < safe_dist[0]-epsilon[0]:
				twist.linear.x = control[0]
			elif ar_pos[0] > safe_dist[0]+epsilon[0]:
				twist.linear.x = control[0]
			else:
				twist.linear.x = 0.0

			print("x=",twist.linear.x)
	
			#Movement condition on Y
			if ar_pos[1] < safe_dist[1]-epsilon[1]:
				twist.linear.y = -control[1]
			elif ar_pos[1] > safe_dist[1]+epsilon[1]:
				twist.linear.y = control[1]
			else :
				twist.linear.y = 0.0
			
			print("y=",twist.linear.y)
	
			#Movement condition on Z
			if ar_pos[2] < safe_dist[1]-epsilon[2]:
				twist.linear.z = -control[2]
			elif ar_pos[2] > safe_dist[1]+epsilon[2]:
				twist.linear.z = control[0]
			else :
				twist.linear.z = 0.0
			
			print("z=",twist.linear.z)

		except:
			print("error")
		print("twist: ", twist.linear)				
	lastAr_pos= ar_pos[:]
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

	rospy.init_node('follow_ar', anonymous= True, disable_signals=True)
	sub = rospy.Subscriber("/ar_pose_marker", AlvarMarkers, callback)

	start = raw_input("Take off? ")
	#start="yes"
	if start == "yes":
		
		print("Take off")
		pubTakeoff.publish()
		
		x = 0.0
		y = 0.0
		z = 0.0
		th = 0.0
		status = 0.0
		
		a = None
		try:
			time.sleep(3)
			"""time_temp = time.time()
			twist = Twist()"""
			while 1:
				"""if time.time()-time_temp > 0.2:
					twist.linear.x = 0.0; twist.linear.y = 0.0; twist.linear.z = 0.0
					twist.angular.x = 0.0; twist.angular.y = 0.0; twist.angular.z = 0.0
					pub.publish(twist)
					time_temp = time.time()"""
				if a:
					break
					


		except KeyboardInterrupt:
			try:
				sub.unregister()
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

