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
objectifPosition=()
OdomcurrentPosition=()
OdomlastPosition=()
following=False
refreshIgnore=0
tempObj=(0.0,0.0,0.0)

safe_dist = (1.0,0.0,0.0)

def PIDController(pos):
	obj=safe_dist
	print ("pos: ",pos)
	print ("obj: ", obj)	

	m = 1 #Number of meter around the point at which the drone start to slow  

	if abs(pos[0]-obj[0])<m:
		Xcoeff=abs((pos[0]-obj[0])/m)
	else:
		Xcoeff=1.0
	if abs(pos[1]-obj[1])<m:				
		Ycoeff=abs((pos[1]-obj[1])/m)
	else:
		Ycoeff=1.0
	if abs(pos[2]-obj[2])<m:
		Zcoeff=abs((pos[2]-obj[2])/m)
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
	global objectifPosition
	global following
	global tempObj
	global refreshIgnore

	for markers in msg.markers:
		if ID_MARKER is markers.id:
			print("AR identifie")

			following=True
			
		

			"""#Difference between the drone location with regard to alvar and wanted location 
			deltaX=markers.pose.pose.position.x-safe_dist[0]
			deltaY=markers.pose.pose.position.y-safe_dist[1]
			deltaZ=markers.pose.pose.position.z-safe_dist[2]

			if refreshIgnore==0:
				objectifPosition=(currentPosition[0]+deltaX,currentPosition[1]+deltaY,currentPosition[2]+deltaZ)

			elif refreshIgnore==5:
    				tempObj()
    				refreshIgnore=0
			else:
    				refreshIgnore+=1"""

			#Updating position with alvar
			currentPosition=(markers.pose.pose.position.x, markers.pose.pose.position.y, markers.pose.pose.position.z)
			#print("Alvar: ",currentPosition)
			"""#Publishing pos for other drones
			pos.x=currentPosition[0]
			pos.y=currentPosition[1]
			pos.z=currentPosition[2]
			pubPos.publish(pos)"""

			#print("pos with ar: ",currentPosition,"\nObjectif point: ",objectifPosition)
			followLeader(currentPosition)


def odometry_callback(msg):

	pos=Point()

	global currentPosition
	global OdomlastPosition
	global objectifPosition
	global following

	#Inverse X so that +X is aiming where the bebop camera aims
	#Y pos is + when going left to where the camera aims
	#Z is + when drones goes up
	#Be careful with calibration
	odomOnX=-msg.pose.pose.position.x
	odomOnY=-msg.pose.pose.position.y
	odomOnZ=msg.pose.pose.position.z

	#Getting the delta corresponding to the movement 
	if OdomlastPosition:
		
		posX=odomOnX-OdomlastPosition[0]
		posY=odomOnY-OdomlastPosition[1]
		posZ=odomOnZ-OdomlastPosition[2]
		currentPosition=(currentPosition[0]-posX,currentPosition[1]-posY,currentPosition[2]+posZ)

	OdomlastPosition=(odomOnX,odomOnY,odomOnZ)

	"""#Publishing pos for other drones
	pos.x=currentPosition[0]
	pos.y=currentPosition[1]
	pos.z=currentPosition[2]
	pubPos.publish(pos)"""
	#print("pos odom: ",msg.pose.pose.position.x," [] ", msg.pose.pose.position.y," [] ", msg.pose.pose.position.z)
	#print("pos with magouille: ",currentPosition)
	followLeader(currentPosition)


def followLeader(pos):


	#print("ici")
	#Follows the drone's leader
	if following == True:
		#print('ici')
    	#Allowed error on position
		epsilon=[0.2,0.2,0.1]

		#init twist X Y Z
		twist = Twist()
		twist.linear.x = 0.0; twist.linear.y = 0.0; twist.linear.z = 0.0
		twist.angular.x = 0.0; twist.angular.y = 0.0; twist.angular.z = 0.0

		#Get PID control
		control=PIDController(currentPosition)

		#Movement condition on X
		if currentPosition[0] < safe_dist[0]-epsilon[0]:
			twist.linear.x = -control[0]
		elif currentPosition[0] > safe_dist[0]+epsilon[0]:
			twist.linear.x = control[0]
		else:
			twist.linear.x = 0.0

		print("x=",twist.linear.x)

		#Movement condition on Y
		if currentPosition[1] < safe_dist[1]-epsilon[1]:
			twist.linear.y = -control[1]
		elif currentPosition[1] > safe_dist[1]+epsilon[1]:
			twist.linear.y = control[1]
		else :
			twist.linear.y = 0.0
		
		print("y=",twist.linear.y)

		#Movement condition on Z
		"""if currentPosition[2] < safe_dist[1]-epsilon[2]:
			twist.linear.z = control[2]
		elif currentPosition[2] > safe_dist[1]+epsilon[2]:
			twist.linear.z = -control[2]
		else :
			twist.linear.z = 0.0"""
		
		#print("z=",twist.linear.z)

		pub.publish(twist)

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

	start = raw_input("Take off? ")
	#start="yes"
	if start == "yes":
		rospy.Subscriber("/bebop/odom", Odometry, odometry_callback)
		print("Take off")
		pubTakeoff.publish()
		
		x = 0.0
		y = 0.0
		z = 0.0
		th = 0.0
		status = 0.0
		
		a = None
		try:
			#time.sleep(3)
			"""time_temp = time.time()
			twist = Twist()"""
			while 1:
				"""if time.time()-time_temp > 0.2:
					twist.linear.x = 0.0; twist.linear.y = 0.0; twist.linear.z = 0.0
					twist.angular.x = 0.0; twist.angular.y = 0.0; twist.angular.z = 0.0
					pub.publish(twist)
					time_temp = time.time()"""
					
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
