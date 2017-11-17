#!/usr/bin/env python
import roslib
import rospy

from geometry_msgs.msg import Twist,Pose, Point
from nav_msgs.msg import Odometry
from std_msgs.msg import Empty
from ar_track_alvar_msgs.msg import AlvarMarkers
from subprocess import call
from numpy import sign

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

#-----GLOBAL VARIABLE-----

#Position variable
currentPosition=(0.0,0.0,0.0)

OdomcurrentPosition=()
OdomlastPosition=()

#Speed variable
Speed=(0.0,0.0,0.0)

#Time variable
posTime=0
lastPosTime=0;

#Log Files
posLog=None
speedLog=None

following=False

#Position offset constant
safe_dist = (1.0,0.0,0.0)

def init_log_file():

	global posLog
	global speedLog
	
	date=time.strftime("%d_%m_%Y")
	hour=time.strftime("%Ih%Mm%Ss")
	suffix=date+ "-" + hour

	call(["mkdir", "log/"+suffix])

	posLog=open("log/"+suffix+"/pos_"+suffix+".txt","w")

	posLog.write("----- Log test "+date+": pos -----\n")

	speedLog=open("log/"+suffix+"/speed_"+suffix+".txt","w")

	speedLog.write("----- Log test "+date+": pos -----\n")


def close_log_file():
    	
	global posLog
	global speedLog

	posLog.close()
	speedLog.close()

"""Currently updates with the difference the distance between Alvar and drone
But needs to be modify by updating with the know coordonates of Alvar + distance between
drone and alvar"""
def alvar_callback(msg):
	global currentPosition
	global objectifPosition
	global following

	for markers in msg.markers:
		if ID_MARKER is markers.id:
			print("AR identifie")
			print(markers.pose.pose.position.x, markers.pose.pose.position.y, markers.pose.pose.position.z)
			print("-------------")
			#if time.time()-arWait>0.5:

			#Updating position with alvar
			updatePosition(markers.pose.pose.position.x, markers.pose.pose.position.y, markers.pose.pose.position.z)
			
			following=True


def odometry_callback(msg):
	print("ODOM")
	global currentPosition
	global OdomlastPosition
	global Speed
	global lastPosTime
	global posTime

	#Inverse X so that +X is aiming where the bebop camera aims
	#Y pos is + when going left to where the camera aims
	#Z is + when drones goes up
	#Be careful with calibration
	odomOnX=-msg.pose.pose.position.x
	odomOnY=-msg.pose.pose.position.y
	odomOnZ=msg.pose.pose.position.z
	posTime=time.time()

	#Getting the delta corresponding to the movement 
	if OdomlastPosition:
		dt=posTime-lastPosTime
		posX=odomOnX-OdomlastPosition[0]
		posY=odomOnY-OdomlastPosition[1]
		posZ=odomOnZ-OdomlastPosition[2]

		if dt>0.1:
			Vx=(posX)/(dt)
			Vy=(posY)/(dt)
			Vz=(posZ)/(dt)

			Speed=(Vx,Vy,Vz)

			hour=time.strftime("%Ih%Mm%Ss")

			speedLog.write(hour+" --- "+str(Speed)+"\n")
			#speedLog.write("Dt:"+ str(posTime-lastPosTime) +"\n")
			print("Speed: ",Speed)


		updatePosition(currentPosition[0]-posX,currentPosition[1]-posY,currentPosition[2]-posZ)

	OdomlastPosition=(odomOnX,odomOnY,odomOnZ)
	lastPosTime=posTime



def PIDController(pos):
    
	global Speed
	a=1#can be change to change reactiveness of the drone
	obj=safe_dist

	print ("pos: ",pos)
	#print ("obj: ", obj)	

	m = 1 #Number of meter around the point at which the drone start to slow

	#Computing wanted speed
	if abs(pos[0]-obj[0])<m:
		WVx=(pos[0]-obj[0])/m
	else:
		WVx=sign(pos[0]-obj[0])
	if abs(pos[1]-obj[1])<m:				
		WVy=(pos[1]-obj[1])/m
	else:
		WVy=sign(pos[1]-obj[1])
	if abs(pos[2]-obj[2])<m:
		WVz=(pos[2]-obj[2])/m
	else:
		WVz=sign(pos[2]-obj[2])

	WVx/=3
	WVy/=3
	WVz/=3

	#Computing controls according to current V
	#Function used is f(x)=x/(x+a)
	Xcoeff=(WVx-Speed[0])/(abs(WVx-Speed[0])+a)
	Ycoeff=(WVy-Speed[1])/(abs(WVy-Speed[1])+a)
	Zcoeff=(WVz-Speed[2])/(abs(WVz-Speed[2])+a)
	
	#print("Xcoef =",Xcoeff,"; Ycoef = ",Ycoeff," ; Zcoef = ", Zcoeff)

	return (Xcoeff,Ycoeff,Zcoeff)

"""Update the global position"""
def updatePosition(posX,posY,posZ):
	global currentPosition
	global posLog
	global speedLog

	currentPosition=(posX, posY, posZ)

	hour=time.strftime("%Ih%Mm%Ss")
	posLog.write(hour+" --- "+str(currentPosition)+"\n")

	"""print("pos")
	print (currentPosition)
	print (lastPosition)
	print("time")
	print (posTime)
	print (lastPosTime)"""

	#Avoid a jump in speed values when switching from
	#odom pos to alvar pos for the 1st time
	if following==True:
		followLeader(currentPosition)


def followLeader(pos):

	#Follows the drone's leader
	if following == True:
		#print('ici')
    	#Allowed error on position
		epsilon=[0.1,0.1,0.1]

		#init twist X Y Z
		twist = Twist()
		twist.linear.x = 0.0; twist.linear.y = 0.0; twist.linear.z = 0.0
		twist.angular.x = 0.0; twist.angular.y = 0.0; twist.angular.z = 0.0

		#Get PID control
		control=PIDController(currentPosition)

		#Movement condition on X
		if currentPosition[0] < safe_dist[0]-epsilon[0]:
			twist.linear.x = control[0]
		elif currentPosition[0] > safe_dist[0]+epsilon[0]:
			twist.linear.x = control[0]
		else:
			twist.linear.x = 0.0

		print("x=",twist.linear.x)

		#Movement condition on Y
		if currentPosition[1] < safe_dist[1]-epsilon[1]:
			twist.linear.y = control[1]
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
		
		print("z=",twist.linear.z)

		pub.publish(twist)

def getKey():
	tty.setraw(sys.stdin.fileno())
	select.select([sys.stdin], [], [], 0)
	key = sys.stdin.read(1)
	termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
	return key


if __name__=="__main__":
	
	#Name init
	if len(sys.argv) == 1:
		name="bebop"

	else:
		name=str(argv[1])

	init_log_file()

	settings = termios.tcgetattr(sys.stdin)

	#Topics init
	pub = rospy.Publisher("/"+name+"/cmd_vel", Twist, queue_size = 1)
	pubTakeoff = rospy.Publisher("/"+name+'/takeoff', Empty, queue_size = 1)
	pubLand = rospy.Publisher('/'+name+'/land', Empty, queue_size = 1)
	pubPos = rospy.Publisher(name+'_Pos', Point, queue_size = 1)	
	rospy.init_node('follow_ar', anonymous= True, disable_signals=True)
	

	start = raw_input("bebop2: Take off? ")

	#start="yes"
	if start == "yes":
		sub_odom = rospy.Subscriber('/'+name+"/odom", Odometry, odometry_callback)
		sub_alvar = rospy.Subscriber("/ar_pose_marker", AlvarMarkers, alvar_callback, queue_size = 1)
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
				sub_alvar.unregister()
				sub_odom.unregister()
				print "SIGNAL!!! \n MODE MANUEL ENCLENCHE"
				print msg
				while(1):
					key = getKey()
					if key in mvtBindings.keys():
						print(key)
						x = mvtBindings[key][0]/3.0
						y = mvtBindings[key][1]/3.0
						z = mvtBindings[key][2]/3.0
						th = mvtBindings[key][3]/3.0
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
					#print(twist)				
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
			close_log_file()   		
			termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
	else:
		print("No start")
