#!/usr/bin/env python
import roslib
import rospy

from geometry_msgs.msg import Twist,Pose, Point
from nav_msgs.msg import Odometry
from std_msgs.msg import Empty
from ar_track_alvar_msgs.msg import AlvarMarkers
from subprocess import call
from numpy import sign,pi,cos,sin,sqrt

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

class flock_manager:

	def __init__(self, name = "bebop"):
		self.name = name

		#PARAMETERS
		self.ID_MARKER = 4

		#Other drone position
		self.leaderPosition=(0.0,0.0,0.0)
		#Position variable
		self.currentPosition=(0.0,0.0,0.0)
		self.OdomLastPosition=()
		self.cap=0.0
		#self.speed variable
		self.speed=(0.0,0.0,0.0)
		#Time variable
		self.posTime=0
		self.lastPosTime=0;

		self.following=False

		#Position offset constant
		self.safeDist = (1.5,0.0,0.0)

		#Topics init
		pub = rospy.Publisher("/"+self.name+"/cmd_vel", Twist, queue_size = 1)
		pubTakeoff = rospy.Publisher("/"+self.name+'/takeoff', Empty, queue_size = 1)
		pubLand = rospy.Publisher('/'+self.name+'/land', Empty, queue_size = 1)
		pubPos = rospy.Publisher(self.name+'_Pos', Point, queue_size = 1)
		rospy.init_node('follow_ar', anonymous= True, disable_signals=True)
		
		start = raw_input(self.name+": Take off? ")

		#start="yes"
		if start == "yes":
			sub_odom = rospy.Subscriber('/'+self.name+"/odom", Odometry, odometry_callback)
			sub_alvar = rospy.Subscriber("/ar_pose_marker", AlvarMarkers, alvar_callback, queue_size = 1)

			print("Take off")
			pubTakeoff.publish()

			try:
				time.sleep(3)
				print("Ready to follow")
				sub_leader = rospy.Subscriber("bebop1_Pos", Point, leader_callback)#bebop 1 is leader
				while 1:
					"""
					infinite loop, wait for Exception ctrl+C	
					"""	
			except KeyboardInterrupt:
				try:
					settings = termios.tcgetattr(sys.stdin)
					#Unregister
					sub_alvar.unregister()
					sub_odom.unregister()
					sub_leader.unregister()
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


	def leader_callback(msg):
		#Get the leader position
		print("Leader pos: ")
		self.leaderPosition=(msg.x,msg.y,msg.z)
		print(self.leaderPosition)
		#self.following=True
		#Avoid a jump in speed values when switching from
		#odom pos to alvar pos for the 1st time
		if self.following==True:
			followLeader(self.leaderPosition)


	"""Currently updates with the difference the distance between Alvar and drone
	But needs to be modify by updating with the know coordonates of Alvar + distance between
	drone and alvar"""
	def alvar_callback(msg):
		for markers in msg.markers:
			if self.ID_MARKER is markers.id:
				print("AR identifie")
				#Updating position with alvar
				leaderInDroneRef = referentiel_global2drone(self.leaderPosition)
				objInDroneRef = (leaderInDroneRef[0]-markers.pose.pose.position.x,leaderInDroneRef[1] + markers.pose.pose.position.y,leaderInDroneRef[2]+markers.pose.pose.position.z)
				print("IN DRONE REF POS:",objInDroneRef)

				obj=referentiel_drone2global(objInDroneRef)
				print("GLOBAL REF POS:",obj)

				updatePosition(obj[0], obj[1],obj[2])
				print("Following")

				self.following=True


	def odometry_callback(msg):

		odomOnX= msg.pose.pose.position.x
		odomOnY= msg.pose.pose.position.y
		odomOnZ=msg.pose.pose.position.z
		self.cap = msg.pose.pose.orientation.z
		self.posTime=time.time()

		#Getting the delta corresponding to the movement 
		if self.OdomLastPosition:
			dt=self.posTime-self.lastPosTime
			posX=odomOnX-self.OdomLastPosition[0]
			posY=odomOnY-self.OdomLastPosition[1]
			posZ=odomOnZ-self.OdomLastPosition[2]

			if dt>0.1:
				Vx=(posX)/(dt)
				Vy=(posY)/(dt)
				Vz=(posZ)/(dt)

				self.speed=(Vx,Vy,Vz)
				#print("self.speed: "+str(self.speed))


			updatePosition(self.currentPosition[0]+posX,self.currentPosition[1]+posY,self.currentPosition[2]+posZ)

		self.OdomLastPosition=(odomOnX,odomOnY,odomOnZ)
		self.lastPosTime=self.posTime


	def PIDController(pos):

		a=1 #can be change to change reactiveness of the drone
		leaderInDroneRef = referentiel_global2drone(self.leaderPosition)
		objInDroneRef = (leaderInDroneRef[0]-self.safeDist[0],leaderInDroneRef[1]-self.safeDist[1],leaderInDroneRef[2]-self.safeDist[2])
		obj = referentiel_drone2global(objInDroneRef)
		#We need the Objective in the global referentiel, since the odometry (so our pos) we use is in the global ref

		print ("pos: ",pos)
		print ("obj: ", obj)	

		m = 1 #Number of meter around the point at which the drone start to slow

		#Computing wanted speed
		if abs(pos[0]-obj[0])<m:
			WVx=(obj[0]-pos[0])/m
		else:
			WVx=sign(obj[0]-pos[0])
		if abs(pos[1]-obj[1])<m:				
			WVy=(obj[1]-pos[1])/m
			#print("WVy : ", WVy)
		else:
			WVy=sign(pos[1]-obj[1])
		if abs(pos[2]-obj[2])<m:
			WVz=(obj[2]-pos[2])/m
		else:
			WVz=sign(obj[2]-pos[2])

		WVx/=5.0
		WVy/=5.0
		WVz/=5.0

		#Computing controls according to current V
		#Function used is f(x)=x/(x+a)
		Xcoeff=(WVx-self.speed[0])/(abs(WVx-self.speed[0])+a)
		Ycoeff=(WVy-self.speed[1])/(abs(WVy-self.speed[1])+a)
		Zcoeff=(WVz-self.speed[2])/(abs(WVz-self.speed[2])+a)
	
		#print("Xcoef =",Xcoeff,"; Ycoef = ",Ycoeff," ; Zcoef = ", Zcoeff)

		return (Xcoeff,Ycoeff,Zcoeff)


	"""Update the global position"""
	def updatePosition(posX,posY,posZ):
		self.currentPosition=(posX, posY, posZ)
		print("Current pos: ", self.currentPosition)


	def followLeader(pos_leader):
	
		leaderInDroneRef=referentiel_global2drone(self.leaderPosition)
		objInDroneRef=(leaderInDroneRef[0]-self.safeDist[0],leaderInDroneRef[1]-self.safeDist[1],leaderInDroneRef[2]-self.safeDist[2])
		obj=referentiel_drone2global(objInDroneRef)

		#Follows the drone's leader
		if self.following == True:
	    		#Allowed error on position
			epsilon=[0.1,0.1,0.1]

			#init twist X Y Z
			twist = Twist()
			twist.linear.x = 0.0; twist.linear.y = 0.0; twist.linear.z = 0.0
			twist.angular.x = 0.0; twist.angular.y = 0.0; twist.angular.z = 0.0

			#Get PID control
			global_control=PIDController(self.currentPosition)
			print("Global control", global_control)
			control = referentiel_global2drone(global_control)
			print("Control : ",control)
		
			norm= sqrt((obj[0]-self.currentPosition[0])**2 + (obj[1]-self.currentPosition[1])**2)
			print("Distance Norm: "+str(norm))
			if norm > 0.2:
				twist.linear.x = control[0]
				twist.linear.y = control[1]
				#twist.linear.z = control[2]

			print(twist)
			pub.publish(twist)


	def referentiel_global2drone(pidControl):
		theta = self.cap*pi
		xdrone= pidControl[0]*cos(theta)+pidControl[1]*sin(theta)
		ydrone= -pidControl[0]*sin(theta)+pidControl[1]*cos(theta)
		return (xdrone,ydrone,pidControl[2])

	def referentiel_drone2global(coordonatesDrone):
		theta = self.cap*pi
		xglobal= coordonatesDrone[0]*cos(theta)-coordonatesDrone[1]*sin(theta)
		yglobal= coordonatesDrone[0]*sin(theta)+coordonatesDrone[1]*cos(theta)
		return (xglobal,yglobal,coordonatesDrone[2])
	
	"""To get the pushed keys on the keyboard"""
	def getKey():
		tty.setraw(sys.stdin.fileno())
		select.select([sys.stdin], [], [], 0)
		key = sys.stdin.read(1)
		termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
		return key



if __name__=="__main__":
	if len(sys.argv) == 1:
		fl_mng = flock_manager()
	else:
		name=str(sys.argv[1])
		fl_mng = flock_manager(name)

