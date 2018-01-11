#!/usr/bin/env python
import roslib
import rospy

from geometry_msgs.msg import Twist,Pose, Point
from std_msgs.msg import Empty
from nav_msgs.msg import Odometry

class bebop:

	def __init__(self, name='bebop'):
		#PARAMETERS
		self.name = name
		if name = 'bebop1':
			self.leader = True
		else:
			self.leader = False
		
		self.currentPosition=(0.0,0.0,0.0)
		self.OdomLastPosition=()
		self.cap=0.0
		#self.speed variable
		self.speed=(0.0,0.0,0.0)
		#Time variable
		self.posTime=0
		self.lastPosTime=0;
		
		self.direction = [0.0,0.0,0.0]
		self.ready = [False,False]

		#Initialisation
		pubPos = rospy.Publisher("/"+self.name+"_Pos", Point, queue_size = 1)
		
		pubCmd = rospy.Publisher("/"+self.name+"/cmd_vel", Twist, queue_size = 1)
		pubTakeoff= rospy.Publisher("/"+self.name+'/takeoff', Empty, queue_size = 1)
		pubLand = rospy.Publisher('/'+self.name+'/land', Empty, queue_size = 1)

		subObj = rospy.Subscriber('/global_dir', Point, obj_callback)
		subObj = rospy.Subscriber('/flock_Manager', Point, flock_callback)

		rospy.init_node(self.name+'_controller', anonymous= True, disable_signals=True)
		sub_odom = rospy.Subscriber('/'+self.name+"/odom", Odometry, odometry_callback)
		
		twist = Twist()
		
		while self.stop is False:
			
			twist.linear.x = 0.0; twist.linear.y = 0.0; twist.linear.z = 0.0
			twist.angular.x = 0.0; twist.angular.y = 0.0; twist.angular.z = 0.0
			
			if False not in self.ready:
				#calcule de la cmd a envoyer
				#
				

				self.direction = [0.0,0.0,0.0]
				self.ready = [False,False]
		
			
			pubCmd.publish(twist)
	


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


	def obj_callback(msg):
		if !self.ready[0]:
			self.direction += msg.x
			self.direction += msg.y
			self.direction += msg.z
			self.ready[0] = True

	def flock_callback(msg):
		if !self.ready[1]:
			#Le leader et le suiveur font des ordres opposes pour se rapprocher ou s'eloigner
			#ATTENTION: dans le referentiel global et represente la distance a faire		
			if self.leader = True:
				self.direction -= msg.x
				self.direction -= msg.y
				
			else:
				self.direction += msg.x
				self.direction += msg.y
				
			self.ready[1] = True

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

			self.currentPosition=(self.currentPosition[0]+posX,self.currentPosition[1]+posY,self.currentPosition[2]+posZ)

		self.OdomLastPosition=(odomOnX,odomOnY,odomOnZ)
		self.lastPosTime=self.posTime




if __name__=="__main__":
	if len(sys.argv) == 1:
		bp = bebop()
	else:
		name_arg=str(sys.argv[1])
		bp = bebop(name = name_arg)


