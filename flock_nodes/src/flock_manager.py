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
		'z':(0.5,0,0,0),
		's':(-0.5,0,0,0),
		'q':(0,0.5,0,0),
		'd':(0,-0.5,0,0),
		'j':(0,0,0,0.5),
		'l':(0,0,0,-0.5),
		'i':(0,0,0.5,0),
		'k':(0,0,-0.5,0),
	       }


class flock_manager:

	def __init__(self, nbDrones = 2):
		#PARAMETERS
		self.nbDrones = nbDrones
		self.drones = []
		self.posDrones = {}

		self.pubCmd = {}
		self.pubTakeoff = {}
		self.pubLand = {}
		self.subPos = {}

		
		#Initialisation
		for i in range(self.nbDrones):
			self.drones.append('bebop%s'%(i+1))

		for i in self.drones:
			pubCmd[i] = rospy.Publisher("/"+i+"/cmd_vel", Twist, queue_size = 1)
			pubTakeoff[i] = rospy.Publisher("/"+i+'/takeoff', Empty, queue_size = 1)
			pubLand[i] = rospy.Publisher('/'+i+'/land', Empty, queue_size = 1)
			#Besoin de 1 callback par drone ??
			subPos[i] = rospy.Subscriber('/'+name+"_Pos", Point, pos_callback)
			rospy.init_node('follow_ar', anonymous= True, disable_signals=True)
		
	
		try:
			#Compute
					
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
			#close_log_file()   		
			termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
	

	def pos_callback(msg):
		




if __name__=="__main__":
	if len(sys.argv) == 1:
		fl_mng = flock_manager()
	else:
		nbDrones = int(sys.argv[1])
		fl_mng = flock_manager(nbDrones)




