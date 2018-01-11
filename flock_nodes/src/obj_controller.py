#!/usr/bin/env python
import roslib
import rospy

from geometry_msgs.msg import Twist,Pose, Point
from std_msgs.msg import Empty
import sys, select, termios, tty, time

mvtBindings = {
		'z':(0.1,0,0,0),
		's':(-0.1,0,0,0),
		'q':(0,0.1,0,0),
		'd':(0,-0.1,0,0),
		'j':(0,0,0,0.1),
		'l':(0,0,0,-0.1),
		'i':(0,0,0.1,0),
		'k':(0,0,-0.1,0),
	       }

class obj_controller:

	def __init__(self,x_=0.0,y_=0.0,z_=0.0,teleop=False):
		#PARAMETERS
		self.stop = False
		self.obj = Point()
		self.obj.x = x_
		self.obj.y = y_
		self.obj.z = z_
		self.teleop = teleop
		
		#Initialisation
		#Vector ???
		pubObj = rospy.Publisher('/global_dir', Point, queue_size = 1)
		rospy.init_node('obj_controller', anonymous= True, disable_signals=True)
		sub_stop = rospy.Subscriber('/bebop_stop', Empty, stop_callback)
		
		if !self.teleop:
			"""
			while stop is False:
				print("Gives Obj")
				pubObj.publish(self.obj)
			"""
			pubObj.publish(self.obj)
		else:
			#TELEOP le point d'objectif
			settings = termios.tcgetattr(sys.stdin)
			while(1):
				key = getKey()
				if key in mvtBindings.keys():
					print(key)
					#Bouge de 10cm
					x = mvtBindings[key][0]
					y = mvtBindings[key][1]
					z = mvtBindings[key][2]
					if (key == '\x03'):
						break

				self.obj.x += x; self.obj.y += y; self.obj.z += z;
				print(self.obj)
				pubObj.publish(self.obj)
				
				time.sleep(0.5)

			#LAND if Ctrl+C

	def stop_callback(msg):
		self.stop= True
	
	
"""To get the pushed keys on the keyboard"""
def getKey():
	tty.setraw(sys.stdin.fileno())
	select.select([sys.stdin], [], [], 0)
	key = sys.stdin.read(1)
	termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
	return key

if __name__=="__main__":
	if len(sys.argv) == 4:
		objCtrl = obj_controller(float(sys.argv[1]),float(sys.argv[2]),float(sys.argv[3]))
	elif str(sys.argv[1]) == "True":
		objCtrl = obj_controller(teleop=True)
	else:
		print("Need 3 paramaters x y z")

