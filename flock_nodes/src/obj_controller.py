#!/usr/bin/env python
import roslib
import rospy

from geometry_msgs.msg import Twist,Pose, Point
from std_msgs.msg import Empty

class obj_controller:

	def __init__(self):
		#PARAMETERS
		self.stop = False
		
		#Initialisation
		#Vector ???
		pubObj = rospy.Publisher('global_dir', Point, queue_size = 1)
		rospy.init_node('obj_controller', anonymous= True, disable_signals=True)
		sub_stop = rospy.Subscriber('/bebop_stop', Empty, stop_callback)
		
		while stop is False:
			print("Gives Obj")
			pubObj.publish(obj)
	

	def stop_callback(msg):
		self.stop= True




if __name__=="__main__":
	obj_ctrl = obj_controller()

