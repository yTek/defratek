#!/usr/bin/env python
import roslib
import rospy

from geometry_msgs.msg import Twist,Pose, Point
from std_msgs.msg import Empty

class bebop:

	def __init__(self, name='bebop'):
		#PARAMETERS
		self.absoluteCmd = False
		self.name = name

		#Initialisation
		#Vector ???
		pubPos = rospy.Publisher(name+'_Pos', Point, queue_size = 1)
		rospy.init_node('obj_controller', anonymous= True, disable_signals=True)
		sub_odom = rospy.Subscriber('/'+self.name+"/odom", Odometry, odometry_callback)
		sub_absoluteCmd = rospy.Subscriber('/bebop_absoluteCmd', Empty, absoluteCmd_callback)
		
		while absoluteCmd is False:
			print("Gives Obj")
			pubObj.publish(obj)
	

	def absoluteCmd_callback(msg):
		self.absoluteCmd= True




if __name__=="__main__":
	if len(sys.argv) == 1:
		bp = bebop()
	else:
		name_arg=str(sys.argv[1])
		bp = bebop(name = name_arg)


