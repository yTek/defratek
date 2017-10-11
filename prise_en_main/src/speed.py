#!/usr/bin/env python
import rospy

import time 

# ROS Image message
from bebop_msgs.msg import Ardrone3PilotingStateSpeedChanged
from bebop_msgs.msg import Ardrone3PilotingStateAltitudeChanged

last_time=time.time()
my_pos=(0.0,0.0,0.0)

#ODOM call back
def speed_callback(msg):
	
	new_time=time.time()-last_time

	delta_time=new_time-last_time

    print("Speed: ",msg)

	speedX=msg.speedX
	deltaX=msg.speedX*delta_time

	speedY=msg.speedY
	deltaY=msg.speedY*delta_time

	speedZ=msg.speedZ
	deltaZ=msg.speedZ*delta_time

	update_pos(deltaX,deltaY,deltaZ)
	
def update_pos(deltaX,deltaY,deltaZ):
	my_pos[0]=my_pos[0]+deltaX
	my_pos[1]=my_pos[1]+deltaY
	my_pos[2]=my_pos[2]+deltaZ
	print("Speed: ")



#ODOM call back
def altitude_callback(msg):
	print("Altitude: ")
    print(msg)

def listener():

    # In ROS, nodes are uniquely named. If two nodes with the same
    # node are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    rospy.init_node('speed_listener', anonymous=True)

    rospy.Subscriber("/bebop/states/ardrone3/PilotingState/SpeedChanged", Ardrone3PilotingStateSpeedChanged, speed_callback)
	
	rospy.Subscriber("/bebop/states/ardrone3/PilotingState/AltitudeChanged", Ardrone3PilotingStateAltitudeChanged, altitude_callback)
	
	
    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    listener()
