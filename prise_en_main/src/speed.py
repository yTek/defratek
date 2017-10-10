#!/usr/bin/env python
import rospy
# ROS Image message
from bebop_msgs.msg import Ardrone3PilotingStateSpeedChanged

#ODOM call back
def speed_callback(msg):
    print(msg)
    
def listener():

    # In ROS, nodes are uniquely named. If two nodes with the same
    # node are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    rospy.init_node('speed_listener', anonymous=True)

    rospy.Subscriber("/bebop/states/ardrone3/PilotingState/SpeedChanged", Ardrone3PilotingStateSpeedChanged, speed_callback)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    listener()
