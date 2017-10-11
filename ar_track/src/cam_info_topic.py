#!/usr/bin/env python
# license removed for brevity
import rospy
from std_msgs.msg import String

def cam_info():
    pub = rospy.Publisher('/cam_info', String)
    rospy.init_node('cam_info_node', anonymous=True)
    rate = rospy.Rate(10) # 10hz
    while not rospy.is_shutdown():
        pub.publish(hello_str)
        rate.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
