#!/usr/bin/env python
import rospy
import random
from std_msgs.msg import Int32, Bool


rospy.init_node('pub_sub')

pub = rospy.Publisher('led', Bool, queue_size=1)
rate = rospy.Rate(1)

def callback(msg):
    """
    turns LED ON if topic data is multiple of 5 or 7 else turn LED Off
    :param msg: <Int32> ROS topic data 
    :return: None
    """
    if msg.data % 5 == 0 or msg.data % 7 == 0:
        # turn LED On
        pub.publish(True)
    else:
        # turn LED Off
        pub.publish(False)

def subscriber():
    sub = rospy.Subscriber('counter', Int32, callback)
    rospy.spin()
    

if __name__ == "__main__":
    subscriber()