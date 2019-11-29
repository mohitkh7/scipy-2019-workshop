#!/usr/bin/env python
import rospy
from std_msgs.msg import Int32


def callback(msg):
    print msg

# initiaite ROS node
rospy.init_node('topic_subscriber')

# create subscriber object
sub = rospy.Subscriber('counter', Int32, callback)

rospy.spin()  # avoids exiting from this script until node is stopped
