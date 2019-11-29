#!/usr/bin/env python
import rospy
from std_msgs.msg import Int32

# initiating ros node
rospy.init_node('publisher')

# creating publisher object
pub = rospy.Publisher('counter', Int32, queue_size=1)
rate = rospy.Rate(1)  # defines frequency of publishing in Hz

count = 0
while not rospy.is_shutdown():
    pub.publish(count)  # publish data over topic
    count += 1
    rate.sleep()
