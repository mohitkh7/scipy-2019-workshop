#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist

rospy.init_node('wander')
rospy.loginfo("Node Initiated")

pub = rospy.Publisher('cmd_vel_mux/input/teleop', Twist, queue_size=2)
rate = rospy.Rate(1)

while not rospy.is_shutdown():
    forward = Twist()
    forward.linear.x = 0.1
    pub.publish(forward)
    rate.sleep()
