#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan

obstacle_distance = 5

def callback(msg):
    global obstacle_distance
    obstacle_distance = min(msg.ranges)
    # rospy.loginfo("min range : %f" % obstacle_distance)

def subscriber():
    sub = rospy.Subscriber('scan', LaserScan, callback)

def publisher():
    pub = rospy.Publisher('cmd_vel_mux/input/teleop', Twist, queue_size=2)
    rate = rospy.Rate(1)
    moving_forward = True

    while not rospy.is_shutdown():
        global obstacle_distance
        moving_forward = True if obstacle_distance > 0.5 else False
        twist = Twist()
        if moving_forward:
            twist.linear.x = 0.2
        else:
            rospy.loginfo("turning robot")
            twist.angular.z = 0.2
        pub.publish(twist)
        rate.sleep()


if __name__ == "__main__":
    rospy.init_node('scipybot')
    rospy.loginfo("Node initiated")
    subscriber()
    publisher()


