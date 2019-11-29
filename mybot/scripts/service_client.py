#!/usr/bin/env python
import rospy
from mybot.srv import WordCount

rospy.init_node('service_client')

rospy.wait_for_service('word_count')
word_counter = rospy.ServiceProxy('word_count', WordCount)

inp = "Welcome to scipy"
print "Input is : %s" % inp

count = word_counter(inp)
print count