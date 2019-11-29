#!/usr/bin/env python
import rospy
from mybot.srv import WordCount, WordCountResponse


def callback(request):
    return WordCountResponse(len(request.words.split(' ')))

rospy.init_node('service_server')

serv = rospy.Service('word_count', WordCount, callback)
rospy.spin()