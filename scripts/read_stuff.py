#!/usr/bin/env python

import rospy
from std_msgs.msg import String

alexa_valid_phrases_topic = "AlexaValidPhrases"
alexa_detected_phrases_topic = "AlexaDetectedPhrases"

def assign_valid_phrases(phrases_str_msg):
    phrases_str = phrases_str_msg.data
    valid_phrases = phrases_str.lower()
    print(valid_phrases)
    open('phrases.txt', 'w').write(valid_phrases)

rospy.init_node('graspit_alexa_controller')

publisher = rospy.Publisher(alexa_detected_phrases_topic, String, queue_size=10)
subscriber = rospy.Subscriber(alexa_valid_phrases_topic, String, assign_valid_phrases)

rate = rospy.Rate(10)

while not rospy.is_shutdown():
    command = open('command.txt','r').read()
    if command != "":
        print(command)
        open('command.txt', 'w').write("")
        publisher.publish(command.title())
        rate.sleep()


