import logging
from random import randint

from flask import Flask, render_template
from flask_ask import Ask, request, session, question, statement

import rospy
from std_msgs.msg import String

import IPython

alexa_valid_phrases_topic = "AlexaValidPhrases"
alexa_detected_phrases_topic = "AlexaDetectedPhrases"
valid_phrases = ["next grasp"]

app = Flask(__name__)
ask = Ask(app, "/")
logging.getLogger('flask_ask').setLevel(logging.DEBUG)

@ask.launch
def launch():
    help_text = render_template('help_text')
    return statement(help_text)

def assign_valid_phrases(phrases_str_msg):
    phrases_str = phrases_str_msg.data
    valid_phrases = phrases_str.lower().split(",")


@ask.intent('SendCommandIntent', mapping={'command':'Command'})
def send_instruction(command):
    print(command)
    if command in valid_phrases:
        publisher.publish(command)
        success_msg = render_template('success_msg', command=command)
        return statement(success_msg)
    else:
        invalid_command_msg = render_template('invalid_command_msg', command=command)
        return statement(invalid_command_msg)

@ask.intent('AMAZON.HelpIntent')
def help():
    help_text = render_template('help_text')
    return question(help_text).reprompt(help_text)


@ask.intent('AMAZON.CancelIntent')
def cancel():
    bye_text = render_template('bye')
    return statement(bye_text)


@ask.session_ended
def session_ended():
    return "", 200


if __name__ == '__main__':
    rospy.init_node('graspit_alexa_controller')
    publisher = rospy.Publisher(alexa_detected_phrases_topic, String, queue_size=10)
    subscriber = rospy.Subscriber(alexa_valid_phrases_topic, String, assign_valid_phrases)
    app.run(debug=True, host='localhost', port=5000)