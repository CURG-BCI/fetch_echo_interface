#!/usr/bin/env python
import logging
from random import randint

from flask import Flask, render_template
from flask_ask import Ask, request, session, question, statement
from flask_sslify import SSLify

import difflib

app = Flask(__name__)
sslify = SSLify(app)
ask = Ask(app, "/")
logging.getLogger('flask_ask').setLevel(logging.DEBUG)

tasks = {}

def levenshteinDistance(s1, s2):
    if len(s1) > len(s2):
        s1, s2 = s2, s1

    distances = range(len(s1) + 1)
    for i2, c2 in enumerate(s2):
        distances_ = [i2+1]
        for i1, c1 in enumerate(s1):
            if c1 == c2:
                distances_.append(distances[i1])
            else:
                distances_.append(1 + min((distances[i1], distances[i1 + 1], distances_[-1])))
        distances = distances_
    return distances[-1]

@ask.launch
def launch():
    help_text = render_template('help_text')
    return statement(help_text)

@ask.intent('SendCommandIntent', mapping={'command':'Command'})
def send_instruction(command):
    print(command)
    valid_phrases = open('phrases.txt', 'r').read().split(',')

    scores = map(lambda w: levenshteinDistance(w, command), valid_phrases)
    close_match = difflib.get_close_matches(command, valid_phrases)
    if len(close_match) > 0:
        command = close_match[0]
    #print(close_match, command, valid_phrases, command[::5] in valid_phrases)
    if command in valid_phrases:
        #publisher.publish(command)
        success_msg = render_template('success_msg', command=command)
	open('command.txt', 'w').write(command)
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

@ask.intent('RecordCommandIntent', mapping={'command_id':'CommandId', 'task_list':'TaskList'})
def record(command_id, task_list):
    tasks[command_id] = task_list
    return statement(render_template('recorded_msg', command_id=command_id))

@ask.intent('ReturnCommandIntent', mapping={'command_id':'CommandId'})
def return_command(command_id):
    if command_id in tasks:
        return statement(render_template('task_list_msg', task_list=tasks[command_id]))
    else:
        return statement(render_template('command_not_found', command_id=command_id))

@ask.session_ended
def session_ended():
    return "", 200

@app.route('/')
def hello_world():
    return 'Hello, World!'

if __name__ == '__main__':
    app.run(debug=True, host="localhost", port=5000)
