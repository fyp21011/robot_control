#! /usr/bin/env python3
# from __future__ import print_function

import random
import rospy
import actionlib
from robot_control.msg import ControlAction, ControlGoal

def send(goal):
    client.send_goal(goal)
    client.wait_for_result()
    print(client.get_result())

if __name__ == '__main__':
    rospy.init_node('robot_control_client')
    client = actionlib.SimpleActionClient('robot_control', ControlAction)
    client.wait_for_server(timeout=rospy.Duration(100))
    help_msg = '''
    'x': exit client
    'h': show cmd options
    'c': calibrate
    'o': align TCP to origin
    's [x] [y] [t]': move TCP with speed (x,y) and time t
    'r [n]': randomly move TCP for n times
    '''
    print(help_msg)
        
    while True:
        goal = ControlGoal()
        goal.velocity = [0., 0., 0., 0., 0., 0.]
        goal.time = 0.
        
        inp = input('>>> ')
        if inp == 'x':  
            break
        elif inp == 'h':
            print(help_msg)
        elif inp == 'c':
            goal.type = 'c'
            send(goal)
        elif inp == 'o':
            goal.type = 'o'
            send(goal)
        elif inp.startswith('s'):
            type, x, y, t = inp.strip().split(' ')
            goal.type = 's'
            goal.velocity = [float(x), float(y), 0., 0., 0., 0.]
            goal.time = float(t)
            send(goal)
        elif inp.startswith('r'):
            type, cnt = inp.strip().split(' ')
            goal.type = 's'
            for i in range(int(cnt)):
                goal.velocity =  [random.uniform(-0.3, 0.3), random.uniform(-0.3, 0.3), 0., 0., 0., 0.]
                goal.time = random.random()
                send(goal)
            
        # def cb(feedback):
        #     print(feedback)
        
        