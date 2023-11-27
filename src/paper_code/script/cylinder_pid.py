#!/usr/bin/env python3
# -*- coding: utf-8 -*-

from cmath import pi
import rospy
import numpy as np
import time
from robot_msgs.msg import Cylinder_States
from robot_msgs.msg import Cylinder_Cmd
from robot_msgs.msg import Joint_Cmd
import tf
import threading
import message_filters
import json

# realtime states and reference
cyl_sta = np.array([0,0,0],dtype=np.float32)
cyl_ref = 150

PID_params = {'kp':0.2, 'ki':0.05, 'kd':0, 'je_limit':10, 'u_limit':[-20,20], 'tolerance':1}
# sub cylinder_states
def msg_callback(msg):
    global cyl_sta
    cyl_sta[0] = msg.Y0

def thread_spin():
    rospy.Subscriber('/cylinder_states', Cylinder_States, msg_callback)
    rospy.spin()


def cyl_pid_controller( error ):
    # pid controller params 
    global PID_params
    kp = PID_params['kp']
    ki = PID_params['ki']
    kd = PID_params['kd']
    je_limit = PID_params['je_limit']
    u_limit = PID_params['u_limit']
    tolerance = PID_params['tolerance']

    # saturation of sum error
    if (error['je']>je_limit):
        error['je'] = je_limit

    u = kp*error['e'] + ki*error['je'] + kd*error['de']

    if (u>u_limit[1]):
        u = u_limit[1]
    if (u<u_limit[0]):
        u = u_limit[0]
    
    if ( abs(error['e'])<=tolerance ):
        u = 0

    return u

# main
def cylinder_control():
    rospy.init_node('cylinder_nmpc_node', anonymous=True)
    cylinder_cmd_pub = rospy.Publisher('/cylinder_cmd', Cylinder_Cmd, queue_size=10)
    cmd_pub = rospy.Publisher('/joint_cmd', Joint_Cmd, queue_size=10)

    rate = rospy.Rate(20)

    # 启动接收线程
    t1 = threading.Thread(target=thread_spin) 
    t1.start()
    
    # 实时控制
    error = {'e':0, 'je':0, 'de':0, 'e0':0}
    while not rospy.is_shutdown():
        # get ref from ros_param
        # PID controller
        global cyl_sta, cyl_ref
        cyl_ref = rospy.get_param('cyl_ref')

        error['e'] = cyl_ref - cyl_sta[0]
        error['de'] = error['e'] - error['e0']
        error['je'] = error['je'] + error['e']
        error['e0'] = error['e']

        u = cyl_pid_controller(error)

        # cmd = Cylinder_Cmd()
        # cmd.mode = 1
        # cmd.current1 = u
        # cmd.header.stamp = rospy.Time.now()
        cmd = Joint_Cmd()
        cmd.enable = True
        for _ in range(5):
            cmd.current.append(-u)
        cmd.header.stamp = rospy.Time.now()

        cmd_pub.publish(cmd)

        # realtime data print
        print('Y0, Ref, cmd', cyl_sta[0], cyl_ref, cmd.current[2])

        rate.sleep()

if __name__ == '__main__':
    try:
        cylinder_control()
    except rospy.ROSInterruptException:
        pass