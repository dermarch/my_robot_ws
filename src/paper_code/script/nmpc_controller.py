#!/usr/bin/env python3
# -*- coding: utf-8 -*-

from cmath import pi
import rospy
import numpy as np
import time
from geometry_msgs import Pose2D
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
from geometry_msgs.msg import PoseStamped
import tf

import casadi as ca
import casadi.tools as ca_tools

import threading
import message_filters

current_states = np.array([0,0,0],dtype=np.float32)
goal_states = np.array([0,0,0],dtype=np.float32)

def callback(msg1,msg2):
    global current_states
    global goal_states

    (roll, pitch, yaw) = tf.transformations.euler_from_quaternion([msg1.orientation.x, msg1.orientation.y, msg1.orientation.z, msg1.orientation.w])
    current_states = np.array([msg1.pose.pose.position.x, msg1.pose.pose.position.y, yaw])

    goal_states = np.array([msg2.x, msg2.y, msg2.theta])

    print("received pose and odom")

#  接收线程，同步接收 odom and pose
def thread_spin():
    t1= message_filters.Subscriber("odom_gnss", Odometry)
    t2 = message_filters.Subscriber("ideal_pose", Pose2D)
    ts = message_filters.ApproximateTimeSynchronizer([t1, t2], 10, 1, allow_headerless=True)
    ts.registerCallback(callback)
    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

def velocity_publisher():
    rospy.init_node('nmpc_controller_node', anonymous=True)
    # 速度指令发布
    cmd_pub = rospy.Publisher('/des_vel', encoder_vel_msg, queue_size=10)

    rate = rospy.Rate(20)                   # 与采样时间 T 对应

    # 启动接收线程
    t1 = threading.Thread(target=thread_spin) 
    t1.start()

    # 移动机器人参数设置
    width = 0.8      # 车宽
    T = 0.05        # sampling time [s]
    N = 20          # prediction horizon
    v_max = 2
    omega_max = np.pi/6.0

    # nmpc设置
    x = ca.SX.sym('x')
    y = ca.SX.sym('y')
    theta = ca.SX.sym('theta')
    states = ca.vertcat(x, y, theta)
    n_states = states.size()[0]

    v = ca.SX.sym('v')
    omega = ca.SX.sym('omega')
    controls = ca.vertcat(v, omega)
    n_controls = controls.size()[0]

    ## rhs
    rhs = ca.vertcat(v*ca.cos(theta), v*ca.sin(theta))
    rhs = ca.vertcat(rhs, omega)

    ## function
    f = ca.Function('f', [states, controls], [rhs], ['input_state', 'control_input'], ['rhs'])

    ## for MPC
    U = ca.SX.sym('U', n_controls, N)
    X = ca.SX.sym('X', n_states, N+1)
    P = ca.SX.sym('P', n_states+n_states)

    ### define
    X[:, 0] = P[:3]         # initial condiction

    #### define the prediction equation within the horizon
    for i in range(N):
        f_value = f(X[:, i], U[:, i])
        X[:, i+1] = X[:, i] + f_value*T

    ff = ca.Function('ff', [U, P], [X], ['input_U', 'target_state'], ['horizon_states'])

    #### compute cost function
    Q = np.array([[1.0, 0.0, 0.0],[0.0, 1.0, 0.0],[0.0, 0.0, 0.01]])
    R = np.array([[0.05, 0.0], [0.0, 0.05]])
    obj = 0 #### cost
    for i in range(N):
        # obj = obj + ca.mtimes([(X[:, i]-P[3:]).T, Q, X[:, i]-P[3:]]) + ca.mtimes([U[:, i].T, R, U[:, i]])
        # new type to calculate the matrix multiplication
        obj = obj + (X[:, i]-P[3:]).T @ Q @ (X[:, i]-P[3:]) + U[:, i].T @ R @ U[:, i]

    #### get constrains
    # states constrains     lbg< gx <ubg
    g = []
    lbg = []
    ubg = []
    for i in range(N+1):
        g.append(X[0, i])
        g.append(X[1, i])
        lbg.append(-1000)
        lbg.append(-1000)
        ubg.append(1000)
        ubg.append(1000)
    # controls constrains       lbx< x <ubx
    lbx = []
    ubx = []
    for _ in range(N):
        lbx.append(-v_max)
        ubx.append(v_max)
        lbx.append(-omega_max)
        ubx.append(omega_max)
    
    # 构建求解器
    nlp_prob = {'f': obj, 'x': ca.reshape(U, -1, 1), 'p':P, 'g':ca.vcat(g)} # here also can use ca.vcat(g) or ca.vertcat(*g)
    opts_setting = {'ipopt.max_iter':100, 'ipopt.print_level':0, 'print_time':0, 'ipopt.acceptable_tol':1e-8, 'ipopt.acceptable_obj_change_tol':1e-6, }

    solver = ca.nlpsol('solver', 'ipopt', nlp_prob, opts_setting)


    # 初始化优化问题
    t0 = 0.0
    x0 = np.array([0.0, 0.0, 0.0]).reshape(-1, 1)# initial state
    xs = np.array([5, 0, 0.0]).reshape(-1, 1) # final state
    u0 = np.array([0.0, 0.0]*N).reshape(-1, 2)# np.ones((N, 2)) # controls

    c_p = np.concatenate((x0, xs))
    init_control = ca.reshape(u0, -1, 1)
    res = solver(x0=init_control, p=c_p, lbg=lbg, lbx=lbx, ubg=ubg, ubx=ubx)
    lam_x_ = res['lam_x']

    # 实时控制
    while not rospy.is_shutdown():
        # 求解nmpc
        ## set parameter
        ## 处理下航向角超范围的问题 [-pi,pi]
        if (goal_states[2]-current_states[2])<pi:
            goal_states[2] = goal_states[2] + 2*pi
        if (goal_states[2]-current_states[2])>pi:
            goal_states[2] = goal_states[2] - 2*pi
        
        x0 = current_states.reshape(-1, 1)  # initial states
        xs = goal_states.reshape(-1,1)      # ideal states

        c_p = np.concatenate((x0, xs))
        init_control = ca.reshape(u0, -1, 1)

        res = solver(x0=init_control, p=c_p, lbg=lbg, lbx=lbx, ubg=ubg, ubx=ubx, lam_x0=lam_x_)

        u_sol = ca.reshape(res['x'], n_controls, N) # one can only have this shape of the output
        uk = u_sol[:, 0]
        vel = uk[0]
        w = uk[1]

        u0 = ca.horzcat(u_sol[:, 1:], u_sol[:, -1])
        u0 = u0.T
        lam_x_ = res['lam_x']

        # 初始化类消息
        cmd_msg = encoder_vel_msg()
        cmd_msg.vleft = vel - 0.5*(w*width)
        cmd_msg.vright = vel + 0.5*(w*width)

        # 发布消息
        cmd_pub.publish(cmd_msg)

        rospy.loginfo("states (%0.3f,%0.3f,%0.3f)" %(current_states[0],current_states[1],current_states[2]))
        rospy.loginfo("goal (%0.3f,%0.3f,%0.3f)" %(goal_states[0],goal_states[1],goal_states[2]))
        rospy.loginfo("cmd (%0.3f,%0.3f)" %(vel,w))

        # 按照循环频率延时
        rate.sleep()

if __name__ == '__main__':
    try:
        velocity_publisher()
    except rospy.ROSInterruptException:
        pass
