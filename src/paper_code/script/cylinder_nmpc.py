#!/usr/bin/env python3
# -*- coding: utf-8 -*-

from cmath import pi
import rospy
import numpy as np
import time
from robot_msgs.msg import Cylinder_States
from robot_msgs.msg import Cylinder_Cmd
import tf
import casadi as ca
import casadi.tools as ca_tools
import threading
import message_filters
import json

# realtime states and reference
cyl_sta = np.array([0,0,0],dtype=np.float32)
cyl_ref = 0

# sub cylinder_states
def msg_callback(msg):
    global cyl_sta
    cyl_sta[0] = msg.Y0

def thread_spin():
    rospy.Subscriber('/cylinder_states', Cylinder_States, msg_callback)
    rospy.spin()

# load params
def load_params():
    global cyl_model_params
    global cyl_nmpc_params
    with open('./src/paper_code/config/cyl_model_params.json', 'r', encoding='utf-8') as f:
        cyl_model_params = json.load(f)
    
    with open('./src/paper_code/config/cyl_nmpc_params.json', 'r', encoding='utf-8') as f:
        cyl_nmpc_params = json.load(f)        

# cylinder model
def cyl_model( params, xk, uk ):
    m = params["m"]
    A = params["A"]
    Cd = params["Cd"]
    w = params["w"]
    b = params["b"]
    betae = params["betae"]
    Ctm = params["Ctm"]
    rou = params["rou"]
    Kv = params["Kv"]
    Fc = params["Fc"]
    Fv = params["Fv"]
    ts = params["ts"]

    Ps = 1e7
    Vt=0.38*A/2

    a1 = 0
    a2 = (b+Fv)/m
    a3 = A/m
    a4 = Fc/m
    a5 = (4*A*betae)/Vt
    a6 = (4*Ctm*betae)/Vt
    a7 = (4*Cd*betae*w*Kv)/Vt/np.sqrt(rou)

    dx = np.array([[0],[0],[0]], dtype=np.float32)

    dx[0] = xk[1]
    dx[1] = -a1*xk[0]-a2*xk[1]+a3*xk[2]-a4*np.sign(xk[1])
    dx[2] = -a5*xk[1]-a6*xk[2]+a7*np.sqrt(abs(Ps-xk[2]*np.sign(uk)))*uk

    return dx

def cyl_mpc_build( cyl_model_params, cyl_nmpc_params):
    # params setting
    ts = cyl_nmpc_params["ts"]
    nx = cyl_nmpc_params["nx"]
    nu = cyl_nmpc_params["nu"]
    umax = cyl_nmpc_params["umax"]
    umin = cyl_nmpc_params["umin"]
    xmin = cyl_nmpc_params["xmin"]
    xmax = cyl_nmpc_params["xmax"]
    Q = cyl_nmpc_params["Q"]
    R = cyl_nmpc_params["R"]
    N = cyl_nmpc_params["N"]

    # casadi variables
    x1 = ca.SX.sym('x1')
    x2 = ca.SX.sym('x2')
    x3 = ca.SX.sym('x3')
    states = ca.vertcat(x1, x2, x3)

    u = ca.SX.sym('u')
    controls = u

    rhs = cyl_model( cyl_model_params, states, controls)

    # function
    f = ca.Function('f', [states, controls], [rhs], ['input_state', 'control_input'], ['rhs'])

    # for MPC
    U = ca.SX.sym('U', nu, N)
    X = ca.SX.sym('X', nx, N+1)
    P = ca.SX.sym('P', 2*nx)

    # compute prediction value symbolically
    X[:,0] = P[:3]
    for i in range(N):
        f_value = f(X[:, i], U[:, i])
        X[:, i+1] = X[:, i] + f_value*ts

    # compute cost function
    obj = 0 
    for i in range(N):
        obj = obj + (X[0, i]-P[3]).T @ Q @ (X[0, i]-P[3]) + U[i].T @ R @ U[i]

    # states constrains        lbg< gx <ubg
    g = []
    lbg = []
    ubg = []
    for i in range(N+1):
        g.append(X[0, i])
        g.append(X[1, i])
        g.append(X[2, i])
        lbg.append(xmin[0])
        lbg.append(xmin[1])
        lbg.append(xmin[2])
        ubg.append(xmax[0])
        ubg.append(xmax[1])
        ubg.append(xmax[2])

    # controls constrains       lbx< x <ubx
    lbx = []
    ubx = []
    for _ in range(N):
        lbx.append(umax)
        ubx.append(umin)
    
    # nlp params
    args = dict{'lbg',lbg, 'ubg',ubg, 'lbx',lbx, 'ubx', ubx}

    # 构建求解器
    nlp_prob = {'f': obj, 'x': ca.reshape(U, -1, 1), 'p':P, 'g':ca.vcat(g)} # here also can use ca.vcat(g) or ca.vertcat(*g)
    opts_setting = {'ipopt.max_iter':100, 'ipopt.print_level':0, 'print_time':0, 'ipopt.acceptable_tol':1e-8, 'ipopt.acceptable_obj_change_tol':1e-6, }

    solver = ca.nlpsol('solver', 'ipopt', nlp_prob, opts_setting)

    return solver, args

# main
def cylinder_control():
    rospy.init_node('cylinder_nmpc_node', anonymous=True)
    cylinder_cmd_pub = rospy.Publisher('/cylinder_cmd', Cylinder_Cmd, queue_size=10)
    rate = rospy.Rate(20)

    # 启动接收线程
    t1 = threading.Thread(target=thread_spin) 
    t1.start()

    # nmpc controller
    solver, args = cyl_mpc_build( cyl_model_params, cyl_nmpc_params )

    # simulation setting
    Nsim = 2000
    u0 = np.zeros(cyl_nmpc_params["N"], 1)
    cyl_x = np.zeros(3, Nsim)
    
    # 实时控制
    # while not rospy.is_shutdown():
    for k in range(Nsim):
        # 求解nmpc
        global cyl_sta, cyl_ref
        xk = cyl_sta.reshape(-1, 1)          # initial states
        ref = cyl_ref                        # ideal states

        c_p = np.concatenate((x0, ref))
        init_control = ca.reshape(u0, -1, 1)

        res = solver(x0=init_control, p=c_p, lbg=args["lbg"], lbx=args["lbx"], ubg=args["ubg"], ubx=args["ubx"])

        u_sol = ca.reshape(res['x'], cyl_nmpc_params["nu"], cyl_nmpc_params["N"]) # one can only have this shape of the output
        uk = u_sol[0]
        u0 = u_sol

        # model simulation
        dx = cyl_model( cyl_model_params, xk, uk)
        cyl_sta = xk + dx*cyl_model_params["ts"]

        cyl_x[:,k] = 

if __name__ == '__main__':
    try:
        load_params()
        print(cyl_model_params, type(cyl_model_params))
        print(cyl_nmpc_params, type(cyl_nmpc_params))

        cylinder_control()
    except rospy.ROSInterruptException:
        pass