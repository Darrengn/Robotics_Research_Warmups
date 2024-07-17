# ---------------------------------
# Car: Code to get simulator running. (Ackermann Drive)
# ---------------------------------

import pybullet as p
import time
import numpy as np
import matplotlib.pyplot as plt
import math
import sys
import pybullet_data

physicsClient = p.connect(p.GUI)#or p.DIRECT for non-graphical version
p.setAdditionalSearchPath(pybullet_data.getDataPath()) #optionally
p.setGravity(0,0,-9.8)
planeId = p.loadURDF("plane.urdf")
startPos = [0,0,1]
startOrientation = p.getQuaternionFromEuler([0,0,0])
robot = p.loadURDF("racecar/racecar.urdf",startPos, startOrientation)

num_joints = p.getNumJoints(robot)
pos, orn = p.getBasePositionAndOrientation(robot)

for i in range(num_joints):
    print("Joint ",i, p.getJointInfo(robot, i))
    print()

def act(v, gamma):
    sign = v / abs(v)
    if abs(v) > 50:
        v = 50*sign
    p.setJointMotorControlArray(robot, [2,3,5,7], p.VELOCITY_CONTROL, targetVelocities=[v] * 4,forces = [1]*4)
    sign = -gamma/abs(gamma)
    if abs(gamma) > 1:
        gamma = sign * 1
    p.setJointMotorControlArray(robot, [4,6], p.POSITION_CONTROL, targetPositions=[gamma] * 2,forces = [1]*2)

def get_state():
    cur_pos, cur_orn= p.getBasePositionAndOrientation(robot)
    rot_matrix = p.getMatrixFromQuaternion(cur_orn)
    rot_matrix = np.array(rot_matrix).reshape(3, 3)
    y_head = rot_matrix[:,1]
    theta = np.arccos(y_head.dot([1,0,0]))

    if y_head[1] < 0:
        theta = -theta
    x,y,z = cur_pos
    return x, y, theta


def useKeyboard():
    inputs = p.getKeyboardEvents()
    if p.B3G_RETURN in inputs:
        moveToPoint(-3,-3)
    if ord('q') in inputs:
        followLine(1,1,3)
    if ord('r') in inputs:
        moveToPose(-3,-3,np.pi)

    

def calcAngleErr(tar, theta):
    err_ang = tar - theta 
    if abs(err_ang) > np.pi:
        err_ang = abs(err_ang) - 2*np.pi
        if tar < 0:
            err_ang = -err_ang
    return err_ang

def moveToPoint(xtar, ytar):
    kp_lin = 5
    ki_lin = 0.01
    err_accum = 0
    kp_ang = -0.2
    dir = 1

    x, y, theta = get_state()

    theta_tar = np.arctan2((ytar-y),(xtar-x))

    err_lin = ((ytar - y)**2 + (xtar - x)**2)**0.5
    err_ang = calcAngleErr(theta_tar,theta)
    #code to drive backwards instead
    if abs(err_ang) > np.pi/2:
        dir = -1
        err_ang = calcAngleErr(err_ang, np.pi)
    else:
        dir = 1

    err_accum += err_lin
    while abs(err_lin) > 0.05:
        act(dir*(kp_lin*err_lin+ ki_lin*err_accum) , kp_ang*err_ang *dir)
        print(np.rad2deg(kp_ang*err_ang *dir))
        p.stepSimulation()
        time.sleep(1./240.)
        
        x, y, theta = get_state()
        # print(x,y)
        theta_tar = np.arctan2((ytar-y),(xtar-x))

        err_lin = ((ytar - y)**2 + (xtar - x)**2)**0.5
        err_ang = calcAngleErr(theta_tar,theta)
        if abs(err_ang) > np.pi/2:
            dir = -1
            err_ang = calcAngleErr(err_ang, np.pi)
        else:
            dir = 1

        err_accum += err_lin
    act(0,kp_ang*err_ang)
    print("DONE, Curpos:", get_state())


def followLine(a, b, c):
    kd = 0.5   #distance const
    kh = 0.5   #heading const
    while(True):
        inputs = p.getKeyboardEvents()
        if ord('e') in inputs:
            break
        x, y, theta = get_state()

        line = np.array([a,b,c])
        curpos = np.array([x,y,1])
        theta_tar = np.arctan2(-a,b)
        lin_dist = line.dot(curpos)/(a**2 + b**2)**0.5
        ang_err = calcAngleErr(theta_tar,theta)

        gamma = kd*lin_dist - kh*ang_err
        act(40, gamma)

        p.stepSimulation()
        time.sleep(1./240.)
    act(0,0)

def moveToPose(xtar,ytar,end_ang):
    kp = 2
    ka = -8
    kb = 11
    ki_lin = 0.01
    err_accum = 0
    dir = -1
    
    x, y, theta = get_state()
    err_lin = ((ytar - y)**2 + (xtar - x)**2)**0.5
    ang_to_goal = np.arctan2((ytar-y),(xtar-x))
    alpha = calcAngleErr(ang_to_goal,theta)
    
    if abs(alpha) > np.pi/2:
        dir = -1
        alpha = calcAngleErr(alpha, np.pi)
    else:
        dir = 1

    beta = calcAngleErr(-theta, alpha)
    beta = calcAngleErr(beta, -end_ang)


    err_accum += err_lin
    while err_lin > 0.1:
        gamma = (ka*alpha + kb*beta)
        act((kp*err_lin + ki_lin*err_accum) * dir, gamma * dir)
        print((ka*alpha + kb*beta) * dir)
        x, y, theta = get_state()
        err_lin = ((ytar - y)**2 + (xtar - x)**2)**0.5
        ang_to_goal = np.arctan2((ytar-y),(xtar-x))
        alpha = calcAngleErr(ang_to_goal,theta)
        
        err_accum += err_lin

        if abs(alpha) > np.pi/2:
            dir = -1
            alpha = calcAngleErr(alpha, np.pi)
        else:
            dir = 1

        print(np.rad2deg(theta))
        beta = calcAngleErr(-theta, alpha)
        beta = calcAngleErr(beta, -end_ang)
        p.stepSimulation()
        time.sleep(1./240.)
    
    act(0,0)
    x,y,theta = get_state()
    print("DONE, Curpos:", x, y, np.rad2deg(theta))

    


p.addUserDebugPoints([[-3,-3,0]], [[1,0,0]],pointSize=10)
p.addUserDebugLine([7,-10,0],[-10,7,0],[1,0,0],lineWidth = 5)

for i in range (10000):
    p.stepSimulation()     
    useKeyboard()
    
    time.sleep(1./240.)
    
p.disconnect()
