import pybullet as p
import numpy as np
import matplotlib.pyplot as plt
import time
import pybullet_data

resolution = 128

def useKeyboard():
    inputs = p.getKeyboardEvents()
    if ord('q') in inputs:
        followLine(1,1,3)
    if ord('r') in inputs:
        moveToPose(3,3,np.pi/2)
    if p.B3G_RETURN in inputs:
        moveToPoint(3,3)
    if p.B3G_DELETE in inputs:
        getCenterDepth()
    if p.B3G_LEFT_ARROW in inputs:
        p.setJointMotorControlArray(robot, [2,3,6,7], p.VELOCITY_CONTROL, targetVelocities=[-20,-20,20,20],forces = [3]*4)
    elif p.B3G_RIGHT_ARROW in inputs:
        p.setJointMotorControlArray(robot, [2,3,6,7], p.VELOCITY_CONTROL, targetVelocities=[20,20,-20,-20],forces = [3]*4)
    elif p.B3G_UP_ARROW in inputs:
        p.setJointMotorControlArray(robot, [2,3,6,7], p.VELOCITY_CONTROL, targetVelocities=[-100,-100,-100,-100],forces = [3]*4)
    elif p.B3G_DOWN_ARROW in inputs:
        p.setJointMotorControlArray(robot, [2,3,6,7], p.VELOCITY_CONTROL, targetVelocities=[20,20,20,20],forces = [3]*4)
    else:
        p.setJointMotorControlArray(robot, [2,3,6,7], p.VELOCITY_CONTROL, targetVelocities=[0,0,0,0],forces = [3]*4)


pmat = p.computeProjectionMatrixFOV(fov=120, aspect=(128 / 128), nearVal=0.01, farVal=20)

# vmat = p.computeViewMatrixFromYawPitchRoll(cameraTargetPosition=[0,0,1], distance=2, yaw=0, pitch=0, roll=0, upAxisIndex=2)

def cameraDisp():

    # code for attaching camera to robot

    com_pos, com_orn, _, _, _, _ = p.getLinkState(robot, 13, computeForwardKinematics=True)
    rot_matrix = p.getMatrixFromQuaternion(com_orn)
    rot_matrix = np.array(rot_matrix).reshape(3, 3)
    
    init_camera_vector = (0, 1, 0) # z-axis
    init_up_vector = (0, 0, 1) # y-axis
    # init_up_vector = rot_matrix.dot(init_camera_vector)
    # Rotated vectors
    camera_vector = rot_matrix.dot(init_camera_vector)

    up_vector = rot_matrix.dot(init_up_vector)
    vmat = p.computeViewMatrix(com_pos + np.array([0,0,0.3]), com_pos + camera_vector, up_vector)
    img = p.getCameraImage(resolution,resolution,vmat,pmat)
    return img

def getCenterDepth():
    _,_,_,dep_map,_ = cameraDisp()
    print(dep_map[int((resolution**2)/2)]) 

def calcAngleErr(tar, theta):
    err_ang = tar - theta 
    if abs(err_ang) > np.pi:
        err_ang = abs(err_ang) - 2*np.pi
        if tar < 0:
            err_ang = -err_ang
    return err_ang

def moveToPoint(xtar, ytar):
    erraccum_lin = 0
    erraccum_ang = 0
    kp_l = 100 #50
    ki_l = 0
    kd_l = 0.0
    kp_a = 2000
    ki_a = 0.0
    kd_a = 0

    curpos, curorn = p.getBasePositionAndOrientation(robot)
    rot_matrix = p.getMatrixFromQuaternion(curorn)
    rot_matrix = np.array(rot_matrix).reshape(3, 3)
    y_head = rot_matrix[:,1]
    theta = np.arccos(y_head.dot([1,0,0]))

    if y_head[1] < 0:
        theta = -theta
    x,y,z = curpos

    theta_tar = np.arctan2((ytar-y),(xtar-x))

    err_lin = ((ytar - y)**2 + (xtar - x)**2)**0.5
    err_ang = calcAngleErr(theta_tar,theta)

    erraccum_lin += err_lin
    erraccum_ang += err_ang
    perr_lin = err_lin  #previous err
    perr_ang = err_ang
    derr_lin = 0    #Linear err delta
    derr_ang = 0
    
    while abs(err_lin) > 0.05 or abs(derr_lin) > 0.001:
        # print("Error:",err_ang, " Acum Err:",erraccum_ang, " D Err:", derr_ang)
        
        v = -kp_l*err_lin - ki_l*erraccum_lin - kd_l*derr_lin
        w = -kp_a*err_ang - ki_a*erraccum_ang - kd_a*derr_ang
        # print("Left Vel:", v-0.22*w, "  Right Vel:", v+0.22*w)
        # print("tar vel: ", v, " tar angvel", w)
        p.setJointMotorControlArray(robot, [2,3,6,7], p.VELOCITY_CONTROL, targetVelocities=[v+0.22*w,v+0.22*w,v-0.22*w,v-0.22*w],forces = [3]*4)
        p.stepSimulation()
        time.sleep(1./240.)

        curpos, curorn = p.getBasePositionAndOrientation(robot)
        rot_matrix = p.getMatrixFromQuaternion(curorn)
        rot_matrix = np.array(rot_matrix).reshape(3, 3)
        y_head = rot_matrix[:,1]
        if v > 0:
            y_head[1] = -y_head[1]
        theta = np.arccos(y_head.dot([1,0,0]))

        if y_head[1] < 0:
            theta = -theta
        x,y,z = curpos

        theta_tar = np.arctan2((ytar-y),(xtar-x))
        # print(theta_tar)
        err_lin = ((ytar - y)**2 + (xtar - x)**2)**0.5
        err_ang = calcAngleErr(theta_tar,theta)

        erraccum_lin += err_lin
        erraccum_ang += err_ang
        derr_lin = err_lin - perr_lin
        derr_ang = err_ang - perr_ang
        perr_lin = err_lin
        perr_ang = err_ang

        cameraDisp()

    p.setJointMotorControlArray(robot, [2,3,6,7], p.VELOCITY_CONTROL, targetVelocities=4*[0],forces = [5]*4)
    print("DONE, Curpos:", curpos)

def moveToPose(xtar, ytar, end_ang):
    kp = 25
    ka = 0.9
    kb = -0.1      #-0.05
    kp_theta = 1000
    
    curpos, curorn = p.getBasePositionAndOrientation(robot)
    rot_matrix = p.getMatrixFromQuaternion(curorn)
    rot_matrix = np.array(rot_matrix).reshape(3, 3)
    y_head = rot_matrix[:,1]
    theta = np.arccos(y_head.dot([1,0,0]))

    if y_head[1] < 0:
        theta = -theta
    x,y,z = curpos

    ang_to_goal = np.arctan2((ytar-y),(xtar-x))

    err = ((ytar - y)**2 + (xtar - x)**2)**0.5
    alpha = calcAngleErr(ang_to_goal,theta)
    beta = calcAngleErr(-theta, alpha)
    beta = calcAngleErr(beta, -end_ang)

    # beta = end_ang
    k_vec = np.array([ka/err, kb*err])
    k_vec = k_vec/np.linalg.norm(k_vec)


    theta_tar = k_vec[0]*alpha + k_vec[1]*beta
    # theta_tar = calcAngleErr(theta_tar, -np.pi/2)
    # err_ang = calcAngleErr(theta_tar,theta)
    err_ang = calcAngleErr(end_ang,theta)
    
    while abs(err) > 0.1 or abs(err_ang) > 0.03:
        # print("Error:",err_ang, " Acum Err:",erraccum_ang, " D Err:", derr_ang)
        
        v = -kp*err
        w = -kp_theta*theta_tar*abs(theta_tar)
        # print("Left Vel:", v-0.22*w, "  Right Vel:", v+0.22*w)
        # print("tar vel: ", v, " tar angvel", w)
        p.setJointMotorControlArray(robot, [2,3,6,7], p.VELOCITY_CONTROL, targetVelocities=[v+0.22*w,v+0.22*w,v-0.22*w,v-0.22*w],forces = [3]*4)
        p.stepSimulation()
        time.sleep(1./240.)

        curpos, curorn = p.getBasePositionAndOrientation(robot)
        rot_matrix = p.getMatrixFromQuaternion(curorn)
        rot_matrix = np.array(rot_matrix).reshape(3, 3)
        y_head = rot_matrix[:,1]
        theta = np.arccos(y_head.dot([1,0,0]))

        if y_head[1] < 0:
            theta = -theta
        x,y,z = curpos

        ang_to_goal = np.arctan2((ytar-y),(xtar-x))

        err = ((ytar - y)**2 + (xtar - x)**2)**0.5
        alpha = calcAngleErr(ang_to_goal,theta)
        beta = calcAngleErr(-theta, alpha)
        beta = calcAngleErr(beta, -end_ang)

        k_vec = np.array([ka/err, kb*err])
        k_vec = k_vec/np.linalg.norm(k_vec)

        if abs(err) < 0.05:
            k_vec = [0, 1]
        theta_tar = k_vec[0]*alpha + k_vec[1]*beta
        # theta_tar = calcAngleErr(theta_tar, -np.pi/2)

        err_ang = calcAngleErr(end_ang,theta)
        
        print("theta tar:",np.rad2deg(theta_tar), "  alpha:", np.rad2deg(alpha), "  beta", np.rad2deg(beta))
        
        cameraDisp()

    p.setJointMotorControlArray(robot, [2,3,6,7], p.VELOCITY_CONTROL, targetVelocities=4*[0],forces = [5]*4)
    print("DONE, Curpos:", curpos, "  Cur Angle:", np.rad2deg(theta))

# where the line is ax + by + c = 0
def followLine(a, b, c):
    kd = 500   #distance const
    kh = 1000   #heading const
    while(True):
        inputs = p.getKeyboardEvents()
        cameraDisp()
        if ord('e') in inputs:
            break
        curpos, curorn = p.getBasePositionAndOrientation(robot)
        rot_matrix = p.getMatrixFromQuaternion(curorn)
        rot_matrix = np.array(rot_matrix).reshape(3, 3)
        y_head = rot_matrix[:,1]
        theta = np.arccos(y_head.dot([1,0,0]))
        theta_tar = np.arctan2(-a,b)

        if y_head[1] < 0:
            theta = -theta
        x,y,z = curpos
        line = np.array([a,b,c])
        curpos = np.array([x,y,1])
        lin_dist = line.dot(curpos)/(a**2 + b**2)**0.5
        ang_err = calcAngleErr(theta_tar,theta)
        
        gamma = kd*lin_dist - kh * ang_err
        # w = np.tan(gamma)
        w = gamma
        print(ang_err)
        
        v = -60
        p.setJointMotorControlArray(robot, [2,3,6,7], p.VELOCITY_CONTROL, targetVelocities=[v+0.22*w,v+0.22*w,v-0.22*w,v-0.22*w],forces = [3]*4)
        p.stepSimulation()
        time.sleep(1./240.)

physicsClient = p.connect(p.GUI)#or p.DIRECT for non-graphical version
p.setAdditionalSearchPath(pybullet_data.getDataPath()) #optionally
p.setGravity(0,0,-9.8)
planeId = p.loadURDF("plane.urdf")
startPos = [0,0,1]
startOrientation = p.getQuaternionFromEuler([0,0,0])
robot = p.loadURDF("r2d2.urdf",startPos, startOrientation)

# Cylindar code
shift = [-5, -5, 0]
meshScale = [0.1, 0.1, 0.1]
visualShapeId = p.createVisualShape(shapeType=p.GEOM_CYLINDER,
                                    visualFramePosition=shift,
                                    meshScale=meshScale)
collisionShapeId = p.createCollisionShape(shapeType=p.GEOM_CYLINDER,
                                          collisionFramePosition=shift,
                                          meshScale=meshScale)
p.createMultiBody(baseMass=1,
                  baseInertialFramePosition=shift,
                  baseCollisionShapeIndex=collisionShapeId,
                  baseVisualShapeIndex=visualShapeId,
                  basePosition=[0,0, 1],
                  useMaximalCoordinates=True)

num_joints = p.getNumJoints(robot)
pos, orn = p.getBasePositionAndOrientation(robot)
print("Postition:",pos, "Orientation",orn)

for i in range(num_joints):
    print("Joint ",i, p.getJointInfo(robot, i))
    print()
# p.setJointMotorControlArray(robot, [13], p.VELOCITY_CONTROL, targetVelocities=[5],forces = [5])

# for i in [2,3,6,7]:
#     pos, _, _, _, _, _ = p.getLinkState(robot, i, computeForwardKinematics=True)
#     print("Position of link:", i, " ", pos)

p.addUserDebugLine([7,-10,0],[-10,7,0],[1,0,0],lineWidth = 5)
# set the center of mass frame (loadURDF sets base link frame) startPos/Ornp.resetBasePositionAndOrientation(boxId, startPos, startOrientation)
time.sleep(0.5)
for i in range (10000):
    p.stepSimulation()
    useKeyboard()
    cameraDisp()
    time.sleep(1./240.)

p.disconnect()
