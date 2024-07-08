import pybullet as p
import numpy as np
import matplotlib.pyplot as plt
import time
import pybullet_data

resolution = 128
def useKeyboard():
    inputs = p.getKeyboardEvents()
    if p.B3G_RETURN in inputs:
        controlPI(0,5)
    if p.B3G_DELETE in inputs:
        getCenterDepth()
    if p.B3G_LEFT_ARROW in inputs:
        p.setJointMotorControlArray(robot, [2,3,6,7], p.VELOCITY_CONTROL, targetVelocities=[-20,-20,20,20],forces = [5]*4)
    elif p.B3G_RIGHT_ARROW in inputs:
        p.setJointMotorControlArray(robot, [2,3,6,7], p.VELOCITY_CONTROL, targetVelocities=[20,20,-20,-20],forces = [5]*4)
    elif p.B3G_UP_ARROW in inputs:
        p.setJointMotorControlArray(robot, [2,3,6,7], p.VELOCITY_CONTROL, targetVelocities=[-100,-100,-100,-100],forces = [5]*4)
    elif p.B3G_DOWN_ARROW in inputs:
        p.setJointMotorControlArray(robot, [2,3,6,7], p.VELOCITY_CONTROL, targetVelocities=[20,20,20,20],forces = [5]*4)
    else:
        p.setJointMotorControlArray(robot, [2,3,6,7], p.VELOCITY_CONTROL, targetVelocities=[0,0,0,0],forces = [5]*4)


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
    print(dep_map) #[int((resolution**2)/2)]


def controlPI(xtar, ytar):
    erraccum = 0
    kp = 100
    ki = 0.001
    kd = 0.0
    curpos, _,_, _, _, _  = p.getLinkState(robot,13, computeForwardKinematics=True)
    x,y,z = curpos
    err = ytar - y
    erraccum += err
    perr = err  #previous err
    derr = 0    #err delta
    print(err)
    while abs(err) > 0.05 or abs(derr) > 0.001:
        print("Error:",err, " Acum Err:",erraccum, " D Err:", derr)
        p.setJointMotorControlArray(robot, [2,3,6,7], p.VELOCITY_CONTROL, targetVelocities=4*[-kp*err - ki*erraccum - kd*derr],forces = [3]*4)
        p.stepSimulation()
        time.sleep(1./240.)
        curpos, _,_, _, _, _  = p.getLinkState(robot,13, computeForwardKinematics=True)
        x,y,z = curpos
        err = ytar - y
        erraccum += err
        derr = err - perr
        perr = err
        cameraDisp()

    p.setJointMotorControlArray(robot, [2,3,6,7], p.VELOCITY_CONTROL, targetVelocities=4*[0],forces = [5]*4)

    


physicsClient = p.connect(p.GUI)#or p.DIRECT for non-graphical version
p.setAdditionalSearchPath(pybullet_data.getDataPath()) #optionally
p.setGravity(0,0,-9.8)
planeId = p.loadURDF("plane.urdf")
startPos = [0,0,1]
startOrientation = p.getQuaternionFromEuler([0,0,0])
robot = p.loadURDF("r2d2.urdf",startPos, startOrientation)

# Cylindar code
shift = [0, 5, 0]
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

# set the center of mass frame (loadURDF sets base link frame) startPos/Ornp.resetBasePositionAndOrientation(boxId, startPos, startOrientation)
time.sleep(0.5)
for i in range (10000):
    p.stepSimulation()
    useKeyboard()
    cameraDisp()
    time.sleep(1./240.)




# 
p.disconnect()


