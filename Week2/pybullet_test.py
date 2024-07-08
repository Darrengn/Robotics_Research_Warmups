import pybullet as p
import numpy as np
import matplotlib.pyplot as plt
import time
import pybullet_data
physicsClient = p.connect(p.GUI)#or p.DIRECT for non-graphical version
p.setAdditionalSearchPath(pybullet_data.getDataPath()) #optionally
p.setGravity(0,0,-10)
planeId = p.loadURDF("plane.urdf")
startPos = [0,0,1]
startOrientation = p.getQuaternionFromEuler([0,0,0])
boxId = p.loadURDF("r2d2.urdf",startPos, startOrientation)

# shift = [0, -0.02, 0]
# meshScale = [0.1, 0.1, 0.1]

# visualShapeId = p.createVisualShape(shapeType=p.GEOM_CYLINDER,
#                                     visualFramePosition=shift,
#                                     meshScale=meshScale)
# collisionShapeId = p.createCollisionShape(shapeType=p.GEOM_CYLINDER,
#                                           collisionFramePosition=shift,
#                                           meshScale=meshScale)

# p.createMultiBody(baseMass=1,
#                   baseInertialFramePosition=[0, 0, 0],
#                   baseCollisionShapeIndex=collisionShapeId,
#                   baseVisualShapeIndex=visualShapeId,
#                   basePosition=[0,0, 1],
#                   useMaximalCoordinates=True)

print(p.getNumJoints(boxId))

vmat = p.computeViewMatrixFromYawPitchRoll(cameraTargetPosition=[0,0,0], distance=2, yaw=0, pitch=0, roll=0, upAxisIndex=2)
pmat = p.computeProjectionMatrixFOV(fov=60, aspect=(128 / 128), nearVal=0.01, farVal=5) 

#set the center of mass frame (loadURDF sets base link frame) startPos/Ornp.resetBasePositionAndOrientation(boxId, startPos, startOrientation)
for i in range (10000):
    p.stepSimulation()
    p.getCameraImage(128,128,vmat,pmat)[2]
    time.sleep(1./240.)



# cubePos, cubeOrn = p.getBasePositionAndOrientation(boxId)
# print(cubePos,cubeOrn)
p.disconnect()
