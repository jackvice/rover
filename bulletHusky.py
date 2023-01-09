import pybullet as p
import pybullet_data
import time

p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath())

p.loadURDF("plane.urdf")
p.setGravity(0, 0, -10)
huskypos = [0, 0, 0.1]

husky = p.loadURDF("husky/husky.urdf", huskypos[0], huskypos[1], huskypos[2])
#camera = p.loadURDF("realsense/urdf/d435Test.urdf.xacro", huskypos[0], huskypos[1], huskypos[2])
#time.sleep(10)
#exit()

numJoints = p.getNumJoints(husky)
for joint in range(numJoints):
  print(p.getJointInfo(husky, joint))
targetVel = 10  #rad/s
maxForce = 100  #Newton


viewMatrix = p.computeViewMatrix(
    cameraEyePosition=[0, 0, 3],
    cameraTargetPosition=[0, 0, 0],
    cameraUpVector=[0, 1, 0])


projectionMatrix = p.computeProjectionMatrixFOV(
    fov=45.0,
    aspect=1.0,
    nearVal=0.1,
    farVal=3.1)



width, height, rgbImg, depthImg, segImg = p.getCameraImage(
    width=224, 
    height=224,
    viewMatrix=viewMatrix,
    projectionMatrix=projectionMatrix)
#for joint in range(2, 6):
#  p.setJointMotorControl(husky, joint, p.VELOCITY_CONTROL, targetVel, maxForce)
#for step in range(300):
#  p.stepSimulation()

targetVel_1 = -0.05 #-10
targetVel_2 = 0.05 #-10
for joint in (2, 4):
    p.setJointMotorControl(husky, joint, p.VELOCITY_CONTROL, targetVel_1, maxForce)
for joint in (3, 5):
    p.setJointMotorControl(husky, joint, p.VELOCITY_CONTROL, targetVel_2, maxForce)
for step in range(400000):
    p.stepSimulation()
    if (step % 100 == 0):
        width, height, rgbImg, depthImg, segImg = p.getCameraImage(
            width=224, 
            height=224,
            viewMatrix=viewMatrix,
            projectionMatrix=projectionMatrix)

time.sleep(2)
p.getContactPoints(husky)

p.disconnect()

