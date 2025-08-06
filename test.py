import pybullet as p
import pybullet_data
import numpy as np
import time
import os
import math
from PyFlyt.core.abstractions.camera import Camera

physicsClient = p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath())
p.loadURDF('plane.urdf')

file_dir = ''
f = os.path.dirname(os.path.realpath(__file__)).split('\\')
for i in range(len(f)-2):
    file_dir += f[i]
    file_dir += '\\'
name = os.path.join(file_dir, "Documents\\RL_UAS\\Models\\camera.urdf")
name2 = os.path.join(file_dir, "Documents\\RL_UAS\\Models\\camera2.urdf")
print(name)
"""
Robot 1 Angle = -90, offset = [0,0,0.051], fov= 75 link=2
Robot 2 Angle = -90, offset = [0,0,0.051], fov= -75 link=2
Robot 3 Camera2, angle =-90, offset = [0,0,0.051], fov=-75, link=2
Robot 4 Camera2, angle =-90, offset = [0,0,0.051], fov=75, link=2
"""
robot = p.loadURDF(name, globalScaling=0.8, baseOrientation=p.getQuaternionFromEuler([0, 1.5708, 0]),
                   basePosition=np.array([2.52, 2, 2]), useFixedBase=True)
robot2 = p.loadURDF(name, globalScaling=0.8, baseOrientation= p.getQuaternionFromEuler([0, -1.5708, 0]), 
                   basePosition=np.array([1.48, 2, 2]), useFixedBase=True)
"""robot3 = p.loadURDF(name2, globalScaling=0.8, baseOrientation = p.getQuaternionFromEuler([1.5708, 0, 0]),
                   basePosition=np.array([2, 1.48, 2]), useFixedBase=True)
robot4 = p.loadURDF(name2, globalScaling=0.8, baseOrientation=p.getQuaternionFromEuler([-1.5708, 0, 0]), 
                   basePosition=np.array([2, 2.52, 2]), useFixedBase=True)"""

cam = Camera(p=p, uav_id=robot, camera_id=2, use_gimbal=False,
            camera_FOV_degrees=75, camera_angle_degrees=-90,
            camera_resolution=(360, 360), camera_position_offset=np.array([0, 0, 0.051]))

camera_state = p.getLinkState(robot, 2)
print(camera_state[0])
print(camera_state[1])
#print(camera_state[2:])
#print(p.getLinkState(robot3, 2))



p.loadURDF(os.path.join(file_dir, "Documents\\RL_UAS\\Models\\box.urdf"), 
                                basePosition=np.array([2, 2, 0]), 
                                useFixedBase=True,
                                globalScaling = 1)
o2 = p.loadURDF(os.path.join(file_dir, "Documents\\RL_UAS\\Models\\box.urdf"), 
                                basePosition=np.array([5, 2, 0]), 
                                useFixedBase=True,
                                globalScaling = 1)
p.changeVisualShape(o2, -1, rgbaColor=[0, 1, 0, 1]) 
o3 = p.loadURDF(os.path.join(file_dir, "Documents\\RL_UAS\\Models\\box.urdf"), 
                                basePosition=np.array([2, 5, 0]), 
                                useFixedBase=True,
                                globalScaling = 1)
p.changeVisualShape(o3, -1, rgbaColor=[1, 0, 0, 1]) 
o4 = p.loadURDF(os.path.join(file_dir, "Documents\\RL_UAS\\Models\\box.urdf"), 
                                basePosition=np.array([-1, 2, 0]), 
                                useFixedBase=True,
                                globalScaling = 1)
p.changeVisualShape(o4, -1, rgbaColor=[0, 0.8, 1, 1]) 
o5 = p.loadURDF(os.path.join(file_dir, "Documents\\RL_UAS\\Models\\box.urdf"), 
                                basePosition=np.array([2, -1, 0]), 
                                useFixedBase=True,
                                globalScaling = 1)
boxes = [o2, o3, o4, o5]
p.changeVisualShape(o5, -1, rgbaColor=[0.8, 0.8, 0, 1]) 


htargetAngle = 20 * (math.pi/180) #[0, 0, -45 * (math.pi/180)]
vtargetAngle = 5 * (math.pi/180)
#print(targetAngle)
#targetAngle = p.getQuaternionFromEuler(targetAngle)
#print(targetAngle)


controlMode = p.POSITION_CONTROL
#p.resetJointStateMultiDof(robot, revoluteJointIndex, targetAngle)
time.sleep(1)  

#print(p.getLinkState(robot, 2))

"""p.setJointMotorControl2(
    bodyUniqueId=robot,
    jointIndex=1,
    controlMode=controlMode,
    targetPosition= targetAngle,
    force= 30, 
    maxVelocity=1.0 
)

p.setJointMotorControl2(
    bodyUniqueId=robot,
    jointIndex=2,
    controlMode=controlMode,
    targetPosition= targetAngle,
    force= 30, 
    maxVelocity=1.0
)"""

"""p.setJointMotorControl2(
    bodyUniqueId=robot,
    jointIndex=1,
    controlMode=controlMode,
    targetPosition= htargetAngle,
    force= -30, 
    maxVelocity=1.0 
)"""

"""p.setJointMotorControl2(
    bodyUniqueId=robot,
    jointIndex=2,
    controlMode=controlMode,
    targetPosition= -vtargetAngle,
    force= 30, 
    maxVelocity=1.0
)"""

p.setJointMotorControl2(
    bodyUniqueId=robot,
    jointIndex=1,
    controlMode=controlMode,
    targetPosition= htargetAngle*2,
    force= 30, 
    maxVelocity=1.0 
)
p.setJointMotorControl2(
    bodyUniqueId=robot2,
    jointIndex=1,
    controlMode=controlMode,
    targetPosition= htargetAngle*2,
    force= 30, 
    maxVelocity=1.0 
)
"""
Positive Angle is right for robot
Positive Angle is left for robot2
"""
"""p.setJointMotorControl2(
    bodyUniqueId=robot3,
    jointIndex=2,
    controlMode=controlMode,
    targetPosition= -vtargetAngle,
    force= 30, 
    maxVelocity=1.0 
)"""


#print(p.getLinkState(robot, 1))
#print(p.getLinkState(robot3, 1))

for _ in range(360):
    p.stepSimulation()
    _, _, x = cam.capture_image()
    """for i in range(len(x)):
        f = False
        for j in range(len(x[i])):
            if (x[i][j] in boxes):
                print("Found")
                f = True
                break
        if (f):
            break"""
    time.sleep(0.25)

# Disconnect from the physics server
p.disconnect()