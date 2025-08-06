from PyFlyt.core.abstractions.camera import Camera
import math
from enum import Enum

"""
Goals: 

(1) [Completed for Cameras in X] Attach camera class to the end of camera model.

(2) [Completed] Develop rotation actions using the apply torque method from pybullet.

    (2.1) [Completed] Use the reset position function for reseting the location of the camera model.

(3) Come up with a of finding the proper locations to spawn the camera model so it appears attached to
    a building along with coming up with the proper way of rotating it.
    [Completed Location]

    (3.1) Will have to develop method such that I know whether torque should be applied along the y or x axis
          for rotating the camera horizontally. 

Cameras can spawn with (0.52*scale plus/minus origin_X, origin_y, 0.4*scale*5) with ord 
(0, (1/-1) * math.pi**0.5, 0, math.pi**0.5) or 
(origin_X, 0.52*scale plus/minus origin_y, 0.4*scale*5) with ord ((1/-1) * math.pi**0.5, 0, 0, math.pi**0.5)

Functions to Add:

    def reset()[Completed]:
            
    def rotate_Vertical()[Completed]:

    def get_Observation()[Completed]:

    def identify_Drones():
"""
cameraLink = 2

class SurveillanceCamera():
    
    def __init__(self, p, use_gimbal, camera_FOV_degrees, camera_angle_degrees, model_file, start_pos, 
                 start_orn, physics_hz, np_random, camera_resolution, force, maxHorzAngle, 
                 maxVertAngle, camera_position_offset = ..., active=True):
        self.p = p
        self.start_pos = start_pos
        self.Id = self.p.loadURDF(fileName= model_file,
                                basePosition=start_pos, 
                                useFixedBase=True, 
                                baseOrientation = start_orn,
                                globalScaling = 0.7)
        self.physics_hz = physics_hz
        self.np_random = np_random
        self.force = force
        self.active = active
        self.maxHorz = maxHorzAngle
        self.maxVert = maxVertAngle
        self.active = True
        self.camera = Camera(
            p=p, uav_id=self.Id, camera_id=0, use_gimbal=use_gimbal, 
            camera_FOV_degrees=camera_FOV_degrees, camera_angle_degrees=camera_angle_degrees,
            camera_resolution=camera_resolution, camera_position_offset=camera_position_offset
        )
    
    """
    def rotate_Horizontal(self, rotateLeft):
        angle = math.pi/180
        if (self.rightIsPos):
            if (rotateLeft):
                self.curHorz -= angle
                self.p.setJointMotorControl2(bodyUniqueId=self.Id,
                    jointIndex=direction.horizontal.value,
                    controlMode=self.p.POSITION_CONTROL,
                    targetPosition=self.curHorz,
                    force= self.force, # Example force, adjust as needed
                    maxVelocity=1.0 # Example max velocity, adjust as needed
                )
            else:
                self.curHorz += angle
                self.p.setJointMotorControl2(bodyUniqueId=self.Id,
                    jointIndex=direction.horizontal.value,
                    controlMode=self.p.POSITION_CONTROL,
                    targetPosition=self.curHorz,
                    force= self.force, # Example force, adjust as needed
                    maxVelocity=1.0 # Example max velocity, adjust as needed
                )
        else:
            if (rotateLeft):
                self.curHorz += angle
                self.p.setJointMotorControl2(bodyUniqueId=self.Id,
                    jointIndex=direction.horizontal.value,
                    controlMode=self.p.POSITION_CONTROL,
                    targetPosition=self.curHorz,
                    force= self.force, # Example force, adjust as needed
                    maxVelocity=1.0 # Example max velocity, adjust as needed
                )
            else:
                self.curHorz -= angle
                self.p.setJointMotorControl2(bodyUniqueId=self.Id,
                    jointIndex=direction.horizontal.value,
                    controlMode=self.p.POSITION_CONTROL,
                    targetPosition=self.curHorz,
                    force= self.force, # Example force, adjust as needed
                    maxVelocity=1.0 # Example max velocity, adjust as needed
                )

    def rotate_Vertical(self, rotateUp):
        angle = math.pi/180
        if (rotateUp):
            self.curVert += angle
            self.p.setJointMotorControl2(bodyUniqueId=self.Id,
                jointIndex=direction.vertical.value,
                controlMode=self.p.POSITION_CONTROL,
                targetPosition=self.curVert,
                force= self.force, # Example force, adjust as needed
                maxVelocity=1.0 # Example max velocity, adjust as needed
            )
        else:
            self.curVert += angle
            self.p.setJointMotorControl2(bodyUniqueId=self.Id,
                jointIndex=direction.vertical.value,
                controlMode=self.p.POSITION_CONTROL,
                targetPosition=self.curVert,
                force= self.force, # Example force, adjust as needed
                maxVelocity=1.0 # Example max velocity, adjust as needed
            )
    """
    
    def step(self, action):
        controlMode= self.p.POSITION_CONTROL
        self.p.setJointMotorControl2(
            bodyUniqueId=self.Id,
            jointIndex=1,
            controlMode=controlMode,
            targetPosition=action[0],
            force=40, 
            maxVelocity=5.0
        )
        self.p.setJointMotorControl2(
            bodyUniqueId=self.Id,
            jointIndex=2,
            controlMode=controlMode,
            targetPosition=action[1],
            force=40, 
            maxVelocity=5.0
        )

    def get_Observation(self, active_uavs, start_uavs):
        uavs = [False]*start_uavs
        if (len(active_uavs) > 0):
            _, _, seg = self.camera.capture_image()
            for i in range(len(seg)):
                for j in range(len(seg[i])):
                    if (seg[i][j][0] in active_uavs):
                        """if (seg[i][j][0] >= len(active_uavs)):
                            print(active_uavs, "au")
                            print(seg[i][j][0], "val")"""
                        uavs[seg[i][j][0]] = True
                        break
        pos, _, _, _, _, _ = self.p.getLinkState(self.Id, cameraLink)
        return uavs, pos
    
    def reset(self):
        self.p.resetJointState(self.Id, 1, 0)
        self.p.resetJointState(self.Id, 2, 0)
        self.active = True
