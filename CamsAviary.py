from __future__ import annotations

import time
from itertools import repeat
from typing import Any, Callable, Sequence
from warnings import warn
from utils.SurveillanceCamera import SurveillanceCamera

import numpy as np
import pybullet as p
import pybullet_data
from pybullet_utils import bullet_client
from PyFlyt.core.aviary import Aviary

from PyFlyt.core.abstractions import DroneClass, WindFieldClass


DroneIndex = int

CameraLink = 2
aAxisJoint = 1
yAxisJoint = 2


"""
The camstart_pos is the added variable this should just be a list of positions where the boxes will spawn.
goal_x is the variable that indicates the x position of the current goal.
"""
class CamsAviary(Aviary):
    def __init__(self, camstart_pos, goal_x, max_HorzAng, max_VertAng, camerafile, 
        boxfile,
        start_pos: np.ndarray,
        start_orn: np.ndarray,
        drone_type: str | Sequence[str],
        drone_type_mappings: None | dict[str, type[DroneClass]] = None,
        drone_options: dict[str, Any] | Sequence[dict[str, Any]] | None = None,
        wind_type: None | str | type[WindFieldClass] = None,
        wind_options: dict[str, Any] = {},
        render: bool = False,
        physics_hz: int = 240,
        world_scale: float = 1.0,
        seed: None | int = None,
        np_random: None | np.random.Generator = None):
        super().__init__(start_pos, start_orn, drone_type, drone_type_mappings, drone_options, wind_type, wind_options,
                         render, physics_hz, world_scale, seed, np_random)
        self.physics_hz = physics_hz
        self.num_cams = len(camstart_pos)
        self.num_drones = len(start_pos)
        self.p = p
        self.camstart_pos = camstart_pos
        self.cams = []
        self.camerafile = camerafile
        self.boxfile = boxfile
        self.np_random = np_random
        self.max_HorzAng = max_HorzAng
        self.max_VertAng = max_VertAng
        for i in range(len(self.camstart_pos)):
            origin = camstart_pos[i][:]
            if (self.camstart_pos[i][0] <= goal_x):
                origin[0] += 0.52
                origin[2] += 2
                self.cams.append(SurveillanceCamera(self.p, False, 75, -90, self.camerafile, np.array(origin), 
                                         self.p.getQuaternionFromEuler([0, 1.5708, 0]), 
                                         self.physics_hz, self.np_random, (360, 360), 10, self.max_HorzAng, 
                                         self.max_VertAng, np.array([0, 0, 0.051])))
                
            else:
                origin[0] -= 0.52
                origin[2] += 2
                self.cams.append(SurveillanceCamera(self.p, False, -75, -90, self.camerafile, np.array(origin), 
                                         self.p.getQuaternionFromEuler([0, -1.5708, 0]),
                                         self.physics_hz, self.np_random, (360, 360), 10, self.max_HorzAng, 
                                         self.max_VertAng, np.array([0, 0, 0.051])))
        for csp in self.camstart_pos:
            self.p.loadURDF(self.boxfile, csp, useFixedBase=True, globalScaling = 1)    

    def camstate(self, camid):
        pos, orn, _, _ = self.p.getLinkState(camid, CameraLink)
        return pos, orn
    
    def updateCams(self, actions):
        for i in range(len(actions)):
            self.cams[i].step(actions[i])
            

    def register_all_new_bodies(self) -> None:
        """Registers all new bodies in the environment to be able to handle collisions later.

        Call this when there is an update in the number of bodies in the environment.
        """
        num_bodies = (
            np.max([self.getBodyUniqueId(i) for i in range(self.getNumBodies())]) + 1
        )
        self.contact_array = np.zeros((num_bodies, num_bodies), dtype=bool)

    def step(self) -> None:
        self.register_all_new_bodies()
        """Steps the environment, this automatically handles physics and control looprates, one step is equivalent to one control loop step."""
        # compute rtf if we're rendering
        
        if self.render:
            elapsed = time.time() - self.now
            self.now = time.time()

            self._sim_elapsed += self.step_period
            self._frame_elapsed += elapsed

            time.sleep(max(self._sim_elapsed - self._frame_elapsed, 0.0))

            # print RTF every 0.5 seconds, this actually adds considerable overhead
            if self._frame_elapsed >= 0.5:
                # calculate real time factor based on realtime/simtime
                RTF = self._sim_elapsed / (self._frame_elapsed + 1e-6)
                self._sim_elapsed = 0.0
                self._frame_elapsed = 0.0

                self.rtf_debug_line = self.addUserDebugText(
                    text=f"RTF: {RTF:.3f}",
                    textPosition=[0, 0, 0],
                    textColorRGB=[1, 0, 0],
                    replaceItemUniqueId=self.rtf_debug_line,
                )

        # reset collisions
        self.contact_array &= False

        # step the environment enough times for one control loop of the slowest controller
        for _ in range(self.updates_per_step):
            # update control and physics
            [drone.update_control(self.physics_steps) for drone in self.armed_drones]
            [drone.update_physics() for drone in self.armed_drones]

            # advance pybullet
            self.stepSimulation()

            # update states and camera
            [drone.update_state() for drone in self.armed_drones]
            [drone.update_last(self.physics_steps) for drone in self.armed_drones]
            # splice out collisions
            for collision in self.getContactPoints():
                if (collision[1] < self.num_drones or collision[2] < self.num_drones):
                    self.contact_array[collision[1], collision[2]] = True
                    self.contact_array[collision[2], collision[1]] = True

            # increment the number of physics steps
            self.physics_steps += 1
            self.elapsed_time = self.physics_steps / self.physics_hz

        self.aviary_steps += 1