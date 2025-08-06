"""QuadX Waypoints Environment With Obstacles"""

from __future__ import annotations

from typing import Any, Literal
import math

import os
import numpy as np

from PyFlyt.gym_envs.quadx_envs.quadx_waypoints_env import QuadXWaypointsEnv

import time
import pybullet as p
#from utils.Obstacle import Obstacle


class QuadXWaypointsObsEnv(QuadXWaypointsEnv):
    def __init__(
        self,
        obs: list[Obstacle],
        sparse_reward: bool = False,
        num_targets: int = 4,
        use_yaw_targets: bool = False,
        goal_reach_distance: float = 0.2,
        goal_reach_angle: float = 0.1,
        flight_mode: int = 0,
        flight_dome_size: float = 5.0,
        max_duration_seconds: float = 10.0,
        angle_representation: Literal["euler", "quaternion"] = "quaternion",
        agent_hz: int = 30,
        render_mode: None | Literal["human", "rgb_array"] = None,
        render_resolution: tuple[int, int] = (480, 480)
    ):
        """__init__.

        Args:
            sparse_reward (bool): whether to use sparse rewards or not.
            num_targets (int): number of waypoints in the environment.
            use_yaw_targets (bool): whether to match yaw targets before a waypoint is considered reached.
            goal_reach_distance (float): distance to the waypoints for it to be considered reached.
            goal_reach_angle (float): angle in radians to the waypoints for it to be considered reached, only in effect if `use_yaw_targets` is used.
            flight_mode (int): the flight mode of the UAV.
            flight_dome_size (float): size of the allowable flying area.
            max_duration_seconds (float): maximum simulation time of the environment.
            angle_representation (Literal["euler", "quaternion"]): can be "euler" or "quaternion".
            agent_hz (int): looprate of the agent to environment interaction.
            render_mode (None | Literal["human", "rgb_array"]): render_mode
            render_resolution (tuple[int, int]): render_resolution.
            obs ([(string, np.array(int, int, int), float, np.array(int, int, int, int))]): A list that contains tuples 
                of the file name of the obstacle you want to load, its position, its scale, and its color.
        """
        super().__init__(sparse_reward=sparse_reward, num_targets=num_targets, use_yaw_targets=use_yaw_targets,
                         goal_reach_distance=goal_reach_distance, goal_reach_angle=goal_reach_angle,
                         flight_mode=flight_mode, flight_dome_size=flight_dome_size, 
                         max_duration_seconds=max_duration_seconds, angle_representation=angle_representation,
                         agent_hz=agent_hz, render_mode=render_mode, render_resolution=render_resolution)
        self.obs = obs
        self.file_dir = ''
        f = os.path.dirname(os.path.realpath(__file__)).split('\\')
        for i in range(len(f)-2):
            self.file_dir += f[i]
            self.file_dir += '\\'
        


    def reset(self, *, seed: None | int = None, options: None | dict[str, Any] = dict()) -> tuple[
        dict[Literal["attitude", "rgba_cam", "target_deltas"], np.ndarray], dict]:

        """Resets the environment.

        Args:
            seed: seed to pass to the base environment.
            options: None

        """
        r = super().reset(seed=seed, options=options)
        #self.generateObstacles()
        self.env.register_all_new_bodies()
        return r
    
    def generateObstacles(self):
        for o in self.obs:
            ob = None
            if (o.ori != None):
                ob = self.env.loadURDF(os.path.join(self.file_dir, o.relative_name), 
                                basePosition=o.pos, 
                                useFixedBase=True, 
                                baseOrientation = o.ori,
                                globalScaling = o.scale)
                self.env.setJointMotorControl2(
                    bodyUniqueId=ob,
                    jointIndex=1,
                    controlMode=self.env.POSITION_CONTROL,
                    targetPosition=2.0,
                    force=5.0, # Example force, adjust as needed
                    maxVelocity=1.0 # Example max velocity, adjust as needed
                )
            else:
                ob = self.env.loadURDF(os.path.join(self.file_dir, o.relative_name), 
                                basePosition=o.pos, 
                                useFixedBase=True,
                                globalScaling = o.scale)
            for i in range(p.getNumJoints(ob)):
                p.changeVisualShape(ob,
                    linkIndex=i,
                    rgbaColor=o.color)
                
if __name__ == "__main__":
    
    """box = Obstacle("RL_UAS\\Models\\box.urdf", np.array([2, 2, 0]), 1, (0, 0, 0.8, 1))
    box2 = Obstacle("RL_UAS\\Models\\box.urdf", np.array([4, 2, 0]), 1, (0, 0, 0.8, 1))
    box3 = Obstacle("RL_UAS\\Models\\box.urdf", np.array([2, 4, 0]), 1, (0, 0, 0.8, 1))
    box4 = Obstacle("RL_UAS\\Models\\box.urdf", np.array([4, 4, 0]), 1, (0, 0, 0.8, 1))
    cam = Obstacle("RL_UAS\\Models\\camera.urdf", np.array([1.5, 1, 2]), 0.8, (0, 0, 0, 1), 
                   (0, math.pi**0.5, 0, math.pi**0.5))
    obst = [box, box2, box3, box4, cam]"""
    
    
    env = QuadXWaypointsObsEnv(num_targets=1, render_mode='human', obs=[])
    
    term, trunc = False, False
    obsr, _ = env.reset()
    
    #while True:
    #    env.render()
    while not (term or trunc):
        obsr, rew, term, trunc, _ = env.step(env.action_space.sample())
        time.sleep(2)