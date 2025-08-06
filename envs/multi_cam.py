from utils.SingleWaypoint import SingleWaypoint
from utils.CamsAviary import CamsAviary
from typing import Any, Sequence
from PyFlyt.pz_envs.quadx_envs.ma_quadx_base_env import MAQuadXBaseEnv
from gymnasium import Space, spaces
import pybullet as p
import numpy as np
import os
import math
import time

cameraLink = 2

"""
Come up with a way for eliminating drones if they have been spotted by cameras or if they collide with objects.
Come up with a way to elimate cameras if drones collide with them.


"""
class SurveillanceEnv(MAQuadXBaseEnv):

    def __init__(self, use_yaw_targets, max_horzAngle, max_vertAngle, goal_reach_distance, goal_reach_angle, 
                 max_height, x, y, box_origins, seed, 
                 start_pos = np.array([[-1.0, -1.0, 1.0], [1.0, -1.0, 1.0], [-1.0, 1.0, 1.0], [1.0, 1.0, 1.0]]), 
                 start_orn = np.array([[0.0, 0.0, 0.0], [0.0, 0.0, 0.0], [0.0, 0.0, 0.0], [0.0, 0.0, 0.0]]), 
                 flight_mode = 0, flight_dome_size = 10, max_duration_seconds = 10, angle_representation = "euler", 
                 agent_hz = 40, render_mode = None, boxfile="RL_UAS\\Models\\box.urdf", 
                 camerafile="RL_UAS\\Models\\camera.urdf", targetfile="RL_UAS\\Models\\waypoint.urdf"):
        super().__init__(start_pos, start_orn, flight_mode, flight_dome_size, max_duration_seconds, 
                         angle_representation, agent_hz, render_mode)
        self.seed = seed
        if (render_mode != None):
            self.render_mode = render_mode
        self.active_uavs = len(start_pos) 
        self.use_yaw_targets = use_yaw_targets
        self.goal_reach_distance = goal_reach_distance
        self.goal_reach_angle = goal_reach_angle
        self.max_height = max_height
        rand = np.random.default_rng(self.seed)
        self.max_horzAngle = max_horzAngle*(math.pi/180)
        self.max_vertAngle = max_vertAngle*(math.pi/180)
        self.x = x
        self.y = y
        self.rand = rand
        self.num_uavs = len(start_pos)
        #Contains a list of tuples holding (camera model id, camera object, bool represnting if camera is active)
        self.box_origins = box_origins
        self.num_cams = len(self.box_origins)
        file_dir = ''
        f = os.path.dirname(os.path.realpath(__file__)).split('\\')
        for i in range(len(f)-2):
            file_dir += f[i]
            file_dir += '\\'
        for i in range(len(self.box_origins)):
            self.possible_agents.append("cam_"+str(i))
            self.agent_name_mapping.update({self.possible_agents[i+self.num_uavs]:i+self.num_uavs})
        self.boxfile = file_dir + boxfile
        self.camerafile = file_dir + camerafile
        self.targetfile = file_dir + targetfile
        self.camera_actions = spaces.Box(low=np.array([-max_horzAngle, -max_vertAngle]), 
                                         high=np.array([max_horzAngle, max_vertAngle]), shape=(2,), dtype=np.float64)
        self.current_camactions = np.zeros((self.num_cams, *self.action_space("cam").shape,))
        self.past_camactions = np.zeros((self.num_cams, *self.action_space("cam").shape,))
    
    def step(self, actions):
        # copy over the past actions
        self.past_actions = self.current_actions.copy()
        self.past_camactions = self.current_camactions.copy()

        # set the new actions and send to aviary
        self.current_actions *= 0.0
        self.current_camactions *= 0.0
        for k, v in actions.items():
            #print(k, v, "ACTION")
            if ("uav" in k):
                self.current_actions[self.agent_name_mapping[k]] = v
            else:
                self.current_camactions[self.agent_name_mapping[k]-self.num_uavs] = v
        self.aviary.set_all_setpoints(self.current_actions)
        #print(self.current_camactions)
        self.aviary.updateCams(self.current_camactions)
        # observation and rewards dictionary
        observations = dict()
        terminations = {k: False for k in self.agents}
        truncations = {k: False for k in self.agents}
        rewards = {k: 0.0 for k in self.agents}
        infos = {k: dict() for k in self.agents}

        # step enough times for one RL step
        for _ in range(self.env_step_ratio):
            self.aviary.step()
            self.update_states()

            # update reward, term, trunc, for each agent
            # TODO: make it so this doesn't have to be computed every aviary step

        for ag in self.agents:
            ag_id = self.agent_name_mapping[ag]

            # compute term trunc reward
            term, trunc, rew, info = self.compute_term_trunc_reward_info_by_id(
                ag_id
            )
            terminations[ag] |= term
            truncations[ag] |= trunc
            rewards[ag] += rew
            infos[ag].update(info)

            # compute observations
            observations[ag] = self.compute_observation_by_id(ag_id)

        # increment step count and cull dead agents for the next round
        ags = self.agents
        for agent in ags:
            if (terminations.get(agent) == True):
                self.agents.remove(agent)
                terminations.pop(agent)
                truncations.pop(agent)
                infos.pop(agent)
                observations.pop(agent)
                rewards.pop(agent)
                actions.pop(agent)
                #print(agent, "Removed")
                #print(self.agent_name_mapping[agent], "ag_id")
                if (self.agent_name_mapping[agent] < self.num_uavs):
                    self.active_uavs -= 1
                    #print(self.active_uavs, "active_uavs less")
                    if (self.active_uavs == 0):
                        #print(agent, "TRUNC")
                        truncations.update({agent:True})
                        break
        self.step_count += 1
        """self.agents = [
            agent
            for agent in self.agents
            if not (terminations[agent] or truncations[agent])
        ]"""

        return observations, rewards, terminations, truncations, infos

    def reset(self, seed: None | int = None, options: None | dict[str, Any] = dict(),
        drone_options: None | dict[str, Any] | Sequence[dict[str, Any]] = dict()):
        if hasattr(self, "aviary"):
            self.aviary.disconnect()
        self.step_count = 0
        self.agents = self.possible_agents[:]

        self.aviary = CamsAviary(
            camerafile = self.camerafile,
            boxfile = self.boxfile,
            max_HorzAng = self.max_horzAngle,
            max_VertAng= self.max_vertAngle,
            camstart_pos = self.box_origins,
            goal_x = self.x,
            start_pos=self.start_pos,
            start_orn=self.start_orn,
            drone_type="quadx",
            render=self.render_mode == "human",
            drone_options=drone_options,
            np_random=self.rand,
        )
        self.waypoint = SingleWaypoint(
            enable_render=self.render_mode is not None,
            use_yaw_targets=self.use_yaw_targets,
            goal_reach_distance=self.goal_reach_distance,
            goal_reach_angle=self.goal_reach_angle,
            flight_dome_size=self.flight_dome_size,
            max_height = self.max_height, 
            min_height=0.1,
            x=self.x,
            y=self.y,
            filename = self.targetfile,
            np_random=self.rand
        )
        self.waypoint.reset(self.aviary.p, self.rand)
        if self.render_mode == "human":
            self.camera_parameters = self.aviary.getDebugVisualizerCamera()
        
        observations = {
            ag: self.compute_observation_by_id(self.agent_name_mapping[ag])
            for ag in self.agents
        }
        self.current_actions = np.zeros((self.num_cams, *self.action_space(None).shape,))
        self.past_actions = np.zeros((self.num_cams, *self.action_space(None).shape,))
        self.current_camactions = np.zeros((self.num_cams, *self.action_space("cam").shape,))
        self.past_camactions = np.zeros((self.num_cams, *self.action_space("cam").shape,))
        self.active_uavs = len(self.start_pos) 
        infos = {ag: dict() for ag in self.agents}
        return observations, infos

    def compute_observation_by_id(self, agent_id: int):
        if (agent_id < self.num_uavs):
            ang_vel, ang_pos, lin_vel, lin_pos, quaternion = super().compute_attitude_by_id(agent_id)
            dist = self.waypoint.distance_to_targets(ang_pos, lin_pos, quaternion)
            aux_state = self.aviary.aux_state(agent_id)
            if self.angle_representation == 0:
                a = np.concatenate(
                    [
                        ang_vel,
                        ang_pos,
                        lin_vel,
                        lin_pos,
                        aux_state,
                        self.past_actions[agent_id],
                        self.start_pos[agent_id],
                        dist
                    ],
                    axis=0,
                )
                return a
            elif self.angle_representation == 1:
                a = np.concatenate(
                    [
                        ang_vel,
                        quaternion,
                        lin_vel,
                        lin_pos,
                        aux_state,
                        self.past_actions[agent_id],
                        self.start_pos[agent_id],
                        dist
                    ],
                    axis=0,
                )
                a = np.reshape(a, (13,2))
                return a
            else:
                raise AssertionError("Not supposed to end up here!")
        else:
            active_drones = []
            for agent in self.agents[:self.active_uavs]:
                active_drones.append(self.agent_name_mapping.get(agent))
            uavs, pos = self.aviary.cams[agent_id-self.num_uavs].get_Observation(active_drones, self.num_uavs)
            obs = np.concatenate([uavs, pos, self.past_camactions[agent_id-self.num_uavs]], axis=0)
            return obs
    
    def compute_term_trunc_reward_info_by_id(self, agent_id):
        reward = 0.0
        term = False
        trunc = self.step_count > self.max_steps
        info = dict()
        if (agent_id < self.num_uavs):
            views = []
            val = np.any(self.aviary.contact_array[self.aviary.drones[agent_id].Id])
            for cam in self.aviary.cams:
                _,_, seg = cam.camera.capture_image()
                views.append(seg)
            # collision
            if val:
                reward -= 100.0
                info["collision"] = True
                term |= True
            # spotted
            for v in views:
                check = False
                for i in range(len(v)):
                    for j in range(len(v[i][0])):
                        if (v[i][j][0] == agent_id):
                            check = True
                            reward -= 100
                            term |= True
                            info["spotted"] = True
                            break
                    if (check):
                        break
                    
            # exceed flight dome
            if np.linalg.norm(self.aviary.state(agent_id)[-1]) > self.flight_dome_size:
                reward -= 100.0
                info["out_of_bounds"] = True
                term |= True

            # reward
            # distance from 0, 0, 1 hover point
            _, ang_pos, _, lin_pos, quaterion = self.compute_attitude_by_id(agent_id)

            dist = self.waypoint.distance_to_targets(ang_pos, lin_pos, quaterion)
            dist = ((dist[0]**2)+(dist[1]**2)+(dist[2]**2))**0.5
            if (dist <= self.goal_reach_distance):
                reward += 500
                trunc |= True
            else:
                reward -= dist
        else:
            # collision
            val = np.any(self.aviary.contact_array[self.aviary.cams[agent_id-self.num_uavs].Id])
            if val:
                reward -= 100.0
                info["collision"] = True
                term |= True
            _, _, seg = self.aviary.cams[agent_id-self.num_uavs].camera.capture_image()
            for i in range(len(seg)):
                for j in range(len(seg[i])):
                    if ((seg[i][j][0]) == self.waypoint.Id):
                        reward += 5
                    elif ((seg[i][j][0]) in self.agents[:self.active_uavs]):
                        reward += 50
        return term, trunc, reward, info

    def action_space(self, agent: Any = None) -> spaces.Box:
        """action_space.

        Args:
            agent (Any): agent

        Returns:
            spaces.Box:

        """
        if (agent is not None and "cam" in agent):
            return self.camera_actions
        return self._action_space
        
