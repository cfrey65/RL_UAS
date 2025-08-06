import torch
from envs.multi_cam import SurveillanceEnv
from Agents.ActorCritic import ActorCritic as A2C
from tqdm import tqdm
import numpy as np


if __name__ == "__main__":
    print("Start")
    env = SurveillanceEnv(use_yaw_targets=False, max_horzAngle=15, max_vertAngle=20, goal_reach_distance=0.25, 
                          render_mode="human",
                          goal_reach_angle=15, max_height=5.5, x=2, y=2, 
                          box_origins=[[5, 1.5, 0], [-1, 5.5, 0],[-1, 1.5, 0], [5, 5.5, 0]], seed=42,
                          start_pos=np.array([[-3,3.5,5], [-3, 3.5, 1.5], [7,3.5,5], [7,3.5,1.5]]))
    print("1")
   
    print("2")
    uavMods = dict()
    camMods = dict()
    env.reset()
    for agent in env.agents:
        if ("uav" in agent):
            uavMods.update({agent:A2C(actor_lr=0.005, critic_lr=0.01, input=26, 
                                      output=4, layerSize=128, discount=0.7, isCamera=False)})
        else:
            camMods.update({agent:A2C(actor_lr=0.005, critic_lr=0.01, input=9, 
                                      output=2, layerSize=128, discount=0.7, 
                                      isCamera=True, maxAng=torch.tensor([env.max_horzAngle, env.max_vertAngle]))})
    n = 100
    for episode in tqdm(range(n)):
        truncations = {"test":False}
        observations, infos = env.reset()
        while not np.any(np.array(list(truncations.values()))):
            # this is where you would insert your policy
            actions = dict()
            action = None
            currentObs = observations
            for agent in env.agents:
                o = currentObs.get(agent)
                if "uav" in agent:
                    action = uavMods.get(agent).getAction(o)
                else:
                    action = camMods.get(agent).getAction(o)
                actions.update({agent:action})
            
            observations, rewards, terminations, truncations, infos = env.step(actions)

            for agent in env.agents:
                co = currentObs.get(agent)
                fo = observations.get(agent)
                r = rewards.get(agent)
                if "uav" in agent:
                    uavMods.get(agent).learn(co, fo, r)
                    uavMods.get(agent).distributions = None
                else:
                    camMods.get(agent).learn(co, fo, r)
                    camMods.get(agent).distributions = None
            print("Step ", env.step_count)
    print("3")
    env.close()