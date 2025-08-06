from __future__ import annotations

from typing import Any, Literal


import numpy as np
from gymnasium import spaces
from PyFlyt.pz_envs.fixedwing_envs.ma_fixedwing_dogfight_env import MAFixedwingDogfightEnv

if __name__ == "__main__":
    #print("PASS")
    env = MAFixedwingDogfightEnv(render_mode="human")
    print("Start")
    
    #while True:
        #print("Check 1")
        #env.render()
    observations, infos = env.reset()

    while env.agents:
        # this is where you would insert your policy
        actions = {agent: env.action_space(agent).sample() for agent in env.agents}

        observations, rewards, terminations, truncations, infos = env.step(actions)
    env.close()