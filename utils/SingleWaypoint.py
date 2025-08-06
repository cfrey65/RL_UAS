from __future__ import annotations
from PyFlyt.gym_envs.utils.waypoint_handler import WaypointHandler
import numpy as np
from pybullet_utils import bullet_client
import math


class SingleWaypoint(WaypointHandler):
    def __init__(self, enable_render, goal_reach_distance, goal_reach_angle, flight_dome_size, 
                 min_height, max_height, np_random, x, y, filename, use_yaw_targets=False):
        super().__init__(enable_render, 1, use_yaw_targets, goal_reach_distance, goal_reach_angle, 
                         flight_dome_size, min_height, np_random)
        self.max_height = max_height
        self.min_height = min_height
        self.x = x
        self.y = y
        self.Id = None
        self.filename = filename

    def reset(
        self,
        p: bullet_client.BulletClient,
        np_random: None | np.random.Generator = None,
    ):
        """Resets the waypoints."""
        # store the client
        self.p = p

        # update the random state
        if np_random is not None:
            self.np_random = np_random

        # reset the error
        self.new_distance = np.inf
        self.old_distance = np.inf
        dist = self.np_random.uniform(low=1.0, high=self.flight_dome_size * 0.9)
        phi = self.np_random.uniform(0.0, 2.0 * math.pi, size=(self.num_targets,))
        z = abs(dist * math.cos(phi))
        z = max(z, self.min_height)
        z = min(z, self.max_height)
        self.z = z
        # we sample from polar coordinates to generate linear targets
        self.targets = np.array([self.x, self.y, z])

        # if we are rendering, load in the targets
        if self.enable_render:
            self.Id = self.p.loadURDF(
                        self.filename,
                        basePosition=self.targets,
                        useFixedBase=True,
                        globalScaling= self.goal_reach_distance / 4.0,
                    )
            self.target_visual = [self.Id]

            for i, visual in enumerate(self.target_visual):
                self.p.changeVisualShape(visual, linkIndex=-1,
                    rgbaColor=(0, 1 - (i / len(self.target_visual)), 0, 1),
                ) 
