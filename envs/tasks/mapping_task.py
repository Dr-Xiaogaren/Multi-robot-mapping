import numpy as np
import pybullet as p

from envs.reward_functions.collision_reward import CollisionReward
from igibson.scenes.gibson_indoor_scene import StaticIndoorScene
from igibson.scenes.igibson_indoor_scene import InteractiveIndoorScene
from igibson.tasks.task_base import BaseTask
from envs.termination_conditions.max_collision import MaxCollision
from envs.termination_conditions.timeout import Timeout
from igibson.utils.utils import cartesian_to_polar, l2_distance, rotate_vector_3d


class MappingTask(BaseTask):
    """
    MappingTask
    The goal is to reconstruct a 2d semantic map
    """

    def __init__(self, env):
        super(MappingTask, self).__init__(env)
        self.reward_type = self.config.get("reward_type", "l2")
        self.termination_conditions = [
            MaxCollision(self.config),
            Timeout(self.config),
        ]
        self.reward_functions = [
            CollisionReward(self.config),
        ]

        self.initial_pos = np.array(self.config.get("initial_pos"))
        self.initial_orn = np.array(self.config.get("initial_orn"))
        self.floor_num = 0

    def reset_scene(self, env):
        """
        Task-specific scene reset: reset scene objects or floor plane

        :param env: environment instance
        """
        if isinstance(env.scene, InteractiveIndoorScene):
            env.scene.reset_scene_objects()
        elif isinstance(env.scene, StaticIndoorScene):
            env.scene.reset_floor(floor=self.floor_num)

    def reset_agent(self, env):
        """
        Task-specific agent reset: land the robot to initial pose, compute initial potential

        :param env: environment instance
        """
        for i, robot in enumerate(env.robots):
            env.land(robot, self.initial_pos[i], self.initial_orn[i])

    def reset_variables(self, env):
        self.robot_pos = [self.initial_pos[i, 0:2] for i in range(env.n_robots)]

    def get_termination(self, env, collision_links=[], action=None, info={}):
        """
        Aggreate termination conditions

        :param env: environment instance
        :param collision_links: collision links after executing action
        :param action: the executed action
        :param info: additional info
        :return done: whether the episode has terminated
        :return info: additional info
        """
        done = [False for i in range(env.n_robots)]
        success = [False for i in range(env.n_robots)]
        for condition in self.termination_conditions:
            # assert the condition is a list with length of n_robots
            d, s = condition.get_termination(self, env)
            for i in range(env.n_robots):
                done[i] = done[i] or d[i]
                success[i] = success[i] or s[i]
        info["done"] = done
        info["success"] = success
        return done, info

    def get_reward(self, env, collision_links=[], action=None, info={}):
        """
        Aggreate reward functions

        :param env: environment instance
        :param collision_links: collision links after executing action
        :param action: the executed action
        :param info: additional info
        :return reward: total reward of the current timestep
        :return info: additional info
        """

        reward = [0.0 for i in range(env.n_robots)]
        for reward_function in self.reward_functions:
            # assert reward is a list with length of robot or just a float
            single_reward = reward_function.get_reward(self, env)
            if isinstance(single_reward, list):
                for i in range(env.n_robots):
                    reward[i] += single_reward[i]
            else:
                for i in range(env.n_robots):
                    reward[i] += single_reward
        return reward, info

    def get_task_obs(self, env):
        """
        Get task-specific observation, including goal position, current velocities, etc.

        :param env: environment instance
        :return: task-specific observation
        """

        # linear velocity along the x-axis
        linear_velocity = rotate_vector_3d(env.robots[0].get_linear_velocity(), *env.robots[0].get_rpy())[0]
        # angular velocity along the z-axis
        angular_velocity = rotate_vector_3d(env.robots[0].get_angular_velocity(), *env.robots[0].get_rpy())[2]
        task_obs = np.array([linear_velocity, angular_velocity])

        return task_obs

    def step(self, env):
        """
        Perform task-specific step: step visualization

        :param env: environment instance
        """
        new_robot_pos = [robot.get_position()[:2] for robot in env.robots]
        self.robot_pos = new_robot_pos
