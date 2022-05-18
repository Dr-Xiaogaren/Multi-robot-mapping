import numpy as np
import pybullet as p

from igibson.objects.visual_marker import VisualMarker
from igibson.reward_functions.collision_reward import CollisionReward
from igibson.reward_functions.point_goal_reward import PointGoalReward
from igibson.reward_functions.potential_reward import PotentialReward
from igibson.scenes.gibson_indoor_scene import StaticIndoorScene
from igibson.scenes.igibson_indoor_scene import InteractiveIndoorScene
from igibson.tasks.task_base import BaseTask
from igibson.termination_conditions.max_collision import MaxCollision
from igibson.termination_conditions.out_of_bound import OutOfBound
from igibson.termination_conditions.point_goal import PointGoal
from igibson.termination_conditions.timeout import Timeout
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

        self.initial_pos = np.array(self.config.get("initial_pos", [0, 0, 0]))
        self.initial_orn = np.array(self.config.get("initial_orn", [0, 0, 0]))
        self.target_pos = np.array(self.config.get("target_pos", [5, 5, 0]))
        self.goal_format = self.config.get("goal_format", "polar")

        self.visible_target = self.config.get("visible_target", False)
        self.visible_path = self.config.get("visible_path", False)
        self.floor_num = 0

    def get_geodesic_potential(self, env):
        """
        Get potential based on geodesic distance

        :param env: environment instance
        :return: geodesic distance to the target position
        """
        _, geodesic_dist = self.get_shortest_path(env)
        return geodesic_dist

    def get_l2_potential(self, env):
        """
        Get potential based on L2 distance

        :param env: environment instance
        :return: L2 distance to the target position
        """
        return l2_distance(env.robots[0].get_position()[:2], self.target_pos[:2])

    def get_potential(self, env):
        """
        Compute task-specific potential: distance to the goal

        :param env: environment instance
        :return: task potential
        """
        if self.reward_type == "l2":
            return self.get_l2_potential(env)
        elif self.reward_type == "geodesic":
            return self.get_geodesic_potential(env)

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
        self.path_length = 0.0
        self.robot_pos = [self.initial_pos[i, 0:2] for i in range(env.n_robots)]
        self.geodesic_dist = self.get_geodesic_potential(env)

    def get_termination(self, env, collision_links=[], action=None, info={}):
        """
        Aggreate termination conditions and fill info
        """
        done, info = super(MappingTask, self).get_termination(env, collision_links, action, info)

        info["path_length"] = self.path_length
        if done:
            info["spl"] = float(info["success"]) * min(1.0, self.geodesic_dist / self.path_length)
        else:
            info["spl"] = 0.0

        return done, info

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

    def get_shortest_path(self, env, from_initial_pos=False, entire_path=False):
        """
        Get the shortest path and geodesic distance from the robot or the initial position to the target position

        :param env: environment instance
        :param from_initial_pos: whether source is initial position rather than current position
        :param entire_path: whether to return the entire shortest path
        :return: shortest path and geodesic distance to the target position
        """
        if from_initial_pos:
            source = self.initial_pos[:2]
        else:
            source = env.robots[0].get_position()[:2]
        target = self.target_pos[:2]
        return env.scene.get_shortest_path(self.floor_num, source, target, entire_path=entire_path)

    def step(self, env):
        """
        Perform task-specific step: step visualization and aggregate path length

        :param env: environment instance
        """
        new_robot_pos = [robot.get_position()[:2] for robot in env.robots]
        # self.path_length += l2_distance(self.robot_pos, new_robot_pos)
        self.robot_pos = new_robot_pos
