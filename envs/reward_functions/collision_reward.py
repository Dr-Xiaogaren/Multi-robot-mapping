from igibson.reward_functions.reward_function_base import BaseRewardFunction


class CollisionReward(BaseRewardFunction):
    """
    Collision reward
    Penalize robot collision. Typically collision_reward_weight is negative.
    """

    def __init__(self, config):
        super(CollisionReward, self).__init__(config)
        self.collision_reward_weight = self.config.get("collision_reward_weight", -0.1)

    def get_reward(self, task, env):
        """
        Reward is self.collision_reward_weight if there is collision
        in the last timestep

        :param task: task instance
        :param env: environment instance
        :return: reward
        """
        reward = [0.0 for i in range(env.n_robots)]
        for i, collision_links in enumerate(env.collision_links):
            if len(collision_links) > 0:
                reward[i] += self.collision_reward_weight
        return reward
