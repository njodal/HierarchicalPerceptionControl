import gymnasium as gym


class BaseEnvironment(object):
    def __init__(self, name, control=None, max_episode_steps=500, render_mode=None):
        self.control = control
        self.env     = gym.make(name, max_episode_steps=max_episode_steps, render_mode=render_mode)

    def get_max_episode_steps(self):
        return self.env.spec.max_episode_steps

    def run_episode(self, initial_values=(), debug=False):
        if debug:
            print('   start episode')

        observation, info = self.env.reset()
        for [i, v] in initial_values:
            observation[i] = v

        steps = 0
        ended = False
        while not ended:
            action = self.control.get_action(observation, info) if self.control is not None \
                else self.env.action_space.sample()
            observation, reward, terminated, truncated, info = self.env.step(action)
            steps += 1
            ended = terminated or truncated
            if debug:
                print('     step:%s obs:%s action:%s' % (steps, observation, action))
                if terminated:
                    print('    terminated at step %s' % steps)
        return steps

    def run_episodes(self, max_number_of_episodes=500):
        for _ in range(max_number_of_episodes):
            self.run_episode(debug=True)
        self.env.close()
