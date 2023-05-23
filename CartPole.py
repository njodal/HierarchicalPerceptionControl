import gymnasium as gym
import unit_test as ut


class BaseEnvironment(object):
    def __init__(self, name, control=None, render_mode=None):
        self.control = control
        self.env     = gym.make(name, render_mode=render_mode)

    def run_episode(self, debug=False):
        if debug:
            print('   start episode')
        observation, info = self.env.reset()
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


class ControlCartPole:
    def __init__(self, control_gains, debug_units=None, pole_angle_reference=0.0):
        self.k1, self.k2, self.k3, self.k4 = control_gains
        debug_flag = debug_units if len(debug_units) >= 4 else [False, False, False, False]
        self.pole_angle_reference = pole_angle_reference
        self.unit1 = ControlUnit('pole angle', control_gains[0], debug=debug_flag[0])
        self.unit2 = ControlUnit('pole speed', control_gains[1], debug=debug_flag[1])
        self.unit3 = ControlUnit('cart pos',   control_gains[2], debug=debug_flag[2])
        self.unit4 = ControlUnit('cart speed', control_gains[3], debug=debug_flag[3])

    def change_pole_angle_reference(self, new_pole_angle_reference):
        self.pole_angle_reference = new_pole_angle_reference

    def get_action(self, observation, _):
        cart_position, cart_speed, pole_angle, pole_speed = observation
        pole_speed_ref = self.unit1.get_output(self.pole_angle_reference, pole_angle)
        cart_pos_ref   = self.unit2.get_output(pole_speed_ref, pole_speed)
        cart_speed_ref = self.unit3.get_output(cart_pos_ref, cart_position)
        action1        = self.unit4.get_output(cart_speed_ref, cart_speed)
        action = 1 if action1 > 0.0 else 0
        return action


class ControlUnit:
    def __init__(self, name, gains, debug=False):
        self.name  = name
        self.k     = gains[0]
        self.ks    = gains[1] if len(gains) > 1 else 1.0
        self.debug = debug
        self.o     = 0.0
        self.e     = 0.0

    def get_output(self, reference, perception):
        self.e = reference - perception
        self.o = self.o + (self.k*self.e - self.o)/self.ks
        if self.debug:
            print('        %s r:%.2f p:%.2f e:%.2f o:%.4f' % (self.name, reference, perception, self.e, self.o))
        return self.o


def test_control_pole_angle(pole_angle_reference, gains, debug_units, render, debug):
    render_mode = 'human' if render else None
    env = BaseEnvironment('CartPole-v1', control=ControlCartPole(gains, pole_angle_reference=pole_angle_reference,
                                                                 debug_units=debug_units),
                          render_mode=render_mode)
    steps = env.run_episode(debug=debug)
    return steps


if __name__ == "__main__":
    ut.UnitTest(__name__, 'tests/CartPole.test', '')
