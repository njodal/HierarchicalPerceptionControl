import GymEnvironment as GymEnv
from ControlUnit import PCUControlUnit, signum
import auto_tune as at
import unit_test as ut


class BaseGymControl(object):
    """
    Base class for all Gymnasium cases
    """

    def __init__(self, reference, overshoot_gain=1.0):
        # to avoid warnings
        self.total_error = 0.0
        self.reference   = 0.0
        self.max_overshoot    = 0.0
        self.current_e        = 0.0
        self.initial_err_sign = 0
        
        self.overshoot_gain = overshoot_gain
        self.set_reference(reference)
        
    def set_reference(self, new_reference):
        self.reference        = new_reference
        self.total_error      = 0.0
        self.initial_err_sign = signum(self.reference)
        self.max_overshoot    = 0.0
        self.current_e        = 0.0

    def get_action(self, observation, info):
        """
        Abstract method for getting an action receiving observation and info
        :param observation:
        :param info:
        :return:
        """
        pass

    def add_to_cost(self, new_error):
        abs_error = abs(new_error)
        if signum(new_error) != self.initial_err_sign:
            gain = self.overshoot_gain
            if abs_error > self.max_overshoot:
                self.max_overshoot = abs_error
        else:
            gain = 1.0
        self.total_error += gain * abs_error
        self.current_e    = new_error

    def get_total_cost(self):
        return self.total_error

    def summary_string(self):
        return 'cost:%.3f overshoot:%.3f current:%.3f' % (self.total_error, self.max_overshoot, self.current_e)


class MoveCartToPosition(BaseGymControl):
    def __init__(self, gains, cart_pos_reference=0.0, overshoot_gain=5.0):
        self.unit1              = PCUControlUnit('cart position', gains=gains[0], debug=False)
        last_gains              = [g for i, g in enumerate(gains) if i > 0]
        self.down_control       = GymControlCartPoleAtAngle(last_gains, debug_units=[False, False, False, False])
        super(MoveCartToPosition, self).__init__(cart_pos_reference, overshoot_gain=overshoot_gain)

    def get_action(self, observation, info):
        cart_position        = observation[0]
        pole_angle_reference = self.unit1.get_output(self.reference, cart_position)
        self.down_control.set_reference(pole_angle_reference)
        action = self.down_control.get_action(observation, info)

        self.add_to_cost(self.unit1.e)

        return action


class GymControlCartPoleAtAngle(BaseGymControl):
    def __init__(self, control_gains, debug_units=None, pole_angle_reference=0.0):
        self.k1, self.k2, self.k3, self.k4 = control_gains
        debug_flag = debug_units if len(debug_units) >= 4 else [False, False, False, False]
        self.unit1 = PCUControlUnit('pole angle', control_gains[0], debug=debug_flag[0])
        self.unit2 = PCUControlUnit('pole speed', control_gains[1], debug=debug_flag[1])
        self.unit3 = PCUControlUnit('cart pos',   control_gains[2], debug=debug_flag[2])
        self.unit4 = PCUControlUnit('cart speed', control_gains[3], debug=debug_flag[3])

        overshoot_gain = 1.0  # not problem to have overshot in this case
        super(GymControlCartPoleAtAngle, self).__init__(pole_angle_reference, overshoot_gain=overshoot_gain)

    def get_action(self, observation, _):
        cart_position, cart_speed, pole_angle, pole_speed = observation
        pole_speed_ref = self.unit1.get_output(self.reference, pole_angle)
        cart_pos_ref   = self.unit2.get_output(pole_speed_ref, pole_speed)
        cart_speed_ref = self.unit3.get_output(cart_pos_ref, cart_position)
        action1        = self.unit4.get_output(cart_speed_ref, cart_speed)
        action = 1 if action1 > 0.0 else 0

        self.add_to_cost(self.unit1.e*self.unit1.e)
        return action


class AutoTunePoleAngleControl:
    def __init__(self, kg, control_gains, pole_angle_reference=0.0, render=False, debug=False):
        self.p_name  = 'k1'
        self.debug   = debug
        self.render  = render
        self.total_i = 0
        self.kg      = kg
        self.other_g = control_gains
        self.ref     = pole_angle_reference
        self.control = None
        self.reset()

    def reset(self):
        gains = self.other_g.copy()
        gains.insert(0, [self.kg])
        self.control = GymControlCartPoleAtAngle(gains, pole_angle_reference=self.ref, debug_units=[])

    def auto_tune(self):
        best_error, best_parameters, steps = at.twiddle(self, threshold=0.01, change=5.0, bad_inc=2.0, mid_inc=1.05)
        if self.debug:
            print('best cost:%.3f parameters:%s tries:%s' % (best_error, best_parameters, steps))
            print('    %s' % self.control.summary_string())
        self.set_parameters(best_parameters)
        if self.render:
            env   = GymEnv.BaseEnvironment('CartPole-v1', control=self.control, render_mode='human')
            steps = env.run_episode(debug=self.debug)
            if self.debug:
                print('    %s' % self.control.summary_string())

    def get_parameters(self):
        return {self.p_name: self.kg}

    def set_parameters(self, new_parameters):
        if self.p_name in new_parameters:
            self.kg = new_parameters[self.p_name]
        self.reset()

    def function(self, parameters):
        self.total_i += 1
        self.set_parameters(parameters)
        env   = GymEnv.BaseEnvironment('CartPole-v1', control=self.control, render_mode=None)
        steps = env.run_episode(initial_values=[[0, 0.0], [1, 0.0], [2, 0.0], [3, 0.0]], debug=False)
        cost  = (env.get_max_episode_steps() - steps) + self.control.total_error
        if self.debug:
            print('   iter: %s total error: %.3f steps: %s cost:%.3f' % (self.total_i, self.control.total_error, steps,
                                                                         cost))
        return cost

    def is_better(self, error1, error2):
        return abs(error1) < abs(error2)

    def is_best(self, error, precision=0.001):
        return abs(error) <= precision


class AutoTuneCartPositionControl:
    def __init__(self, kg, ks, control_gains, max_iter=500, cart_pos_reference=0.0, not_stable_gain=100.0, render=False,
                 debug=False):
        self.p_name1  = 'kg'
        self.p_name2  = 'ks'
        self.debug    = debug
        self.render   = render
        self.render_m = 'human' if self.render else None
        self.max_iter = max_iter
        self.total_i  = 0
        self.kg       = kg
        self.ks       = ks
        self.other_g  = control_gains
        self.ref      = cart_pos_reference
        self.bad_g    = not_stable_gain
        self.control  = None
        self.reset()

    def reset(self):
        gains = self.other_g.copy()
        gains.insert(0, [self.kg, self.ks])
        self.control = MoveCartToPosition(gains, cart_pos_reference=self.ref)

    def get_parameters(self):
        return {self.p_name1: self.kg, self.p_name2: self.ks}

    def set_parameters(self, new_parameters):
        if self.p_name1 in new_parameters:
            self.kg = new_parameters[self.p_name1]
        if self.p_name2 in new_parameters:
            self.ks = new_parameters[self.p_name2]
        self.reset()

    def auto_tune(self):
        best_error, best_parameters, steps = at.twiddle(self, threshold=0.01, change=10.0, bad_inc=2.0, mid_inc=1.05)
        if self.debug:
            print('best cost:%.3f parameters:%s tries:%s' % (best_error, best_parameters, steps))
        self.set_parameters(best_parameters)
        env   = GymEnv.BaseEnvironment('CartPole-v1', control=self.control, max_episode_steps=self.max_iter,
                                       render_mode=self.render_m)
        steps = env.run_episode(initial_values=[[0, 0.0], [1, 0.0], [2, 0.0]], debug=self.debug)
        print('    %s steps:%s' % (self.control.summary_string(), steps))

    def function(self, parameters):
        self.total_i += 1
        self.set_parameters(parameters)
        env   = GymEnv.BaseEnvironment('CartPole-v1', control=self.control, render_mode=None)
        steps = env.run_episode(initial_values=[[0, 0.0], [1, 0.0], [2, 0.0], [3, 0.0]], debug=False)
        cost  = self.bad_g*(env.get_max_episode_steps() - steps) + self.control.total_error
        if self.debug:
            print('   iter: %s total error: %.3f steps: %s cost:%.3f' % (self.total_i, self.control.total_error, steps,
                                                                         cost))
        return cost

    def is_better(self, error1, error2):
        return abs(error1) < abs(error2)

    def is_best(self, error, precision=0.001):
        return abs(error) <= precision


# tests
def test_autotune_pole_angle_first_gain(pole_angle_reference, kg, gains, render, debug):
    to_tune = AutoTunePoleAngleControl(kg, gains, pole_angle_reference=pole_angle_reference, render=render, debug=debug)
    to_tune.auto_tune()
    return to_tune.kg


def test_autotune_move_cart_first_gain(cart_pos_reference, max_iter, kg, ks, gains, render, debug):
    to_tune = AutoTuneCartPositionControl(kg, ks, gains, cart_pos_reference=cart_pos_reference, render=render,
                                          max_iter=max_iter, debug=debug)
    to_tune.auto_tune()
    return to_tune.kg, to_tune.ks


def test_control_pole_angle(pole_angle_reference, gains, debug_units, render, debug):
    render_mode = 'human' if render else None
    control     = GymControlCartPoleAtAngle(gains, pole_angle_reference=pole_angle_reference, debug_units=debug_units)
    env         = GymEnv.BaseEnvironment('CartPole-v1', control=control, render_mode=render_mode)
    steps       = env.run_episode(debug=debug)
    print('total error: %.3f' % control.total_error)
    return steps


def test_move_cart(car_pos_reference, max_iter, gains, debug_units, render, debug):
    render_mode = 'human' if render else None
    control     = MoveCartToPosition(gains, cart_pos_reference=car_pos_reference)
    env         = GymEnv.BaseEnvironment('CartPole-v1', control=control, max_episode_steps=max_iter,
                                         render_mode=render_mode)
    steps       = env.run_episode(debug=debug)
    if debug or 1 == 1:
        print('%s' % control.summary_string())
    return steps


if __name__ == "__main__":
    ut.UnitTest(__name__, 'tests/CartPole.test', '')
