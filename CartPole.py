import math

import GymEnvironment as GymEnv
from ControlUnit import PCU, BangBang, signum, PID
import auto_tune as at
import unit_test as ut


class BaseGymControl(object):
    """
    Base class for all Gymnasium cases
    """

    def __init__(self, reference, overshoot_gain=1.0):
        # to avoid warnings
        self.total_error      = 0.0
        self.avg_error        = 0.0
        self.steps            = 0
        self.reference        = 0.0
        self.max_overshoot    = 0.0
        self.current_e        = 0.0
        self.initial_err_sign = 0
        
        self.overshoot_gain = overshoot_gain
        self.set_reference(reference)
        
    def set_reference(self, new_reference):
        self.reference        = new_reference
        self.total_error      = 0.0
        self.avg_error        = 0.0
        self.steps            = 0
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
        self.avg_error += new_error
        self.steps += 1
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
        avg = self.avg_error/self.steps if self.steps > 0 else 0.0
        return 'cost:%.3f overshoot:%.3f current:%.3f avg:%.3f' % (self.total_error, self.max_overshoot, self.current_e,
                                                                   avg)


class MoveCartToPosition(BaseGymControl):
    def __init__(self, gains, cart_pos_reference=0.0, max_pole_angle_allowed=5, overshoot_gain=5.0):
        max_pole_angle    = math.radians(max_pole_angle_allowed)
        self.unit1        = PCU('cart position', gains=gains[0],
                                bounds=(-max_pole_angle, max_pole_angle), debug=False)
        lower_levels_gains = [g for i, g in enumerate(gains) if i > 0]
        self.down_control  = GymControlCartPoleAtAngle(lower_levels_gains, debug_units=[False, False, False, False])
        super(MoveCartToPosition, self).__init__(cart_pos_reference, overshoot_gain=overshoot_gain)

    def get_last_error(self):
        return self.unit1.e

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
        self.k_p, self.k_i, self.k_d = get_pid_gains(self.k1)
        self.unit1 = PID(key='pole angle', p=self.k_p, i=self.k_i, d=self.k_d, debug=debug_flag[0])
        self.unit2 = PCU('pole speed', control_gains[1], debug=debug_flag[1])
        self.unit3 = PCU('cart pos', control_gains[2], debug=debug_flag[2])
        self.unit4 = PCU('cart speed', control_gains[3], debug=debug_flag[3])
        self.unit5 = BangBang(bellow_value=0, above_value=1, hysteresis=0.0, key='cart action')

        overshoot_gain = 1.0  # not problem to have overshot in this case
        super(GymControlCartPoleAtAngle, self).__init__(pole_angle_reference, overshoot_gain=overshoot_gain)

    def get_parameters(self):
        return self.unit1.get_parameters()

    def set_parameters(self, new_parameters):
        self.unit1.set_parameters(new_parameters)

    def get_last_error(self):
        return self.unit1.e

    def get_action(self, observation, _):
        cart_position, cart_speed, pole_angle, pole_speed = observation
        pole_speed_ref = self.unit1.get_output(self.reference, pole_angle)
        cart_pos_ref   = self.unit2.get_output(pole_speed_ref, pole_speed)
        cart_speed_ref = self.unit3.get_output(cart_pos_ref, 0.0)  # position is relative not absolute
        action1        = self.unit4.get_output(cart_speed_ref, cart_speed)
        action         = self.unit5.get_output(0.0, action1)
        # action = 1 if action1 > 0.0 else 0
        # print('  angle:%.3f speed:%.3f cart pos ref:%.3f speed ref:%.3f' % (math.degrees(pole_angle),
        #                                                                    math.degrees(pole_speed_ref), cart_pos_ref,
        #                                                                    cart_speed_ref))
        self.add_to_cost(self.unit1.e*self.unit1.e)
        return action


class AutoTunePoleAngleControl(at.AutoTuneFunction):
    def __init__(self, first_gains, control_gains, pole_angle_reference=0.0, render=False, debug=False):
        self.debug   = debug
        self.render  = render
        self.total_i = 0
        self.kg      = first_gains
        self.other_g = control_gains
        self.ref     = pole_angle_reference
        self.control = None
        self.reset()
        super(AutoTunePoleAngleControl, self).__init__(threshold=0.01, change=2.0, bad_inc=2.0, mid_inc=1.05)

    def reset(self):
        gains = self.other_g.copy()
        gains.insert(0, self.kg)
        self.control = GymControlCartPoleAtAngle(gains, pole_angle_reference=self.ref, debug_units=[])

    def get_parameters(self):
        return self.control.get_parameters()

    def set_parameters(self, new_parameters):
        self.reset()
        self.control.set_parameters(new_parameters)

    def run_function_with_parameters(self, parameters):
        self.total_i += 1
        self.set_parameters(parameters)
        env      = GymEnv.BaseEnvironment('CartPole-v1', control=self.control, render_mode=None)
        steps, _ = env.run_episode(initial_values=[[0, 0.0], [1, 0.0], [2, 0.0], [3, 0.0]], debug=False)
        cost     = (env.get_max_episode_steps() - steps) + self.control.total_error
        if self.debug:
            print('   iter: %s total error: %.3f steps: %s cost:%.3f' % (self.total_i, self.control.total_error, steps,
                                                                         cost))
        return cost


class AutoTuneCartPositionControl(at.AutoTuneFunction):
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
        super(AutoTuneCartPositionControl, self).__init__(threshold=0.01, change=10.0, bad_inc=2.0, mid_inc=1.05)

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

    def run_function_with_parameters(self, parameters):
        self.total_i += 1
        self.set_parameters(parameters)
        env      = GymEnv.BaseEnvironment('CartPole-v1', control=self.control, render_mode=None)
        steps, _ = env.run_episode(initial_values=[[0, 0.0], [1, 0.0], [2, 0.0], [3, 0.0]], debug=False)
        cost     = self.bad_g*(env.get_max_episode_steps() - steps) + self.control.total_error
        if self.debug:
            print('   iter: %s total error: %.3f steps: %s cost:%.3f' % (self.total_i, self.control.total_error, steps,
                                                                         cost))
        return cost


def get_pid_gains(gains):
    length = len(gains)
    if length == 0:
        raise Exception('No gains provided for PID controller')
    else:
        ks = [gains[i] if i < length else 0.0 for i in range(0, 3)]
    return ks


def run_one_move_cart(car_pos_reference, render, gains, max_iter, max_angle=5, debug=False):
    render_mode = 'human' if render else None
    control     = MoveCartToPosition(gains, cart_pos_reference=car_pos_reference, max_pole_angle_allowed=max_angle)
    env         = GymEnv.BaseEnvironment('CartPole-v1', control=control, max_episode_steps=max_iter,
                                         render_mode=render_mode)
    steps, obs  = env.run_episode(debug=debug)
    result_msg  = get_result_msg(steps, max_iter, obs)
    summary     = '%s (%s)' % (result_msg, control.summary_string())
    return steps, env.error_history, summary


def run_one_pole_control(pole_angle_reference, gains, render, debug_units, max_iter=500, debug=False):
    render_mode = 'human' if render else None
    control     = GymControlCartPoleAtAngle(gains, pole_angle_reference=pole_angle_reference, debug_units=debug_units)
    env         = GymEnv.BaseEnvironment('CartPole-v1', control=control, max_episode_steps=max_iter,
                                         render_mode=render_mode)
    steps, obs  = env.run_episode(debug=debug)
    result_msg  = get_result_msg(steps, max_iter, obs)
    summary     = '%s (%s)' % (result_msg, control.summary_string())
    return steps, env.error_history, control.total_error, summary


def get_result_msg(steps, max_iter, observation):
    return 'Succeed' if steps >= max_iter else 'Failed at %s because %s' % (steps, limit_msg(observation))


def limit_msg(observation, max_d=2.4, max_a=12):
    if abs(observation[0]) > max_d:
        msg = 'max distance reached (%s)' % max_d
    elif math.degrees(abs(observation[2])) > max_a:
        msg = 'pole angle (%.2f) more than the allowed limit (%.2f)' % (math.degrees(abs(observation[2])), max_a)
    else:
        msg = ''
    return msg


# tests
def test_autotune_pole_angle_first_gain(pole_angle_reference, kg, gains, render, debug):
    to_tune = AutoTunePoleAngleControl(kg, gains, pole_angle_reference=pole_angle_reference, render=render, debug=debug)
    best_parameters = to_tune.auto_tune()
    to_tune.set_parameters(best_parameters)
    if debug:
        print('best parameters: %s' % best_parameters)
    if render:
        max_iter = 500
        env      = GymEnv.BaseEnvironment('CartPole-v1', control=to_tune.control, max_episode_steps=max_iter,
                                          render_mode='human')
        env.run_episode(debug=debug)
        if debug:
            print('    %s' % to_tune.control.summary_string())

    return best_parameters['p'], best_parameters['i'], best_parameters['d']


def test_autotune_move_cart_first_gain(cart_pos_reference, max_iter, kg, ks, gains, render, debug):
    to_tune = AutoTuneCartPositionControl(kg, ks, gains, cart_pos_reference=cart_pos_reference, render=render,
                                          max_iter=max_iter, debug=debug)
    best_parameters = to_tune.auto_tune()
    to_tune.set_parameters(best_parameters)
    render_m = 'human' if render else None
    env = GymEnv.BaseEnvironment('CartPole-v1', control=to_tune.control, max_episode_steps=max_iter,
                                 render_mode=render_m)
    steps, _ = env.run_episode(initial_values=[[0, 0.0], [1, 0.0], [2, 0.0]], debug=debug)
    print('    %s steps:%s' % (to_tune.control.summary_string(), steps))

    return to_tune.kg, to_tune.ks


def test_control_pole_angle(pole_angle_reference, gains, debug_units, render, debug):
    steps, _, total_error, summary = run_one_pole_control(pole_angle_reference, gains, render, debug_units, debug=debug)
    print('total error: %.3f' % total_error)
    return steps


def test_move_cart(car_pos_reference, max_iter, gains, debug_units, render, debug):
    steps, _, summary_string = run_one_move_cart(car_pos_reference, render, gains, max_iter, debug=debug)
    if debug:
        print('%s' % summary_string)
    return steps


if __name__ == "__main__":
    ut.UnitTest(__name__, 'tests/CartPole.test', '')
