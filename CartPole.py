import math

import GymEnvironment as GymEnv
import hierarchical_control
from ControlUnit import signum
import auto_tune as at
import unit_test as ut


class BaseGymControl(object):
    """
    Base class for all Gymnasium cases
    """

    def __init__(self, control, overshoot_gain=1.0):
        # to avoid warnings
        self.total_error      = 0.0
        self.avg_error        = 0.0
        self.steps            = 0
        self.reference        = 0.0
        self.max_overshoot    = 0.0
        self.current_e        = 0.0
        self.initial_err_sign = 0

        self.control        = control
        self.overshoot_gain = overshoot_gain

    def set_reference(self, reference_name, new_reference):
        self.control.set_reference(reference_name, new_reference)
        self.reference        = new_reference
        self.total_error      = 0.0
        self.avg_error        = 0.0
        self.steps            = 0
        self.initial_err_sign = signum(self.reference)
        self.max_overshoot    = 0.0
        self.current_e        = 0.0

    def get_parameters(self):
        return self.control.get_parameters()

    def set_parameters(self, new_parameters):
        self.control.set_parameters(new_parameters)

    def get_autotune_parameters(self):
        return self.control.get_autotune_parameters()

    def get_last_error(self):
        return self.control.get_last_error()

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


class CartPoleGymControl(BaseGymControl):
    def __init__(self, control_def_name, initial_parameters=None, reference_value=0.0, reference_name='ref_name',
                 overshoot_gain=5.0, dir_name='car_pole_control'):
        control = hierarchical_control.HierarchicalControl(control_def_name, initial_parameters=initial_parameters,
                                                           dir_name=dir_name)
        super(CartPoleGymControl, self).__init__(control, overshoot_gain=overshoot_gain)
        self.set_reference(reference_name, reference_value)

    def get_action(self, observation, _):
        cart_position, cart_speed, pole_angle, pole_speed = observation
        sensors = {'cart_pos': cart_position, 'cart_speed': cart_speed, 'pole_angle': pole_angle,
                   'pole_speed': pole_speed}
        actuators = self.control.get_actuators(sensors)
        action    = actuators['action']
        error     = self.get_last_error()
        self.add_to_cost(error*error)
        # print('  action: %s' % action)
        return action


class AutoTunePoleAngleControl(at.AutoTuneFunction):
    def __init__(self, control_def_file_name, pole_angle_reference=0.0, state=None, render=False, debug=False):
        self.debug   = debug
        self.render  = render
        self.total_i = 0
        self.ref     = pole_angle_reference
        self.control_def = control_def_file_name
        self.control     = CartPoleGymControl(self.control_def, initial_parameters=state,
                                              reference_name='ref_pole_angle', reference_value=pole_angle_reference,
                                              overshoot_gain=1.0)
        super(AutoTunePoleAngleControl, self).__init__(changes_threshold=0.01, change=2.0, bad_inc=2.0, mid_inc=1.05)

    def get_parameters(self):
        return self.control.get_autotune_parameters()

    def run_function_with_parameters(self, parameters):
        self.total_i += 1
        steps, _, total_error, _ = run_one_pole_control(self.ref, self.control_def, parameters, False, debug=False)
        cost = (self.max_iter - steps) + total_error
        if self.debug:
            print('   iter: %s total error: %.3f steps: %s cost:%.3f' % (self.total_i, self.control.total_error, steps,
                                                                         cost))
        return cost


class AutoTuneCartPositionControl(at.AutoTuneFunction):
    def __init__(self, control_def_file_name, max_iter=500, cart_pos_reference=0.0, not_stable_gain=100.0, render=False,
                 debug=False):
        self.p_name1  = 'kg'
        self.p_name2  = 'ks'
        self.debug    = debug
        self.render   = render
        self.render_m = 'human' if self.render else None
        self.max_iter = max_iter
        self.total_i  = 0
        self.ref      = cart_pos_reference
        self.bad_g    = not_stable_gain
        self.control_def = control_def_file_name
        self.control     = CartPoleGymControl(self.control_def, initial_parameters=None,
                                              reference_name='ref_final_pos', reference_value=self.ref)
        super(AutoTuneCartPositionControl, self).__init__(changes_threshold=0.01, change=10.0, bad_inc=2.0, mid_inc=1.05)

    def run_function_with_parameters(self, parameters):
        self.total_i += 1
        steps, _, total_error, _ = run_one_move_cart(self.ref, self.control_def, parameters, False, self.max_iter,
                                                     debug=False)
        self.set_parameters(parameters)
        env      = GymEnv.BaseEnvironment('CartPole-v1', control=self.control, render_mode=None)
        steps, _ = env.run_episode(initial_values=[[0, 0.0], [1, 0.0], [2, 0.0], [3, 0.0]], debug=False)
        cost     = self.bad_g*(env.get_max_episode_steps() - steps) + self.control.total_error
        if self.debug:
            print('   iter: %s total error: %.3f steps: %s cost:%.3f' % (self.total_i, self.control.total_error, steps,
                                                                         cost))
        return cost


def run_one_move_cart(car_pos_reference, control_def_name, state, render, max_iter, max_angle=5, debug=False):
    render_mode        = 'human' if render else None
    max_pole_angle     = math.radians(max_angle)
    initial_parameters = {'min_pole_angle': - max_pole_angle, 'max_pole_angle': max_pole_angle}
    initial_parameters.update(state)
    control = CartPoleGymControl(control_def_name, initial_parameters=initial_parameters,
                                 reference_name='ref_final_pos', reference_value=car_pos_reference)
    env     = GymEnv.BaseEnvironment('CartPole-v1', control=control, max_episode_steps=max_iter,
                                     render_mode=render_mode)
    steps, obs  = env.run_episode(debug=debug)
    result_msg  = get_result_msg(steps, max_iter, obs)
    summary     = '%s (%s)' % (result_msg, control.summary_string())
    return steps, env.error_history, control.total_error, summary


def run_one_pole_control(pole_angle_reference, control_def_name, state, render, max_iter=500, debug=False):
    render_mode = 'human' if render else None
    control     = CartPoleGymControl(control_def_name, initial_parameters=state, reference_name='ref_pole_angle',
                                     reference_value=pole_angle_reference, overshoot_gain=1.0)
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
def test_autotune_pole_angle(pole_angle_reference, control_def_name, state, render, debug):
    to_tune = AutoTunePoleAngleControl(control_def_name, pole_angle_reference=pole_angle_reference, state=state,
                                       render=render, debug=debug)
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

    return [kv[1] for kv in best_parameters.items()]


def test_autotune_move_cart(cart_pos_reference, control_def_file_name, max_iter, render, debug):
    to_tune = AutoTuneCartPositionControl(control_def_file_name, cart_pos_reference=cart_pos_reference, render=render,
                                          max_iter=max_iter, debug=debug)
    best_parameters = to_tune.auto_tune()
    to_tune.set_parameters(best_parameters)
    render_m = 'human' if render else None
    env = GymEnv.BaseEnvironment('CartPole-v1', control=to_tune.control, max_episode_steps=max_iter,
                                 render_mode=render_m)
    steps, _ = env.run_episode(initial_values=[[0, 0.0], [1, 0.0], [2, 0.0]], debug=debug)
    print('    %s steps:%s' % (to_tune.control.summary_string(), steps))

    return [kv[1] for kv in best_parameters.items()]


def test_control_pole_angle(pole_angle_reference, control_def_name, state, render, debug):
    steps, _, total_error, summary = run_one_pole_control(pole_angle_reference, control_def_name, state, render,
                                                          debug=debug)
    if debug:
        print('total error: %.3f' % total_error)
    return steps


def test_move_cart(car_pos_reference, max_iter, control_def_name, state, render, debug):
    steps, _, _, summary_string = run_one_move_cart(car_pos_reference, control_def_name, state, render, max_iter,
                                                    debug=debug)
    if debug:
        print('%s' % summary_string)
    return steps


if __name__ == "__main__":
    ut.UnitTest(__name__, 'tests/CartPole.test', '')
