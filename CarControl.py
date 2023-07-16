from ControlUnit import create_control
from CarModel import CarEnvironment
import hierarchical_control as hc
import signals as sg
import auto_tune as at

import yaml_functions as yf
import WinDeklar.WindowForm as WinForm
import WinDeklar.graph_aux as ga
import unit_test as ut


class CarSpeedControl(hc.BaseHierarchicalControl):
    """
    Control the speed of a Car changing its acceleration and braking pedals
    """

    def __init__(self, speed_reference, speed_control_def, overshoot_gain=10.0):
        speed_control = create_control(speed_control_def)
        low_levels    = []
        super(CarSpeedControl, self).__init__(speed_control, low_levels_controls=low_levels,
                                              reference=speed_reference, overshoot_gain=overshoot_gain)

    def get_main_observation(self, observation):
        return observation[1]  # current car speed

    def get_actions_from_output(self, acceleration):
        acc     = acceleration if acceleration > 0 else 0
        brake   = - acceleration if acceleration < 0 else 0
        actions = {'acc': acc, 'brake': brake}
        # print('actions: %s' % actions)
        return actions

    def set_kg(self, new_value):
        self.main_control.set_kg(new_value)

    def set_ks(self, new_value):
        self.main_control.set_ks(new_value)


class CarAccPedalControl(hc.BaseHierarchicalControl):
    """
    Control the speed of a Car changing its acceleration
    """

    def __init__(self, accelerator_control_def, overshoot_gain=1.0):
        acc_control = create_control(accelerator_control_def)
        super(CarAccPedalControl, self).__init__(acc_control, low_levels_controls=[], reference=0.0,
                                                 overshoot_gain=overshoot_gain)

    def get_main_observation(self, observation):
        return observation[2]  # current car acceleration


class AutoTuneCarSpeed(at.AutoTuneFunction):
    def __init__(self, car_name, control, output_lag, reference_changes, slope=0.0, dt=0.1, max_iter=500, debug=False):
        self.dt                = dt
        self.output_lag        = output_lag
        self.slope             = slope
        self.control           = control
        self.car_name          = car_name
        self.reference_changes = reference_changes
        self.debug             = debug

        super(AutoTuneCarSpeed, self).__init__(max_iter=max_iter)

    def get_parameters(self):
        return self.control.get_parameters()

    def set_parameters(self, parameters):
        return self.control.set_parameters(parameters)

    def auto_tune(self, debug=False):
        best_err, best_p, j = self.auto_tune_with_twiddle()
        if debug:
            print('best cost: %.2f' % best_err)
        return best_p

    def run_one_episode(self, parameters):
        self.control.set_parameters(parameters)
        self.control.reset()
        env = CarEnvironment(self.control, car_name=self.car_name, output_lag=self.output_lag, slope=self.slope,
                             dt=self.dt, max_steps=self.max_iter)
        steps, observation, cost, speed_evo = env.run_episode(reference_changes=self.reference_changes, debug=False)
        return cost, speed_evo

    def run_function_with_parameters(self, parameters):
        cost, _ = self.run_one_episode(parameters)
        # print('  run, parameters: %s cost: %.2f' % (parameters, cost))
        return cost


class CarPositionalControl:
    """
    Control the position of a Car changing its acceleration
    """

    def __init__(self, position_reference, position_control_def, speed_control_def):
        self.control_position = create_control(position_control_def)
        self.control_speed    = create_control(speed_control_def)

        self.position_reference = position_reference
        self.sqr_errors         = 0.0

    def get_action(self, observation):
        current_position, current_speed, _ = observation
        self.sqr_errors += abs(self.position_reference - current_position)
        speed_reference = self.control_position.get_output(self.position_reference, current_position)
        acc_reference   = self.control_speed.get_output(speed_reference, current_speed)
        return acc_reference


def get_car_controller(controller_name):
    cars_file  = yf.get_yaml_file('cars/car_controllers.yaml')
    controller = yf.get_record(cars_file, controller_name, 'controllers', 'controller')
    return CarSpeedControl(0.0, controller['def'])


# tests
def test_speed_control(speed_reference, output_lag, dt, max_steps, speed_control_def, debug):
    control = CarSpeedControl(speed_reference, speed_control_def)
    env     = CarEnvironment(control, output_lag=output_lag, dt=dt, max_steps=max_steps)
    steps, observation, errors, _ = env.run_episode()
    if debug:
        print('speed:%.2f (ref:%.2f) at step %s (total error:%.3f)' % (observation[1], speed_reference, steps, errors))
    return observation[1]


def test_auto_tune_speed_control(car_name, output_lag, slope, dt, max_iter, reference_changes, speed_control_def,
                                 debug):
    control         = CarSpeedControl(0.0, speed_control_def)
    auto_tune       = AutoTuneCarSpeed(car_name, control, output_lag, reference_changes, dt=dt, max_iter=max_iter,
                                       slope=slope, debug=debug)
    best_parameters = auto_tune.auto_tune(debug=debug)
    if debug:
        show_auto_tune(car_name, output_lag, dt, reference_changes, max_iter, best_parameters, control, auto_tune)

    return best_parameters['p'], best_parameters['i'], best_parameters['d']


def show_auto_tune(car_name, output_lag, dt, reference_changes, max_iter, best_parameters, control, auto_tune):
    control.set_parameters(best_parameters)
    title = 'Auto tune speed control (car: %s, lag:%s, parms: %s)' % (car_name, output_lag, control.parm_string())
    window = WinForm.SimpleFigure(title=title, size=(5, 2), inc=1.2, adjust_size=True)
    # graph reference changes
    last_y = 0.0
    ref_points = []
    for [x, y] in reference_changes:
        ref_points.append([x*dt, last_y])
        ref_points.append([x*dt, y])
        last_y = y
    ref_points.append([max_iter*dt, last_y])
    ga.graph_points(window.ax, ref_points, color='Blue')
    window.resize(ref_points)
    window.resize([[-1, -1]])

    # graph speed evolution
    _, speed_evo = auto_tune.run_one_episode(best_parameters)
    ga.graph_points(window.ax, speed_evo, color='Red')
    window.resize(speed_evo)

    window.show()


def test_positional_control(pos_reference, output_lag, dt, max_steps, position_control_def, speed_control_def, debug):
    control = CarPositionalControl(pos_reference, position_control_def, speed_control_def)
    env     = CarEnvironment(control, output_lag=output_lag, dt=dt, max_steps=max_steps)
    steps, observation, errors, _ = env.run_episode()
    if debug:
        print('position:%.2f at step %s (total error:%.3f)' % (observation[0], steps, errors))
    return observation[0]


if __name__ == "__main__":
    ut.UnitTest(__name__, 'tests/CarControl.test', '')
