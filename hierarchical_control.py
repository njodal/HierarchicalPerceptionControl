
from ControlUnit import signum


class BaseHierarchicalControl(object):
    """
    Base class for all Gymnasium cases
    """

    def __init__(self, main_control, low_levels_controls=None, reference=0.0, overshoot_gain=1.0):
        # to avoid warnings
        self.total_error = 0.0
        self.reference = 0.0
        self.max_overshoot = 0.0
        self.current_e = 0.0
        self.initial_err_sign = 0

        self.main_control        = main_control
        self.low_levels_controls = low_levels_controls

        self.overshoot_gain = overshoot_gain
        self.set_reference(reference)

    def reset(self):
        self.total_error = 0.0
        self.main_control.reset()
        if self.low_levels_controls is not None:
            self.low_levels_controls.reset()

    def set_reference(self, new_reference):
        self.reference        = new_reference
        self.initial_err_sign = signum(self.reference)
        self.max_overshoot    = 0.0
        self.current_e        = 0.0
        self.main_control.set_reference(self.reference)

    def get_main_observation(self, observation):
        return observation[0]

    def get_action(self, observation, info):
        """
        Abstract method for getting an action receiving observation and info
        :param observation:
        :param info:
        :return:
        """
        main_observation = self.get_main_observation(observation)
        second_reference = self.main_control.get_output(self.reference, main_observation)
        if self.low_levels_controls is None:
            action = second_reference
        else:
            self.low_levels_controls.set_reference(second_reference)
            action = self.low_levels_controls.get_action(observation, info)

        self.add_to_cost(self.main_control.e)

        return action

    def add_to_cost(self, new_error):
        abs_error = abs(new_error)
        if signum(new_error) != self.initial_err_sign:
            gain = self.overshoot_gain
            if abs_error > self.max_overshoot:
                self.max_overshoot = abs_error
        else:
            gain = 1.0
        self.total_error += gain * abs_error
        self.current_e = new_error

    def get_total_cost(self):
        return self.total_error

    # methods for autotune
    def get_parameters(self):
        return self.main_control.get_parameters()

    def set_parameters(self, parameters):
        self.main_control.set_parameters(parameters)

    # string methods
    def parm_string(self):
        return self.main_control.parm_string()

    def summary_string(self):
        return 'cost:%.3f overshoot:%.3f current:%.3f' % (self.total_error, self.max_overshoot, self.current_e)
