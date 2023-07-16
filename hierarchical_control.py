
from ControlUnit import signum


class BaseHierarchicalControl(object):
    """
    Base class for all Gymnasium cases
    """

    def __init__(self, main_control, low_levels_controls=(), reference=0.0, overshoot_gain=1.0, action_name='action'):
        # to avoid warnings
        self.total_error      = 0.0
        self.reference        = 0.0
        self.max_overshoot    = 0.0
        self.current_e        = 0.0
        self.initial_err_sign = 0

        self.action_name         = action_name
        self.main_control        = main_control
        self.low_levels_controls = low_levels_controls
        self.overshoot_gain      = overshoot_gain
        self.set_reference(reference)

    def reset(self):
        self.total_error = 0.0
        self.main_control.reset()
        [control.reset() for control in self.low_levels_controls]

    def set_reference(self, new_reference):
        self.reference        = new_reference
        self.initial_err_sign = signum(self.reference)
        self.max_overshoot    = 0.0
        self.current_e        = 0.0
        self.main_control.set_reference(self.reference)

    def get_main_observation(self, observation):
        return observation[0]

    def get_actions(self, observation, info):
        """
        Returns all the actions to apply to actuators
        :param observation:
        :param info:
        :return:
        """
        second_reference = self.get_second_reference(observation)
        if len(self.low_levels_controls) == 0:
            actions = self.get_actions_from_output(second_reference)
        else:
            actions = {}
            for control in self.low_levels_controls:
                control.set_reference(second_reference)
                actions.update(control.get_actions(observation, info))

        self.add_to_cost(self.main_control.e)

        return actions

    def get_second_reference(self, observation):
        main_observation = self.get_main_observation(observation)
        second_reference = self.main_control.get_output(self.reference, main_observation)
        return second_reference

    def get_actions_from_output(self, output):
        return {self.action_name: output}

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
