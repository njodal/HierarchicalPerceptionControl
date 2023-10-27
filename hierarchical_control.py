
import yaml_functions as yaml
from ControlUnit import signum, create_control
import unit_test as ut


class HierarchicalControl:
    """
    Handle a hierarchical control as defined in a given file
    """
    k_references = 'references'
    k_reference  = 'reference'
    k_sensors    = 'sensors'
    k_sensor     = 'sensor'
    k_actuators  = 'actuators'
    k_actuator   = 'actuator'
    k_parameters = 'parameters'
    k_parameter  = 'parameter'
    k_controls   = 'controls'
    k_control    = 'control'
    k_name       = 'name'
    k_value      = 'value'
    k_definition = 'definition'
    k_output     = 'output'
    k_autotune   = 'autotune'

    def __init__(self, file_name, dir_name, initial_parameters=None):

        # to avoid warnings
        self.total_error      = 0.0
        self._state           = {}   # keeps all values
        self._controls        = []
        self.initial_err_sign = 0
        self.max_overshoot    = 0.0
        self.current_e        = 0.0

        self.hc_def = yaml.get_yaml_file(file_name, directory=dir_name)

        self.actuator_names  = get_item_names(self.k_name, self.k_actuators, self.k_actuator, self.hc_def)
        self.parm_names      = get_item_names(self.k_name, self.k_parameters, self.k_parameter, self.hc_def)
        self.reference_names = get_item_names(self.k_name, self.k_references, self.k_reference, self.hc_def)
        # self.autotune_names  = get_item_names(self.k_name, self.k_autotune, self.k_parameter, self.hc_def)

        self.reset(initial_parameters=initial_parameters)

    def reset(self, initial_parameters=None):
        self._state = {}
        for [group, item] in [[self.k_references, self.k_reference], [self.k_sensors, self.k_sensor],
                              [self.k_actuators, self.k_actuator], [self.k_parameters, self.k_parameter]]:
            self.update_state_with_definition(group, item, self.hc_def)
        if initial_parameters is not None:
            self._state.update(initial_parameters)
        self.create_controls()  # must go after init_state to property init controllers

    def create_controls(self):
        self._controls = [ControlUnit(control_def, self._state) for control_def in
                          get_item_def(self.k_controls, self.k_control, self.hc_def)]

    def update_state_with_definition(self, group_name, item_name, definition):
        for item in get_item_def(group_name, item_name, definition):
            self._state[item[self.k_name]] = item.get(self.k_value, 0.0)

    def set_value(self, name, value):
        self._state[name] = value

    def get_value(self, name):
        return self._state[name]

    def set_reference(self, reference_name, new_reference):
        if reference_name not in self.reference_names:
            raise Exception('Reference name "%s" not known, must be one of %s' % (reference_name, self.reference_names))
        self.set_value(reference_name, new_reference)
        self.initial_err_sign = signum(new_reference)
        self.max_overshoot    = 0.0
        self.current_e        = 0.0
        for control in self._controls:
            if control.reference_name == reference_name:
                control.set_reference(new_reference)

    def get_actuators(self, new_sensor_values):
        """
        Returns the actuators values after applying the new sensor values through all the controls
        :param new_sensor_values:
        :return:
        """
        # print('   new sensor values: %s' % new_sensor_values)
        for k, v in new_sensor_values.items():
            self._state[k] = v
        [control.run(self._state) for control in self._controls]
        return self.get_actuator_values()

    def get_actions(self, new_sensor_values, _):
        """
        Same as get_actuators, just to be compatible with OpenAI Gym terminology
        :param new_sensor_values:
        :return:
        """
        return self.get_actuators(new_sensor_values)

    def get_actuator_values(self):
        return {name: self._state[name] for name in self.actuator_names}

    def get_parameters(self):
        # ToDo: to be implemented
        return {}

    def set_parameters(self, new_parameters):
        # ToDo: to be implemented
        return None

    def get_total_cost(self):
        return self.total_error

    def get_last_error(self):
        return self._controls[0].get_last_error()

    def parm_string(self):
        return self._controls[0].parm_string()


class ControlUnit:
    """
    A control unit used in HierarchicalControl
    """
    def __init__(self, control_def_all, state):
        control_def         = control_def_all[HierarchicalControl.k_definition]
        control_def['key']  = control_def_all.get(HierarchicalControl.k_name, 'NoName')
        # print('  control def: %s' % control_def)
        self.control        = create_control(control_def, state=state)
        self.sensor_name    = control_def_all.get(HierarchicalControl.k_sensor, None)
        self.output_name    = control_def_all.get(HierarchicalControl.k_output, None)
        self.reference_name = control_def_all.get(HierarchicalControl.k_reference, 'NoRef')
        reference_value     = state.get(self.reference_name, 0.0)
        # print('   ref_name: %s sensor_name: %s output_name: %s' %
        #      (self.reference_name, self.sensor_name, self.output_name))
        self.control.set_reference(reference_value)

    def reset(self):
        self.control.reset()

    def set_reference(self, new_reference):
        self.control.set_reference(new_reference)

    def run(self, state):
        reference_value         = state.get(self.reference_name, 0.0)
        sensor_value            = state.get(self.sensor_name, 0.0)
        output_value            = self.control.get_output(reference_value, sensor_value)
        state[self.output_name] = output_value

    def get_last_error(self):
        return self.control.e

    def parm_string(self):
        return self.control.parm_string()


class BaseHierarchicalControl(object):
    """
    Base class for all Hierarchical Controls
    """

    def __init__(self, main_control, low_levels_controls=(), reference=0.0, overshoot_gain=1.0, action_name='action'):
        """

        :param main_control:        higher level controller
        :param low_levels_controls: lower levels controllers
        :param reference:           reference value for main_control
        :param overshoot_gain:      penalty gain for overshoot, used in autotune
        :param action_name:         name of the action to be applied for the main controller, only used in 1 level
        """
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
        Returns all the actions to apply to actuators given an observation from sensors
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

        self.add_to_cost(self.get_current_cost())

        return actions

    def get_second_reference(self, observation):
        main_observation = self.get_main_observation(observation)
        second_reference = self.main_control.get_output(self.reference, main_observation)
        return second_reference

    def get_actions_from_output(self, output):
        return {self.action_name: output}

    def get_current_cost(self):
        return self.main_control.e

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


def get_item_def(group_name, item_name, definition):
    if group_name in definition:
        for item in definition[group_name]:
            yield item[item_name]


def get_item_names(k_name, k_group, k_item, definition):
    return [item[k_name] for item in get_item_def(k_group, k_item, definition)]


# tests
def test_hc(file_name, dir_name):
    hc = HierarchicalControl(file_name, dir_name)
    print('state: %s' % hc._state)
    print('actuator names: %s' % hc.actuator_names)
    return len(hc._state), len(hc._controls)


def test_get_items(file_name, dir_name, group, item):
    h_def = yaml.get_yaml_file(file_name, directory=dir_name)
    items = [item for item in get_item_def(group, item, h_def)]
    print(items)
    return len(items)


if __name__ == "__main__":
    ut.UnitTest(__name__, 'tests/hierarchical_control.test', '')
