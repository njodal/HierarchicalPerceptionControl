import FuzzyControl
import signals as sg
import auto_tune as at
import unit_test as ut


class GenericControlUnit(object):
    """
    Abstract class to Control Units like PID or PCU
    """
    type     = None
    k_type   = 'type'
    k_dt     = 'dt'
    k_min_dt = 'min_dt'

    @staticmethod
    def get_class(control_unit_type):
        """
        Given a control_unit_type ('PID', 'PCU', etc.), returns the corresponding class
        :param control_unit_type: :type String
        :return:
        """
        for cls in GenericControlUnit.__subclasses__():
            if cls.type == control_unit_type:
                return cls
            # ToDo: find a way to avoid using get_subclass
            cls = GenericControlUnit.get_subclass(control_unit_type, cls)
            if cls is not None:
                return cls

        return None

    @staticmethod
    def get_subclass(control_unit_type, super_class):
        """
        Given a control_unit_type and a super_class, returns the corresponding class searching down in the hierarchy
        :param control_unit_type:
        :param super_class:
        :return:
        """
        for cls in super_class.__subclasses__():
            if cls.type == control_unit_type:
                return cls
            cls = GenericControlUnit.get_subclass(control_unit_type, cls)
            if cls is not None:
                return cls

        return None

    def __init__(self, control_params, dt=0.1, min_dt=0.0001, state=None):
        """
        init
        :param control_params: dict with control definitions
        :param dt:     interval between outputs
        :param min_dt: minimum dt to be useful, if less than this value a dt of 0.0 is assumed
        :param state:  dict which can have some parameters values
        """
        # to avoid warnings
        self.o  = 0.0  # output
        self.r  = 0.0  # reference (or set point)
        self.e  = 0.0  # error: difference between reference and perception (in that order)
        self.p  = 0.0  # perception

        # common parameters
        self.control_params = control_params
        self.key        = self.control_params.get('key', 'NoName')   # name, used for debugging
        self.max_change = get_param(self.control_params, 'max_change', 0.0, state=state)  # output cannot change faster
        self.bounds     = get_param(self.control_params, 'bounds', [], state=state)       # min and max values of output
        self.lag        = get_param(self.control_params, 'lag', 0.0, state=state)         # cycle needed to react
        self.dt         = self.control_params.get(self.k_dt, dt)                         # default interval control time
        self.min_dt     = self.control_params.get(self.k_min_dt, min_dt)                  # do not react to dt smaller
        self.min_error  = get_param(self.control_params, 'min_error', 0.0, state=state)   # don't react to smaller error
        self.debug      = get_param(self.control_params, 'debug', False, state=state)     # print controller info
        # print('  bounds for %s:%s' % (self.key, self.bounds))

        self.reference_changed = True

        # initialization
        self.reset()

        if self.debug:
            print('%s params: %s' % (self.type, self.parm_string()))

    def update(self, new_p, dt=0.0):
        """
        Given a p (perception) updates the new o (output)
        :param new_p:   new perception
        :param dt:  interval since last perception
                      * 0.0 means caller is not providing time so the predefined one is used
                      * if > 0.0 the predefined one is also changed
        :return: output
        """
        if dt > self.min_dt:
            # updates the predefined one
            self.dt = dt

        p_speed    = (new_p - self.p)/self.dt
        p_expected = new_p + p_speed*self.lag  # predictive value of p after lag (assuming speed remains the same)
        self.p     = new_p
        new_error  = self.r - p_expected
        if abs(new_error) < self.min_error:
            new_error = 0.0

        new_o = self.calc_output(new_error)

        if self.max_change > 0.00001:
            # output change is bounded, cannot change too much
            # old_o = self.o
            self.o = sg.bound_value(new_o, [self.o - self.max_change, self.o + self.max_change])
            # print('old_o:%.2f max:%.2f new_o:%.2f self.o:%.2f' % (old_o, self.max_change, new_o, self.o))
        else:
            self.o = new_o

        self.o = sg.bound_value(self.o, self.bounds)

        self.reference_changed = False

        if self.debug:
            more = self.more_info_for_debug()
            print('o:%.3f r:%.3f p:%.3f e:%.3f %s' % (self.o, self.r, new_p, new_error, more))

        # print('e:%s p:%s o:%s g:%s bounds:%s key:%s' % (self.e, p, self.o, self.g, self.bounds, self.key))
        return self.o

    def calc_output(self, _):
        """
        Abstract method, each particular controller must provide it own method
        :param _:
        :return:
        """
        return 0.0

    def get_output(self, r, p, dt=0.0, bounds=None):
        """
        Just a convenient form go get the Output (most of the time a reference is set and later an update is called)
        :param r:  reference value
        :param p:  perception value (input)
        :param dt:
        :param bounds:
        :return:   output
        """
        self.set_bounds(bounds)
        self.set_reference(r)
        return self.update(p, dt)

    def output(self):
        return self.o

    def set_output(self, new_output):
        """
        Hard setting of output (instead of calculating by the controller)
        ex: someone moved the throttle pedal in a car
        :param new_output:
        :return:
        """
        self.o = new_output

    def adjust_output(self, status):
        """
        Given a dict (status), adjust output if value is in dict
        :param status:
        :return:
        """
        if self.key in status:
            self.set_output(status[self.key])

    def set_reference(self, r, precision=0.01):
        if abs(self.r - r) > precision:
            self.reference_changed = True
        self.r = r

    def set_bounds(self, bounds):
        """
        Set new max and min value for output
        :param bounds:
        :return:
        """
        if bounds is not None:
            self.bounds = bounds  # min/max values for output ([] = no bounds)

    def reset(self):
        """
        Reset the Controller
        :return:
        """
        self.o = 0.0
        self.e = 0.0
        self.reference_changed = True
        self.reset_specific()

    def reset_specific(self):
        """
        Abstract method
        Many controllers have specific requirements at reset time, so this method is provided
        :return: None
        """
        pass

    def get_parameters(self):
        """
        Abstract method, returns the ControlUnit parameters that can be tuned
        :return: dict with controller's parameters
        """
        return {}

    def set_parameters(self, parameters):
        """
        Abstract method, sets a new parameters
        :param parameters:
        :return:
        """
        return {}

    def get_params_as_widgets(self):
        """
        Returns params as a dict used to be display in a WinDeklar form, useful to interactively change param values to
        see the effect of changes
        :return:
        """
        return [widget_from_param(par_name, par_value) for par_name, par_value in self.get_parameters().items()]

    def more_info_for_debug(self):
        """
        Abstract method, returns particular info for debug
        :return:
        """
        return ''

    def parm_string(self):
        """
        Used to print controller's parameters
        :return:
        """
        return dict_to_label(self.get_parameters())


class PID(GenericControlUnit):
    """
    PID controller as defined in http://en.wikipedia.org/wiki/PID_controller but many enhancements like:
     * bounds for output values (in its value or in its rate of change),
     * protection against integrator windup
        see: https://controlguru.com/integral-reset-windup-jacketing-logic-and-the-velocity-pi-form/
    """

    type    = 'PID'
    kp_key  = 'p'
    ki_key  = 'i'
    kd_key  = 'd'
    k_gains = 'gains'
    k_i_windup_key = 'i_windup'
    k_i_length     = 'integrator_length'
    k_i_reset      = 'integrator_reset'

    def __init__(self, control_params, state=None, integrator_length=0, integrator_windup=300.0,
                 integrator_reset=False):
        """
        Create a PID controller
        :param integrator_length: number of last values to be taken in integration
        :param integrator_windup: max value (positive and negative) the integrator can have
        :param integrator_reset:  if True means the integrator value is to 0 every time the reference values changes
        """
        check_mandatory_param(self.k_gains, control_params, self.type)

        gains = get_param(control_params, self.k_gains, [], state=state)
        self.k_p = gains[0]
        self.k_i = gains[1] if len(gains) > 1 else 0.0
        self.k_d = gains[2] if len(gains) > 2 else 0.0

        self.i_windup          = control_params.get(self.k_i_windup_key, integrator_windup)
        self.integrator_length = control_params.get(self.k_i_length, integrator_length)
        self.integrator_reset  = control_params.get(self.k_i_reset, integrator_reset)

        self.first_time = True
        super(PID, self).__init__(control_params, state=state)

        # there are two kind of integrators, the first one is a traditional one (just summing all values)
        # and the second one just take in count the lasts (integrator_length) values
        self.integrator        = 0.0
        self.integrator_values = sg.SignalHistory(length=self.integrator_length)

        self.p_value = 0.0
        self.i_value = 0.0
        self.d_value = 0.0

    def calc_output(self, new_error):
        delta_error = new_error - self.e
        self.e = new_error

        self.p_value = self.k_p * self.e
        if self.first_time:
            self.first_time = False
            return self.p_value

        self.set_integrator(self.e * self.dt)
        self.i_value = self.k_i * self.integrator

        self.d_value = self.k_d * delta_error / self.dt if self.dt > self.min_dt else 0.0

        o = self.p_value + self.i_value + self.d_value

        return o

    def set_integrator(self, value):
        """
        Given the new perception (value), update the integrator taken in count all the integrator parameters
        :param value: perception
        :return: None, but updates integrator
        """
        if abs(self.k_i) < 0.0001:
            return

        if self.integrator_reset and self.reference_changed:
            self.integrator = 0.0
        elif self.integrator_values.get_len() > 1:
            self.integrator_values.append(value)
            self.integrator = self.integrator_values.sum()
        else:
            self.integrator += value
        self.integrator = sg.bound_value(self.integrator, [-self.i_windup, self.i_windup])
        if self.debug:
            print('   integrator:%.3f v:%.3f (e:%.3f dt:%s, key:%s)' % (self.integrator, value, self.e, self.dt,
                                                                        self.key))

    def reset_specific(self):
        self.integrator        = 0.0
        self.integrator_values = sg.SignalHistory(length=self.integrator_length)

    def get_parameters(self):
        return {self.kp_key: self.k_p, self.ki_key: self.k_i, self.kd_key: self.k_d}

    def set_parameters(self, parameters):
        self.k_p = parameters.get(self.kp_key, self.k_p)
        self.k_i = parameters.get(self.ki_key, self.k_i)
        self.k_d = parameters.get(self.kd_key, self.k_d)
        self.i_windup = parameters.get(self.k_i_windup_key, self.i_windup)


class IncrementalPID(GenericControlUnit):
    """
    Incremental PID as defined in https://d-nb.info/1208071297/34
    Basic idea: output change is a function of error, derivative and second derivative of error
    """

    type   = 'IncrementalPID'
    kp_key = 'p'
    ki_key = 'i'
    kd_key = 'd'
    k_gains = 'gains'

    def __init__(self, control_params, state=None):
        check_mandatory_param(self.k_gains, control_params, self.type)

        gains = get_param(control_params, self.k_gains, [], state=state)
        self.k_p = gains[0]
        self.k_i = gains[1] if len(gains) > 1 else 0.0
        self.k_d = gains[2] if len(gains) > 2 else 0.0

        self.last_e      = 0.0
        self.last_last_e = 0.0
        self.first_time = True
        super(IncrementalPID, self).__init__(control_params, state=state)

        self.p_value = 0.0
        self.i_value = 0.0
        self.d_value = 0.0

    def calc_output(self, new_error):
        self.e = new_error

        self.p_value = self.k_p * (self.e - self.last_e)
        self.i_value = self.k_i * self.e
        self.d_value = self.k_d * (self.e - 2*self.last_e + self.last_last_e)

        delta_o = self.p_value + self.i_value + self.d_value
        o       = self.o + delta_o

        if self.debug:
            print('   e:%.3f last_e:%.3f last_last_e:%.3f d_o:%.3f' % (self.e, self.last_e, self.last_last_e, delta_o))

        self.last_last_e = self.last_e
        self.last_e      = self.e

        return o

    def reset_specific(self):
        self.last_e = 0.0
        self.last_last_e = 0.0

    def get_parameters(self):
        return {self.kp_key: self.k_p, self.ki_key: self.k_i, self.kd_key: self.k_d}

    def set_parameters(self, parameters):
        self.k_p = parameters.get(self.kp_key, self.k_p)
        self.k_i = parameters.get(self.ki_key, self.k_i)
        self.k_d = parameters.get(self.kd_key, self.k_d)


class P(PID):
    """
    Just a Proportional controller (it is implemented over a PID with just the proportional gain available)
    The only difference with a PID with I and D in 0.0 is the only gain that can be used in twiddle is P
    """
    type = 'P'
    k_p  = 'gain'

    def __init__(self, control_params, state=None):
        check_mandatory_param(self.k_p, control_params, self.type)

        kp = get_param(control_params, self.k_p, 1.0, state=state)
        control_params[self.k_gains] = [kp]
        super(P, self).__init__(control_params, state=state)

    def get_parameters(self):
        return {self.kp_key: self.k_p}

    def set_parameters(self, parameters):
        self.k_p = parameters.get(self.kp_key, self.k_p)


class PCU(GenericControlUnit):
    """
    Perceptual Control Unit (PCU) as used in Perceptual Control Theory:
        o = o + (kg*e - o)/ks
    """

    type = 'PCU'
    k_g  = 'g'
    k_s  = 's'

    def __init__(self, control_params, state=None):
        check_mandatory_param(self.k_g, control_params, self.type)

        self.kg = get_param(control_params, self.k_g, 1.0, state=state)
        self.ks = get_param(control_params, self.k_s, 1.0, state=state)

        super(PCU, self).__init__(control_params, state=state)

    def get_output(self, reference, perception, dt=0.0, bounds=None, min_ks=0.01):
        self.e = reference - perception

        # to avoid dividing by zero
        if 0.0 <= self.ks < min_ks:
            ks = min_ks
        elif -min_ks < self.ks <= 0.0:
            ks = -min_ks
        else:
            ks = self.ks

        self.o += (self.kg * self.e - self.o) / ks
        if len(self.bounds) > 0:
            if self.o < self.bounds[0]:
                self.o = self.bounds[0]
            elif self.o > self.bounds[1]:
                self.o = self.bounds[1]
        if self.debug:
            print('        %s r:%.2f p:%.2f e:%.2f o:%.4f' % (self.key, reference, perception, self.e, self.o))
        return self.o

    def get_parameters(self):
        return {self.k_g: self.kg, self.k_s: self.ks}

    def set_parameters(self, parameters):
        self.kg = parameters.get(self.k_g, self.kg)
        self.ks = parameters.get(self.k_s, self.ks)

    def set_kg(self, new_kg):
        self.kg = new_kg

    def set_ks(self, new_ks):
        self.ks = new_ks


class BangBang(GenericControlUnit):
    """
    BangBang controller as defined in https://en.wikipedia.org/wiki/Bang%E2%80%93bang_control
        output can have only two values (typically On or Off)
    """
    type             = 'BangBang'
    bellow_value_key = 'bellow_value'
    above_value_key  = 'above_value'
    hysteresis_key   = 'hysteresis'

    def __init__(self, control_params, state=None, bellow_value=1, above_value=0, hysteresis=0.0):
        """
        :param bellow_value:  output when p is bellow reference
        :param above_value:   output when p is above reference
        :param hysteresis:    range from reference in which send the last output
        """
        self.bellow_value = control_params.get(self.bellow_value_key, bellow_value)
        self.above_value  = control_params.get(self.above_value_key, above_value)
        self.hysteresis   = control_params.get(self.hysteresis_key, hysteresis)

        super(BangBang, self).__init__(control_params, state=state)

    def calc_output(self, new_error):
        self.e = new_error

        if self.hysteresis < 0:
            low  = self.r + self.hysteresis
            high = self.r
        else:
            low  = self.r
            high = self.r + self.hysteresis

        p = self.r - self.e
        if p <= low:
            o = self.bellow_value
        elif p > high:
            o = self.above_value
        else:
            o = self.o
        # print '  low:%s high:%s p:%s o:%s' % (low, high, p, o)
        return o

    def get_parameters(self):
        return {'bellow': self.bellow_value, 'above': self.above_value, 'hysteresis': self.hysteresis}

    def set_parameters(self, parameters):
        self.bellow_value = parameters.get('bellow',     self.bellow_value)
        self.above_value  = parameters.get('above',      self.above_value)
        self.hysteresis   = parameters.get('hysteresis', self.hysteresis)


class LinealControlUnit(GenericControlUnit):
    """
    A lineal controller: the output is a lineal function of reference
    """
    type       = 'Lineal'
    k_gain     = 'gain'
    k_i_bounds = 'input_bounds'

    def __init__(self, control_params, state=None):
        self.gain         = control_params.get(self.k_gain, 1.0)
        self.input_bounds = control_params.get(self.k_i_bounds, [])
        super(LinealControlUnit, self).__init__(control_params, state=state)

    def update(self, new_p, dt=0.0):
        bounded_r = sg.bound_value(self.r, self.input_bounds)
        self.o    = sg.bound_value(self.gain*bounded_r, bounds=self.bounds)
        if self.debug:
            print('o:%.3f r:%.3f' % (self.o, self.r))
        return self.o

    def get_parameters(self):
        return {self.k_gain: self.gain, self.k_i_bounds: self.input_bounds}

    def set_parameters(self, parameters):
        self.gain = parameters.get('gain', self.gain)


class FuzzyController(GenericControlUnit):
    """
    Implements a fuzzy controller which is defined in a yaml file
    """

    type        = 'Fuzzy'
    k_file_name = 'file_name'
    k_dir_name  = 'dir'

    def __init__(self, control_params, def_dir_name='fuzzy_rules', state=None):
        check_mandatory_param(self.k_file_name, control_params, self.type)

        file_name             = control_params[self.k_file_name]
        dir_name              = control_params.get(self.k_dir_name, def_dir_name)
        self.fuzzy_controller = FuzzyControl.FuzzyControl.create_from_file(file_name, directory=dir_name)

        self.last_e  = 0.0
        self.delta_e = 0.0  # stored for debug
        super(FuzzyController, self).__init__(control_params, state=state)

    def calc_output(self, new_error):
        self.e        = new_error
        self.delta_e  = self.e - self.last_e
        out_dict      = self.fuzzy_controller.compute({'error': self.e, 'delta_error': self.delta_e})
        defuzz_output = next(iter(out_dict.values()))  # assumed first value in dict is the corresponding output
        self.last_e   = self.e
        return self.o + defuzz_output if self.fuzzy_controller.delta else defuzz_output

    def reset_specific(self):
        self.last_e = 0.0

    def more_info_for_debug(self):
        return 'delta_e:%.2f' % self.delta_e


class AdaptiveControlUnit(GenericControlUnit):
    """
    Adaptive control based on 'Artificial Cerebellum' by Bill Powers
    https://www.iapct.org/themes/biology-neuroscience/an-artificial-cerebellum-adaptive-stabilization-of-a-control-system/
    (and also from Rupert Young PhD Thesis
    """

    type            = 'AdaptiveP'
    k_gain          = 'gain'
    k_learning_rate = 'learning_rate'
    k_max_change    = 'max_change'
    k_decay_rate    = 'decay_rate'
    k_past_length   = 'past_length'

    def __init__(self, control_params, state=None, gain=1.0, learning_rate=0.01, decay_rate=0.0, past_length=10,
                 initial_weights=(1.0, 0.0)):
        self.gain          = gain
        self.learning_rate = learning_rate
        self.decay_rate    = decay_rate
        self.past_length   = past_length

        super(AdaptiveControlUnit, self).__init__(control_params, state=state)
        self.set_constants()
        self.past_errors = sg.SignalHistory(length=self.past_length)
        self.weights     = [0.0 for _ in range(self.past_length+1)]
        for i, v in enumerate(initial_weights):
            self.weights[i] = v
        self.reference_changed = True

    def calc_output(self, new_error):
        self.past_errors.append(new_error)
        self.adjust_weights(new_error)

        weighted_sum = self.past_errors.weighted_sum(self.weights, lifo_order=True)
        self.o       = self.gain*weighted_sum
        self.e       = new_error

        if self.debug:
            print('   o:%.3f w_sum:%.3f e:%.3f r:%.3f p:%.3f' % (self.o, weighted_sum, self.e, self.r, self.p))
        return self.o

    def adjust_weights(self, new_error):
        for i, e in enumerate(self.past_errors.get_items_in_lifo_order()):
            change = self.learning_rate*new_error*e
            self.weights[i] += change
            self.weights[i] -= self.weights[i] * self.decay_rate
            # if self.weights[i] < 0.0:
            #    self.weights[i] = 0.0
            if self.debug:
                print('      i:%s e:%.3f w:%.3f v:%.3f change:%.3f' % (i, e, self.weights[i], self.weights[i]*e,
                                                                       change))

    def get_parameters(self):
        return {self.k_gain: self.gain, self.k_past_length: self.past_length, self.k_learning_rate: self.learning_rate,
                self.k_decay_rate: self.decay_rate}

    def set_parameters(self, parameters):
        for k, v in parameters.items():
            if not k not in self.control_params:
                continue
            if k == 'past_length':
                v = int(v)
            self.control_params[k] = v
        self.set_constants()

    def set_constants(self):
        self.gain          = self.control_params.get(self.k_gain, self.gain)
        self.learning_rate = self.control_params.get(self.k_learning_rate, self.learning_rate)
        self.max_change    = self.control_params.get(self.k_max_change, self.max_change)
        self.decay_rate    = self.control_params.get(self.k_decay_rate, self.decay_rate)
        self.past_length   = self.control_params.get(self.k_past_length, self.past_length)


def check_mandatory_param(parameter_key, params, controller_type):
    if parameter_key not in params:
        raise Exception(missing_parameter_msg(parameter_key, controller_type))


def missing_parameter_msg(parameter_key, controller_type):
    return 'Parameter "%s" must be present for type %s' % (parameter_key, controller_type)


def create_control(control_params, state=None):
    if GenericControlUnit.k_type not in control_params:
        raise Exception('No type defined for controller %s' % control_params)
    control_type       = control_params[GenericControlUnit.k_type]
    control_unit_class = GenericControlUnit.get_class(control_type)
    if control_unit_class is None:
        raise Exception('Control type %s is not implemented' % control_type)
    # print('Create %s ' % control_unit_class)
    return control_unit_class(control_params, state=state)


def widget_from_param(par_name, par_value, default_type='EditNumberSpin'):
    values = {'name': par_name, 'value': par_value, 'type': default_type, 'params': {'step': 0.1}}
    return {'widget': values}


class AutoTuneControl(at.AutoTuneFunction):
    """
    Auto tune a given ControlUnit to a given step response
    """

    def __init__(self, control, reference=0.0, disturbance=0.0, dt=0.1, max_iter=500, change=1.0, output_lag=0,
                 overshoot_cost=2.0, overshoot_max_cost=10.0, debug=False):
        self.dt            = dt
        self.output_lag    = output_lag
        self.control       = control
        self.reference     = reference
        self.disturbance   = disturbance
        self.over_cost     = overshoot_cost
        self.over_max_cost = overshoot_max_cost
        self.debug         = debug

        super(AutoTuneControl, self).__init__(max_iter=max_iter, change=change)

    def get_parameters(self):
        return self.control.get_parameters()

    def set_parameters(self, parameters):
        return self.control.set_parameters(parameters)

    def run_one_episode(self, parameters):
        self.control.set_parameters(parameters)
        self.control.reset()
        initial_error_sign  = signum(self.reference)
        error_cost          = 0.0
        overshoot_error     = 0.0
        max_overshoot_value = 0.0
        for [p, o] in step_response_values(self.reference, self.disturbance, self.max_iter, self.dt, self.debug,
                                           self.control):
            abs_error   = abs(self.control.e)
            error_cost += abs_error * self.dt
            if initial_error_sign != signum(self.control.e):
                # there's an overshoot
                overshoot_error += abs_error * self.dt
                if abs_error > max_overshoot_value:
                    # new overshoot reached
                    max_overshoot_value = abs_error
        return error_cost + self.over_cost*overshoot_error + self.over_max_cost*max_overshoot_value

    def run_function_with_parameters(self, parameters):
        cost = self.run_one_episode(parameters)
        # print('  run, parameters: %s cost: %.2f' % (parameters, cost))
        return cost


def step_response_values(r, d, times, dt, debug, control_unit, max_output_change=5.0, control_delta=False):
    """
    Very basic model to simulate a system that react to an output force (with disturbance)
    :param control_delta: whether control output directly or it's change
    :param r:       reference value the perception must reach
    :param d:       disturbance force
    :param times:   number of cycles to run the simulation
    :param dt:      interval between cycles
    :param debug:
    :param control_unit: controller to use
    :param max_output_change: max change per second the actuator (output) is able to execute
    :return:
    """
    max_output_change_in_cycle = max_output_change*dt
    current_o = 0.0
    p         = 0.0
    for i in range(0, times):
        controller_o = control_unit.get_output(r, p)
        if control_delta:
            delta_o    = max(-max_output_change_in_cycle, min(controller_o, max_output_change_in_cycle))
            current_o += delta_o
            # print(f'  delta o: {delta_o}')
        else:
            current_o = controller_o
        p = simple_change_model(p, current_o, d, dt)
        if debug:
            print(f'  {i}: p={p} o={current_o} ')
        yield p, current_o


def simple_change_model(p, o, d, dt):
    """
    Pretty basic model where the new perception is just the old one plus its derivative (with disturbance)
    :param p:   perception
    :param o:   output (value to send to the actuator)
    :param d:   disturbance (external value that influence in the derivative, like a slope in a road)
    :param dt:  delta time
    :return:    new perception after applying o and d
    """
    return p + (o + d)*dt


class DeConvolution:
    def __init__(self, learning_rate=0.01, decay_rate=0.0, past_length=10, max_change=0.01, debug=False):
        self.debug   = debug
        self.o       = 0.0
        self.error   = 999.0  # any big
        self.past_length   = past_length
        self.learning_rate = learning_rate
        self.max_change    = max_change
        self.decay_rate    = decay_rate
        self.past_inputs   = sg.SignalHistory(length=self.past_length)
        self.weights       = [0.0 for _ in range(self.past_length+1)]

    def get_output(self, input_value, output_value):
        if self.debug:
            print('   input:%s output:%s' % (input_value, output_value))
        self.o = self.past_inputs.weighted_sum(self.weights, lifo_order=True)
        self.adjust_weights(input_value, output_value, self.o)
        new_error = abs(output_value - self.o)
        if self.debug:
            improve = 'Good' if new_error < self.error else 'Bad'
            print('     o:%.3f error:%.4f %s' % (self.o, output_value - self.o, improve))
        self.error = new_error
        return self.o

    def adjust_weights(self, input_value, actual_output_value, estimated_output_value):
        self.past_inputs.append(input_value)
        delta_output_value = actual_output_value - estimated_output_value
        for i, old_input in enumerate(self.past_inputs.get_items_in_lifo_order()):
            change = self.learning_rate*delta_output_value*signum(old_input)
            if abs(change) > self.max_change:
                change = self.max_change if change >= 0.0 else - self.max_change

            self.weights[i] += change
            self.weights[i] = self.weights[i]*(1 - self.decay_rate)
            # if self.weights[i] < 0.0:
            #    self.weights[i] = 0.0
            if self.debug:
                print('      i:%s in:%.3f w:%.3f change:%.3f' % (i, old_input, self.weights[i], change))


def signum(number, min_v=0.000001):
    if number < - min_v:
        return -1
    elif number > min_v:
        return 1
    else:
        return 0


def dict_to_label(values, exclude_keys=(), max_length=80):
    message = ''
    for k, v in values.items():
        if len(message) > max_length:
            break
        if k in exclude_keys:
            continue
        v1 = '%.2f' % v if isinstance(v, float) else v
        message += '%s: %s ' % (k, v1)
    return message


def get_param(params, key, default, state=None):
    """
    Returns params[key], but with some exceptions:
        * if key not in params returns the default value
        * if params[key] is a string and its in state, returns state[value]
    This is useful for setting parameters, ex:
        params{gains: [kp, 1.0, 0.1] ... }
        state(kp: 2.0, ... }
        it will return 2.0
    :param params:
    :param key:
    :param state:
    :param default:
    :return:
    """
    if key not in params:
        return default

    value = params[key]
    if state is not None:
        if isinstance(default, list):
            value = [state.get(item, default) if isinstance(item, str) else item for item in params[key]]
        elif isinstance(value, str):
            value = state.get(value, default)
    return value


# Tests
def test_create_control(control_params):
    control_unit = create_control(control_params)
    return control_unit.key


def test_control(values, control_params):
    control_unit = create_control(control_params)
    o = 0.0
    for [r, p] in values:
        o = control_unit.get_output(r, p)
    return o


def test_step_response(r, d, times, dt, debug, control_params):
    control_unit = create_control(control_params)
    p_last = 0.0
    for [p, o] in step_response_values(r, d, times, dt, debug, control_unit):
        p_last = p
    return p_last


def test_deconvolution(function_name, max_iter, learning_rate, decay_rate, max_change, past_length, debug):
    deconvolution = DeConvolution(learning_rate=learning_rate, decay_rate=decay_rate, past_length=past_length,
                                  max_change=max_change, debug=debug)
    error = 999.0  # any big
    for i in range(max_iter):
        actual_input, actual_output = call_function(function_name, i)
        estimated_output = deconvolution.get_output(actual_input, actual_output)
        error = actual_output - estimated_output
    return error


def lineal2(i):
    return i, i*2


def call_function(func_name, i):
    func = globals().get(func_name)
    if func is not None and callable(func):
        return func(i)
    else:
        raise Exception("%s is not implemented" % func_name)


if __name__ == "__main__":
    ut.UnitTest(__name__, 'tests/ControlUnit.test', '')
