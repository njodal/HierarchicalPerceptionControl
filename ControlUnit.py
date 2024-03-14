import FuzzyControl
import signals as sg
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
        self.key        = control_params.get('key', 'NoName')
        self.min_error  = get_param(control_params, 'min_error', 0.0, state=state)
        self.max_change = get_param(control_params, 'max_change', 0.0, state=state)
        self.lag        = get_param(control_params, 'lag', 0.0, state=state)
        self.bounds     = get_param(control_params, 'bounds', [], state=state)
        self.dt         = control_params.get(self.k_dt, dt)
        self.min_dt     = control_params.get(self.k_min_dt, min_dt)
        self.debug      = get_param(control_params, 'debug', False, state=state)
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


class AdaptiveControlUnit:
    """
    Adaptive control based on 'Artificial Cerebellum' by Bill Powers
    https://www.iapct.org/themes/biology-neuroscience/an-artificial-cerebellum-adaptive-stabilization-of-a-control-system/
    (and also from Rupert Young PhD Thesis
    """

    type = 'AdaptiveP'

    def __init__(self, name, gain=1.0, learning_rate=0.01, decay_rate=0.0, past_length=10, max_change=0.01,
                 debug=False):
        self.name          = name
        self.gain          = gain
        self.learning_rate = learning_rate
        self.max_change    = max_change
        self.decay_rate    = decay_rate
        self.past_length   = past_length
        self.past_errors   = sg.SignalHistory(length=self.past_length)
        self.weights       = [0.0 for _ in range(self.past_length+1)]
        self.reference_changed = True
        self.debug         = debug

        self.r = 0.0
        self.o = 0.0
        self.e = 0.0
        self.p = 0.0

    def set_reference(self, r, precision=0.01):
        if abs(self.r - r) > precision:
            self.reference_changed = True
        self.r = r

    def get_output(self, reference, perception):
        self.r = reference
        self.p = perception
        self.e = self.r - self.p
        self.past_errors.append(self.e)

        self.adjust_weights()

        weighted_sum = self.past_errors.weighted_sum(self.weights, lifo_order=True)
        self.o       = self.gain*weighted_sum

        if self.debug:
            print('   o:%.3f w_sum:%.3f e:%.3f r:%.3f p:%.3f' % (self.o, weighted_sum, self.e, self.r, self.p))
        return self.o

    def adjust_weights(self):
        for i, e in enumerate(self.past_errors.get_items_in_lifo_order()):
            change = self.learning_rate*self.e*e
            self.weights[i] += change
            self.weights[i] -= self.weights[i] * self.decay_rate
            # if self.weights[i] < 0.0:
            #    self.weights[i] = 0.0
            if self.debug:
                print('      i:%s e:%.3f w:%.3f v:%.3f change:%.3f' % (i, e, self.weights[i], self.weights[i]*e,
                                                                       change))

    def get_parameters(self):
        return {'gain': self.gain, 'past_length': self.past_length, 'learning_rate': self.learning_rate}

    def set_parameters(self, parameters):
        pass


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
