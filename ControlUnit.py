import signals as sg
import unit_test as ut


class GenericControlUnit(object):
    """
    Abstract class to control units like PID or PUC
    """

    def __init__(self, bounds=(), min_error=0.0, dt=0.1, min_dt=0.0001, max_change=0.0, key='', debug=False):
        """
        :param bounds:      min and max value for output
        :param min_error:   defines the error too small to react
        :param dt:          interval between outputs
        :param: min_dt:     minimum dt to be useful, if less than this value a dt of 0.0 is assumed
        :param max_change:  how much output can change in one cycle (0=unbounded)
        :param key:         name used to recognize unit in a dictionary (useful for HCPU)
        """
        self.key        = key
        self.min_error  = min_error
        self.max_change = max_change
        self.bounds     = bounds
        self.debug      = debug
        # print('  bounds for %s:%s' % (self.key, self.bounds))

        # to avoid warnings
        self.o  = 0.0  # output
        self.r  = 0.0  # reference (or set point)
        self.e  = 0.0  # error: difference between reference and perception (in that order)

        self.dt     = dt
        self.min_dt = min_dt
        self.reference_changed = True

        # initialization
        self.reset()

    def update(self, p, dt=0.0):
        """
        Given a p (perception) updates the new o (output)
        :param p:   new perception
        :param dt:  interval since last perception
                      * 0.0 means caller is not providing time so the predefined one is used
                      * if > 0.0 the predefined one is also changed
        :return: output
        """
        if dt > self.min_dt:
            # updates the predefined one
            self.dt = dt

        new_error = self.r - p
        if abs(new_error) < self.min_error:
            new_error = 0.0

        new_o = self.calc_output(new_error)

        if self.debug:
            print('o:%.3f r:%.3f p:%.3f e:%.3f' % (new_o, self.r, p, new_error))

        if self.max_change > 0.0:
            # output change is bounded, cannot change too much
            # old_o = self.o
            self.o = sg.bound_value(new_o, [self.o - self.max_change, self.o + self.max_change])
            # print('old_o:%.2f max:%.2f new_o:%.2f self.o:%.2f' % (old_o, self.max_change, new_o, self.o))
        else:
            self.o = new_o

        self.o = sg.bound_value(self.o, self.bounds)

        self.reference_changed = False
        # print('e:%s p:%s o:%s g:%s bounds:%s key:%s' % (self.e, p, self.o, self.g, self.bounds, self.key))
        return self.o

    def calc_output(self, _):
        """
        Abstract method, each particular controller must provide it own method
        :param _:
        :return:
        """
        pass

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
        # given a dict (status), adjust output if value is in dict
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
        Reset the controller
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

    type = 'PID'

    def __init__(self, p=0.1, i=0.0, d=0.0, dt=0.1, bounds=(), integrator_length=20, integrator_windup=300,
                 integrator_reset=False, max_change=0.0, min_error=0.0, key='PID', debug=False):
        """
        :param p: Proportional gain
        :param i: Integrator gain
        :param d: Derivative gain
        :param dt:     delta time
        :param bounds: min and max values for output
        :param integrator_length: number of last values to be taken in integration
        :param integrator_windup: max value (positive and negative) the integrator can have
        :param integrator_reset:  if True means the integrator value is to 0 every time the reference values changes
        :param max_change: max change output can have in one interval
        :param min_error:  error less than this is too small to react
        :param key:
        :param debug:
        """
        self.k_p = p
        self.k_i = i
        self.k_d = d
        self.i_windup          = integrator_windup
        self.integrator_length = integrator_length
        self.integrator_reset  = integrator_reset
        self.first_time = True
        super(PID, self).__init__(bounds=bounds, dt=dt, max_change=max_change, min_error=min_error, key=key,
                                  debug=debug)

        # there are two kind of integrators, the first one is a traditional one (just summing all values)
        # and the second one just take in count the lasts (integrator_length) values
        self.integrator        = 0.0
        self.integrator_values = sg.SignalHistory(length=self.integrator_length)

        self.p_value = 0.0
        self.i_value = 0.0
        self.d_value = 0.0

        if self.debug:
            print('PID params: %s' % self.parm_string())

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
        elif self.integrator_values.get_len() > 1 and False:
            self.integrator_values.append(value)
            self.integrator = self.integrator_values.sum()
        else:
            self.integrator += value
        # self.integrator = sg.bound_value(self.integrator, [-self.i_windup, self.i_windup])
        if self.debug:
            print('   i:%.3f v:%.3f (e:%.3f dt:%s, key:%s)' % (self.integrator, value, self.e, self.dt, self.key))

    def reset_specific(self):
        self.integrator        = 0.0
        self.integrator_values = sg.SignalHistory(length=self.integrator_length)

    def get_parameters(self):
        return {'p': self.k_p, 'i': self.k_i, 'd': self.k_d}

    def set_parameters(self, parameters):
        self.k_p = parameters.get('p', self.k_p)
        self.k_i = parameters.get('i', self.k_i)
        self.k_d = parameters.get('d', self.k_d)
        self.i_windup = parameters.get('i_windup', self.i_windup)


class IncrementalPID(GenericControlUnit):
    """
    Incremental PID as defined in https://d-nb.info/1208071297/34
    """

    type = 'IncrementalPID'

    def __init__(self, p=0.1, i=0.0, d=0.0, dt=0.1, bounds=(), max_change=0.0, min_error=0.0, key='PID', debug=False):
        """
        :param p: Proportional gain
        :param i: Integrator gain
        :param d: Derivative gain
        :param dt:     delta time
        :param bounds: min and max values for output
        :param max_change: max change output can have in one interval
        :param min_error:  error less than this is too small to react
        :param key:
        :param debug:
        """
        self.k_p = p
        self.k_i = i
        self.k_d = d

        self.last_e      = 0.0
        self.last_last_e = 0.0
        self.first_time = True
        super(IncrementalPID, self).__init__(bounds=bounds, dt=dt, max_change=max_change, min_error=min_error, key=key,
                                             debug=debug)

        self.p_value = 0.0
        self.i_value = 0.0
        self.d_value = 0.0

        if self.debug:
            print('Incremental PID params: %s' % self.parm_string())

    def calc_output(self, new_error):
        self.e = new_error

        self.p_value = self.k_p * (self.e - self.last_e)
        self.i_value = self.k_i * self.e
        self.d_value = self.k_d * (self.e - 2*self.last_e + self.last_last_e)

        delta_o = self.p_value + self.i_value + self.d_value
        o       = self.o + delta_o

        self.last_last_e = self.last_e
        self.last_e      = self.e

        return o

    def reset_specific(self):
        self.last_e = 0.0
        self.last_last_e = 0.0

    def get_parameters(self):
        return {'p': self.k_p, 'i': self.k_i, 'd': self.k_d}

    def set_parameters(self, parameters):
        self.k_p = parameters.get('p', self.k_p)
        self.k_i = parameters.get('i', self.k_i)
        self.k_d = parameters.get('d', self.k_d)


class P(PID):
    """
    Just a Proportional controller (it is implemented over a PID with just the proportional gain available)
    The only difference with a PID with I and D in 0.0 is the only gain that can be used in twiddle is P
    """

    def __init__(self, p=0.1, dt=0.1, bounds=(), max_change=0.0, min_error=0.0, key='P', debug=False):
        super(P, self).__init__(p=p, bounds=bounds, dt=dt, max_change=max_change, min_error=min_error, key=key,
                                debug=debug)

    def get_parameters(self):
        return {'p': self.k_p}

    def set_parameters(self, parameters):
        self.k_p = parameters.get('p', self.k_p)


class PCUControlUnit(GenericControlUnit):
    """
    Control Unit as used in Perceptual Control Theory:
        o = o + (kg*e - o)/ks
    """

    type = 'PCU'
    key_g = 'g'
    key_s = 's'

    def __init__(self, name, gains, bounds=(), dt=0.1, max_change=0.0, min_error=0.0, debug=False):
        self.kg     = gains[0]
        self.ks     = gains[1] if len(gains) > 1 else 1.0
        super(PCUControlUnit, self).__init__(bounds=bounds, dt=dt, max_change=max_change, min_error=min_error,
                                             key=name, debug=debug)

    def get_output(self, reference, perception, dt=0.0, bounds=None, min_ks=0.01):
        self.e = reference - perception
        if 0.0 <= self.ks < min_ks:
            ks = min_ks
        elif 0.0 > self.ks > - min_ks:
            ks = -min_ks
        else:
            ks = self.ks

        self.o = self.o + (self.kg * self.e - self.o) / ks
        if len(self.bounds) > 0:
            if self.o < self.bounds[0]:
                self.o = self.bounds[0]
            elif self.o > self.bounds[1]:
                self.o = self.bounds[1]
        if self.debug:
            print('        %s r:%.2f p:%.2f e:%.2f o:%.4f' % (self.key, reference, perception, self.e, self.o))
        return self.o

    def get_parameters(self):
        return {self.key_g: self.kg, self.key_s: self.ks}

    def set_parameters(self, parameters):
        self.kg = parameters.get(self.key_g, self.kg)
        self.ks = parameters.get(self.key_s, self.ks)

    def set_kg(self, new_kg):
        self.kg = new_kg

    def set_ks(self, new_ks):
        self.ks = new_ks


class BangBang(GenericControlUnit):
    """
    BangBang controller as defined in https://en.wikipedia.org/wiki/Bang%E2%80%93bang_control
        output can have only two values (typically On or Off)
    """

    def __init__(self, bellow_value=1, above_value=0, hysteresis=0.0, key='BB'):
        """
        :param bellow_value:  output when p is bellow reference
        :param above_value:   output when p is above reference
        :param hysteresis:    range from reference in which send the last output
        :param key:
        """
        super(BangBang, self).__init__(key=key)
        self.bellow_value = bellow_value
        self.above_value  = above_value
        self.hysteresis   = hysteresis

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
        return {}

    def set_parameters(self, parameters):
        pass


def create_control(control_params):
    if control_params is None:
        return None

    # print(control_params)
    control_type   = control_params['type']
    control_name   = control_params.get('name', 'NoName')
    control_bounds = control_params.get('bounds', [])
    control_debug  = control_params.get('debug', False)

    if control_type == PCUControlUnit.type:
        control = PCUControlUnit(control_name, gains=control_params.get('gains'), bounds=control_bounds,
                                 debug=control_debug)
    elif control_type == PID.type:
        gains   = control_params.get('gains')
        control = PID(key=control_name, p=gains[0], i=gains[1], d=gains[2], bounds=control_bounds, debug=control_debug)
    elif control_type == IncrementalPID.type:
        gains = control_params.get('gains')
        control = IncrementalPID(key=control_name, p=gains[0], i=gains[1], d=gains[2], bounds=control_bounds,
                                 debug=control_debug)
    elif control_type == AdaptiveControlUnit.type:
        gain    = control_params.get('gain', 1.0)
        l_rate  = control_params.get('learning_rate', 0.01)
        d_rate  = control_params.get('decay_rate', 0.0)
        length  = control_params.get('past_length', 10)
        control = AdaptiveControlUnit(control_name, gain=gain, learning_rate=l_rate, decay_rate=d_rate,
                                      past_length=length, debug=control_debug)
    else:
        raise Exception('%s control type not implemented' % control_type)
    return control


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


# Tests
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
