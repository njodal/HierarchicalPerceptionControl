from signals import SignalHistory


class PCUControlUnit:
    """
    Control Unit as used in Perceptual Control Theory:
        o = o + (kg*e - o)/ks
    """

    type = 'PCU'

    def __init__(self, name, gains, debug=False):
        self.name  = name
        self.kg    = gains[0]
        self.ks    = gains[1] if len(gains) > 1 else 1.0
        self.debug = debug
        self.o     = 0.0
        self.e     = 0.0

    def get_output(self, reference, perception):
        self.e = reference - perception
        self.o = self.o + (self.kg * self.e - self.o) / self.ks
        if self.debug:
            print('        %s r:%.2f p:%.2f e:%.2f o:%.4f' % (self.name, reference, perception, self.e, self.o))
        return self.o


class AdaptiveControlUnit:
    """
    Adaptive control based on 'Artificial Cerebellum' by Bill Powers
    https://www.iapct.org/themes/biology-neuroscience/an-artificial-cerebellum-adaptive-stabilization-of-a-control-system/
    (and also from Rupert Young PhD Thesis
    """

    type = 'AdaptiveP'

    def __init__(self, name, gain=1.0, learning_rate=0.01, decay_rate=0.0, past_length=10, debug=False):
        self.name          = name
        self.gain          = gain
        self.learning_rate = learning_rate
        self.decay_rate    = decay_rate
        self.past_length   = past_length
        self.past_errors   = SignalHistory(length=self.past_length)
        self.weights       = [0.0 for _ in range(self.past_length+1)]
        self.debug         = debug

        self.r = 0.0
        self.o = 0.0
        self.e = 0.0
        self.p = 0.0

    def get_output(self, reference, perception):
        self.r = reference
        self.p = perception
        self.e = self.r - self.p
        self.past_errors.append(self.e)

        self.adjust_weights()

        weighted_sum = self.past_errors.weighted_sum(self.weights, lifo_order=True)

        self.o = self.gain*weighted_sum

        if self.debug:
            print('   o:%.3f w_sum:%.3f e:%.3f r:%.3f p:%.3f' % (self.o, weighted_sum, self.e, self.r, self.p))
        return self.o

    def adjust_weights(self):
        # print('values length:%s weight_length:%s' % (len(self.past_errors.values), len(self.weights)))
        for i, e in enumerate(self.past_errors.get_items_in_lifo_order()):
            change = self.learning_rate*self.e*e
            self.weights[i] += change
            self.weights[i] -= self.weights[i] * self.decay_rate
            # if self.weights[i] < 0.0:
            #    self.weights[i] = 0.0
            if self.debug:
                print('      i:%s e:%.3f w:%.3f v:%.3f change:%.3f' % (i, e, self.weights[i], self.weights[i]*e,
                                                                       change))

        # for i, e in enumerate(self.past_errors.values):
        #    self.weights[i] += self.learning_rate*e
        #    self.weights[i] -= self.weights[i] * self.decay_rate
        #    if self.debug:
        #        print('      i:%s e:%.3f w:%.3f v:%.3f' % (i, e, self.weights[i], self.weights[i]*e))


def create_control(control_params):
    # print(control_params)
    control_type  = control_params['type']
    control_name  = control_params.get('name', 'NoName')
    control_debug = control_params.get('debug', False)
    if control_type == PCUControlUnit.type:
        control = PCUControlUnit(control_name, gains=control_params.get('gains'), debug=control_debug)
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
    def __init__(self, inputs, outputs, learning_rate=0.01, decay_rate=0.0, past_length=10, debug=False):
        self.inputs  = inputs
        self.outputs = outputs
        self.debug   = debug
        self.o       = 0.0
        self.past_length   = past_length
        self.learning_rate = learning_rate
        self.decay_rate    = decay_rate
        self.past_inputs   = SignalHistory(length=self.past_length)
        self.weights       = [0.0 for _ in range(self.past_length+1)]

    def get_output(self, input_value, output_value):
        self.o = self.past_inputs.weighted_sum(self.weights, lifo_order=True)
        self.adjust_weights(input_value, output_value, self.o)
        return self.o

    def adjust_weights(self, input_value, actual_output_value, current_output_value):
        self.past_inputs.append(input_value)
        delta_output_value = actual_output_value - current_output_value
        for i, old_input in enumerate(self.past_inputs.get_items_in_lifo_order()):
            change = self.learning_rate*delta_output_value*signum(old_input)
            self.weights[i] += change
            self.weights[i] -= self.weights[i] * self.decay_rate
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
