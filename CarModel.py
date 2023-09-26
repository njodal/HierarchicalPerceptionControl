import numpy as np

import signals as sg
import yaml_functions as yf
import unit_test as ut


class CarEnvironment:
    def __init__(self, control, car_name='simple', output_lag=0, slope=0.0, dt=0.1, max_steps=500):
        self.slope      = slope
        self.car_name   = car_name
        self.output_lag = output_lag
        self.dt         = dt
        self.control    = control
        self.max_iter   = max_steps
        self.last_episode_observations = []

    def run_episode(self, reference_changes=(), slope_changes=(), debug=False):
        """
        Run a feedback control loop for a number of steps
        :param slope_changes:
        :param reference_changes: list (iter, reference) where the reference will be changed (ex: [[0, 5], [40, 0]]
                                  means at step 0 reference will be set at 5 and in step 40 will be set at 0)
        :param debug:
        :return:
        """
        self.last_episode_observations = []
        car_type    = get_car_type(self.car_name)
        car_model   = CarModel(car_type, slope=self.slope, output_lag=self.output_lag)
        observation = car_model.get_state()
        ended       = False
        steps       = 0
        t           = 0.0
        while not ended:
            for [i, reference] in reference_changes:
                if i == steps:
                    self.control.set_reference(reference)
                    # print('set reference to: %s' % reference)
            for [i, slope] in slope_changes:
                if i == steps:
                    car_model.set_slope(slope)

            actions = self.control.get_actions(observation, [])
            car_model.apply_actions(actions, self.dt)
            observation = car_model.get_state()
            steps += 1
            t     += self.dt
            self.last_episode_observations.append([t, observation])
            if steps > self.max_iter:
                ended = True
        if debug:
            print('   episode cost: %.3f for control: %s' % (self.control.get_total_cost(), self.control.parm_string()))
        return steps, observation, self.control.get_total_cost()

    def get_speed_evolution(self):
        for [t, observation] in self.last_episode_observations:
            yield [t, observation[1]]

    def get_position_evolution(self):
        return [[t, observation[0]] for [t, observation] in self.last_episode_observations]


class CarModel:
    gravity         = 9.8
    acc_key         = 'acceleration'
    acc_pedal_key   = 'acc'
    brake_pedal_key = 'brake'
    max_pedal_value = 10

    def __init__(self, car_type, slope=0.0, friction=0.1, output_lag=0):
        """
        Basic model of a car moving straight
        :param car_type: defines the static properties of the vehicle :type CarType
        :param output_lag: delay it takes to apply the desired acceleration
        """
        self.car_type   = car_type
        self.olag       = output_lag
        self.acc_values = sg.DelayedSignal(delay=self.olag)

        # to avoid warnings
        self.acc_pedal   = 0
        self.brake_pedal = 0
        self.current_pos = 0.0
        self.current_v   = 0.0
        self.current_acc = 0.0
        self.slope_acc   = 0.0
        self.friction    = friction

        self.set_slope(slope)

        self.reset()

    def reset(self, pos=0.0, v=0.0, acc=0.0, acc_pedal=0, brake_pedal=0):
        # state values
        self.acc_pedal   = acc_pedal
        self.brake_pedal = brake_pedal
        self.current_pos = pos
        self.current_v   = v
        self.current_acc = acc

    def apply_actions(self, actions, dt=0.1):
        self.acc_pedal   = sg.bound_value(actions.get(self.acc_pedal_key, 0), (0, self.max_pedal_value))
        acc              = sg.lineal_proportional_bounded(self.acc_pedal, 0, self.max_pedal_value, 0.0,
                                                          self.car_type.max_acc)
        self.brake_pedal = sg.bound_value(actions.get(self.brake_pedal_key, 0), (0, self.max_pedal_value))
        brake_acc        = sg.lineal_proportional_bounded(self.brake_pedal, 0, self.max_pedal_value, 0.0,
                                                          self.car_type.max_brake)
        self.apply_acc(acc, brake_acc, dt=dt)

    def apply_acc(self, acc, brake_acc, dt=0.1):

        self.acc_values.append([acc, brake_acc])     # store current value
        acc_after_lag = self.acc_values.get_value()  # actual value taking in count transport lag
        actual_acc, actual_braking_acc = acc_after_lag if acc_after_lag is not None else [0.0, 0.0]

        valid_acc         = self.car_type.valid_acc(actual_acc)
        forward_acc       = valid_acc - self.slope_acc                           # forces that move the vehicle
        backward_acc      = self.friction * self.current_v + actual_braking_acc  # forces that resist the movement
        new_v             = self.current_v + forward_acc*dt
        new_v_after_brake = new_v - np.sign(new_v) * backward_acc * dt
        if new_v_after_brake < 0.0 < new_v or new_v < 0.0 < new_v_after_brake:
            # braking cannot change direction of movement
            new_v_after_brake = 0.0

        self.current_acc  = (new_v_after_brake - self.current_v)/dt  # forward_acc - np.sign(new_v) * backward_acc
        self.current_v    = new_v_after_brake
        self.current_pos += self.current_v*dt
        # print('  acc:%s forward_acc:%.2f total_acc:%.2f slope:%.2f f:%.2f v:%.2f' %
        #      (acc_after_lag, forward_acc, self.current_acc, self.slope_acc, backward_acc, self.current_v))

    def get_state(self):
        return self.current_pos, self.current_v, self.current_acc

    def get_actuator(self, name):
        """
        Returns current actuators values, useful for plotting evolution
        :param name:
        :return:
        """
        if name == self.acc_pedal_key:
            return self.acc_pedal
        elif name == self.brake_pedal_key:
            return self.brake_pedal
        elif name == self.acc_key:
            return self.current_acc
        else:
            raise Exception('Actuator %s not known' % name)

    def set_output_lag(self, new_output_lag):
        self.olag = new_output_lag
        self.acc_values.set_delay(self.olag)

    def set_slope(self, new_slope):
        self.slope_acc = np.sin(np.radians(new_slope)) * self.gravity

    def __str__(self):
        return 'pos:%.2f v:%.2f acc:%.2f' % (self.current_pos, self.current_v, self.current_acc)


class CarType:
    name_key        = 'name'
    max_speed_key   = 'max_speed'
    max_acc_key     = 'max_acc'
    max_r_speed_key = 'max_reverse_speed'
    max_brake_key   = 'max_brake'

    def __init__(self, car_spec):
        """
        Car spec definition (max speed, acceleration, etc.)
        :param car_spec:  car parameters :type dict
        """
        self.car_spec = car_spec
        self.name              = self.car_spec.get(self.name_key, 'NoName')
        self.max_reverse_speed = self.car_spec.get(self.max_r_speed_key, 1.0)
        self.max_speed         = self.car_spec.get(self.max_speed_key, 1.0)
        self.max_acc           = self.car_spec.get(self.max_acc_key, 1.0)
        self.max_brake         = self.car_spec.get(self.max_brake_key, 1.0)

    def valid_acc(self, acceleration):
        return sg.bound_value(acceleration, [-self.max_brake, self.max_acc])

    def valid_speed(self, speed):
        return sg.bound_value(speed, [-self.max_reverse_speed, self.max_speed])


def get_car_type(car_name):
    cars_file = yf.get_yaml_file('cars/cars_definition.yaml')
    car_spec  = yf.get_record(cars_file, car_name, 'cars', 'car')
    return CarType(car_spec)


# tests
def test_lineal_move(acc, until_t, total_t, dt, output_lag, debug):
    car_type = get_car_type('simple')
    car      = CarModel(car_type, output_lag=output_lag)
    t        = 0.0
    while t < total_t:
        acc1 = acc if t < until_t else 0.0
        car.apply_acc(acc1, 0.0, dt=dt)
        if debug:
            print('  t:%.2f %s' % (t, car))
        t += dt
    return car.current_pos


if __name__ == "__main__":
    ut.UnitTest(__name__, 'tests/CarModel.test', '')
