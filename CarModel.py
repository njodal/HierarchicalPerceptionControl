import numpy as np

import signals as sg
import unit_test as ut


class CarEnvironment:
    def __init__(self, control, car_name='simple', output_lag=0, slope=0.0, dt=0.1, max_steps=500):
        self.slope      = slope
        self.car_name   = car_name
        self.car_type   = get_car_type(self.car_name)
        self.output_lag = output_lag
        self.dt         = dt
        self.control    = control
        self.max_iter   = max_steps

    def run_episode(self, reference_changes=(), debug=False):
        """
        Run a feedback control loop for a number of steps
        :param reference_changes: list (iter, reference) where the reference will be changed (ex: [[0, 5], [40, 0]]
                                  means at step 0 reference will be set at 5 and in step 40 will be set at 0)
        :param debug:
        :return:
        """
        car_type    = get_car_type(self.car_type)
        car_model   = CarModel(car_type, slope=self.slope, output_lag=self.output_lag)
        observation = car_model.get_state()
        ended       = False
        steps       = 0
        t           = 0.0
        speed_evo   = [[t, 0.0]]
        while not ended:
            for [i, reference] in reference_changes:
                if i == steps:
                    self.control.set_reference(reference)
                    # print('set reference to: %s' % reference)

            acceleration = self.control.get_action(observation, [])
            car_model.apply_acc(acceleration, self.dt)
            observation = car_model.get_state()
            steps += 1
            t     += self.dt
            speed_evo.append([t, observation[1]])
            if steps > self.max_iter:
                ended = True
        if debug:
            print('   episode errors: %.3f for control: %s' % (self.control.sqr_errors,
                                                               self.control.control_speed.parm_string()))
        return steps, observation, self.control.get_total_cost(), speed_evo


class CarModel:
    gravity = 9.8

    def __init__(self, car_type, slope=0.0, friction=0.01, output_lag=0):
        """
        Basic model of a car moving straight
        :param car_type: defines the static properties of the vehicle :type CarType
        :param output_lag: delay it takes to apply the desired acceleration
        """
        self.car_type   = car_type
        self.olag       = output_lag
        self.acc_values = sg.DelayedSignal(delay=self.olag)

        # to avoid warnings
        self.current_pos = 0.0
        self.current_v   = 0.0
        self.current_acc = 0.0
        self.slope_acc   = 0.0
        self.friction    = friction

        self.set_slope(slope)

        self.reset()

    def reset(self, pos=0.0, v=0.0, acc=0.0):
        # state values
        self.current_pos = pos
        self.current_v   = v
        self.current_acc = acc

    def apply_acc(self, acc, dt=0.1):
        self.acc_values.append(acc)
        desired_acc = self.acc_values.get_value()
        if desired_acc is None:
            desired_acc = 0.0

        self.current_acc  = self.car_type.valid_acc(desired_acc)
        friction_force    = self.friction * self.current_v
        total_acc         = self.current_acc - self.slope_acc - friction_force
        desired_speed     = self.current_v + total_acc*dt
        self.current_v    = self.car_type.valid_speed(desired_speed)
        self.current_pos += self.current_v*dt
        # print('  acc:%.2f current_acc:%.2f total_acc:%.2f' % (acc, self.current_acc, total_acc))

    def get_state(self):
        return self.current_pos, self.current_v, self.current_acc

    def set_output_lag(self, new_output_lag):
        self.olag = new_output_lag
        self.acc_values.set_delay(self.olag)

    def set_slope(self, new_slope):
        self.slope_acc = np.sin(np.radians(new_slope)) * self.gravity

    def __str__(self):
        return 'pos:%.2f v:%.2f acc:%.2f' % (self.current_pos, self.current_v, self.current_acc)


class CarType:
    def __init__(self, max_speed=10.0, max_reverse_speed=2.0, max_acc=5.0, max_brake=5.0):
        """
        Basic model of a car moving straight
        :param max_speed:  max speed the vehicle is capable to achieve
        :param max_acc:    idem for acceleration
        :param max_brake:  idem for braking deceleration
        """
        self.max_reverse_speed = max_reverse_speed
        self.max_speed         = max_speed
        self.max_acc           = max_acc
        self.max_brake         = max_brake

    def valid_acc(self, acceleration):
        return sg.bound_value(acceleration, [-self.max_brake, self.max_acc])

    def valid_speed(self, speed):
        return sg.bound_value(speed, [-self.max_reverse_speed, self.max_speed])


def get_car_type(car_name):
    # ToDo: create a def file with different car types
    return CarType(max_speed=15.0, max_reverse_speed=2.0, max_acc=5.0, max_brake=8.0)


# tests
def test_lineal_move(acc, until_t, total_t, dt, output_lag, debug):
    car_type = get_car_type('simple')
    car      = CarModel(car_type, output_lag=output_lag)
    t        = 0.0
    while t < total_t:
        acc1 = acc if t < until_t else 0.0
        car.apply_acc(acc1, dt=dt)
        if debug:
            print('  t:%.2f %s' % (t, car))
        t += dt
    return car.current_pos


if __name__ == "__main__":
    ut.UnitTest(__name__, 'tests/CarModel.test', '')
