import signals as sg
import unit_test as ut


class CarEnvironment:
    def __init__(self, output_lag=0, dt=0.1, control=None, max_steps=500):
        self.output_lag = output_lag
        self.dt         = dt
        self.control    = control
        self.max_iter   = max_steps

    def run_episode(self):
        car_model   = CarModel(output_lag=self.output_lag)
        observation = car_model.get_state()
        ended       = False
        steps       = 0
        while not ended:
            acceleration = self.control.get_action(observation)
            car_model.apply_acc(acceleration, self.dt)
            observation = car_model.get_state()
            steps += 1
            if steps > self.max_iter:
                ended = True
        return steps, observation, self.control.sqr_errors


class CarModel:
    def __init__(self, output_lag=0):
        """
        Basic model of a car moving straight
        :param output_lag: delay it takes to apply the desired acceleration
        """
        self.olag = output_lag
        self.acc_values = sg.DelayedSignal(delay=self.olag)

        # to avoid warnings
        self.current_pos = 0.0
        self.current_v   = 0.0
        self.current_acc = 0.0

        self.reset()

    def reset(self, pos=0.0, v=0.0, acc=0.0):
        # state values
        self.current_pos = pos
        self.current_v   = v
        self.current_acc = acc

    def apply_acc(self, acc, dt=0.1):
        self.acc_values.append(acc)
        actual_acc = self.acc_values.get_value()
        if actual_acc is None:
            # no action yet
            return
        self.current_acc  = actual_acc
        self.current_v   += self.current_acc*dt
        self.current_pos += self.current_v*dt

    def get_state(self):
        return self.current_pos, self.current_v

    def __str__(self):
        return 'pos:%.2f v:%.2f acc:%.2f' % (self.current_pos, self.current_v, self.current_acc)


def test_lineal_move(acc, until_t, total_t, dt, output_lag, debug):
    car = CarModel(output_lag=output_lag)
    t   = 0.0
    while t < total_t:
        acc1 = acc if t < until_t else 0.0
        car.apply_acc(acc1, dt=dt)
        if debug:
            print('  t:%.2f %s' % (t, car))
        t += dt
    return car.current_pos


if __name__ == "__main__":
    ut.UnitTest(__name__, 'tests/CarModel.test', '')
