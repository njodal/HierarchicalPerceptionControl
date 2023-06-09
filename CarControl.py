from ControlUnit import create_control
from CarModel import CarEnvironment
import unit_test as ut


class CarSpeedControl:
    """
    Control the speed of a Car changing its acceleration
    """

    def __init__(self, speed_reference, speed_control_def):
        self.control_speed = create_control(speed_control_def)

        self.speed_reference = speed_reference
        self.sqr_errors = 0.0

    def get_action(self, observation):
        _, current_speed = observation
        self.sqr_errors += abs(self.speed_reference - current_speed)
        acc_reference = self.control_speed.get_output(self.speed_reference, current_speed)
        return acc_reference


class CarPositionalControl:
    """
    Control the position of a Car changing its acceleration
    """

    def __init__(self, position_reference, position_control_def, speed_control_def):
        self.control_position = create_control(position_control_def)
        self.control_speed    = create_control(speed_control_def)

        self.position_reference = position_reference
        self.sqr_errors         = 0.0

    def get_action(self, observation):
        current_position, current_speed = observation
        self.sqr_errors += abs(self.position_reference - current_position)
        speed_reference = self.control_position.get_output(self.position_reference, current_position)
        acc_reference   = self.control_speed.get_output(speed_reference, current_speed)
        return acc_reference


def test_speed_control(speed_reference, output_lag, dt, max_steps, speed_control_def, debug):
    control = CarSpeedControl(speed_reference, speed_control_def)
    env     = CarEnvironment(control=control, output_lag=output_lag, dt=dt, max_steps=max_steps)
    steps, observation, errors = env.run_episode()
    if debug:
        print('speed:%.2f (ref:%.2f) at step %s (total error:%.3f)' % (observation[1], speed_reference, steps, errors))
    return observation[1]


def test_positional_control(pos_reference, output_lag, dt, max_steps, position_control_def, speed_control_def, debug):
    control = CarPositionalControl(pos_reference, position_control_def, speed_control_def)
    env     = CarEnvironment(control=control, output_lag=output_lag, dt=dt, max_steps=max_steps)
    steps, observation, errors = env.run_episode()
    if debug:
        print('position:%.2f at step %s (total error:%.3f)' % (observation[0], steps, errors))
    return observation[0]


if __name__ == "__main__":
    ut.UnitTest(__name__, 'tests/CarControl.test', '')





