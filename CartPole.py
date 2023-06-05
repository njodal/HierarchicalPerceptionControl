import GymEnvironment as GymEnv
from ControlUnit import PCUControlUnit
import unit_test as ut


class MoveCartToPosition:
    def __init__(self, gains, cart_pos_reference=0.0):
        self.unit1              = PCUControlUnit('cart position', gains=gains[0], debug=True)
        last_gains              = [g for i, g in enumerate(gains) if i > 0]
        self.down_control       = ControlCartPoleAtAngle(last_gains, debug_units=[True, False, False, False])
        self.cart_pos_reference = cart_pos_reference

    def change_cart_pos_reference(self, new_car_pos_reference):
        self.cart_pos_reference = new_car_pos_reference

    def get_action(self, observation, info):
        cart_position        = observation[0]
        pole_angle_reference = self.unit1.get_output(self.cart_pos_reference, cart_position)
        self.down_control.change_pole_angle_reference(pole_angle_reference)
        action = self.down_control.get_action(observation, info)
        return action


class ControlCartPoleAtAngle:
    def __init__(self, control_gains, debug_units=None, pole_angle_reference=0.0):
        self.k1, self.k2, self.k3, self.k4 = control_gains
        debug_flag = debug_units if len(debug_units) >= 4 else [False, False, False, False]
        self.pole_angle_reference = pole_angle_reference
        self.unit1 = PCUControlUnit('pole angle', control_gains[0], debug=debug_flag[0])
        self.unit2 = PCUControlUnit('pole speed', control_gains[1], debug=debug_flag[1])
        self.unit3 = PCUControlUnit('cart pos',   control_gains[2], debug=debug_flag[2])
        self.unit4 = PCUControlUnit('cart speed', control_gains[3], debug=debug_flag[3])

    def change_pole_angle_reference(self, new_pole_angle_reference):
        self.pole_angle_reference = new_pole_angle_reference

    def get_action(self, observation, _):
        cart_position, cart_speed, pole_angle, pole_speed = observation
        pole_speed_ref = self.unit1.get_output(self.pole_angle_reference, pole_angle)
        cart_pos_ref   = self.unit2.get_output(pole_speed_ref, pole_speed)
        cart_speed_ref = self.unit3.get_output(cart_pos_ref, cart_position)
        action1        = self.unit4.get_output(cart_speed_ref, cart_speed)
        action = 1 if action1 > 0.0 else 0
        return action


def test_control_pole_angle(pole_angle_reference, gains, debug_units, render, debug):
    render_mode = 'human' if render else None
    control     = ControlCartPoleAtAngle(gains, pole_angle_reference=pole_angle_reference, debug_units=debug_units)
    env         = GymEnv.BaseEnvironment('CartPole-v1', control=control, render_mode=render_mode)
    steps       = env.run_episode(debug=debug)
    return steps


def test_move_cart(car_pos_reference, gains, debug_units, render, debug):
    render_mode = 'human' if render else None
    # control     = ControlCartPoleAtAngle(gains, pole_angle_reference=pole_angle_reference, debug_units=debug_units)
    control     = MoveCartToPosition(gains, cart_pos_reference=car_pos_reference)
    env         = GymEnv.BaseEnvironment('CartPole-v1', control=control, render_mode=render_mode)
    steps       = env.run_episode(debug=debug)
    return steps


if __name__ == "__main__":
    ut.UnitTest(__name__, 'tests/CartPole.test', '')
