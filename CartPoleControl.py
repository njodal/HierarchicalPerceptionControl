import hierarchical_control as hc
import ControlUnit as cu


class CartPoleControl(hc.BaseHierarchicalControl):
    def __init__(self, pole_angel_control_def, pole_angle_reference=0.0, overshoot_gain=10.0):
        pole_angle_control = cu.create_control(pole_angel_control_def)
        low_levels         = []

        super(CartPoleControl, self).__init__(pole_angle_control, low_levels_controls=low_levels,
                                              reference=pole_angle_reference, overshoot_gain=overshoot_gain)

    def get_main_observation(self, observation):
        """
        Main observation is pole_angle
        :param observation: cart_position, cart_speed, pole_angle, pole_speed
        :return:
        """
        return observation[2]

    def get_action(self, pole_angle_reference, observation):
        self.set_reference(pole_angle_reference)
        action = self.get_actions(observation, info=None)
        return action
