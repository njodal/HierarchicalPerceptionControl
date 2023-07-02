#!/usr/bin/env python

import sys

import WinDeklar.QTAux as QTAux
import WinDeklar.WindowForm as WinForm
import WinDeklar.graph_aux as ga

import CarControl as CarCon
import CarModel as CarMod


class CarSpeedControlHost(WinForm.HostModel):
    """
    Shows car speed control
    """

    def __init__(self, dt=0.1):
        # keys (names used in the yaml definition file)
        self.ref_speed_key         = 'ref_speed'
        self.olag_key              = 'olag'
        self.kg_key                = 'kg'
        self.ks_key                = 'ks'
        self.start_stop_key        = 'start_stop'
        self.start_stop_action_key = 'start_stop_action'

        # particular data
        self.dt = dt

        self.speed_reference = ga.RealTimeConstantDataProvider(dt=self.dt, color='Black')
        self.speed_control   = RealTimeControlSpeedDataProvider(output_lag=0, color='Red')

        initial_values = {}
        super(CarSpeedControlHost, self).__init__(initial_values=initial_values)

    def get_data_provider(self, interval=100, min_x=0.0, max_x=10.0, data_provider=None):
        """
        Returns the basic setup of the graph, most important one is the data provider
        :param interval:       time in milliseconds to show each point in the graph
        :param min_x:          start value in the x-axis
        :param max_x:          max value in the x-axis, after that it starts to scroll left
        :param data_provider:  subclass of RealTimeDataProvider
        :return:
        """
        dt             = interval/1000  # seconds
        max_points     = int(max_x/dt) + 1
        x_bounds       = [min_x, max_x]
        data_provider1 = [self.speed_reference, self.speed_control]
        self.set_references()

        return interval, max_points, x_bounds, data_provider1

    def get_current_output_lag(self):
        return int(self.state.get(self.olag_key, 0))

    def control_changed(self, name, value):
        if name == self.ref_speed_key:
            self.set_references()
        elif name == self.olag_key:
            self.speed_control.set_output_lag(int(value))
        elif name == self.kg_key:
            self.speed_control.set_kg(value)
        elif name == self.ks_key:
            self.speed_control.set_ks(value)

    def set_references(self):
        ref_speed = self.state.get(self.ref_speed_key, 0.0)
        self.speed_reference.set_reference(ref_speed)  # show new reference in win
        self.speed_control.set_reference(ref_speed)          # set new reference to control

    def start_stop_animation(self):
        """
        Logit to start/stop the graph with only one button (who changes its title depending on the graph is
        running or not_
        :return:
        """
        if self.anim_is_running():
            self.stop_animation()
            self.set_control_title(self.start_stop_key, 'Restart Animation')
            self.set_control_title(self.start_stop_action_key, 'Restart Animation')
        else:
            self.start_animation()
            self.set_control_title(self.start_stop_key, 'Pause Animation')
            self.set_control_title(self.start_stop_action_key, 'Pause Animation')


class RealTimeControlSpeedDataProvider(ga.RealTimeDataProvider):
    def __init__(self, dt=0.1, gains=(2.0, 1.0), reference=0.0, output_lag=0, min_y=-1.0, max_y=15.0, color='Black'):
        self.reference = reference
        def_control    = {'type': 'PCU', 'name': 'speed', 'gains': gains}
        def_control    = {'type': 'AdaptiveP', 'name': 'speed', 'learning_rate': 0.001, 'decay_rate': 0.0,
                          'past_length': 10, 'gain': 0.1, 'debug': True}
        self.control   = CarCon.CarSpeedControl(self.reference, def_control)
        self.model     = CarMod.CarModel(output_lag=output_lag)
        super(RealTimeControlSpeedDataProvider, self).__init__(dt=dt, min_y=min_y, max_y=max_y, color=color)

    def set_output_lag(self, new_output_lag):
        self.model.set_output_lag(new_output_lag)

    def set_kg(self, new_kg):
        # self.control.set_kg(new_kg)
        pass

    def set_ks(self, new_kg):
        # self.control.set_ks(new_kg)
        pass

    def set_reference(self, new_reference):
        self.reference = new_reference
        self.control.set_reference(self.reference)

    def get_next_values(self, i):
        x      = self.t
        acceleration = self.control.get_action(self.model.get_state())
        self.model.apply_acc(acceleration, self.dt)
        position, speed = self.model.get_state()

        self.t += self.dt
        return x, speed


if __name__ == '__main__':
    app = QTAux.def_app()
    provider = CarSpeedControlHost()        # class to handle events
    WinForm.run_winform(__file__, provider)
    sys.exit(app.exec_())
