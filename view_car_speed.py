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

    def __init__(self, dt=0.1, control_type='PID', car_name='simple'):
        # keys (names used in the yaml definition file)
        self.ref_speed_key         = 'ref_speed'
        self.olag_key              = 'olag'
        self.slope_key             = 'slope'
        self.control_type_key      = 'control_type'
        self.kp_key                = 'p'
        self.ki_key                = 'i'
        self.kd_key                = 'd'
        self.start_stop_key        = 'start_stop'
        self.start_stop_action_key = 'start_stop_action'
        self.graph_speed_v         = 'graph_speed'
        self.graph_speed_acc       = 'graph_acc'

        self.gain_keys = [self.kp_key, self.ki_key, self.kd_key]

        # particular data
        self.dt              = dt
        car_type             = CarMod.get_car_type(car_name)
        self.model           = CarMod.CarModel(car_type, output_lag=0)
        self.control         = CarCon.get_car_controller(control_type)
        self.speed_reference = ga.RealTimeConstantDataProvider(dt=self.dt, color='Black')
        self.speed_control   = RealTimeControlSpeedDataProvider(self.model, self.control, color='Red')
        self.acc_values      = RealTimeActuatorDataProvider(self.model, self.model.acc_pedal_key, dt=self.dt,
                                                            color='Black')
        self.brake_values    = RealTimeActuatorDataProvider(self.model, self.model.brake_pedal_key, dt=self.dt,
                                                            color='Red')

        initial_values = self.control.get_parameters()
        initial_values[self.control_type_key] = control_type
        print('initial state:%s' % initial_values)
        super(CarSpeedControlHost, self).__init__(initial_values=initial_values)

    def get_data_provider(self, figure, interval=100, min_x=0.0, max_x=10.0, data_provider=None):
        """
        Returns the basic setup of the graph, most important one is the data provider
        :param figure:
        :param interval:       time in milliseconds to show each point in the graph
        :param min_x:          start value in the x-axis
        :param max_x:          max value in the x-axis, after that it starts to scroll left
        :param data_provider:  subclass of RealTimeDataProvider
        :return:
        """
        dt             = interval/1000  # seconds
        max_points     = int(max_x/dt) + 1
        x_bounds       = [min_x, max_x]
        if figure.name == self.graph_speed_v:
            data_provider1 = [self.speed_reference, self.speed_control]
            self.set_references()
        elif figure.name == self.graph_speed_acc:
            data_provider1 = [self.acc_values, self.brake_values]
        else:
            raise Exception('"%s" graph name not implemented' % figure.name)

        return interval, max_points, x_bounds, data_provider1

    def get_current_output_lag(self):
        return int(self.state.get(self.olag_key, 0))

    def control_changed(self, name, value):
        if name == self.ref_speed_key:
            self.set_references()
        elif name == self.slope_key:
            self.model.set_slope(value)
        elif name == self.olag_key:
            self.model.set_output_lag(int(value))
        elif name in self.gain_keys:
            # msg = '%s changed to %.3f' % (name, value)
            # print(msg)
            # self.show_status_bar_msg(msg)
            self.speed_control.set_gain(name, value)

    def set_references(self):
        ref_speed = self.state.get(self.ref_speed_key, 0.0)
        self.speed_reference.set_reference(ref_speed)        # show new reference in win
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
    def __init__(self, model, control, dt=0.1, reference=0.0, min_y=-1.0, max_y=15.0, color='Black'):
        self.reference = reference
        self.model     = model
        self.control   = control
        super(RealTimeControlSpeedDataProvider, self).__init__(dt=dt, min_y=min_y, max_y=max_y, color=color)

    def set_gain(self, name, value):
        self.control.set_parameters({name: value})

    def set_reference(self, new_reference):
        self.reference = new_reference
        self.control.set_reference(self.reference)

    def get_next_values(self, i):
        x       = self.t
        actions = self.control.get_actions(self.model.get_state(), [])
        self.model.apply_actions(actions, self.dt)
        position, speed, _ = self.model.get_state()

        self.t += self.dt
        return x, speed


class RealTimeActuatorDataProvider(ga.RealTimeDataProvider):
    def __init__(self, model, name, dt=0.1, inc=2, color='Black'):
        self.model = model
        self.name  = name
        min_y = - inc
        max_y = self.model.max_pedal_value + inc
        super(RealTimeActuatorDataProvider, self).__init__(dt=dt, min_y=min_y, max_y=max_y, color=color)

    def get_next_values(self, i):
        x       = self.t
        value   = self.model.get_actuator(self.name)
        self.t += self.dt
        return x, value


if __name__ == '__main__':
    app = QTAux.def_app()
    par_control_type = WinForm.get_arg_value(1, None)
    def_control_type = 'PID' if par_control_type is None else par_control_type
    provider = CarSpeedControlHost(control_type=def_control_type)        # class to handle events
    WinForm.run_winform(__file__, provider)
    sys.exit(app.exec_())
