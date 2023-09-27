#!/usr/bin/env python

import sys

import WinDeklar.QTAux as QTAux
import WinDeklar.WindowForm as WinForm
import WinDeklar.graph_aux as ga

import CarControl as CarCon
import CarModel as CarMod


class CarPositionControlHost(WinForm.HostModel):
    """
    Shows car speed control
    """

    def __init__(self, dt=0.1, control_type='PID', car_name='simple'):
        # keys (names used in the yaml definition file)
        self.ref_speed_key         = 'ref_speed'
        self.ref_pos_key           = 'ref_position'
        self.olag_key              = 'olag'
        self.slope_key             = 'slope'
        self.control_type_key      = 'control_type'
        self.kp_key                = 'p'
        self.ki_key                = 'i'
        self.kd_key                = 'd'
        self.kg_key                = 'g'
        self.ks_key                = 's'
        self.start_stop_key        = 'start_stop'
        self.start_stop_action_key = 'start_stop_action'
        self.graph_position        = 'graph_position'
        self.graph_speed_v         = 'graph_speed'
        self.graph_speed_acc       = 'graph_acc'

        self.gain_keys = [self.kp_key, self.ki_key, self.kd_key, self.kg_key, self.ks_key]

        # particular data
        self.dt             = dt
        self.car_name       = car_name
        car_type            = CarMod.get_car_type(car_name)
        self.model          = CarMod.CarModel(car_type, output_lag=0)
        pos_reference       = 10.0
        expected_output_lag = 0.3
        pos_control_def, speed_control_def = CarCon.get_car_position_controllers(control_type)
        self.control_pos     = CarCon.CarPositionalControl(pos_reference, pos_control_def, speed_control_def,
                                                           output_lag=expected_output_lag)
        self.pos_control     = RealTimeControlPositionDataProvider(self.model, self.control_pos, color='Red')

        self.pos_reference   = ga.RealTimeConstantDataProvider(dt=self.dt, color='Black')
        self.pos_reference.set_reference(pos_reference)
        speed_control        = self.control_pos.control_speed
        self.speed_reference = RealTimeControlInternalDataProvider(speed_control, data_key='r', dt=self.dt,
                                                                   color='Black')
        self.speed_p         = RealTimeControlInternalDataProvider(speed_control, data_key='p', dt=self.dt,
                                                                   color='Red')
        self.acc_values      = self.model.get_real_time_data(self.model.acc_pedal_key, dt=self.dt, color='Black')
        self.brake_values    = self.model.get_real_time_data(self.model.brake_pedal_key, dt=self.dt, color='Red')

        initial_values = self.control_pos.get_parameters()
        initial_values[self.control_type_key] = control_type
        print('initial state:%s' % initial_values)
        super(CarPositionControlHost, self).__init__(initial_values=initial_values)

    def initialize(self):
        conditional_widgets = [self.kp_key, self.ki_key, self.kd_key, self.kg_key, self.ks_key]
        parameters           = self.control_pos.get_parameters()
        for widget_name in conditional_widgets:
            if widget_name not in parameters:
                screen_widget = self.get_widget_by_name(widget_name)
                screen_widget.set_visible(False)

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
            data_provider1 = [self.speed_reference, self.speed_p]
        elif figure.name == self.graph_position:
            data_provider1 = [self.pos_control]
        elif figure.name == self.graph_speed_acc:
            data_provider1 = [self.acc_values, self.brake_values]
        else:
            raise Exception('"%s" graph name not implemented' % figure.name)

        return interval, max_points, x_bounds, data_provider1

    def get_current_output_lag(self):
        return int(self.get_value(self.olag_key))

    def widget_changed(self, name, value):
        if name == self.ref_pos_key:
            self.set_references()
        elif name == self.ref_speed_key:
            self.control_pos.set_min_max_speed(0.0, value)
        elif name == self.slope_key:
            self.model.set_slope(value)
        elif name == self.olag_key:
            self.model.set_output_lag(int(value))
        elif name in self.gain_keys:
            msg = '%s changed to %.3f' % (name, value)
            print(msg)
            self.show_status_bar_msg(msg)
            self.pos_control.set_gain(name, value)
            pass

    def set_references(self):
        new_pos_reference = self.get_value(self.ref_pos_key)
        self.pos_reference.set_reference(new_pos_reference)
        self.control_pos.set_reference(new_pos_reference)

    # Actions
    def auto_tune(self):
        self.start_stop_animation()
        self.show_status_bar_msg('Auto tuning ...')
        output_lag        = int(self.get_value(self.olag_key))
        slope             = self.get_value(self.slope_key)
        max_iter          = 500
        reference_changes = [[0, 5], [300, 2], [350, 7], [400, 6], [410, 5], [420, 3]]
        slope_changes     = [[100, 10], [200, -10], [300, 0]]
        auto_tune = CarCon.AutoTuneCar(self.car_name, self.control_pos, output_lag, reference_changes, dt=self.dt,
                                       max_iter=max_iter, slope=slope, slope_changes=slope_changes, debug=False)
        best_parameters = auto_tune.auto_tune(debug=False)
        self.set_parameters(best_parameters)
        self.start_stop_animation()
        self.show_status_bar_msg('Done')

    def set_parameters(self, new_parameters):
        for k, v in new_parameters.items():
            if k not in self._state:
                continue
            self._state[k] = v
            self.set_value(k, v)
        self.control_pos.set_parameters(new_parameters)

    def start_stop_animation(self):
        """
        Logic to start/stop the graph with only one button (who changes its title depending on the graph is
        running or not_
        :return:
        """
        if self.anim_is_running():
            self.stop_animation()
            self.set_widget_title(self.start_stop_key, 'Restart Animation')
            self.set_widget_title(self.start_stop_action_key, 'Restart Animation')
        else:
            self.start_animation()
            self.set_widget_title(self.start_stop_key, 'Pause Animation')
            self.set_widget_title(self.start_stop_action_key, 'Pause Animation')


class RealTimeControlPositionDataProvider(ga.RealTimeDataProvider):
    def __init__(self, model, control, dt=0.1, reference=0.0, min_y=-1.0, max_y=25.0, color='Black'):
        self.reference = reference
        self.model     = model
        self.control   = control
        super(RealTimeControlPositionDataProvider, self).__init__(dt=dt, min_y=min_y, max_y=max_y, color=color)

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
        print('pos:%.2f speed:%.2f' % (position, speed))

        self.t += self.dt
        return x, self.control.main_control.e


class RealTimeControlInternalDataProvider(ga.RealTimeDataProvider):
    def __init__(self, control, data_key='r', dt=0.1, min_y=-1.0, max_y=15.0, color='Black'):
        self.control  = control
        self.data_key = data_key
        super(RealTimeControlInternalDataProvider, self).__init__(dt=dt, min_y=min_y, max_y=max_y, color=color)

    def get_internal_data(self):
        if self.data_key == 'r':
            return self.control.r
        elif self.data_key == 'e':
            return self.control.e
        elif self.data_key == 'p':
            return self.control.p
        elif self.data_key == 'o':
            return self.control.o
        else:
            raise Exception('Data name %s not implemented' % self.data_key)

    def get_next_values(self, i):
        x       = self.t
        self.t += self.dt
        return x, self.get_internal_data()


if __name__ == '__main__':
    app = QTAux.def_app()
    par_control_type = WinForm.get_arg_value(1, None)
    def_control_type = 'PID' if par_control_type is None else par_control_type
    provider = CarPositionControlHost(control_type=def_control_type)        # class to handle events
    WinForm.run_winform(__file__, provider)
    sys.exit(app.exec_())
