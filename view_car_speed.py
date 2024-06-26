#!/usr/bin/env python

import sys

import WinDeklar.QTAux as QTAux
import WinDeklar.WindowForm as WinForm
import WinDeklar.graph_aux as ga
from WinDeklar.WindowForm import get_win_config, ConfigurableWindow
from WinDeklar.yaml_functions import get_file_name_with_other_extension

import hierarchical_control as hc
import view_aux_functions as va
import CarControl as CarCon
import CarModel as CarMod


class CarSpeedControlHost(WinForm.HostModel):
    """
    Shows car speed control
    """

    def __init__(self, dt=0.1, control_def_file_name='PID', car_name='simple'):
        # keys (names used in the yaml definition file)
        self.ref_speed_key         = 'ref_speed'
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
        self.graph_speed_v         = 'graph_speed'
        self.graph_speed_acc       = 'graph_acc'
        self.graph_speed_acc1      = 'graph_acc1'

        self.parameter_keys = []

        # particular data
        self.dt              = dt
        self.car_name        = car_name
        car_type             = CarMod.get_car_type(car_name)
        self.model           = CarMod.CarModel(car_type, output_lag=0, acc_pedal_key='accelerator',
                                               brake_pedal_key='brake')
        self.control         = hc.HierarchicalControl(control_def_file_name, dir_name='cars')
        self.speed_reference = ga.RealTimeConstantDataProvider(dt=self.dt, color='Black')
        self.speed_control   = RealTimeControlSpeedDataProvider(self.model, self.control, color='Red')

        max_acc = 3.0
        self.acceleration_values = va.RealTimeObjectAttributeDataProvider(self.control, 'ref_acceleration',
                                                                          dt=self.dt, min_y=-max_acc, max_y=max_acc,
                                                                          color='Black')
        self.current_acc_values  = va.RealTimeObjectAttributeDataProvider(self.model, self.model.acc_sensor_key,
                                                                          dt=self.dt, min_y=-2.0, max_y=2.0,
                                                                          color='Red')
        min_y = -2
        max_y = self.model.max_pedal_value - min_y
        self.acc_values   = va.RealTimeObjectAttributeDataProvider(self.model, self.model.acc_pedal_key, dt=self.dt,
                                                                   min_y=min_y, max_y=max_y, color='Black')
        self.brake_values = va.RealTimeObjectAttributeDataProvider(self.model, self.model.brake_pedal_key, dt=self.dt,
                                                                   min_y=min_y, max_y=max_y, color='Red')

        initial_values = self.control.get_parameters()
        initial_values[self.control_type_key] = control_def_file_name
        # print('initial state:%s' % initial_values)
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
        elif figure.name == self.graph_speed_acc1:
            data_provider1 = [self.acceleration_values, self.current_acc_values]
        else:
            raise Exception('"%s" graph name not implemented' % figure.name)

        return interval, max_points, x_bounds, data_provider1

    def get_current_output_lag(self):
        return int(self.get_value(self.olag_key))

    def widget_changed(self, name, value):
        if name == self.ref_speed_key:
            self.set_references()
        elif name == self.slope_key:
            self.model.set_slope(value)
        elif name == self.olag_key:
            self.model.set_output_lag(int(value))
        elif name in self.parameter_keys:
            msg = '%s changed to %.3f' % (name, value)
            # print(msg)
            self.show_status_bar_msg(msg)
            self.speed_control.set_parameter(name, value)

    def set_references(self):
        ref_speed = self.get_value(self.ref_speed_key)
        self.speed_reference.set_reference(ref_speed)        # show new reference in win
        self.speed_control.set_reference(ref_speed)          # set new reference to control

    def on_mouse_move(self, event, ax):
        if event.xdata is None or event.ydata is None:
            return
        self.show_status_bar_msg('x:%.2f y:%.2f' % (event.xdata, event.ydata))

    def add_new_widgets(self, win_config1, debug=False):
        """
        Adds the HierarchicalControl parameters as widgets, so user can experiment with the effect of changing each
        value
        :param win_config1:
        :param debug:
        :return:
        """
        widgets = win_config1['layout'][0]['item']['layout'][0]['item']['widgets']
        if debug:
            print(widgets)
        new_widgets = self.control.get_parameters_as_widget()
        if debug:
            print('new_widgets: %s' % new_widgets)
        widgets.extend(new_widgets)
        for new_widget in new_widgets:
            name = new_widget['widget']['name']
            self.parameter_keys.append(name)

    # Actions
    def auto_tune(self):
        self.start_stop_animation()
        self.show_status_bar_msg('Auto tuning ...')
        output_lag        = int(self.get_value(self.olag_key))
        slope             = self.get_value(self.slope_key)
        max_iter          = 500
        reference_changes = [[0, 5], [300, 2], [350, 7], [400, 6], [410, 5], [420, 3]]
        slope_changes     = [[100, 10], [200, -10], [300, 0]]
        auto_tune = CarCon.AutoTuneCar(self.car_name, self.control, output_lag, reference_changes, dt=self.dt,
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
        self.control.set_parameters(new_parameters)

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


class RealTimeControlSpeedDataProvider(ga.RealTimeDataProvider):
    def __init__(self, model, control, dt=0.1, reference=0.0, min_y=-1.0, max_y=15.0, color='Black'):
        self.reference = reference
        self.model     = model
        self.control   = control
        super(RealTimeControlSpeedDataProvider, self).__init__(dt=dt, min_y=min_y, max_y=max_y, color=color)

    def set_parameter(self, name, value):
        self.control.set_parameters({name: value})

    def set_reference(self, new_reference):
        self.reference = new_reference
        self.control.set_reference('ref_speed', self.reference)

    def get_next_values(self, i):
        x       = self.t
        actions = self.control.get_actions(self.model.get_sensors(), [])
        self.model.apply_actions(actions, self.dt)
        position, speed, _ = self.model.get_state()

        self.t += self.dt
        return x, speed


if __name__ == '__main__':
    app = QTAux.def_app()
    par_control_type       = WinForm.get_arg_value(1, None)
    control_def_file_name1 = 'simple_speed_control.yaml' if par_control_type is None else par_control_type
    provider = CarSpeedControlHost(control_def_file_name=control_def_file_name1)        # class to handle events
    # WinForm.run_winform(__file__, provider)
    win_config_name = get_file_name_with_other_extension(__file__, 'yaml')
    win_config      = get_win_config(win_config_name)
    provider.add_new_widgets(win_config, debug=True)
    ConfigurableWindow(win_config, provider)

    sys.exit(app.exec_())
