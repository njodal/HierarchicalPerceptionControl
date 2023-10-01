#!/usr/bin/env python

import sys

import WinDeklar.QTAux as QTAux
import WinDeklar.WindowForm as WinForm

import FuzzyControl


class FuzzyControllerHost(WinForm.HostModel):
    """
    Shows car speed control
    """

    def __init__(self, controller_name):
        # keys (names used in the yaml definition file)
        self.error_key       = 'error'
        self.delta_error_key = 'delta_error'
        self.control_key     = 'control_name'
        self.output_key      = 'acceleration'

        self.error_graph_key       = 'error'
        self.delta_error_graph_key = 'delta_error'
        self.output_graph_key      = 'output'

        # particular data
        self.fuzzy_controller = FuzzyControl.FuzzyControl.create_from_file(controller_name)

        initial_values = {self.control_key: controller_name}
        # print('initial state:%s' % initial_values)
        super(FuzzyControllerHost, self).__init__(initial_values=initial_values)

    def update_view(self, figure, ax):
        error_value       = self.get_value(self.error_key)
        delta_error_value = self.get_value(self.delta_error_key)
        output_values     = self.fuzzy_controller.compute({'error': error_value, 'delta_error': delta_error_value})
        output_value      = output_values[self.output_key]
        if figure.name == self.error_graph_key:
            self.fuzzy_controller.view_fuzzy_variable(ax, self.error_key, value=error_value)
        elif figure.name == self.delta_error_graph_key:
            self.fuzzy_controller.view_fuzzy_variable(ax, self.delta_error_key, value=delta_error_value)
        elif figure.name == self.output_graph_key:
            self.fuzzy_controller.view_fuzzy_variable(ax, self.output_key, value=output_value)
        else:
            print('Graph %s not implemented yet' % figure.name)

    def on_mouse_move(self, event, ax):
        if event.xdata is None or event.ydata is None:
            return
        self.show_status_bar_msg('x:%.2f y:%.2f' % (event.xdata, event.ydata))


if __name__ == '__main__':
    app = QTAux.def_app()
    par_controller_name = WinForm.get_arg_value(1, None)
    def_controller_name = 'car_speed_control_delta.yaml' if par_controller_name is None else par_controller_name
    provider = FuzzyControllerHost(controller_name=def_controller_name)        # class to handle events
    WinForm.run_winform(__file__, provider)
    sys.exit(app.exec_())
