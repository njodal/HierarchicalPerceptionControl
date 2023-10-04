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

    def initialize(self):
        """
        Set axis limits for all fuzzy variables and graphs
        :return:
        """
        for [key, scale] in [[self.error_key, 10], [self.delta_error_key, 100]]:
            slider = self.get_widget_by_name(key)
            if slider is None:
                continue
            a_min, a_max = self.fuzzy_controller.get_fuzzy_variable_min_max(key)
            slider.set_min_max(a_min*scale, a_max*scale)
        for graph in self.main_window.fig_views:
            fuzzy_variable = self.fuzzy_controller.get_fuzzy_variable(graph.name)
            if fuzzy_variable is None:
                continue

    def update_view(self, figure, ax):
        error_value       = self.get_value(self.error_key)
        delta_error_value = self.get_value(self.delta_error_key)
        input_values      = {self.error_key: error_value, self.delta_error_key: delta_error_value}
        output_values     = self.fuzzy_controller.compute(input_values)
        output_value      = output_values[self.output_key]
        for [graph_name, key, value] in [[self.error_graph_key, self.error_key, error_value],
                                         [self.delta_error_graph_key, self.delta_error_key, delta_error_value],
                                         [self.output_graph_key, self.output_key, output_value]]:
            if figure.name == graph_name:
                self.fuzzy_controller.view_fuzzy_variable(ax, key, value=value)
                figure.show_text([[key, '%.2f' % value]], position=[value, 0.5])


if __name__ == '__main__':
    app = QTAux.def_app()
    par_controller_name = WinForm.get_arg_value(1, None)
    def_controller_name = 'car_speed_control_delta.yaml' if par_controller_name is None else par_controller_name
    provider = FuzzyControllerHost(controller_name=def_controller_name)        # class to handle events
    WinForm.run_winform(__file__, provider)
    sys.exit(app.exec_())
