#!/usr/bin/env python

import sys
import time

import WinDeklar.WindowForm as WinForm
import WinDeklar.graph_aux as ga
import WinDeklar.QTAux as QTAux
import WinDeklar.record as rc
import WinDeklar.yaml_functions as ya

import CartPole as CarPole


class CarPoleMoveHost(WinForm.HostModel):
    """
    Visualization of control a Cart Pole moving to a given place
    """

    def __init__(self, control_def_file_name, default_directory='/tmp', file_extension='yaml'):
        # keys (names used in the yaml definition file)
        self.pos_ref_key   = 'pos_reference'
        self.kg_key        = 'kg'
        self.ks_key        = 'ks'
        self.max_iter_key  = 'max_iter'
        self.max_angle_key = 'max_angle'

        self.check1_key = 'check1'
        self.points_key = 'points'
        self.axis_key   = 'show_axis'
        self.type_key   = 'graph_type'
        self.action_key = 'action'
        self.width_key  = 'line_width'

        # particular data
        self.control_def_file_name = control_def_file_name
        self.action_name        = 'Action'
        self.last_action_number = 0
        self.directory          = default_directory
        self.file_extension     = file_extension
        self.file_filter        = '*.%s' % self.file_extension
        self.file_name          = None

        initial_values = {}  # used in case some control have an initial value programmatically
        super(CarPoleMoveHost, self).__init__(initial_values=initial_values)

    def update_view(self, figure, ax):
        pos_ref  = self.get_value(self.pos_ref_key)
        first_g  = [self.get_value(self.kg_key), self.get_value(self.ks_key)]
        gains    = [[3.0], [6.0], [0.2], [-0.1, 4.0]]
        gains.insert(0, first_g)
        max_iter  = self.get_value(self.max_iter_key)
        max_angle = self.get_value(self.max_angle_key)
        state     = {}
        render    = False
        debug     = True
        steps, error_history, _, summary = CarPole.run_one_move_cart(pos_ref, self.control_def_file_name, state, render,
                                                                     max_iter, max_angle=max_angle, debug=debug)
        ref_signal = [[0.0, 0.0], [max_iter, 0.0]]
        ga.graph_points(ax, ref_signal, scale_type='tight', x_visible=True, y_visible=True, color='Black')
        ga.graph_points(ax, error_history, scale_type='tight', x_visible=True, y_visible=True)
        error_history.extend(ref_signal)
        figure.resize_axis(error_history)

        self.show_status_bar_msg(summary)

    def redraw(self):
        """
        Event defined in the yaml file to be called when button redraw is pressed
        :return:
        """
        self.refresh()

    # actions
    def event_open_file(self):
        """
        Event defined in the yaml file to be called when File/Open is clicked
        :return:
        """
        file_name = self.get_file_name(title='Open an YAML', file_filter=self.file_filter, directory=self.directory)
        if file_name is None:
            return
        self.file_name = file_name
        # particular code to open the file
        self.open_yaml_file(self.file_name, progress_bar=self.get_progress_bar())

    def event_save_file_as(self):
        file_name = self.get_file_name_to_save(title='Save File', file_filter=self.file_filter,
                                               directory=self.directory)
        if file_name is None:
            return
        self.save_file(file_name, progress_bar=self.get_progress_bar())

    def event_save_file(self):
        if self.file_name is None:
            self.event_save_file_as()
        else:
            self.save_file(self.file_name, progress_bar=self.get_progress_bar())

    def change_action(self):
        self.last_action_number += 1
        value = '%s %s' % (self.action_name, self.last_action_number)
        self.set_and_refresh_widget(self.action_key, value)

    def on_mouse_move(self, event, ax):
        if event.xdata is None or event.ydata is None:
            return
        self.show_status_bar_msg('x:%.2f y:%.2f' % (event.xdata, event.ydata))

    # particular code
    def open_yaml_file(self, file_name, progress_bar):
        """
        Example on how to use the progress bar
        :param file_name:
        :param progress_bar: use to give feedback about the opening process
        :return:
        """
        file = ya.get_yaml_file(file_name, must_exist=True, verbose=True)
        self.set_values(file['state'])
        self.refresh()

        # just an example of how to use the ProgressBar, no actually needed in this case
        progress_bar_example(progress_bar, max_value=100, inc=20, sleep_time=0.2)

        msg = '%s opened' % file_name
        self.show_status_bar_msg(msg)

    def save_file(self, file_name, progress_bar):
        """
        Example on how to use the progress bar
        :param file_name:
        :param progress_bar: use to give feedback about the opening process
        :return:
        """
        record = rc.Record(file_name, dir=None, add_time_stamp=False)
        record.write_ln('version: 1')  # just to avoid warnings with editing in pycharm
        record.write_group('state', self._state, level=0)

        # just an example of how to use the ProgressBar, no actually needed in this case
        progress_bar_example(progress_bar, max_value=100, inc=20, sleep_time=0.2)

        msg = '%s saved' % file_name
        self.show_status_bar_msg(msg)

    def get_graph_points(self):
        """
        Returns the points (x,y) to be graphed depending on the graph type and number of points
        :return:
        """
        function_name    = self.get_value(self.type_key)
        number_of_points = int(self.get_value(self.points_key))
        points, msg      = ga.graph_points_for_many_functions(function_name, number_of_points)
        if points is None:
            self.show_status_bar_msg('%s not implemented' % function_name)
            points = []
        return points


def progress_bar_example(progress_bar, max_value=100, inc=20, sleep_time=0.2):
    progress_bar.set_max(max_value)
    for i in range(0, max_value, inc):
        progress_bar.set_value(i)
        time.sleep(sleep_time)


if __name__ == '__main__':
    app = QTAux.def_app()
    par_control_type       = WinForm.get_arg_value(1, None)
    control_def_file_name1 = 'pct_cart_pole_move.yaml' if par_control_type is None else par_control_type
    provider = CarPoleMoveHost(control_def_file_name1)        # class to handle events
    WinForm.run_winform(__file__, provider)
    sys.exit(app.exec_())
