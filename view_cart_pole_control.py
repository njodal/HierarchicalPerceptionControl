#!/usr/bin/env python

import sys
import time
import math

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

    def __init__(self, default_directory='/tmp', file_extension='yaml'):
        # keys (names used in the yaml definition file)
        self.angle_ref_key   = 'angle_ref'
        self.pos_ref_key   = 'pos_reference'
        self.kg_key        = 'kg'
        self.ks_key        = 'ks'
        self.kd_key        = 'kd'
        self.max_iter_key  = 'max_iter'

        self.action_key = 'action'

        # particular data
        self.action_name        = 'Action'
        self.last_action_number = 0
        self.directory          = default_directory
        self.file_extension     = file_extension
        self.file_filter        = '*.%s' % self.file_extension
        self.file_name          = None

        initial_values = {}  # used in case some control have an initial value programmatically
        super(CarPoleMoveHost, self).__init__(initial_values=initial_values)

    def update_view(self, _, ax):
        """
        Update the figure
        Notes:
            - This method is called when any property changes or refresh() is used
            - All form variables are in dict self.state
        :param ax:
        :return:
        """
        angle_ref  = math.radians(self.state.get(self.angle_ref_key, 0.0))
        first_g  = [self.state.get(self.kg_key, 1.0), self.state.get(self.ks_key, 100),
                    self.state.get(self.kd_key, 2.5)]
        # first_g  = [86.0, 76.0, 71.0]
        gains    = [[0.5], [2.0], [-0.05, 4.0]]  # [[6.0], [0.2], [-0.1, 4.0]]
        gains.insert(0, first_g)
        max_iter  = self.state.get(self.max_iter_key, 1000)
        debug     = True
        steps, error_history, _, summary = CarPole.run_one_pole_control(angle_ref, gains, None, [], max_iter=max_iter,
                                                                        debug=debug)
        errors_degrees = [[x, math.degrees(y)] for [x, y] in error_history]
        ref_signal = [[0.0, 0.0], [max_iter, 0.0]]
        ga.graph_points(ax, ref_signal, scale_type='tight', x_visible=True, y_visible=True, color='Black')
        ga.graph_points(ax, errors_degrees, scale_type='tight', x_visible=True, y_visible=True)
        errors_degrees.extend(ref_signal)
        self.resize_figure(ax, errors_degrees)

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
        self.set_and_refresh_control(self.action_key, value)

    # particular code
    def open_yaml_file(self, file_name, progress_bar):
        """
        Example on how to use the progress bar
        :param file_name:
        :param progress_bar: use to give feedback about the opening process
        :return:
        """
        file = ya.get_yaml_file(file_name, must_exist=True, verbose=True)
        self.state.update(file['state'])
        print(self.state)
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
        record.write_group('state', self.state, level=0)

        # just an example of how to use the ProgressBar, no actually needed in this case
        progress_bar_example(progress_bar, max_value=100, inc=20, sleep_time=0.2)

        msg = '%s saved' % file_name
        self.show_status_bar_msg(msg)


def progress_bar_example(progress_bar, max_value=100, inc=20, sleep_time=0.2):
    progress_bar.set_max(max_value)
    for i in range(0, max_value, inc):
        progress_bar.set_value(i)
        time.sleep(sleep_time)


if __name__ == '__main__':
    app = QTAux.def_app()
    provider = CarPoleMoveHost()        # class to handle events
    WinForm.run_winform(__file__, provider)
    sys.exit(app.exec_())
