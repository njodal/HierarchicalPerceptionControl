#!/usr/bin/env python

import sys
import math

import WinDeklar.QTAux as QTAux
import WinDeklar.WindowForm as WinForm

import ControlUnit as cu


class CarEnvelopHost(WinForm.TestHost):
    """
    Shows vehicle longitudinal and rotational speed envelop
    """

    def __init__(self):
        # keys (names used in the yaml definition file)
        self.k_reference   = 'reference'
        self.k_disturbance = 'disturbance'
        self.k_dt          = 'dt'
        self.k_cycles      = 'cycles'
        self.k_debug       = 'debug'
        self.k_type        = 'type'
        self.k_output      = 'o'
        self.k_perception  = 'p'
        self.k_t           = 't'
        self.k_controller_params = ['p1', 'p2', 'p3']

        # particular data
        self.r           = 0.0
        self.d           = 0.0
        self.dt          = 0.1
        self.cycles      = 10
        self.control_def = None
        self.debug       = False
        self.parm_map    = []
        self.j = 0

        super(CarEnvelopHost, self).__init__(initial_values={}, base_dir=None)

    def update_view(self, figure, ax):
        if self.control_def is None:
            return
        values = self.run_simulation()
        # Set exactly the same position and size for both plots
        # [left, bottom, width, height] in figure coordinates (0 to 1)
        position = [0.125, 0.11, 0.775, 0.8]
        # ax.set_position(position)
        self.j += 1
        print(f'try {self.j}')
        figure.figure.clear()
        ax1 = figure.figure.add_axes(position)

        if figure.name == 'perception':
            # ax1.legend(title='Perception', loc='upper center')
            figure.figure.suptitle('Perception', fontsize='medium')
            self.show_values(ax1, values, [[self.k_perception, 'red'], [self.k_reference, 'black']])
        if figure.name == 'output':
            figure.figure.suptitle('Output', fontsize='medium')
            self.show_values(ax1, values, [[self.k_output, 'red'], [self.k_disturbance, 'black']])

    def run_simulation(self):
        cycles = int(self.get_value(self.k_cycles))
        r      = self.get_value(self.k_reference)
        d      = self.get_value(self.k_disturbance)
        dt     = self.get_value(self.k_dt)
        debug  = self.get_value(self.k_debug)
        control_unit = cu.create_control(self.control_def)
        self.set_control_unit_parameters(control_unit)
        values = [{self.k_t: 0.0, self.k_reference: r, self.k_disturbance: d, self.k_perception: 0.0,
                   self.k_output: 0.0}]
        cycle  = 1
        for [p, o] in cu.step_response_values(r, d, cycles, dt, debug, control_unit, control_delta=False):
            values.append({self.k_t: cycle*dt, self.k_reference: r, self.k_disturbance: d, self.k_perception: p,
                           self.k_output: o})
            cycle += 1
        return values

    def show_values(self, ax, values, ref_values, linewidth=1):
        self.box_size.reset()
        for i in range(1, len(values)):
            for [ref, ref_color] in ref_values:
                p0 = [values[i-1][self.k_t], values[i-1][ref]]
                p1 = [values[i][self.k_t], values[i][ref]]
                draw_line(ax, p0, p1, color=ref_color, box_size=self.box_size, linewidth=linewidth)
                self.box_size.add_points([p0, p1])
        self.box_size.set_bounds(ax, inc=1.1)

    def set_control_unit_parameters(self, control_unit):
        control_params = {k1: self.get_value(k) for [k, k1] in self.parm_map}
        control_unit.set_parameters(control_params)

    def set_ui_parameters(self, parameters):
        for [k, k1] in self.parm_map:
            if k1 not in parameters:
                continue
            self.set_value(k, parameters[k1])

    def set_case(self, test_input, output, test_description):
        r, d, cycles, dt, debug, self.control_def = test_input
        self.set_value(self.k_reference, r)
        self.set_value(self.k_disturbance, d)
        self.set_value(self.k_cycles, cycles)
        self.set_value(self.k_dt, dt)
        self.set_value(self.k_type, self.control_def.get(self.k_type, 'Unknown'))
        self.set_value(self.k_debug, debug)
        controller       = cu.create_control(self.control_def)
        controller_parms = controller.get_parameters()
        # change default values with controller specific values
        self.parm_map = []
        i = 0
        for k, v in controller_parms.items():
            if i >= len(self.k_controller_params):
                break
            widget_name = self.k_controller_params[i]
            ui_control  = self.get_widget_by_name(widget_name)
            ui_control.label.setText(k)
            ui_control.set_visible(True)
            self.set_value(widget_name, v)
            self.parm_map.append([widget_name, k])
            i += 1
        # hide not used default widgets
        for i in range(len(controller_parms), len(self.k_controller_params)):
            widget_name = self.k_controller_params[i]
            ui_control  = self.get_widget_by_name(widget_name)
            ui_control.set_visible(False)
        self.j = 0

        return self.get_current_description(test_description)

    def set_no_case(self):
        self.control_def = None

    def auto_tune(self):
        controller = cu.create_control(self.control_def)
        r          = self.get_value(self.k_reference)
        d          = self.get_value(self.k_disturbance)
        dt         = self.get_value(self.k_dt)
        auto_tune  = cu.AutoTuneControl(controller, reference=r, disturbance=d, dt=dt)
        best_parms = auto_tune.auto_tune(debug=False)
        cost       = auto_tune.run_one_episode(best_parms)
        print(f'best cost: {cost} parameters: {best_parms}')
        self.set_ui_parameters(best_parms)


def draw_line(ax, p1, p2, color='Black', linewidth=1, box_size=None):
    ax.plot([p1[0], p2[0]], [p1[1], p2[1]], color, linewidth=linewidth)
    if box_size is not None:
        box_size.add_points([p1, p2])


if __name__ == '__main__':
    app = QTAux.def_app()
    provider = CarEnvelopHost()        # class to handle events
    WinForm.run_winform(__file__, provider)
    sys.exit(app.exec_())
