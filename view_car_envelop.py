#!/usr/bin/env python

import sys
import math

import WinDeklar.QTAux as QTAux
import WinDeklar.WindowForm as WinForm


class CarEnvelopHost(WinForm.HostModel):
    """
    Shows vehicle longitudinal and rotational speed envelop
    """

    def __init__(self):
        # keys (names used in the yaml definition file)
        self.max_v_key       = 'max_v'
        self.max_w_key       = 'max_w'
        self.max_lat_acc_key = 'max_lateral_acc'

        self.inc_w = 0.3

        # particular data
        super(CarEnvelopHost, self).__init__(initial_values={})

    def widget_changed(self, name, value):
        pass

    def update_view(self, figure, ax):
        if figure.name == 'speed_envelop':
            self.show_speed_envelop(ax)
        elif figure.name == 'acc_envelop':
            self.show_acc_envelop(ax, v=1.0, w=10)

    def show_acc_envelop(self, ax, v=1.0, w=10, lat_acc_inc=0.1, v_scale=10, line_width=1, color='Black', alpha=0.5):
        self.box_size.reset()
        self.box_size.add_point([0, 0])
        max_lateral_acc = self.get_value(self.max_lat_acc_key)/10
        max_v           = self.get_value(self.max_v_key)
        max_w           = self.get_value(self.max_w_key)
        max_lon_acc     = max_v - v
        max_lat_acc     = max_w - w
        lat_acc         = - max_lat_acc - lat_acc_inc
        while lat_acc <= max_lat_acc:
            lat_acc += lat_acc_inc
            w1      = w + lat_acc
            if abs(w1) > max_lat_acc:
                continue
            min_valid_lon_acc = 0.0
            max_valid_lon_acc = 0.0
            lon_acc = - v - lat_acc_inc
            while lon_acc <= max_lon_acc:
                lon_acc += lat_acc_inc
                v1 = v + lon_acc
                v_w_lat_acc = v1*math.radians(w1)
                if abs(v_w_lat_acc) > max_lateral_acc:
                    # not valid value
                    continue
                if lon_acc < min_valid_lon_acc:
                    min_valid_lon_acc = lon_acc
                if lon_acc > max_valid_lon_acc:
                    max_valid_lon_acc = lon_acc

            # print([[lat_acc, min_valid_lon_acc], [lat_acc, max_valid_lon_acc]])
            self.box_size.add_points([[lat_acc, min_valid_lon_acc], [lat_acc, max_valid_lon_acc]])
            ax.vlines(x=lat_acc, ymin=min_valid_lon_acc, ymax=max_valid_lon_acc, linewidth=line_width, colors=color,
                      alpha=alpha)

        self.box_size.set_bounds(ax, inc=1.1)

    def show_speed_envelop(self, ax, color='green', line_width=2, alpha=0.3, v_scale=10):
        self.box_size.reset()
        max_lateral_acc = self.get_value(self.max_lat_acc_key)/10
        max_v           = self.get_value(self.max_v_key)
        max_w           = self.get_value(self.max_w_key)
        w = - max_w
        while w <= max_w:
            v = min(abs(max_lateral_acc/math.radians(w)), max_v) if abs(w) > 0.001 else max_v
            v *= v_scale
            self.box_size.add_point([w, v])
            ax.vlines(x=w, ymin=0, ymax=v, linewidth=line_width, colors=color, alpha=alpha)
            w += self.inc_w
        self.box_size.set_bounds(ax, inc=1.1)


if __name__ == '__main__':
    app = QTAux.def_app()
    provider = CarEnvelopHost()        # class to handle events
    WinForm.run_winform(__file__, provider)
    sys.exit(app.exec_())
