
import WinDeklar.graph_aux as ga


class RealTimeObjectAttributeDataProvider(ga.RealTimeDataProvider):
    """
    Returns the real time value of a given attribute of a given object as needed in an animated graph
    Note: object must implement get_value()
    """

    def __init__(self, provider_class, attribute_key, dt=0.1, min_y=-2.0, max_y=10.0, color='Black'):
        """
        Initialization
        :param provider_class: any class that implements get_value(value_key)
        :param attribute_key:  name of the attribute to return (speed, accelerator_pedal, etc.)
        :param dt:
        :param min_y:
        :param max_y:
        :param color:
        """
        self.provider_class = provider_class
        self.attribute_key  = attribute_key
        super(RealTimeObjectAttributeDataProvider, self).__init__(dt=dt, min_y=min_y, max_y=max_y, color=color)

    def get_next_values(self, _):
        x       = self.t
        value   = self.provider_class.get_value(self.attribute_key)
        self.t += self.dt
        return x, value
