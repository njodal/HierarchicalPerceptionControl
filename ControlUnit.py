

class PCUControlUnit:
    """
    Control Unit as used in Perceptual Control Theory:
        o = o + (kg*e - o)/ks
    """

    def __init__(self, name, gains, debug=False):
        self.name  = name
        self.kg    = gains[0]
        self.ks    = gains[1] if len(gains) > 1 else 1.0
        self.debug = debug
        self.o     = 0.0
        self.e     = 0.0

    def get_output(self, reference, perception):
        self.e = reference - perception
        self.o = self.o + (self.kg * self.e - self.o) / self.ks
        if self.debug:
            print('        %s r:%.2f p:%.2f e:%.2f o:%.4f' % (self.name, reference, perception, self.e, self.o))
        return self.o
