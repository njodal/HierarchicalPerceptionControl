import random as rn


class Ecoli:
    """
    Implements EColi learning algorithms as defined in Perceptual Control Theory
    General idea:
        * given a set of parameters to optimize, randomly change any of it a little
        * the rate of change depends on how close is the output to the goal
            if function is getting closer not change to much, if it getting farther increase the rate of change
    """

    def __init__(self, function, rate_of_change=10, max_change_percentage=0.1):
        self.function              = function
        self.rate_of_change        = rate_of_change
        self.max_change_percentage = max_change_percentage
