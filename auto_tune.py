

class Ecoli:
    """
    Attention: not implemented yet
    Implements EColi learning algorithms as defined in Perceptual Control Theory
    General idea:
        * given a set of parameters to optimize, randomly change any of it a little
        * the rate of change depends on how close is the output to the goal
            if function is getting closer not change to0 much, if it's getting farther increase the rate of change
    """

    def __init__(self, function, rate_of_change=10, max_change_percentage=0.1):
        self.function              = function
        self.rate_of_change        = rate_of_change
        self.max_change_percentage = max_change_percentage


def twiddle(function_object, threshold=0.1, change=10.0, max_iterations=1000, good_inc=1.1, bad_inc=2.0, mid_inc=1.05,
            dec_inc=0.95, debug=False):
    """
    Twiddle Learning algorithm
            see: https://martin-thoma.com/twiddle/
     given a function with parameters, optimize the parameters
     function is actually an Object who need to implement:
        Object.get_parameters()
            dict with the parameters names and initial values
        Object.function(parameters)
            is the real function to optimize (with its parameters), must return a value
        Object.is_better(value1, value2)
            return True if value1 is better than value2
        Object.is_best(value)
            return True if value is the best possible one

    :param function_object:
    :param threshold: stop searching if sum of changes is less than this
    :param change:    max changing value
    :param max_iterations: don't try more than this
    :param good_inc: percentage to increase value in case of improvement
    :param bad_inc:  idem for non improvement
    :param mid_inc:
    :param dec_inc:
    :param debug:
    :return:
    """

    # init
    p  = function_object.get_parameters()
    dp = {}
    for k, _ in p.items():
        dp[k] = change

    best_err = function_object.function(p)
    best_p   = p

    j = 0  # for returning the iteration that succeeded
    for j in range(0, max_iterations):
        sdp = dict_sum(dp)
        if sdp <= threshold:
            if debug:
                print('threshold %s reached at %s (s:%s err:%s) ' % (threshold, j, sdp, best_err))
            break
        elif function_object.is_best(best_err):
            if debug:
                print('best error (%s) reached at %s' % (best_err, j))
            break

        for k, v in p.items():
            p[k] += dp[k]    # change one parameter
            err   = function_object.function(p)  # try what happens
            if function_object.is_better(err, best_err):
                # There was some improvement (best result so far), increment the change of this parameter
                best_err = err
                best_p   = p
                dp[k]   *= good_inc
                # print 'best %s' %(best_err)
            else:
                # There was no improvement, try a change in the opposite direction
                p[k] -= bad_inc*dp[k]  # Go into the other direction
                err   = function_object.function(p)

                if function_object.is_better(err, best_err):
                    # There was some improvement
                    best_err = err
                    best_p   = p
                    dp[k]   *= mid_inc
                    # print 'bad best %s' %(best_err)
                else:  # There was no improvement
                    p[k] += dp[k]
                    # As there was no improvement, the step size in either
                    # direction, the step size might simply be too big.
                    dp[k] *= dec_inc
                    # print 'bad bad %s'  %(best_err)
    return best_err, best_p, j


def dict_sum(a_dict):
    s = 0
    for _, v in a_dict.items():
        s += v
    return s
