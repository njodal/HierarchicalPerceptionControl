#!/usr/bin/env python


import sys
import numpy as np
import skfuzzy as fz
from skfuzzy import control as ctrl
import FuzzyControl


def generic_control(file_name, debug=False):
    control = FuzzyControl.FuzzyControl.create_from_file(file_name, debug=debug)
    control.run_tests()


def tip_control_from_file(food_quality=6.5, service_quality=9.8, file_name='tipping_control.yaml'):
    control = FuzzyControl.FuzzyControl.create_from_file(file_name)
    output  = control.compute({'food_quality': food_quality, 'service_quality': service_quality})
    tip     = output['tip']
    print('Given quality of food is %.1f and service is %.1f the suggested_tip is %.0f (from file)' %
          (food_quality, service_quality, tip))
    return tip


def tip_control_low_level(food_quality=6.5, service_quality=9.8, debug=False):
    """
    Tipping example: given a value for quality of food and service, returns the suggested tip percentage
    http://pythonhosted.org/scikit-fuzzy/auto_examples/plot_tipping_problem_newapi.html#example-plot-tipping-problem-newapi-py
    :param service_quality:
    :param food_quality:
    :param debug:
    :return: suggested tipping percentage
    """
    # New Antecedent/Consequent objects hold universe variables and membership functions
    quality = ctrl.Antecedent(np.arange(0, 11, 1), 'quality')
    service = ctrl.Antecedent(np.arange(0, 11, 1), 'service')
    tip     = ctrl.Consequent(np.arange(0, 26, 1), 'tip')

    # Auto-membership function population is possible with .automf(3, 5, or 7)
    quality.automf(3)
    service.automf(3)

    # Custom membership functions can be built interactively with a familiar, Pythonic API
    tip['low']    = fz.trimf(tip.universe, [0, 0, 13])
    tip['medium'] = fz.trimf(tip.universe, [0, 13, 25])
    tip['high']   = fz.trimf(tip.universe, [13, 25, 25])

    if debug:
        # You can see how these look with .view()
        quality['average'].view()

    rule1 = ctrl.Rule(quality['poor'] | service['poor'], tip['low'])
    rule2 = ctrl.Rule(service['average'], tip['medium'])
    rule3 = ctrl.Rule(service['good'] | quality['good'], tip['high'])

    tipping_ctrl = ctrl.ControlSystem([rule1, rule2, rule3])
    tipping      = ctrl.ControlSystemSimulation(tipping_ctrl)

    # Pass inputs to the ControlSystem using Antecedent labels with Pythonic API
    # Note: if you like passing many inputs all at once, use .inputs(dict_of_data)
    tipping.input['quality'] = food_quality
    tipping.input['service'] = service_quality

    # Crunch the numbers
    tipping.compute()

    suggested_tip = tipping.output['tip']

    print('Given quality of food is %.1f and service is %.1f the suggested_tip is %.0f (low level)' %
          (food_quality, service_quality, suggested_tip))
    return suggested_tip


"""
# fuzzy_control = FuzzyControl.create_from_file('SimpleTest.yaml')
# fuzzy_control.run_tests(False)
print(rule_code('angle:rb & p:vl', True))
print(rule_code('w:hr', False))
print(string_to_dict('angle:90 p:100'))
print(string_to_dict('angle:-90 p:100'))

fuzzy_control = FuzzyControl.create_from_file('WallFollowingRules2.yaml')
fuzzy_control.run_tests(False)
"""

if __name__ == '__main__':
    # tip_control_low_level()
    # tip_control_from_file()
    par_file_name = sys.argv[1] if len(sys.argv) > 1 else 'tipping_control.yaml'
    generic_control(par_file_name, debug=True)
