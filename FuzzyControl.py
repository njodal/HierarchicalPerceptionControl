#!/usr/bin/env python

import re
import numpy as np
import matplotlib.pyplot as plt
import skfuzzy as fuzz
from skfuzzy import control as ctrl
import yaml_functions as yaml


class FuzzyControl:
    """
    Manage a Fuzzy Controller
    """

    # statics methods
    @classmethod
    def create_from_file(cls, file_name, directory='fuzzy_rules', debug=False):
        """
        Creates a FuzzyControl object from an YAML file
        :param directory:
        :param file_name:
        :param debug:
        :return:
        """
        fuzzy_file = yaml.get_yaml_file(file_name, directory=directory)

        name         = fuzzy_file['general'].get('name', 'No Name')
        description  = fuzzy_file['general'].get('description', '')
        colors       = fuzzy_file['general'].get('membership_colors', [])
        delta_output = fuzzy_file['general'].get('output_control_delta', False)

        # create antecedent & consequent
        antecedents = define_fuzzy_variables(fuzzy_file['antecedents'], True)
        consequents = define_fuzzy_variables(fuzzy_file['consequents'], False)

        rules = define_rules(fuzzy_file.get('rules', []), antecedents, consequents)
        tests = define_tests(fuzzy_file.get('tests', []))

        return FuzzyControl(name, description, rules, antecedents, consequents, delta_output, tests, colors,
                            debug=debug)

    def __init__(self, name, description, rules, antecedents, consequents, delta_output, tests, colors, debug=False):
        self.name        = name
        self.description = description
        self.delta       = delta_output
        self.colors      = colors
        self.rules       = rules
        self.antecedents = antecedents
        self.consequents = consequents
        self.tests       = tests
        self.control     = ctrl.ControlSystem(self.rules)
        self.simulation  = ctrl.ControlSystemSimulation(self.control)
        if debug:
            self.view_memberships()

    def compute(self, input_values):
        for k, v in input_values.items():
            if k in self.antecedents:
                # print('   check for %s:%.2f (min:%.2f max:%.2f)' % (k, v, self.antecedents[k].universe[0],
                #                                                    self.antecedents[k].universe[-1]))
                if v < self.antecedents[k].universe[0]:
                    input_values[k] = self.antecedents[k].universe[0]
                    print('    min value reached in %s:%.2f (min:%.2f)' % (k, v, input_values[k]))
                if v > self.antecedents[k].universe[-1]:
                    input_values[k] = self.antecedents[k].universe[-1]

        self.simulation.inputs(input_values)
        self.simulation.compute()
        return self.simulation.output

    # tests
    def add_test(self, test):
        self.tests.append(test)

    def run_tests(self, print_state=False):
        print('Tests for %s' % self.description)

        for test in self.tests:
            inputs, output, comment = test
            self.print_outs(inputs, output, comment)
        if print_state:
            self.simulation.print_state()

    def print_outs(self, inputs, output, comment):
        print('inputs: %s' % inputs)
        outputs = self.compute(inputs)
        in_str  = self.dict_string(inputs)
        out_str = self.dict_string(outputs)
        print('   if %s then %s (valid: %s) desc: %s' % (in_str, out_str, output, comment))

    def view_memberships(self):
        """
        Show the memberships functions of all antecedent and consequent fuzzy variables
        :return:
        """
        fuzzy_variables = [a for a in self.control.antecedents]
        fuzzy_variables.extend([c for c in self.control.consequents])
        fig, axis = plt.subplots(nrows=len(fuzzy_variables), figsize=(8, 9))
        for i, fuzzy_variable in enumerate(fuzzy_variables):
            view_fuzzy_variable(axis[i], fuzzy_variable, self.colors)
        show_window(axis, self.name)

    @staticmethod
    def dict_string(dictionary):
        out_str = ''
        for k in dictionary:
            value     = dictionary[k]
            str_value = '%.2f' % value if isinstance(value, float) else value
            out1      = '%s:%s ' % (k, str_value)
            out_str  += out1
        return out_str


def define_fuzzy_variables(vars_config, is_antecedent):
    fuzzy_variables = {}
    for var_data in vars_config:
        var_config = var_data['variable']
        var_name   = var_config['name']
        min_value  = var_config['min']
        max_value  = var_config['max']
        inc        = var_config['inc']

        var_range = np.arange(min_value, max_value + inc, inc)
        fuzzy_var = ctrl.Antecedent(var_range, var_name) if is_antecedent else ctrl.Consequent(var_range, var_name)

        adjectives = var_config['adjectives']
        for adjective_data in adjectives:
            adjective  = adjective_data['adjective']
            adj_name   = adjective['name']
            par_values = adjective['values']
            values     = par_values if isinstance(par_values, list) else eval(par_values)
            if len(values) == 3:
                fuzzy_var[adj_name] = fuzz.trimf(fuzzy_var.universe, values)
            elif len(values) == 4:
                fuzzy_var[adj_name] = fuzz.trapmf(fuzzy_var.universe, values)
            else:
                print('error: values len (%s) not implemented' % (len(values)))

        print('  universe for %s %s' % (var_name, fuzzy_var.universe))
        fuzzy_variables[var_name] = fuzzy_var
    return fuzzy_variables


def define_rules(rules_config, antecedents, consequents, debug=True):
    """
    Returns the rules as needed
    ex:     - rule: 'error:pb, delta_error:z  => acceleration:accelerate'
            ctrl.Rule(antecedents['error']['pb'] & antecedents['delta_error']['z'],
                      consequents['acceleration']['accelerate'])
    Warning: antecedents and consequents need to be present because they are used in eval
    :param rules_config:
    :param antecedents:
    :param consequents:
    :param debug:
    :return:
    """
    # - if:   'angle:rb & p:vl'
    #  then: 'w:hr'
    # rules.append(ctrl.Rule(antecedent['angle']['rb'] & antecedent['p']['vl'],
    #                       consequent['w']['hr']))
    rules = []
    for rule_config in rules_config:
        rule_string = rule_config['rule']
        tokens      = rule_string.split('=>')
        if debug:
            print('  rule tokens %s' % tokens)
        if len(tokens) != 2:
            print('  Warning: rule %s not valid, ignored' % rule_string)

        if_code   = rule_code(tokens[0], True)
        then_code = rule_code(tokens[1], False)

        if debug:
            print('   if %s then %s' % (if_code, then_code))
        rules.append(ctrl.Rule(eval(if_code), eval(then_code)))
    return rules


def define_tests(tests_config, comment_key='comment', output_key='output'):
    tests = []
    for test_config1 in tests_config:
        test_config = test_config1['test']
        inputs = string_to_dict(test_config['inputs'])

        comment = test_config.get(comment_key, '')
        output  = test_config.get(output_key, None)

        tests.append((inputs, output, comment))
    return tests


def rule_code(rule_string, is_antecedent):
    """
    Converts 'angle:rb & p:vl' to "antecedent['angle']['rb'] & antecedent['p']['vl']
    :param rule_string:
    :param is_antecedent:
    :return:
    """
    dict_name  = 'antecedents' if is_antecedent else 'consequents'
    rule_dict  = string_to_dict(rule_string)
    separator  = ''
    rule_code1 = ''
    for k, v in rule_dict.items():
        rule_code1 += "%s %s['%s']['%s']" % (separator, dict_name, k, v)
        separator   = ' &'
    # print('   rule code:%s' % rule_code1)
    # print('rule string %s code %s' %(rule_string, rule_code1))
    return rule_code1


def string_to_dict(input_string, delimiters=',|&| '):
    """
    Converts 'angle:90 p:100' to {'angle': 90, 'p':100}
    :param delimiters:
    :param input_string:
    :return: :type dict
    """
    var_dict = {}
    tokens = [item for item in re.split(delimiters, input_string) if item]
    # print('tokens: %s' % tokens)
    for token in tokens:
        kv = token.split(':')
        if len(kv) != 2:
            print('  invalid token %s' % token)
        var_dict[kv[0]] = string_to_value(kv[1])
    # print(var_dict)
    return var_dict


def string_to_value(string):
    # returns a number if it's possible, if not just returns a string
    try:
        return float(string)
    except ValueError:
        return string


def view_fuzzy_variable(ax, fuzzy_variable, colors):
    for j, term in enumerate(fuzzy_variable.terms):
        color = colors[j] if j < len(colors) else 'black'
        ax.plot(fuzzy_variable.universe, fuzzy_variable[term].mf, color, linewidth=1.5, label=term)
    ax.set_title(fuzzy_variable.label)
    ax.legend()


def show_window(axis, name):
    # Turn off top/right axes
    for ax in axis:
        ax.spines['top'].set_visible(False)
        ax.spines['right'].set_visible(False)
        ax.get_xaxis().tick_bottom()
        ax.get_yaxis().tick_left()

    plt.get_current_fig_manager().set_window_title(name)
    plt.tight_layout()
    plt.show()
