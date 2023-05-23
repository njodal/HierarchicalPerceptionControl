#!/usr/bin/env python

import yaml_functions as yf
import sys
from enum import IntEnum

k_general   = 'general'
k_tests     = 'tests'
k_test      = 'test'
k_name      = 'name'
k_call      = 'call'
k_equal     = 'equality'
k_precision = 'precision'
k_cases     = 'cases'
k_case      = 'case'
k_input     = 'input'
k_output    = 'output'
k_desc      = 'desc'


class ShowResult(IntEnum):
    Summary    = 0
    OnlyFailed = 1
    Detailed   = 2


class UnitTest:

    def run_tests(self, given_test_name, show_result):
        all_tests = self.test[k_general]
        all_tests1 = all_tests[k_tests]
        for test_item in all_tests1:
            test = test_item[k_test]
            if k_call not in test:
                raise Exception('Missing function to call in test')
            test_name = test.get(k_name, test[k_call])
            if given_test_name is None or given_test_name == '' or test_name == given_test_name:
                print('Test: %s' % test_name)
                precision        = test.get(k_precision, 0.01)
                function_name    = test[k_call]
                function_to_test = getattr(self.module, function_name)

                all_ok     = 0
                all_failed = 0

                for case_item in test[k_cases]:
                    case = case_item[k_case]
                    test_input    = case.get(k_input, [])
                    valid_output  = case.get(k_output, [])
                    actual_output = function_to_test(*test_input)
                    if k_output not in case:
                        # if no output is defined in test the intention was just to show the output
                        all_ok += 1
                        # if show_result == ShowResult.Detailed:
                        print('      i:%s o:%s' % (test_input, actual_output))
                    elif is_same_values(valid_output, actual_output, precision=precision):
                        all_ok += 1
                        if show_result == ShowResult.Detailed:
                            print('      i:%s o:%s Ok!' % (test_input, actual_output))
                    else:
                        all_failed += 1
                        if show_result in [ShowResult.Detailed, ShowResult.OnlyFailed]:
                            print('      i:%s valid o:%s actual o:%s Failed' %
                                  (test_input, valid_output, actual_output))
                            if k_desc in case:
                                print('          %s' % case[k_desc])
                if all_failed == 0:
                    print('   summary: all tests (%s) ok!' % all_ok)
                else:
                    print('   summary: %s tests failed, ok %s (total %s) ' % (all_failed, all_ok, all_failed + all_ok))

    def __init__(self, module_name, test_file_name, test_name=None, show_result=ShowResult.OnlyFailed):
        self.module        = sys.modules[module_name]
        self.test          = yf.get_yaml_file(test_file_name)
        cmd_line_test_name = get_test_name_from_cmd_line()
        test_name1         = cmd_line_test_name if cmd_line_test_name is not None else test_name
        self.run_tests(test_name1, show_result)


def get_test_result_msg(valid, actual, precision=0.01):
    """
    Returns a message according the result of a test.
    Parameters:
        valid  - test valid value
        actual - test actual value
        precision - precision used for comparison
    """
    if is_same_values(valid, actual, precision=precision):
        return [['result', 'Ok!'], ['output', valid]]
    else:
        return [['result', 'Failed'], ['valid', valid], ['actual', actual]]


def is_same_values(value1, value2, precision=0.001):
    # test for equal values but depending on the type of values
    # print('v1:%s v2:%s' % (value1, value2))
    if is_float(value1) or is_float(value2):
        # print('one is float')
        if value1 == 'None' or value2 == 'None':
            return False
        return similar_values(value1, value2, precision)
    elif is_list(value1) or is_list(value2):
        return is_same_lists(value1, value2, precision)
    else:
        return value1 == value2


def is_float(number):
    return isinstance(number, float)


def is_list(list1):
    return isinstance(list1, list)


def similar_values(v1, v2, precision=0.001):
    return abs(v1 - v2) <= precision


def is_same_lists(l1, l2, precision):
    if l1 is None:
        return True if l2 is None else False
    if l2 is None:
        return True if l1 is None else False
    if len(l1) != len(l2):
        return False
    for i, value1 in enumerate(l1):
        value2 = l2[i]
        if not is_same_values(value1, value2, precision=precision):
            # print('v1: %s v2:%s' % (value1, value2))
            return False
    return True


def get_test_name_from_cmd_line():
    if len(sys.argv) < 2:
        return None
    else:
        return sys.argv[1]
