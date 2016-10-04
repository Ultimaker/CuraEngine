#!/usr/bin/python3

## runtest.py
# The runtest.py script runs regression tests on the CuraEngine.
# It parses the json file for settings to know which settings can be passed towards the engine.
# It runs the following test:
# * Defaults
# * Bounds/possible values
# * Single random value
# * All settings random

import ast #For safe function evaluation.
import math #For evaluating setting inheritance functions.
import sys
import subprocess
import os
import time
import stat
import argparse
import random
import json
import threading
from xml.etree import ElementTree


# Mock function that occurs in fdmprinter.def.json
# Use wrapper to provide locals
def extruderValueWrapper(_locals):
    def extruderValue(extruder_nr, parameter):
        return eval(parameter, globals(), _locals)
    return extruderValue


# Mock function that occurs in fdmprinter.def.json
# Use wrapper to provide locals
def extruderValuesWrapper(_locals):
    def extruderValues(parameter):
        return [eval(parameter, globals(), _locals)]
    return extruderValues


# Mock function that occurs in fdmprinter.def.json
# Use wrapper to provide locals
def resolveOrValueWrapper(_locals):
    def resolveOrValue(parameter):
        return eval(parameter, globals(), _locals)
    return resolveOrValue


## The TestSuite class stores the test results of a single set of tests.
#  TestSuite objects are created by the TestResults class.
class TestSuite:
    def __init__(self, name):
        self._name = name
        self._successes = []
        self._failures = []
    
    ## Add a successful test result to the test suite.
    def success(self, class_name, test_name):
        self._successes.append((class_name, test_name))

    ## Add a failed test result to the test suite.
    def failure(self, class_name, test_name, error_message):
        self._failures.append((class_name, test_name, error_message))

    ## Return the number of tests in this test suite
    def getTestCount(self):
        return self.getSuccessCount() + self.getFailureCount()

    ## Return the number of successful tests in this test suite
    def getSuccessCount(self):
        return len(self._successes)

    ## Return the number of failed tests in this test suite
    def getFailureCount(self):
        return len(self._failures)


## The TestResults class stores a group of TestSuite objects, each TestSuite object contains failed and successful test.
#  This class can output the result of the tests in a JUnit xml format for parsing in Jenkins.
class TestResults:
    def __init__(self):
        self._testsuites = []
    
    ## Create a new test suite with the name.
    def addTestSuite(self, name):
        suite = TestSuite(name)
        self._testsuites.append(suite)
        return suite

    def getFailureCount(self):
        result = 0
        for testsuite in self._testsuites:
            result += testsuite.getFailureCount()
        return result

    ## Save the test results to the file given in the filename.
    def saveXML(self, filename):
        xml = ElementTree.Element("testsuites")
        xml.text = "\n"
        for testsuite in self._testsuites:
            testsuite_xml = ElementTree.SubElement(xml, "testsuite", {"name": testsuite._name, "errors": "0", "tests": str(testsuite.getTestCount()), "failures": str(testsuite.getFailureCount()), "time": "0", "timestamp": time.strftime("%Y-%m-%dT%H:%M:%S", time.gmtime())})
            testsuite_xml.text = "\n"
            testsuite_xml.tail = "\n"
            for class_name, test_name in testsuite._successes:
                testcase_xml = ElementTree.SubElement(testsuite_xml, "testcase", {"classname": class_name, "name": test_name})
                testcase_xml.text = "\n"
                testcase_xml.tail = "\n"
            for class_name, test_name, error_message in testsuite._failures:
                testcase_xml = ElementTree.SubElement(testsuite_xml, "testcase", {"classname": class_name, "name": test_name})
                testcase_xml.text = "\n"
                testcase_xml.tail = "\n"
                failure = ElementTree.SubElement(testcase_xml, "failure", {"message": "test failure"})
                failure.text = error_message
                failure.tail = "\n"
        return ElementTree.ElementTree(xml).write(filename, "utf-8", True)


class Setting:
    ##  Creates a new setting from a JSON node.
    #
    #   Some parts of the setting may have to be evaluated as functions. For
    #   these, the default values of all settings are added as local variables.
    #
    #   \param key The name of the setting.
    #   \param data The JSON node of the setting, containing the default value,
    #   the setting type, the minimum value, maximum value, the minimum warning
    #   value, the maximum warning value and (for enum types) the options.
    #   \param locals The local variables for eventual function evaluation.
    def __init__(self, key, data, locals):
        self._key = key
        if "value" in data: #Evaluate "value" if we can, otherwise just take default_value.
            self._default = self._evaluateFunction(data.get("value", "0"), locals)
        else:
            self._default = data.get("default_value", 0)
        self._type = data["type"]
        self._min_value = self._evaluateFunction(data.get("minimum_value", None), locals)
        self._max_value = self._evaluateFunction(data.get("maximum_value", None), locals)
        self._min_value_warning = self._evaluateFunction(data.get("minimum_value_warning", None), locals)
        self._max_value_warning = self._evaluateFunction(data.get("maximum_value_warning", None), locals)
        self._options = data.get("options", None)
        if self._options is not None:
            self._options = list(self._options.keys())

    ##  Gets the default value of this setting, according to the source JSON.
    #
    #   \return The default value.
    def getDefault(self):
        return self._default

    ## Return a list of possible values for this setting. This list depends on the setting type.
    #  For number values it contains the minimal and maximal values.
    #  For enums and booleans it will contain the exact possible values.
    #  For string settings only the default value is returned.
    def getSettingValues(self):
        if self._type == "bool":
            return ["True", "False"]
        if self._type == "float" or self._type == "int":
            ret = [self._default]
            if self._min_value is not None:
                ret.append(self._min_value)
            else:
                ret.append(-2)
            if self._min_value_warning is not None:
                ret.append(self._min_value_warning)
            if self._max_value is not None:
                ret.append(self._max_value)
            else:
                if self._max_value_warning is None:
                    ret.append(10000)
                elif self._type == "float":
                    ret.append(float(self._max_value_warning) + 100)
                    ret.append(float(self._max_value_warning) * 10)
                elif self._type == "int":
                    ret.append(int(self._max_value_warning) + 100)
                    ret.append(int(self._max_value_warning) * 10)
                # If the type is boolean, string or enum, the warning values make no sense anyway, so don't test them.
            if self._max_value_warning is not None:
                ret.append(self._max_value_warning)
            
            if self._type == "int":
                for n in range(0, len(ret)):
                    ret[n] = int(ret[n])
            return ret
        if self._type == "enum":
            return self._options
        if self._type == "str":
            return [self._default]
        if self._type == "extruder":
            return [self._default] # TODO: also allow for other values below machine_extruder_count
        if self._type == "polygon":
            return [self._default]
        if self._type == "polygons":
            return [self._default]
        print("Unknown setting type:", self._type)

    ## Return a random value for this setting. The returned value will be a valid value according to the settings json file.
    def getRandomValue(self):
        if self._type == "float" or self._type == "int":
            min = -2
            if self._min_value_warning is not None:
                min = self._min_value_warning
            if self._min_value is not None:
                min = self._min_value
            max = 10000
            if self._max_value_warning is not None:
                max = self._max_value_warning
            if self._max_value is not None:
                max = self._max_value
            if self._type == "int":
                return random.randint(int(min), int(max))
            return random.uniform(float(min), float(max))
        return random.choice(self.getSettingValues())

    ##  Evaluates a setting value that is described as a function.
    #
    #   Note that this function should behave EXACTLY the same as it does in
    #   UM/Settings/Setting.py:_createFunction. The only differences should be
    #   that this evaluation always uses the default values instead of the
    #   current profile values, and that this function directly evaluates the
    #   setting instead of returning a function with which to evaluate the
    #   setting. Also, this function doesn't need to compile the list of
    #   settings that this setting depends on.
    #
    #   \param code The string to evaluate as a function of default values of
    #   other settings.
    #   \param locals The default values of other settings, as dictionary keyed
    #   by the setting names.
    #   \return The evaluated value of the setting, or None if \p code was None.
    def _evaluateFunction(self, code, locals):
        if not code: #The input was None. This setting value doesn't exist in the JSON.
            return None

        try:
            tree = ast.parse(code, "eval")
            compiled = compile(code, self._key, "eval")
        except (SyntaxError, TypeError) as e:
            print("Parse error in function (" + code + ") for setting", self._key + ":", str(e))
        except IllegalMethodError as e:
            print("Use of illegal method", str(e), "in function (" + code + ") for setting", self._key)
        except Exception as e:
            print("Exception in function (" + code + ") for setting", self._key + ":", str(e))

        return eval(compiled, globals(), locals)

class EngineTest:
    def __init__(self, json_filename, engine_filename, models):
        self._json_filename = json_filename
        self._json = json.load(open(json_filename, "r"))
        self._locals = {}
        self._addAllLocals() #Fills the _locals dictionary.
        self._addLocalsFunctions()  # Add mock functions used in fdmprinter
        self._engine = engine_filename
        self._models = models
        self._settings = {}
        self._test_results = TestResults()
        
        self._flattenAllSettings()

    def _flattenAllSettings(self):
        for key, data in self._json["settings"].items(): # top level settings are categories
            self._flattenSettings(data["children"]) # actual settings are children of top level category-settings
    
    def _flattenSettings(self, settings):
            for key, setting in settings.items():
                if not ("type" in setting and setting["type"] == "category"):
                    self._settings[key] = Setting(key, setting, self._locals)
                if "children" in setting:
                    self._flattenSettings(setting["children"])

    def testDefaults(self):
        suite = self._test_results.addTestSuite("Defaults")
        self._runTest(suite, "defaults", {})
        return suite.getFailureCount()

    def testSingleChanges(self):
        suite = self._test_results.addTestSuite("SingleSetting")
        for key, setting in self._settings.items():
            for value in setting.getSettingValues():
                self._runTest(suite, key, {key: value})
        return suite.getFailureCount()
    
    def testSingleRandom(self):
        suite = self._test_results.addTestSuite("SingleRandom")
        for key, setting in self._settings.items():
            self._runTest(suite, key, {key: setting.getRandomValue()})
        return suite.getFailureCount()

    def testDualRandom(self):
        suite = self._test_results.addTestSuite("DualRandom")
        for key, setting in self._settings.items():
            for key2, setting2 in self._settings.items():
                if key != key2:
                    self._runTest(suite, key, {key: setting.getRandomValue(), key2: setting2.getRandomValue()})
        return suite.getFailureCount()

    def testAllRandom(self, amount):
        suite = self._test_results.addTestSuite("AllRandom_%d" % (amount))
        for n in range(0, amount):
            settings = {}
            for key, setting in self._settings.items():
                settings[key] = setting.getRandomValue()
            self._runTest(suite, "Random", settings)
        return suite.getFailureCount()

    def _runTest(self, suite, class_name, settings):
        test_name = ', '.join("{!s}={!r}".format(key, val) for (key,val) in settings.items())
        for model in self._models:
            this_test_name = '%s.%s' % (os.path.basename(model), test_name)
            cmd = [self._engine, "slice", "-j", self._json_filename, "-o", "/dev/null"]
            for key, value in settings.items():
                cmd += ['-s', '%s=%s' % (key, value)]
            cmd += ["-l", model]
            error = self._runProcess(cmd)
            if error is not None:
                suite.failure(class_name, this_test_name, error)
            else:
                suite.success(class_name, this_test_name)

    def _runProcess(self, cmd):
        p = subprocess.Popen(cmd, stdin=subprocess.PIPE, stdout=subprocess.PIPE, stderr=subprocess.PIPE)
        p.error = ""
        t = threading.Thread(target=self._abortProcess, args=(p,))
        p.aborting = False
        t.start()
        stdout, stderr = p.communicate()
        p.aborting = True
        if p.error == "Timeout":
            return "Timeout: %s" % (' '.join(cmd))
        if p.wait() != 0:
            return "Execution failed: %s" % (' '.join(cmd))
        return None
    
    def _abortProcess(self, p):
        for i in range(0, 60):
            time.sleep(1) #Check every 1000ms if we need to abort the thread.
            if p.aborting:
                break
        if p.poll() is None:
            p.terminate()
            p.error = "Timeout"

    def getResults(self):
        return self._test_results

    ##  Adds all default values for all settings to the locals.
    #
    #   The results are stored in self._locals, keyed by the setting name.
    def _addAllLocals(self):
        for key, data in self._json["settings"].items(): # top level categories
            self._addLocals(data["children"]) # the actual settings in each category

    def _addLocalsFunctions(self):
        extruderValue = extruderValueWrapper(self._locals)
        self._locals['extruderValue'] = extruderValue

        extruderValues = extruderValuesWrapper(self._locals)
        self._locals['extruderValues'] = extruderValues

        resolveOrValue = resolveOrValueWrapper(self._locals)
        self._locals['resolveOrValue'] = resolveOrValue

    ##  Adds the default values in a node of the setting tree to the locals.
    #
    #   The results are stored in self._locals, keyed by the setting name.
    #
    #   \param settings The JSON node of which to add the default values.
    def _addLocals(self, settings):
        for key, setting in settings.items():
            if not ("type" in setting and setting["type"] == "category"): # skip category-settings
                self._locals[key] = setting["default_value"]
            if "children" in setting:
                self._addLocals(setting["children"]) #Recursively go down the tree.

def main(engine, model_path):
    filenames = sorted(os.listdir(model_path), key=lambda filename: os.stat(os.path.join(model_path, filename)).st_size)
    filenames = list(filter(lambda filename: filename.lower().endswith(".stl"), filenames))
    for filename in filenames:
        print("Slicing: %s (%d/%d)" % (filename, filenames.index(filename), len(filenames)))
        t = time.time()
        p = subprocess.Popen([engine, "-vv", os.path.join(model_path, filename)], stdin=subprocess.PIPE, stdout=subprocess.PIPE, stderr=subprocess.PIPE)
        stdout, stderr = p.communicate()
        if p.wait() != 0:
            print ("Engine failed to report success on test object: %s" % (filename))
            print(stderr.decode("utf-8", "replace").split("\n")[-5:])
            sys.exit(1)
        else:
            print("Slicing took: %f" % (time.time() - t))
if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="CuraEngine testing script")
    parser.add_argument("--simple", action="store_true", help="Only run the single test, exit")
    parser.add_argument("json", type=str, help="Machine JSON file to use")
    parser.add_argument("engine", type=str, help="Engine executable")
    parser.add_argument("models", type=str, nargs="+", help="List of models to use for testing")
    args = parser.parse_args()

    et = EngineTest(args.json, args.engine, args.models)
    if et.testDefaults() == 0:
        if not args.simple:
            et.testSingleChanges()
            if et.testSingleRandom() == 0:
                et.testDualRandom()
            if et.testAllRandom(10) == 0:
                et.testAllRandom(100)
    et.getResults().saveXML("output.xml")
    if args.simple:
        if et.getResults().getFailureCount() > 0:
            sys.exit(1)