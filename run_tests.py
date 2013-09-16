# vi:et:ts=4 sw=4 sts=4
import sys
import os
import string
import subprocess
import unittest

EXECUTABLE = './CuraEngine'
TEST_DIR = 'testcase_models'

class TestGCode(unittest.TestCase):
    def _tuplify(self, first, second):
        return (first, second)

    def _main(self, base):
        stl_file = '{}.stl'.format(base)
        gcode_file = '{}.gcode'.format(base)

        with open(os.path.join(TEST_DIR, gcode_file)) as ifp:
            gcode_output = map(string.strip, ifp.readlines())

        cmd = [EXECUTABLE, os.path.join(TEST_DIR, stl_file)]
        stl_output = subprocess.check_output(cmd, stderr=subprocess.PIPE)
        stl_output = stl_output.splitlines()

        results = map(self._tuplify, gcode_output, stl_output)
        for gcode_line, stl_line in results:
            self.assertEqual(gcode_line, stl_line)


    def test_wolt(self):
        self._main('wolt')


    def test_woltBaseline(self):
        self._main('woltBaseline')


    def test_woltNotFlat(self):
        self._main('woltNotFlat')


    def test_wolt_scaled200Perc(self):
        self._main('wolt_scaled200Perc')


    def test_wolt_smoothingOn(self):
        self._main('wolt_smoothingOn')


if __name__ == '__main__':
    if len(sys.argv) == 2:
        EXECUTABLE = sys.argv[1]
        sys.argv.remove(EXECUTABLE)

	unittest.main()

