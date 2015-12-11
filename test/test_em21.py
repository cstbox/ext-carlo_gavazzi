#!/usr/bin/env python
# -*- coding: utf-8 -*-

import unittest
import json
import os
import logging

from pycstbox.cgavazzi.em21 import EM21Instrument


class EM21MedataTestCase(unittest.TestCase):
    PATH = os.path.abspath(os.path.join(
        os.path.dirname(__file__),
        "../lib/python/pycstbox/devcfg.d/modbus.d/cgavazzi.em21"
    ))

    def test_01(self):
        json.load(file(self.PATH))


class EM21TestCase(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.em21 = EM21Instrument('/dev/ttyUSB0', 3, baudrate=19200, inter_requests_delay=0.1)
        cls.em21.logger.setLevel(logging.DEBUG)

    def test_01(self):
        values = self.em21.poll()
        self.assertEqual(len(values), 30)


if __name__ == '__main__':
    unittest.main()
