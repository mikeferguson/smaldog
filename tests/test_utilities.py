import os
import unittest

from smaldog.utilities import *
from smaldog.robot_defs import *

class UtilitiesTest(unittest.TestCase):

    def test_neutral(self):
        defs = SMALdog()
        sol = [0.0 for i in range(12)]
        ax12 = convertToAX12(sol, defs)
        self.assertEqual(defs.neutrals, ax12)
