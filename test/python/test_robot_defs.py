import os
import unittest

from smaldog.robot_defs import *

class RobotDefsTest(unittest.TestCase):

    def test_neutral(self):
        defs = SMALdog()

        sol = dict()
        sol["lf_pitch_joint"] = 0.0
        sol["lf_flex_joint"] = 0.0
        sol["lf_knee_joint"] = 0.0
        sol["lr_pitch_joint"] = 0.0
        sol["lr_flex_joint"] = 0.0
        sol["lr_knee_joint"] = 0.0
        sol["rf_pitch_joint"] = 0.0
        sol["rf_flex_joint"] = 0.0
        sol["rf_knee_joint"] = 0.0
        sol["rr_pitch_joint"] = 0.0
        sol["rr_flex_joint"] = 0.0
        sol["rr_knee_joint"] = 0.0

        ax12 = convertToAX12(sol, defs)
        self.assertEqual(defs.neutrals, ax12)
