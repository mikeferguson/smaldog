import os
import unittest

from smaldog.leg_ik import *
from smaldog.robot_defs import *

class IkTest(unittest.TestCase):

    def test_neutral(self):
        defs = SMALdog()
        lf_ik = LegIK("lf", defs.X_SHOULDER, defs.Y_SHOULDER, defs.L_SHOULDER, defs.L_FEMUR, defs.L_TIBIA)
        angles = lf_ik.getIK(defs.X_SHOULDER, defs.Y_SHOULDER+defs.L_SHOULDER,-(defs.L_FEMUR + defs.L_TIBIA)*0.9999999999)
        for val in angles.values():
            self.assertAlmostEqual(val, 0.0, 4)

        rf_ik = LegIK("rf", defs.X_SHOULDER, -defs.Y_SHOULDER, defs.L_SHOULDER, defs.L_FEMUR, defs.L_TIBIA)
        angles = rf_ik.getIK(defs.X_SHOULDER, -defs.Y_SHOULDER-defs.L_SHOULDER,-(defs.L_FEMUR + defs.L_TIBIA)*0.9999999999)
        for val in angles.values():
            self.assertAlmostEqual(val, 0.0, 4)

        rr_ik = LegIK("rr", -defs.X_SHOULDER, -defs.Y_SHOULDER, defs.L_SHOULDER, defs.L_FEMUR, defs.L_TIBIA)
        angles = rr_ik.getIK(-defs.X_SHOULDER, -defs.Y_SHOULDER-defs.L_SHOULDER,-(defs.L_FEMUR + defs.L_TIBIA)*0.9999999999)
        for val in angles.values():
            self.assertAlmostEqual(val, 0.0, 4)

