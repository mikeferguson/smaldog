import os
import unittest

from smaldog.robot_defs import *

class IkTest(unittest.TestCase):

    def test_neutral(self):
        defs = SMALdog()
        lf_ik = defs.ik["lf"]
        angles = lf_ik.getIK(defs.X_SHOULDER, defs.Y_SHOULDER+defs.L_SHOULDER,-(defs.L_FEMUR + defs.L_TIBIA)*0.9999999999)
        for val in angles.values():
            self.assertAlmostEqual(val, 0.0, 4)

        rf_ik = defs.ik["rf"]
        angles = rf_ik.getIK(defs.X_SHOULDER, -defs.Y_SHOULDER-defs.L_SHOULDER,-(defs.L_FEMUR + defs.L_TIBIA)*0.9999999999)
        for val in angles.values():
            self.assertAlmostEqual(val, 0.0, 4)

        rr_ik = defs.ik["rr"]
        angles = rr_ik.getIK(-defs.X_SHOULDER, -defs.Y_SHOULDER-defs.L_SHOULDER,-(defs.L_FEMUR + defs.L_TIBIA)*0.9999999999)
        for val in angles.values():
            self.assertAlmostEqual(val, 0.0, 4)

