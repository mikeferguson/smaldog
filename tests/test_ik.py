import os
import unittest

from smaldog.ik import *
from smaldog.robot_defs import *

class IkTest(unittest.TestCase):

    def test_neutral(self):
        defs = SMALdog()

        zero_stance = [[defs.X_COXA, -defs.Y_COXA-defs.L_COXA, 0.0],    # Right Front
                       [-defs.X_COXA, -defs.Y_COXA-defs.L_COXA, 0.0],   # Right Rear
                       [defs.X_COXA, defs.Y_COXA+defs.L_COXA, 0.0],     # Left Front
                       [-defs.X_COXA, defs.Y_COXA+defs.L_COXA, 0.0]]    # Left Rear

        solver = MammalIK(defs)
        solver.bodyPosZ = (defs.L_FEMUR + defs.L_TIBIA)*0.9999999999
        solution = solver.fullIK(zero_stance)

        for i in range(12):
            self.assertAlmostEqual(solution[i], 0.0, 4)

