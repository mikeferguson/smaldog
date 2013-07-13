import os
import unittest

from smaldog.geometry import *

class GeometryTest(unittest.TestCase):

    def test_inside_triangle(self):
        self.assertEqual(insideTriangle([0,0], [1,1], [1,-1], [-2,1]), True)
        self.assertEqual(insideTriangle([2,0], [1,1], [1,-1], [-2,1]), False)

    def test_midway_line(self):
        self.assertEqual(midwayLine([1,1], [1,-1]), [1,0])
        self.assertEqual(midwayLine([1,-1], [-2,1]), [-0.5,0])
        self.assertEqual(midwayLine([-2,1], [1,1]), [-0.5,1])

    def test_scale_line(self):
        self.assertEqual(scaleLine([1,1], [1,-1], 0.25), [1,.5])
        self.assertEqual(scaleLine([1,-1], [-2,1], 0.5), [-0.5,0])
        self.assertEqual(scaleLine([-2,1], [1,1], 0.5), [-0.5,1])

    def test_centroid_triangle(self):
        self.assertEqual(centroidTriangle([1,1], [1,-1], [-2,0]), [0.0, 0])

    def test_length_line(self):
        self.assertEqual(lengthLine([0,0], [2,0]), 2.0)

    def test_scale_triange(self):
        self.assertEqual(scaleTriangle([2,2], [2,-2], [-4,0], 0.5), [[1,1],[1,-1],[-2,0]])

    def test_reduce_triange(self):
        new_triangle = reduceTriangle([2,2], [2,-2], [-4,0], 0.2)
        d = lengthLine(new_triangle[2], [-4,0])
        self.assertAlmostEqual(d, 0.4)
        
