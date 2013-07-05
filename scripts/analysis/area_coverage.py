#!/usr/bin/env python

# Copyright (c) 2008-2013, Michael E. Ferguson. All right reserved.
#
# This program is free software; you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation; either version 2 of the License, or
# (at your option) any later version.
#
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.
#
# You should have received a copy of the GNU General Public License
# along with this program; if not, write to the Free Software Foundation,
# Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA

from math import pi, isnan
from smaldog.ik import *
from smaldog.interpolation import *
from smaldog.utilities import *
from smaldog.robot_defs import *
from smaldog import ax12

def iterate_z():
    for t in range(12):
        yield 0.05 + t*0.005

def iterate_x():
    for t in range(35):
        yield 0.255 - t*0.01

def iterate_y():
    for t in range(35):
        yield -0.1 + t*0.01

def found_solution(sol):
    for val in sol:
        if isnan(val):
            return False
    return True

if __name__=="__main__":
    defs = SMALdog()
    solver = MammalIK(defs)

    zero_stance = [[defs.X_COXA, -defs.Y_COXA-defs.L_COXA, 0.0],    # Right Front
                   [-defs.X_COXA, -defs.Y_COXA-defs.L_COXA, 0.0],   # Right Rear
                   [defs.X_COXA, defs.Y_COXA+defs.L_COXA, 0.0],     # Left Front
                   [-defs.X_COXA, defs.Y_COXA+defs.L_COXA, 0.0]]    # Left Rear

    points = list()
    best = 0
    best_z = 0.08
    data = dict()
    # step through heights
    for z in iterate_z():
        print "Z height at", z
        solver.bodyPosZ = z
        c = 0
        k = 0
        kt = 0
        for x in iterate_x():
            zero_stance[0][0] = x
            for y in iterate_y():
                zero_stance[0][1] = y
                if abs(x-defs.X_COXA)<0.005 and abs(y-defs.Y_COXA-defs.L_COXA)<0.005:
                    print "X",
                elif abs(y) < 0.005: # mark center of robot
                    if x > 0.25:
                        print "C",
                    else:
                        print ":", 
                elif found_solution(solver.fullIK(zero_stance)):
                    c = c + 1
                    if abs(y-defs.Y_COXA-defs.L_COXA)<0.005: # mark the typical plane the leg is in
                        print "|",
                        k = k + 1
                        kt = kt + x
                    else:
                        print "o",
                else:
                    print " ",
            print ""
        print ""
        print "Area Coverage = ", c/(35*35.0)
        print "Length of step available = ", k*0.01, "m"
        if k == 0:
            k = 1
        print "Optimal x = ", kt/k
        if c > best:
            best = c
            best_z = z
        data[z] = (c/(75*60.0), k*4, kt/k)
    print "Best height = ", best_z  
