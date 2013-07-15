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
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
# GNU General Public License for more details.
#
# You should have received a copy of the GNU General Public License
# along with this program; if not, write to the Free Software Foundation,
# Inc., 59 Temple Place, Suite 330, Boston, MA 02111-1307 USA

## @file visualization/stability.py Visualization of stability.

from pylab import *
from smaldog.geometry import *

class StabilityVisualization:

    def __init__(self, robot):
        self.robot = robot

        self.default_X = [s[0] for s in robot.DEFAULT_STANCE]
        self.default_Y = [-s[1] for s in robot.DEFAULT_STANCE]
        self.default_X.append(0)
        self.default_Y.append(0)
        self.default_c = 'yo'

        self.legs_X = [list(), list(), list(), list()]
        self.legs_Y = [list(), list(), list(), list()]
        self.legs_c = ['bo', 'go', 'ro', 'ko']

        self.center_X = [0, ]
        self.center_Y = [0, ]
        self.center_c = 'y-'

    def draw(self, stance, controller):
        if controller.swing_leg == None:
            return

        for i in range(4):
            self.legs_X[i].append(stance[i][0]+controller.x)
            self.legs_Y[i].append(-stance[i][1]-controller.y)

        legs = controller.cross_poses[controller.swing_leg]
        support = [stance[z] for z in legs]

        stability_X = [s[0]+controller.x for s in support]
        stability_Y = [-s[1]-controller.y for s in support]
        stability_X.append(stability_X[0]); stability_Y.append(stability_Y[0])
        stability_c = 'b'

        isostable = reduceTriangle(support[0], support[1], support[2], controller.stability)
        iso_X = [p[0]+controller.x for p in isostable]
        iso_Y = [-p[1]-controller.y for p in isostable]
        iso_X.append(iso_X[0]); iso_Y.append(iso_Y[0])
        iso_c = 'r'
  
        self.center_X.append(controller.x)
        self.center_Y.append(-controller.y)

        swing_X = [stance[controller.swing_leg][0]+controller.x, ]
        swing_Y = [-stance[controller.swing_leg][1]-controller.y, ]
        if stance[controller.swing_leg][2] != self.robot.DEFAULT_STANCE[controller.swing_leg][2]:
          swing_c = 'ro'
        else:
          swing_c = 'bo'

        plot(self.legs_Y[0], self.legs_X[0], self.legs_c[0],
             self.legs_Y[1], self.legs_X[1], self.legs_c[1],
             self.legs_Y[2], self.legs_X[2], self.legs_c[2],
             self.legs_Y[3], self.legs_X[3], self.legs_c[3],
             #self.default_Y, self.default_X, self.default_c,
             stability_Y, stability_X, stability_c,
             iso_Y, iso_X, iso_c,
             self.center_Y, self.center_X, self.center_c,
             swing_Y, swing_X, swing_c)
        axis('equal')
        show()
