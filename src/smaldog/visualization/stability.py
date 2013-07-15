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

def draw(stance, controller, robot):
    if controller.swing_leg == None:
      return

    default_X = [s[0] for s in robot.DEFAULT_STANCE]
    default_Y = [-s[1] for s in robot.DEFAULT_STANCE]
    default_X.append(0)
    default_Y.append(0)
    default_c = 'yo'

    stance_X = [s[0]+controller.x for s in stance]
    stance_Y = [-s[1]-controller.y for s in stance]
    stance_c = 'bo'

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
  
    center_X = [controller.x,]
    center_Y = [-controller.y,]
    center_c = 'ro'

    swing_X = [stance[controller.swing_leg][0]+controller.x, ]
    swing_Y = [-stance[controller.swing_leg][1]-controller.y, ]
    if stance[controller.swing_leg][2] != robot.DEFAULT_STANCE[controller.swing_leg][2]:
      swing_c = 'ro'
    else:
      swing_c = 'bo'

    plot(default_Y, default_X, default_c,
       stability_Y, stability_X, stability_c,
       iso_Y, iso_X, iso_c,
       stance_Y, stance_X, stance_c,
       center_Y, center_X, center_c,
       swing_Y, swing_X, swing_c)
    axis('equal')
    show()
