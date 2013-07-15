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
from pylab import *

from smaldog.robot_defs import *
from smaldog.controllers.muybridge import *

if __name__=="__main__":
    robot = SMALdog()
    zero_stance = [[robot.X_SHOULDER, -0.05, -0.095],   # Right Front
                   [-robot.X_SHOULDER, -0.05, -0.095],  # Right Rear
                   [robot.X_SHOULDER, 0.05, -0.095],    # Left Front
                   [-robot.X_SHOULDER, 0.05, -0.095]]   # Left Rear
    controller = MuybridgeGaitController(robot, zero_stance)

    stance_X = [s[0] for s in zero_stance]
    stance_Y = [s[1] for s in zero_stance]
    stance_c = 'yo'

    figure()
    for swing_leg in range(4):
        legs = controller.cross_poses[swing_leg]
        support = [zero_stance[z] for z in legs]

        stability_X = [s[0] for s in support]
        stability_Y = [s[1] for s in support]
        stability_X.append(stability_X[0]); stability_Y.append(stability_Y[0])
        stability_c = 'b'

        isostable = reduceTriangle(support[0], support[1], support[2], 0.01)
        iso_X = [p[0] for p in isostable]
        iso_Y = [p[1] for p in isostable]
        iso_X.append(iso_X[0]); iso_Y.append(iso_Y[0])
        iso_c = 'r'

        if swing_leg == 1 or swing_leg == 3:
            # For hind legs, shift halfway to the midpoint of the front stability boundary
            front_mid = midwayLine(isostable[0], isostable[2])
            offset = midwayLine([0,0], front_mid)
            print "  hind leg causes body shift of", offset
        else:
            # For front legs, shift to the midpoint of the front stability boundary
            #pts = [self.last_pose[k] for k in self.cross_poses[self.swing_leg]]
            #isostable = reduceTriangle(pts[0], pts[1], pts[2], 0.01)
        #        print "  isostable", isostable
        #        print "  center of isostable", centroidTriangle(isostable[0], isostable[1], isostable[2])
        #        print "  midpoint (back)", midwayLine(isostable[0], isostable[2])
        #        print "           (side)", midwayLine(isostable[1], isostable[2])
            offset = midwayLine(isostable[0], isostable[1])
         #        print "  front leg causes body shift of", offset

        offset_X = [offset[0], ]
        offset_Y = [offset[1], ]
        offset_c = 'go'
    
        center_X = [0,]
        center_Y = [0,]
        center_c = 'ro'

        #iso_center_X = [0,]
        #iso_center_Y = [0,]



        #C = ['b', 'b', 'b', 'r', 'r', 'r', 'g']

        #y = [-k for k in Y]

        subplot(221 + swing_leg)
        plot(stability_Y, stability_X, stability_c,
             iso_Y, iso_X, iso_c,
             stance_Y, stance_X, stance_c,
             offset_Y, offset_X, offset_c,
             center_Y, center_X, center_c)
        axis('equal')
        
    show()


  #subplot(111)
  #plot(iso_Y, iso_X, "r")
  #subplot(111)
  #plot(center_Y, center_X, "bo", iso_center_Y, iso_center_X, "ro")

#subplot(121, aspect='equal')
  #plot(Y,X, "go")
  #scatter(Y, X, c=C)


