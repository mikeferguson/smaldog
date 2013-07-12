#!/usr/bin/env python

# Copyright (c) 2013, Michael E. Ferguson. All right reserved.
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

## @file robot_defs.py Robot-specific information.

class SMALdog:
    servo_res = 1024
    mins =     [152, 450,   1,  336, 193, 160, 450, 231,  323,   1, 159, 192]
    maxs =     [575, 868, 689, 1023, 857, 824, 813, 575, 1023, 683, 819, 855]
    neutrals = [512, 512, 442,  582, 302, 722, 512, 512,  582, 442, 722, 302]
    signs =    [ -1,   1,  -1,    1,   1,  -1,   1,  -1,    1,  -1,  -1,   1]

    # Servo ID's
    RF_PITCH = 1    # -
    LF_PITCH = 2    # +
    RF_FLEX = 3     # -
    LF_FLEX = 4     # +
    RF_KNEE = 5     # +
    LF_KNEE = 6     # -
    RR_PITCH = 7    # +
    LR_PITCH = 8    # -
    RR_FLEX = 9     # +
    LR_FLEX = 10    # -
    RR_KNEE = 11    # -
    LR_KNEE = 12    # +

    X_SHOULDER = 0.086  # Meters between front and back legs /2
    Y_SHOULDER = 0.019  # Meters between front/back legs /2

    L_SHOULDER = 0.050  # Meters distance from shoulder pitch servo to shoulder flex servo
    L_FEMUR = 0.064     # Meters distance from flex servo to knee servo
    L_TIBIA = 0.086     # Meters distance from knee servo to foot

