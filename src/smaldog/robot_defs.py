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
    signs =    [ -1,   1,   1,    -1,  1,  -1,   1,  -1,   -1,   1,  -1,   1]

    # Servo ID's
    RF_COXA = 1
    RF_FEMUR = 3
    RF_TIBIA = 5
    LF_COXA = 2
    LF_FEMUR = 4
    LF_TIBIA = 6
    RR_COXA = 7
    RR_FEMUR = 9
    RR_TIBIA = 11
    LR_COXA = 8
    LR_FEMUR = 10
    LR_TIBIA = 12

    X_COXA = 0.086      # Meters between front and back legs /2
    Y_COXA = 0.019      # Meters between front/back legs /2

    L_COXA = 0.050      # Meters distance from coxa servo to femur servo 
    L_FEMUR = 0.064     # Meters distance from femur servo to tibia servo 
    L_TIBIA = 0.105     # Meters distance from tibia servo to foot

