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

## @file leg_ik.py Inverse Kinematics for SMALdog.

from math import cos, sin, atan2, sqrt, acos, pi

def sq(x):
    return x*x

## @brief Class for computing IK of a single leg
class LegIK:

    def __init__(self, name, shoulder_x, shoulder_y, l_shoulder, l_femur, l_tibia):
        # Leg name, typically something like "lf"
        self.name = name

        # The location of the shoulder in the body frame
        self.SHOULDER_X = shoulder_x
        self.SHOULDER_Y = shoulder_y

        # The lengths of the leg
        self.L_SHOULDER = l_shoulder
        self.L_FEMUR = l_femur
        self.L_TIBIA = l_tibia

    ## @brief Get the leg joint angles.
    ## @param X Position forwards of the center of mass (in meters).
    ## @param Y Position left of the center of mass (in meters).
    ## @param Z Position downward from the center of mass (in meters).
    ## @returns Joint angles in radians
    def getIK(self, X, Y, Z, bodyR=0.0, bodyP=0.0, bodyY=0.0):
        angles = dict()
        angles[self.name + "_pitch_joint"] = float('nan')
        angles[self.name + "_flex_joint"] = float('nan')
        angles[self.name + "_knee_joint"] = float('nan')

        # Get shoulder position
        cosR = cos(bodyR)
        sinR = sin(bodyR)
        cosP = cos(bodyP)
        sinP = sin(bodyP)
        cosY = cos(bodyY)
        sinY = sin(bodyY)
        shoulder_pos = [ self.SHOULDER_X*cosY*cosP - self.SHOULDER_Y*sinY,
                         self.SHOULDER_X*sinY + self.SHOULDER_Y*cosY*cosR,
                         self.SHOULDER_Y*sinR - self.SHOULDER_X*sinP ] # X, Y, Z
        
        # The remainder of this is in quadrant 1, need to convert to that
        if self.SHOULDER_X > 0:
            X = X - shoulder_pos[0]
        else:
            X = -(X - shoulder_pos[0])
        if self.SHOULDER_Y > 0:
            Y = Y - shoulder_pos[1]
        else:
            Y = -(Y - shoulder_pos[1])

        try:
            # first, make this a 2DOF problem... by solving shoulder
            im1 = sqrt(sq(Y) + sq(-Z))
            trueZ = sqrt(sq(im1) - sq(self.L_SHOULDER))
            q1 = atan2(Y,-Z)
            q2 = acos(self.L_SHOULDER/im1)
            angles[self.name + "_pitch_joint"] = q1 + q2 - pi/2

            # get femure angle from vertical
            im2 = sqrt(sq(X) + sq(trueZ))
            q3 = atan2(-X, trueZ)
            d1 = sq(self.L_FEMUR) - sq(self.L_TIBIA) + sq(im2)
            d2 = 2 * self.L_FEMUR * im2
            q4 = acos(d1/d2)
            angles[self.name + "_flex_joint"] = q3 + q4

            # and tibia angle from femur
            d3 = sq(self.L_FEMUR) - sq(im2) + sq(self.L_TIBIA)
            d4 = 2 * self.L_TIBIA * self.L_FEMUR
            angles[self.name + "_knee_joint"] = pi - acos(d3/d4)
        except Exception as e:
            pass

        return angles
