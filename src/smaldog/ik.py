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

## @file ik.py Inverse Kinematics for SMALdog.

from math import cos, sin, atan2, sqrt, acos, pi

def sq(x):
    return x*x

## @brief Class for computing IK
##
## All kinematics are in a frame centered on the center of mass, and
## horizontal to the ground plane (regardless of any terrain of the
## plane).
class MammalIK:
    bodyRotX = 0.0
    bodyRotY = 0.0
    bodyRotZ = 0.0

#    bodyPosX = 0.0
#    bodyPosY = 0.0
#    bodyPosZ = 0.0

    def __init__(self, robot):
        self.robot = robot

    ## @brief Body IK: used to compute of the point where the leg
    ## attaches to the bodygiven a particular body rotation and
    ## foot position.
    ## @returns position in meters
    def bodyIK(self, X, Y, Z, Xdisp, Ydisp):
        ans = [0.0, 0.0, 0.0]  # (X,Y,Z)

        cosR = cos(self.bodyRotX)
        sinR = sin(self.bodyRotX)
        cosP = cos(self.bodyRotY)
        sinP = sin(self.bodyRotY)
        cosY = cos(self.bodyRotZ)
        sinY = sin(self.bodyRotZ)

        # Body can be rotated, Translate Xdisp & Ydisp
        X_offset = Xdisp*cosY*cosP - Ydisp*sinY
        Y_offset = Xdisp*sinY + Ydisp*cosY*cosR
        Z_offset = Ydisp*sinR - Xdisp*sinP

        ans[0] = X - X_offset
        ans[1] = Y - Y_offset
        ans[2] = self.bodyPosZ - Z - Z_offset
        return ans

    ## @brief Get the Leg IK for a Mammal.
    ## @param X Position forwards of the center of mass (in meters).
    ## @param Y Position left of the center of mass (in meters).
    ## @param Z Position downward from the center of mass (in meters).
    ## @returns Joint angles in radians
    def legIK(self, X, Y, Z, robot=None):
        if robot == None:
            robot = self.robot

        # (coxa, femur, tibia)
        ans = [float('nan') for i in range(3)]
        try:
            # first, make this a 2DOF problem... by solving coxa
            im1 = sqrt(sq(Y) + sq(Z))
            trueZ = sqrt(sq(im1) - sq(robot.L_COXA))
            q1 = atan2(Y,Z)
            q2 = acos(robot.L_COXA/im1)
            ans[0] = q1 + q2 - pi/2

            # get femure angle from vertical
            im2 = sqrt(sq(X) + sq(trueZ))
            q3 = atan2(-X, trueZ)
            d1 = sq(robot.L_FEMUR) - sq(robot.L_TIBIA) + sq(im2)
            d2 = 2 * robot.L_FEMUR * im2
            q4 = acos(d1/d2)
            ans[1] = q3 + q4

            # and tibia angle from femur
            d3 = sq(robot.L_FEMUR) - sq(im2) + sq(robot.L_TIBIA)
            d4 = 2 * robot.L_TIBIA * robot.L_FEMUR
            ans[2] = pi - acos(d3/d4)
        except Exception as e:
            pass #print e

        # return leg angles
        return ans

    ## @brief Full IK for the robot
    ## @param foot_pos 
    def fullIK(self, foot_pos, robot=None):
        if robot == None:
            robot = self.robot
        sol = list()

        # right front leg
        pos = foot_pos[0]
        req = self.bodyIK(pos[0], pos[1], pos[2], robot.X_COXA, -robot.Y_COXA)
        sol += self.legIK(req[0], -req[1], req[2])

        # right rear leg
        pos = foot_pos[1]
        req = self.bodyIK(pos[0], pos[1], pos[2], -robot.X_COXA, -robot.Y_COXA)
        sol += self.legIK(-req[0], -req[1], req[2])

        # left front leg
        pos = foot_pos[2]
        req = self.bodyIK(pos[0], pos[1], pos[2], robot.X_COXA, robot.Y_COXA)
        sol += self.legIK(req[0], req[1], req[2])

        # left rear leg
        pos = foot_pos[3]
        req = self.bodyIK(pos[0], pos[1], pos[2], -robot.X_COXA, robot.Y_COXA)
        sol += self.legIK(-req[0], req[1], req[2])

        # solution is list of servo angles
        return sol

