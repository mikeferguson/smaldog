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

from math import isnan
from leg_ik import *
import ax12


## @brief Convert radians to servo position offset.
def radToServoAX12(rads):
    return int((rads/5.235987755982989) * 1024)

## @brief Convert servo position to radians
def servoToRadAX12(servo):
    return (servo/1024.0) * 5.235987755982989

## @brief Convert radians to servo ticks, apply neutrals, check limits.
def convertToAX12(values, robot):
    sol = [-1 for i in range(12)]

    for i in range(12):
        name = robot.names[i]
        s = robot.neutrals[i] + robot.signs[i] * radToServoAX12(values[name])
        if s < robot.maxs[i] and s > robot.mins[i]:
            sol[i] = s
        else:
            pass

    return sol

## @brief Get sync write packet
## @param positions Values for servos 1 to n
def makeSyncWritePacket(positions):
    output = list()
    output.append(ax12.P_GOAL_POSITION_L)
    output.append(2)
    for i in range(len(positions)):
        output.append(i+1) # id
        output.append(positions[i]&0xff)
        output.append((positions[i]>>8)&0xff)
    return output

## @brief The definitions for robot geometry and layout
class SMALdog:
    legs = ["rf", "lr", "lf", "rr"]
    servo_res = 1024
    names =    ["rf_pitch_joint", "lf_pitch_joint",
                "rf_flex_joint", "lf_flex_joint",
                "rf_knee_joint", "lf_knee_joint",
                "rr_pitch_joint", "lr_pitch_joint",
                "rr_flex_joint", "lr_flex_joint",
                "rr_knee_joint", "lr_knee_joint"]
    # Determined on real robot:
    mins =     [152, 450,   1,  336, 193, 160, 450, 231,  323,   1, 159, 192]
    maxs =     [575, 868, 689, 1023, 857, 824, 813, 575, 1023, 683, 819, 855]
    # From CAD:
    #  - Shoulder flex is 20.400209 degrees off the 0 when the line between shoulder
    #    flex and knee flex is upright. 20.400209/300 * 1024 = 69.63
    #  - Knee flex is 30.177936 degrees off the 0 when the line between the knee
    #    flex and the foot is upright. 30.177936/300 * 1024 = 204.19
    neutrals = [512, 512, 442,  582, 308, 716, 512, 512,  582, 442, 716, 308]
    # Determined on real robot:
    signs =    [ -1,   1,  -1,    1,   1,  -1,   1,  -1,    1,  -1,  -1,   1]

    X_SHOULDER = 0.086  # Meters between front and back legs /2
    Y_SHOULDER = 0.019  # Meters between front/back legs /2

    L_SHOULDER = 0.050  # Meters distance from shoulder pitch servo to shoulder flex servo
    L_FEMUR = 0.065     # Meters distance from flex servo to knee servo (from CAD)
    L_TIBIA = 0.088     # Meters distance from knee servo to foot (from CAD)

    DEFAULT_STANCE = [[X_SHOULDER+.01, -0.06, -0.11],   # Right Front
                      [-X_SHOULDER+.01, 0.06, -0.11],   # Left Rear
                      [X_SHOULDER+.01, 0.06, -0.11],    # Left Front
                      [-X_SHOULDER+.01, -0.06, -0.11]]  # Right Rear

    def __init__(self):
        # initialize IK solvers for each leg
        self.ik = dict()
        self.ik["rf"] = LegIK("rf", self.X_SHOULDER, -self.Y_SHOULDER, self.L_SHOULDER, self.L_FEMUR, self.L_TIBIA)
        self.ik["lr"] = LegIK("lr", -self.X_SHOULDER, self.Y_SHOULDER, self.L_SHOULDER, self.L_FEMUR, self.L_TIBIA)
        self.ik["lf"] = LegIK("lf", self.X_SHOULDER, self.Y_SHOULDER, self.L_SHOULDER, self.L_FEMUR, self.L_TIBIA)
        self.ik["rr"] = LegIK("rr", -self.X_SHOULDER, -self.Y_SHOULDER, self.L_SHOULDER, self.L_FEMUR, self.L_TIBIA)

    def getIK(self, footposes):
        angles = dict()
        for i in range(4):
            angles.update(self.ik[self.legs[i]].getIK(footposes[i][0], footposes[i][1], footposes[i][2]))
        return angles

    def checkLimits(self, angles):
        for name, pos in angles.items():
            i = self.names.index(name)
            if self.signs[i] > 0:
                mini = servoToRadAX12(self.mins[i] - self.neutrals[i])
                maxi = servoToRadAX12(self.maxs[i] - self.neutrals[i])
            else:
                maxi = -servoToRadAX12(self.mins[i] - self.neutrals[i])
                mini = -servoToRadAX12(self.maxs[i] - self.neutrals[i])
            if isnan(pos):
                return False
            if pos > maxi:
                print "Servo", name, "has limits of", mini, maxi, "but", pos, "requested."
                return False
            if pos < mini:
                print "Servo", name, "has limits of", mini, maxi, "but", pos, "requested."
                return False
        return True

