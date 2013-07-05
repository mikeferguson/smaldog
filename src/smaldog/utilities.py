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

## @file utilities.py Functions for conversion between servo/radian values.

import ax12

## @brief Convert radians to servo position offset.
def radToServoAX12(rads):
    return int((rads/5.235987755982989) * 1024)

## @brief Convert radians to servo ticks, apply neutrals, check limits.
def convertToAX12(values, robot):
    sol = [-1 for i in range(12)]

    servos = [robot.RF_COXA, robot.RF_FEMUR, robot.RF_TIBIA,
              robot.RR_COXA, robot.RR_FEMUR, robot.RR_TIBIA,
              robot.LF_COXA, robot.LF_FEMUR, robot.LF_TIBIA,
              robot.LR_COXA, robot.LR_FEMUR, robot.LR_TIBIA]

    for i in range(12):
        s = robot.neutrals[servos[i]-1] + robot.signs[servos[i]-1] * radToServoAX12(values[i])
        if s < robot.maxs[servos[i]-1] and s > robot.mins[servos[i]-1]:
            sol[servos[i]-1] = s
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
