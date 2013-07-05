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

from math import pi
from smaldog.eth_bridge import *
from smaldog.ik import *
from smaldog.interpolation import *
from smaldog.utilities import *
from smaldog.robot_defs import *
from smaldog import ax12

defs = SMALdog()
solver = MammalIK(defs)
eth = EthBridge()

zero_stance = [[defs.X_COXA, -defs.Y_COXA-defs.L_COXA, 0.0],    # Right Front
               [-defs.X_COXA, -defs.Y_COXA-defs.L_COXA, 0.0],   # Right Rear
               [defs.X_COXA, defs.Y_COXA+defs.L_COXA, 0.0],     # Left Front
               [-defs.X_COXA, defs.Y_COXA+defs.L_COXA, 0.0]]    # Left Rear

solver.bodyPosZ = (defs.L_FEMUR + defs.L_TIBIA)*0.8

# start in standing state
# TODO: read actual state
start_pose = solver.fullIK(zero_stance)

# move around
solver.bodyRotY = 0.25
next_pose = solver.fullIK(zero_stance)

for pose in interpolate(start_pose, next_pose, 50):
    packet = makeSyncWritePacket(convertToAX12(pose, defs))
    eth.send(eth.makePacket(254, ax12.AX_SYNC_WRITE, packet))
    time.sleep(0.01)

time.sleep(1)

for pose in interpolate(next_pose, start_pose, 50):
    packet = makeSyncWritePacket(convertToAX12(pose, defs))
    eth.send(eth.makePacket(254, ax12.AX_SYNC_WRITE, packet))
    time.sleep(0.01)

