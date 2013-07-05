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

## @file interpolation.py Interpolation for SMALdog.

## @brief Interpolation via generator
def interpolate(start_pose, end_pose, iterations):
    diffs = list()
    for i in range(len(start_pose)):
        diffs.append((end_pose[i] - start_pose[i])/iterations)
    for i in range(iterations):
        pose = list()
        for j in range(len(start_pose)):
            pose.append(start_pose[j] + diffs[j]*i)
        yield pose
    
