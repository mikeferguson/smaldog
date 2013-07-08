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

## @file gaits.py Some simple gaits for SMALdog.

import copy

## @brief Generator based gaits
## @param starting_pose Typical [4][3] foot position matrix.
class RippleGait:

    def __init__(self, starting_pose):
        self.default_pose = copy.deepcopy(starting_pose)
        self.last_pose = copy.deepcopy(starting_pose)
        self.leg_no = [0, 3, 3, 0]
        self.steps_total = 6
        self.steps_push = 3
        self.step = 0
        self.lift = 0.02 # height to lift foot
        self.step_time = 0.25 # time to complete a step (1 full cycle/sec)

    def next(self, x_vel, y_vel, r_vel):
        moving = abs(x_vel) > 0.001 or abs(y_vel) > 0.001 or abs(r_vel) > 0.001
        x_step = x_vel # TODO
        y_step = y_vel
        r_step = r_vel / self.steps_push
        for i in range(4):
            if moving:
                if self.step == self.leg_no[i]: # up and forward
                    self.last_pose[i][0] = self.default_pose[i][0] # x
                    self.last_pose[i][1] = self.default_pose[i][1] # y                
                    self.last_pose[i][2] = self.lift # z
                elif self.step == (self.leg_no[i] + 1): # forward
                    self.last_pose[i][0] = self.default_pose[i][0] + x_step/2 # x
                    self.last_pose[i][1] = self.default_pose[i][1] + y_step/2 # y  
                elif self.step == (self.leg_no[i] + 2): # down
                    self.last_pose[i][2] = 0 # z
                else:
                    # move body forward
                    # TODO add rotation element
                    self.last_pose[i][0] = self.last_pose[i][0] - x_step/self.steps_push # x
                    self.last_pose[i][1] = self.last_pose[i][1] - y_step/self.steps_push # y                   
            else:
                self.last_pose[i][2] = 0 # foot down
        self.step += 1
        if self.step >= self.steps_total:
            self.step = 0
        return self.last_pose

