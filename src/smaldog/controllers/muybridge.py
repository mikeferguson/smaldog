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

## @file muybridge.py The standard gait.

import copy
from smaldog.geometry import *

## @brief The standard gait controller. Implements a simple fixed gait
## which shifts the body only while all feet are on the ground.
class MuybridgeGaitController:
    standard_time = 0.2
    # indexes of points cross legs, first 2 are diagonal to leg
    cross_poses = [ [3, 2, 1],
                    [2, 3, 0],
                    [0, 1, 3],
                    [0, 1, 2] ]
    # pose of body in 2d space
    x = 0
    y = 0
    r = 0

    ## @brief Constructor
    ## @param starting_pose The pose of the robot feet relative to the body
    ##        of the robot. Should be of the form: 
    ##        [ [RF_x, RF_y, RF_z], [RR_x, RR_y, RR_z], [LF_x...], [LR_x...] ]
    ## @param ik solvers
    def __init__(self, robot, starting_pose):
        self.robot = robot

        # last_pose will be adapted and returned in next()
        self.last_pose = copy.deepcopy(starting_pose)
        self.last_x_vel = 0
        self.last_y_vel = 0
        self.last_r_vel = 0

        # First phase is quad shift phase (all feet on ground, move the body if needed)
        # Second phase is lifting the swing leg
        # Third phase is moving the swing leg forward
        # Fourth phase is setting the swing leg down
        self.QUAD_SHIFT = 0
        self.SWING_LIFT = 1
        self.SWING_FORWARD = 2
        self.SWING_DROP = 3
        self.phase = 0
        self.phase_count = 4

        self.lift = 0.02 # height to lift foot
        self.stability = 0.01

        self.swing_leg = None

    ## @brief Get the next update from the gait.
    ## @param x_vel Desired velocity in X direction (forward).
    ## @param y_vel Desired velocity in Y direction (left).
    ## @param r_vel Desired rotational velocity.
    ## @returns Time to carry out motion, along with a new set of feet positions,
    ##          in same form as starting_pose was. Optionally, some gaits may also
    ##          return a 3rd list element which is the desired body rotation.
    def next(self, x_vel, y_vel, r_vel):

        # Are we not moving?
        if abs(x_vel) < 0.001 and abs(y_vel) < 0.001 and abs(r_vel) < 0.001:
            if self.phase == self.SWING_FORWARD:
                # put that foot down, but go back to lift so that when we start walking we use it
                self.last_pose[self.swing_leg][2] -= self.lift
                self.phase = self.SWING_LIFT
                return [self.standard_time, self.last_pose]
            elif self.phase == self.SWING_DROP:
                # put foot down, go to quad shift
                self.last_pose[self.swing_leg][2] -= self.lift
                self.phase = self.QUAD_SHIFT
                return [self.standard_time, self.last_pose]
            else:
                # do nothing!
                return [self.standard_time, self.last_pose]

        phase_time = self.standard_time  # might want to modify phase_time for some shifts?

        if self.phase == self.QUAD_SHIFT:
            # Update velocities (stow for duration of this cycle)
            self.last_x_vel = x_vel
            self.last_y_vel = y_vel
            self.last_r_vel = r_vel
            # Do setup of choosing which leg is the swing leg
            if self.swing_leg == None:
                self.swing_leg = 0
            else:
                self.swing_leg = (self.swing_leg + 1) % 4;
            # Check if we need to shift
            points = [self.last_pose[k] for k in self.cross_poses[self.swing_leg]]
            isostable = reduceTriangle(points[0], points[1], points[2], self.stability)
            if not insideTriangle([0,0], isostable[0], isostable[1], isostable[2]):
                #offset = self.stableQuadShift(points[0], points[1], points[2])
                offset = self.fastQuadShift(points[0], points[1], points[2], self.stability*1.5)
                for j in range(4):
                    self.last_pose[j][0] -= offset[0]
                    self.last_pose[j][1] -= offset[1]
                self.x += offset[0]
                self.y += offset[1]
                # TODO r
                shift_time = (lengthLine([0,0], offset) / 0.08) # 8cm/s
                print "Swing leg is", self.robot.legs[self.swing_leg], "doing shift of", offset, "in time of", shift_time
                self.phase = self.SWING_LIFT
                return [shift_time, self.last_pose]
            # No need to quad shift
            self.phase = self.SWING_LIFT
            print "Swing leg is", self.robot.legs[self.swing_leg]

        if self.phase == self.SWING_LIFT:
            # Swing leg is computed, raise it
            self.last_pose[self.swing_leg][2] += self.lift

        elif self.phase == self.SWING_FORWARD:
            # Move swing leg forward
            vector = [self.last_x_vel, 0] # TODO: update this to do x/y/r velocity
            mag = self.getMaxDist(self.robot.ik[self.robot.legs[self.swing_leg]], self.last_pose[self.swing_leg], vector)
            if mag > 1.0:
                mag = 1.0
            update = [v*mag for v in vector]
            print "Swinging", self.robot.legs[self.swing_leg], "distance of", lengthLine([0,0], update)
            for i in range(len(update)):
                self.last_pose[self.swing_leg][i] += update[i]

        else: # DROP
            self.last_pose[self.swing_leg][2] -= self.lift
        
        # advance
        self.phase = (self.phase + 1) % self.phase_count

        # return our pose
        return [phase_time, self.last_pose]

    ## @brief Compute the Fast Quad Shift (Fig 4. in Rebula et al 2007)
    ## @param p1 Point [x,y] of the first of the diagonal feet
    ## @param p2 Point [x,y] of the second of the diagonal feet
    ## @param p3 Point [x,y] of the non-diagonal foot
    ## @param margin Iso-stability margin
    ## @returns [x,y] to move body to
    def fastQuadShift(self, p1, p2, p3, margin):
        # get the iso-stability triangle
        isostable = reduceTriangle(p1, p2, p3, margin)
        # find midway of isotable[0] and [1]
        return midwayLine(isostable[0], isostable[1])

    ## @brief Compute the Stable Quad Shift (Fig 5. in Rebula et al 2007)
    ## @param p1 Point [x,y] of the first  foot
    ## @param p2 Point [x,y] of the second foot
    ## @param p3 Point [x,y] of the third foot
    ## @returns [x,y] to move body to
    def stableQuadShift(self, p1, p2, p3):
        return centroidTriangle(p1, p2, p3)

    ## @brief Get the max distance that a leg can move
    ## @param ik_eval IK function
    ## @param pose The current pose of the leg
    ## @param vector The direction that the robot wants to move in
    def getMaxDist(self, ik_eval, pose, vector):
        d = 0
        for v in range(500):
            d_ = 0.01 * (v+1)
            vec = [d_*val for val in vector]
            res = ik_eval.getIK(pose[0] + vec[0], pose[1] + vec[1], pose[2]) #, self.roll, self.pitch, self.yaw)
            if not self.robot.checkLimits(res):
                break
            d = d_
        return d

