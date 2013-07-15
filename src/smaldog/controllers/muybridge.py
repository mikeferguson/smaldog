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

## @brief The standard gait controller, with some thoughts from Rebula et al, 2007
class MuybridgeGaitController:
    legs = ["rf", "rr", "lf", "lr"]
    standard_time = 0.25
    # indexes of points cross legs, first 2 are diagonal to leg
    cross_poses = [ [1, 2, 3],
                    [0, 3, 2],
                    [0, 3, 1],
                    [1, 2, 0] ]
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
        self.step_time = 0.25 # time to complete a step (1 full cycle/sec)

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
                print "FORWARD: Not moving, setting foot down."
                # put that foot down, but go back to lift so that when we start walking we use it
                self.last_pose[self.swing_leg][2] -= self.lift
                self.phase = self.SWING_LIFT
                return [self.standard_time, self.last_pose]
            elif self.phase == self.SWING_DROP:
                print "DROP: Not moving, setting foot down."
                # put foot down, go to quad shift
                self.last_pose[self.swing_leg][2] -= self.lift
                self.phase = self.QUAD_SHIFT
                return [self.standard_time, self.last_pose]
            else:
                # do nothing!
                return [self.standard_time, self.last_pose]

        phase_time = self.standard_time  # might want to modify phase_time for some shifts?

        if self.phase == self.QUAD_SHIFT:
            # Has direction changed?
            dir_changed = self.compareDirections(x_vel, y_vel, r_vel, self.last_x_vel, self.last_y_vel, self.last_r_vel)
            # Update velocities (stow for duration of this cycle)
            self.last_x_vel = x_vel
            self.last_y_vel = y_vel
            self.last_r_vel = r_vel
            # Do setup of choosing which leg is the swing leg
            swing_leg = self.computeSwingLeg(self.last_pose, self.swing_leg, dir_changed)
            if swing_leg == None:
                print "No swing leg found, shifting..."
                dists = list()
                offsets = list()
                # Need to do a quad shift, iterate through legs to find best choice
                for i in range(4):
                    print "  evaluating", self.legs[i]
                    new_pose = copy.deepcopy(self.last_pose)
                    points = [new_pose[k] for k in self.cross_poses[i]]
                    offset = self.fastQuadShift(points[0], points[1], points[2], 0.01)
                    print "    offset of", offset
                    offsets.append(offset)
                    for j in range(4):
                        new_pose[j][0] -= offset[0]
                        new_pose[j][1] -= offset[1]
                    vector = [self.last_x_vel, 0] # TODO: update this to do x/y/r velocity
                    dist = self.getMaxDist(self.robot.ik[self.legs[i]], new_pose[i], vector)
                    print "    allows movement of", dist
                    dists.append(dist)
                
                self.swing_leg = dists.index(max(dists))
                print "Swing leg is", self.legs[self.swing_leg] 
                # Apply shift
                for j in range(4):
                    self.last_pose[j][0] -= offsets[self.swing_leg][0]
                    self.last_pose[j][1] -= offsets[self.swing_leg][1]
                self.x += offsets[self.swing_leg][0]
                self.y += offsets[self.swing_leg][1]
                # TODO r
                self.phase = self.SWING_LIFT
                return [phase_time, self.last_pose]
            # We have a swing leg, and no need to quad shift
            self.swing_leg = swing_leg
            self.phase = self.SWING_LIFT
            print "Swing leg is", self.legs[self.swing_leg]

        if self.phase == self.SWING_LIFT:
            # Swing leg is computed, raise it
            self.last_pose[self.swing_leg][2] += self.lift
            print "Lifting", self.legs[self.swing_leg]

        elif self.phase == self.SWING_FORWARD:
            print "Moving", self.legs[self.swing_leg], "forward"
            # Move swing leg forward
            vector = [self.last_x_vel, 0] # TODO: update this to do x/y/r velocity
            mag = self.getMaxDist(self.robot.ik[self.legs[self.swing_leg]], self.last_pose[self.swing_leg], vector)
            if mag > 1.0:
                mag = 1.0
            update = [v*mag for v in vector]
            print "  leg movement is", update
            for i in range(len(update)):
                self.last_pose[self.swing_leg][i] += update[i]

            # Compute shift of body, apply to all legs
            if self.swing_leg == 1 or self.swing_leg == 3:
                # For hind legs, shift halfway to the midpoint of the front stability boundary
                pts = [self.last_pose[k] for k in self.cross_poses[self.swing_leg]]
                isostable = reduceTriangle(pts[0], pts[1], pts[2], 0.01)
                front_mid = midwayLine(isostable[0], isostable[2])
                offset = midwayLine([0,0], front_mid)
                print "  hind leg causes body shift of", offset
            else:
                # For front legs, shift to the midpoint of the front stability boundary
                pts = [self.last_pose[k] for k in self.cross_poses[self.swing_leg]]
                isostable = reduceTriangle(pts[0], pts[1], pts[2], 0.01)
                print "  isostable", isostable
                print "  center of isostable", centroidTriangle(isostable[0], isostable[1], isostable[2])
                print "  midpoint (back)", midwayLine(isostable[0], isostable[2])
                print "           (side)", midwayLine(isostable[1], isostable[2])
                offset = midwayLine(isostable[0], isostable[1])
                print "  front leg causes body shift of", offset
            for j in range(4): #self.cross_poses[self.swing_leg]:
                # TODO: we should compute body shift first, so we get max movement
                self.last_pose[j][0] -= offset[0]
                self.last_pose[j][1] -= offset[1]
            self.x += offset[0]
            self.y += offset[1]
            # TODO r

        else: # DROP
            print "Dropping", self.legs[self.swing_leg]
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
            res = ik_eval.getIK(pose[0] + vec[0], pose[1] + vec[1], pose[2]) #, self.roll, self.pitch, self.yaw)x
            if not self.robot.checkLimits(res):
                break
            d = d_
        return d

    ## @brief Compute the next swing leg to use
    ## @param poses The poses of the legs
    ## @param prev_swing The previous swing leg.
    ## @param dir_changed Has the direction changed since last cycle?
    def computeSwingLeg(self, poses, prev_swing, dir_changed):
        print "  Computing swing leg..."
        # Which legs can be a swing leg?
        candidates = list()
        for i in range(4):
            # If no direction change, remove previous swing leg from option
            if self.swing_leg == i and not dir_changed:
                continue
            # Test if support polygon is OK
            pts = [poses[j] for j in self.cross_poses[i]]
            iso = reduceTriangle(pts[0], pts[1], pts[2], 0.01)
            if insideTriangle([0,0], iso[0], iso[1], iso[2]):
                print "      ", self.legs[i], "is candidate"
                candidates.append(i)
        # Of the candidates, which can go the farthest?
        max_dist = list()
        for c in candidates:
            vector = [self.last_x_vel, 0] # TODO: update this to do x/y/r velocity
            max_dist.append(self.getMaxDist(self.robot.ik[self.legs[c]], poses[c], vector))
        print "    candidate distances:", max_dist
        if len(max_dist) == 0:
            return None
        if max(max_dist) < 0.001:
            return None
        return candidates[max_dist.index(max(max_dist))]

    ## @brief Determine if the direction of desired velocity has changed
    def compareDirections(self, x1, y1, r1, x2, y2, r2):
        # TODO
        return False

