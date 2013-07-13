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

## @file geometry.py Various geometry helper functions (mainly for stability margins).

from math import sqrt

## @brief Is a 2d point inside a 2d triangle?
## @param pt The point [x, y]
## @param v1 First vertex of triangle [x, y]
## @param v2 Second vertex of triangle [x, y]
## @param v3 Third vertex of triangle [x, y]
def insideTriangle(pt, v1, v2, v3):
    def sign(p1, p2, p3):
        return (p1[0] - p3[0]) * (p2[1] - p3[1]) - (p2[0] - p3[0]) * (p1[1] - p3[1])
    b1 = sign(pt, v1, v2) < 0.0
    b2 = sign(pt, v2, v3) < 0.0
    b3 = sign(pt, v3, v1) < 0.0
    return ((b1 == b2) and (b2 == b3))

## @brief Find midway point of line
## @param p1 First point forming line [x, y]
## @param p2 Second point forming line [x, y]
def midwayLine(p1, p2):
    return [(p1[0] + p2[0])/2.0, (p1[1] + p2[1])/2.0]

## @brief Find a new endpoint for a line after scaling
## @param p1 First point forming line [x, y]
## @param p2 Second point forming line [x, y]
## @param scale Amount to scale the line
def scaleLine(p1, p2, scale):
    vector = [p2[0] - p1[0], p2[1] - p1[1]]
    for i in range(len(vector)):
        vector[i] = vector[i] * scale + p1[i]
    return vector

## @brief Find the length of a line
## @param p1 First point forming line [x, y]
## @param p2 Second point forming line [x, y]
def lengthLine(p1, p2):
    x = p2[0] - p1[0]
    y = p2[1] - p1[1]
    print "len", x, y
    return sqrt(x*x + y*y)

## @brief Find the centroid of the triangle
## @param v1 First vertex of triangle [x, y]
## @param v2 Second vertex of triangle [x, y]
## @param v3 Third vertex of triangle [x, y]
def centroidTriangle(v1, v2, v3):
    m1 = midwayLine(v2, v3)
    # v1 and m1 form a line
    e1 = scaleLine(v1, m1, 2/3.0)
    return e1

## @brief Scale a triangle, while keeping it centered about it's centroid
## @param v1 First vertex of triangle [x, y]
## @param v2 Second vertex of triangle [x, y]
## @param v3 Third vertex of triangle [x, y]
## @param scale Amount to scale the triangle
def scaleTriangle(v1, v2, v3, scale):
    # scale v2, v3
    v2s = scaleLine(v1, v2, scale)
    v3s = scaleLine(v1, v3, scale)
    # offset towards centroid   
    c = centroidTriangle(v1, v2, v3)
    offset = scaleLine(v1, c, scale)
    v1n = [v1[0] - offset[0], v1[1] - offset[1]]
    v2n = [v2s[0] - offset[0], v2s[1] - offset[1]]
    v3n = [v3s[0] - offset[0], v3s[1] - offset[1]]
    return [v1n, v2n, v3n]

## @brief Reduce a triangle, while keeping it centered about it's centroid.
## @param v1 First vertex of triangle [x, y]
## @param v2 Second vertex of triangle [x, y]
## @param v3 Third vertex of triangle [x, y]
## @param r Distance of iso-stability margin
def reduceTriangle(v1, v2, v3, r):
    c = centroidTriangle(v1, v2, v3)
    m = midwayLine(v1, v2)
    l = lengthLine(m, c)
    scale = (l-r)/l
    print scale
    return scaleTriangle(v1, v2, v3, scale)
