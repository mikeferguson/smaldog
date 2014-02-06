/*
 * Copyright (c) 2014 Michael E. Ferguson.  All right reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software Foundation,
 * Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA 
 */

#ifndef SMALDOG_STABILITY_FAST_QUAD_SHIFT_H_
#define SMALDOG_STABILITY_FAST_QUAD_SHIFT_H_

#include <kdl/frames.hpp>
#include <smaldog/robot_state.h>

namespace smaldog
{

/**
 *  \brief Compute the fast quad shift for a particular robot state, as described
 *         in Rebula et. Al 2007.
 *  \param state The state of the robot joints and which legs are in contact
 *         with the ground.
 *  \param shift The computed shift. This represents a movement of the body COG
 *         within the body-centric coordinate frame.
 *  \returns true if shift is properly computed, false if there is some sort of
 *           error (typically numerical)
 */
bool computeFastQuadShift(const RobotState& state, KDL::Vector& shift);

}  // namespace smaldog

#endif  // SMALDOG_STABILITY_FAST_QUAD_SHIFT_H_
