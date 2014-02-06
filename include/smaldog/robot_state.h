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

#ifndef SMALDOG_ROBOT_STATE_H_
#define SMALDOG_ROBOT_STATE_H_

#include <vector>
#include <string>
#include <tf/transform_datatypes.h>

namespace smaldog
{

/**
 *  \brief State of the robot. This is used both as a state during planning,
 *         and for representing the robot's present state at execution time.
 */
class RobotState
{
public:
  /** \brief Robot joint names */
  std::vector<std::string> joint_names;

  /** \brief Robot joint angles, order matches joint names above */
  std::vector<double> joint_positions;

  /** \brief Names of legs */
  std::vector<std::string> leg_names;

  /** \brief Which legs are in contact with the ground? */
  std::vector<double> leg_contact_likelihood;

  /** \brief Body odometry transform (body_link -> odom) */
  tf::Transform odom_transform;

  // TODO: IMU?
};

}  // namespace smaldog

#endif  // SMALDOG_ROBOT_STATE_H_
