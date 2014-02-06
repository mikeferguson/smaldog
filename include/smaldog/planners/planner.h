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

#ifndef SMALDOG_PLANNING_PLANNER_H_
#define SMALDOG_PLANNING_PLANNER_H_

#include <smaldog/robot_state.h>

namespace smaldog
{

/**
 *  \brief Base class for a gait planner.
 *
 *  A planner would typically be used by a control program which
 *  then updates the goals, calls the planner to get the next step,
 *  and then executes the trajectory.
 */
class Planner
{
public:
  Planner() {}
  virtual ~Planner() {}

  /**
   *  \brief Set the state of the robot, including where joints are, which
   *         legs are on the ground, etc.
   */
  bool setState(const RobotState& state) = 0;

  bool setForwardVelocity(double x) = 0;
  bool setStrafingVelocity(double y) = 0;
  bool setTurningVelocity(double theta) = 0;

  /**
   *  \brief Generates a trajectory for the next step.
   */
  bool getNextStep(trajectory_msgs::JointTrajectory& trajectory) = 0;

private:

};

}  // namespace smaldog

#endif  // SMALDOG_PLANNING_PLANNER_H_
