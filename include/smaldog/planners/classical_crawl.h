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

#ifndef SMALDOG_CLASSICAL_CRAWL_H_
#define SMALDOG_CLASSICAL_CRAWL_H_

#include <smaldog/robot_state.h>
#include <smaldog/kinematics/kinematics_solver.h>
#include <smaldog/planners/planner.h>

namespace smaldog
{

/**
 *  \brief The classical crawl gait, as a simple planner.
 */
class ClassicalCrawl : public Planner
{
public:
  ClassicalCrawl(KinematicsSolver * solver) : solver_(solver),
    x_(0.0), y_(0.0), theta_(0.0), swing_leg_(LEFT_FRONT)
  {
    /* Get default stance */
    RobotState state;
    for (size_t i = 0; i < state.joint_positions.size(); ++i)
      state.joint_positions[i] = 0.0;
    solver_->solveFK(state, poses_[0], poses_[1], poses_[2], poses_[3]);
    for (size_t i = 0; i < 4; ++i)
    {
      poses_[i].z(poses_[i].z() * 0.65);
      poses_[i].x(poses_[i].x() + 0.0125);
    }
  }

  bool setForwardVelocity(double x)
  {
    x_ = x;
    return true;
  }

  bool setStrafingVelocity(double y)
  {
    y_ = y;
    return true;
  }

  bool setTurningVelocity(double theta)
  {
    theta_ = theta;
    return true;
  }

  bool getNextStep(RobotState& state,
                   trajectory_msgs::JointTrajectory& trajectory)
  {
    if (moving())
    {
      // TODO: swing COG

      RobotState temp;
      trajectory.joint_names = temp.joint_names;
      trajectory.points.resize(4);

      /* First point, where we are */
      if (!solver_->solveIK(poses_[0], poses_[1], poses_[2], poses_[3], 0.0, 0.0, 0.0, temp))
        return false;
      trajectory.points[0].positions = temp.joint_positions;
      trajectory.points[0].time_from_start = ros::Duration(0.0);

      /* Second point, raise swing leg */
      for (size_t i = 0; i < 4; ++i)
      {
        if (i == swing_leg_)
        {
          poses_[i].x(poses_[i].x() + x_/4);
          poses_[i].z(poses_[i].z() + 0.02);
        }
      }
      if (!solver_->solveIK(poses_[0], poses_[1], poses_[2], poses_[3], 0.0, 0.0, 0.0, temp))
        return false;
      trajectory.points[1].positions = temp.joint_positions;
      trajectory.points[1].time_from_start = ros::Duration(0.25);

      /* Third point, swing legs forward */
      for (size_t i = 0; i < 4; ++i)
      {
        if (i == swing_leg_)
          poses_[i].x(poses_[i].x() + x_/2);
        else
          poses_[i].x(poses_[i].x() - x_/3.0);
      }
      if (!solver_->solveIK(poses_[0], poses_[1], poses_[2], poses_[3], 0.0, 0.0, 0.0, temp))
        return false;
      trajectory.points[2].positions = temp.joint_positions;
      trajectory.points[2].time_from_start = ros::Duration(0.75);

      /* Fourth point, drop swing leg */
      for (size_t i = 0; i < 4; ++i)
      {
        if (i == swing_leg_)
        {
          poses_[i].x(poses_[i].x() + x_/4);
          poses_[i].z(poses_[i].z() - 0.02);
        }
      }
      if (!solver_->solveIK(poses_[0], poses_[1], poses_[2], poses_[3], 0.0, 0.0, 0.0, temp))
        return false;
      trajectory.points[3].positions = temp.joint_positions;
      trajectory.points[3].time_from_start = ros::Duration(1.0);

      /* Update contact legs */
      for (size_t i = 0; i < state.leg_contact_likelihood.size(); ++i)
      {
        if (i == swing_leg_)
          state.leg_contact_likelihood[i] = 0.0;
        else
          state.leg_contact_likelihood[i] = 1.0;
      }

      /* Update swing leg */
      swing_leg_ = (swing_leg_+1)%4;

      trajectory.header.stamp = ros::Time::now();
      return true;
    }
    else
    {
      /* Return last pose */
      RobotState s;
      if (solver_->solveIK(poses_[0], poses_[1], poses_[2], poses_[3],
                           0.0, 0.0, 0.0, s))
      {
        trajectory.joint_names = state.joint_names;
        trajectory.points.resize(1);
        trajectory.points[0].positions = s.joint_positions;
        trajectory.points[0].time_from_start = ros::Duration(1.0);
        trajectory.header.stamp = ros::Time::now();
        return true;
      }
      return false;
    }
  }

private:
  bool moving() const
  {
    if (fabs(x_) > 0.001 || fabs(y_) > 0.001 || fabs(theta_) > 0.01)
      return true;
    return false;
  }

  double x_, y_, theta_;
  int swing_leg_;

  KDL::Vector poses_[4];
  KinematicsSolver* solver_;
};

}  // namespace smaldog

#endif  // SMALDOG_ROBOT_STATE_H_
