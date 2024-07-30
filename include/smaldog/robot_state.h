/*
 * Copyright (c) 2014-2024 Michael E. Ferguson.  All right reserved.
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

#ifndef SMALDOG_ROBOT_STATE_H
#define SMALDOG_ROBOT_STATE_H

#include <vector>
#include <string>
#include <kdl/frames.hpp>

namespace smaldog
{

enum Legs
{
  LEFT_FRONT,
  RIGHT_REAR,
  RIGHT_FRONT,
  LEFT_REAR
};

/**
 *  \brief State of the robot. This is used both as a state during planning,
 *         and for representing the robot's present state at execution time.
 */
class RobotState
{
public:
  RobotState() : odom_transform(KDL::Rotation::Identity(), KDL::Vector::Zero()),
                 body_transform(KDL::Rotation::Identity(), KDL::Vector::Zero())
  {
    /* This is the servo order */
    joint_names.push_back("rf_pitch_joint");
    joint_names.push_back("lf_pitch_joint");
    joint_names.push_back("rf_flex_joint");
    joint_names.push_back("lf_flex_joint");
    joint_names.push_back("rf_knee_joint");
    joint_names.push_back("lf_knee_joint");
    joint_names.push_back("rr_pitch_joint");
    joint_names.push_back("lr_pitch_joint");
    joint_names.push_back("rr_flex_joint");
    joint_names.push_back("lr_flex_joint");
    joint_names.push_back("rr_knee_joint");
    joint_names.push_back("lr_knee_joint");

    joint_positions.resize(joint_names.size());
    leg_contact_likelihood.resize(4);
  }

  /** \brief Robot joint names */
  std::vector<std::string> joint_names;

  /** \brief Robot joint angles, order matches joint names above */
  std::vector<double> joint_positions;

  /** \brief Which legs are in contact with the ground? */
  std::vector<double> leg_contact_likelihood;

  /** \brief Body odometry transform (body_link -> odom). */
  KDL::Frame odom_transform;

  /** \brief Body rotation about body_link frame */
  KDL::Frame body_transform;


  // TODO: IMU?
};

}  // namespace smaldog

#endif  // SMALDOG_ROBOT_STATE_H
