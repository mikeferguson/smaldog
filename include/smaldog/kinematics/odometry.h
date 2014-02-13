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

#ifndef SMALDOG_KINEMATICS_ODOMETRY_H_
#define SMALDOG_KINEMATICS_ODOMETRY_H_

#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <tf_conversions/tf_kdl.h>

#include <smaldog/robot_state.h>
#include <smaldog/kinematics/kinematics_solver.h>

namespace smaldog
{

/**
 *  \brief A class for computing odometry of a walking robot.
 */
class WalkingOdometry
{
public:
  WalkingOdometry(KinematicsSolver * solver) :
    initialized_(false), solver_(solver)
  {
  }

  /**
   *  \brief Compute odometry based off a new state. Updates the odom_transform
   *         of RobotState passed in, publishes new TF message for odometry.
   */
  bool update(RobotState& state)
  {
    if (!initialized_)
    {
      last_state_.joint_positions = state.joint_positions;
      last_state_.odom_transform = state.odom_transform;
      last_state_.leg_contact_likelihood = state.leg_contact_likelihood;

      /* Use forward kinematics to get starting position */
      KDL::Vector poses[4];
      if (!solver_->solveFK(state, poses[0], poses[1], poses[2], poses[3]))
      {
        ROS_ERROR("Unable to compute forward kinematics");
        return false;
      }

      /* Set a starting Z to the first leg that is on ground */
      for (size_t i = 0; i < 4; ++i)
      {
        if (state.leg_contact_likelihood[i] > 0.5)
        {
          last_state_.odom_transform.p.z(-poses[i].z());
          initialized_ = true;
          ROS_INFO("Odometry Initialized");
          return true;
        }
      }

      /* No legs on ground, can't initialize */
      ROS_WARN_THROTTLE(5, "Cannot initialize odometry as no legs are on ground");
      return false;
    }

    for (size_t i = 0; i < 4; ++i)
    {
      /* Is this leg fixed? */
      if (state.leg_contact_likelihood[i] > 0.5 &&
          last_state_.leg_contact_likelihood[i] > 0.5)
      {
        KDL::Vector prev;
        KDL::Vector poses[4];
        /* Use forward kinematics to get last position */
        if (!solver_->solveFK(last_state_, poses[0], poses[1], poses[2], poses[3]))
        {
          ROS_ERROR("Unable to compute forward kinematics");
          return false;
        }
        prev = poses[i];

        /* Use forward kinematics to get new position */
        if (!solver_->solveFK(state, poses[0], poses[1], poses[2], poses[3]))
        {
          ROS_ERROR("Unable to compute forward kinematics");
          return false;
        }

        /* Compute motion */
        KDL::Vector motion = prev - poses[i];
        motion.z(0.0);
        last_state_.odom_transform.p += motion;
        state.odom_transform.p += motion;
        break;
      }
    }

    last_state_.joint_positions = state.joint_positions;
    last_state_.leg_contact_likelihood = state.leg_contact_likelihood;

    /* Publish odometry */
    tf::Transform t;
    tf::transformKDLToTF(last_state_.odom_transform * last_state_.body_transform, t);
    broadcaster_.sendTransform(tf::StampedTransform(t, ros::Time::now(), "odom", "body_link"));

    return true;
  }

private:
  bool initialized_;
  KinematicsSolver* solver_;
  RobotState last_state_;
  tf::TransformBroadcaster broadcaster_;
};

}  // namespace smaldog

#endif  // SMALDOG_KINEMATICS_ODOMETRY_H_
