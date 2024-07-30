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

#ifndef SMALDOG_KINEMATICS_ODOMETRY_H
#define SMALDOG_KINEMATICS_ODOMETRY_H

#include <memory>

#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_kdl/tf2_kdl.hpp>

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
  bool update(RobotState& state, rclcpp::Node::SharedPtr node)
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
        //RCLCPP_ERROR("Unable to compute forward kinematics");
        return false;
      }

      /* Set a starting Z to the first leg that is on ground */
      for (size_t i = 0; i < 4; ++i)
      {
        if (state.leg_contact_likelihood[i] > 0.5)
        {
          last_state_.odom_transform.p.z(-poses[i].z());
          initialized_ = true;
          RCLCPP_INFO(node->get_logger(), "Odometry Initialized");
          return true;
        }
      }

      /* No legs on ground, can't initialize */
      //RCLCPP_WARN_THROTTLE(5, "Cannot initialize odometry as no legs are on ground");
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
          RCLCPP_ERROR(node->get_logger(), "Unable to compute forward kinematics");
          return false;
        }
        prev = poses[i];

        /* Use forward kinematics to get new position */
        if (!solver_->solveFK(state, poses[0], poses[1], poses[2], poses[3]))
        {
          RCLCPP_ERROR(node->get_logger(), "Unable to compute forward kinematics");
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

    /* Initialize broadcaster if needed */
    if (!broadcaster_)
    {
      broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(node);
    }

    /* Publish odometry */
    geometry_msgs::msg::TransformStamped t =
      tf2::kdlToTransform(last_state_.odom_transform * last_state_.body_transform);
    t.header.stamp = node->now();
    t.header.frame_id = "odom";
    t.child_frame_id = "body_link";
    broadcaster_->sendTransform(t);

    return true;
  }

private:
  bool initialized_;
  KinematicsSolver* solver_;
  RobotState last_state_;
  std::shared_ptr<tf2_ros::TransformBroadcaster> broadcaster_;
};

}  // namespace smaldog

#endif  // SMALDOG_KINEMATICS_ODOMETRY_H
