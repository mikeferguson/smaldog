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

#include "smaldog/trajectory_sampler.h"

namespace smaldog
{

TrajectorySampler::TrajectorySampler(const std::string ns, const std::vector<std::string> joints)
 : namespace_(ns), joints_(joints)
{
  /* Setup action feedback */ 
  feedback_.desired.positions.resize(joints_.size());    
  feedback_.actual.positions.resize(joints_.size());    
  feedback_.error.positions.resize(joints_.size());
}

bool TrajectorySampler::init(ros::NodeHandle& nh)
{
  /* Setup action server */
  server_.reset(new server_t(nh, namespace_+"/follow_joint_trajectory",
                             boost::bind(&TrajectorySampler::actionCb, this, _1),
                             false));
  server_->start();

  /* Setup trajectory message callback */
  subscriber_ = nh.subscribe<trajectory_msgs::JointTrajectory>(
    namespace_+"/trajectory", 5, boost::bind(&TrajectorySampler::messageCb, this, _1));

  return true;
}

bool TrajectorySampler::sample(ros::Time time, std::vector<double>& positions)
{
  if ( false /* can sample */ )
  {
    positions.resize(joints_.size());
    // TODO: sample
    return true;
  }
  return false;
}

void TrajectorySampler::actionCb(const control_msgs::FollowJointTrajectoryGoalConstPtr& goal)
{
  control_msgs::FollowJointTrajectoryResult result;

  if (goal->trajectory.points.empty())
  {
    // TODO: Stop
    return;
  }

  if (goal->trajectory.joint_names.size() != joints_.size())
  {
    result.error_code = control_msgs::FollowJointTrajectoryResult::INVALID_JOINTS;
    server_->setAborted(result, "Invalid number of joints");
    ROS_ERROR("Invalid number of joints");
    return;
  }

  // TODO: update path
}

void TrajectorySampler::messageCb(const trajectory_msgs::JointTrajectoryConstPtr& msg)
{
  if (msg->points.empty())
  {
    // TODO: Stop
    return;
  }

  if (msg->joint_names.size() != joints_.size())
  {
    ROS_ERROR("Invalid number of joints");
    return;
  }

  // TODO: update path
}

}  // namespace smaldog
