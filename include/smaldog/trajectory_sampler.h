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

#ifndef SMALDOG_TRAJECTORY_SAMPLER_H_
#define SMALDOG_TRAJECTORY_SAMPLER_H_

#include <vector>
#include <string>

#include <boost/shared_ptr.hpp>

#include <ros/ros.h>
#include <control_msgs/FollowJointTrajectoryAction.h>
#include <actionlib/server/simple_action_server.h>

namespace smaldog
{

class TrajectorySampler
{
  typedef actionlib::SimpleActionServer<control_msgs::FollowJointTrajectoryAction> server_t;

public:
  /**
   *  \brief Constructor for a trajectory sampler.
   *  \param ns The namespace for this action.
   *  \param joints Name of joints that should be sampled from.
   */
  TrajectorySampler(std::string ns, std::vector<std::string> joints);
  ~TrajectorySampler() {}

  /**
   *  \brief Setup ros publishers and subscribers.
   *  \param nh Node-local node handle.
   */
  bool init(ros::NodeHandle& nh);

  /**
   *  \brief Sample at this time step.
   *  \param time The time to sample at.
   *  \param positions Vector of joint positions returned in same order as joint names passed in.
   *  \returns True if we were able to sample for this time.
   */
  bool sample(ros::Time time, std::vector<double>& positions);

private:
  /**
   *  \brief Callback for the trajectory action server.
   */
  void actionCb(const control_msgs::FollowJointTrajectoryGoalConstPtr& goal);

  /**
   *  \brief Callback for a raw trajectory message.
   */
  void messageCb(const trajectory_msgs::JointTrajectoryConstPtr& msg);

  std::string namespace_;
  std::vector<std::string> joints_;

  boost::shared_ptr<server_t> server_;
  ros::Subscriber subscriber_;

  control_msgs::FollowJointTrajectoryFeedback feedback_;
};

}  // namespace smaldog

#endif  // SMALDOG_TRAJECTORY_SAMPLER_H_
