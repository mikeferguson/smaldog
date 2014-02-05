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
  feedback_.joint_names = joints_;
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
  /* Attempt to update actual positions */
  if (positions.size() == feedback_.joint_names.size())
  {
    for (size_t j = 0; j < positions.size(); ++j)
      feedback_.actual.positions[j] = positions[j];
  }

  /* If we have no trajectory to sample, stop now */
  if (trajectory_.points.size() == 0)
    return false;

  /* Lock and sample */
  trajectory_mutex_.lock();
  /* Determine which segment we are in */
  while ((trajectory_.segment + 1 < trajectory_.points.size()) &&
         (trajectory_.points[trajectory_.segment+1].time < time))
    ++trajectory_.segment;
  /* Sample the segment */
  if (trajectory_.segment == -1)
  {
    /* Time not started, return first point */
    feedback_.desired.positions = trajectory_.points[0].positions;
  }
  else if (trajectory_.segment + 1 >= trajectory_.points.size())
  {
    /* End of trajectory, return last point */
    feedback_.desired.positions = trajectory_.points[trajectory_.segment].positions;
    /* If we are executing an action, respond */
    if (server_->isActive())
    {
      control_msgs::FollowJointTrajectoryResult result;
      result.error_code = control_msgs::FollowJointTrajectoryResult::SUCCESSFUL;
      server_->setSucceeded(result, "Completed.");
      ROS_DEBUG("Completed");
    }
  }
  else
  {
    double t = (trajectory_.points[trajectory_.segment+1].time - trajectory_.points[trajectory_.segment].time).toSec();
    for (size_t i = 0; i < positions.size(); ++i)
    {
      feedback_.desired.positions[i] = trajectory_.points[trajectory_.segment].positions[i] +
        (trajectory_.points[trajectory_.segment+1].positions[i] - trajectory_.points[trajectory_.segment].positions[i]) * t;
    }
  }
  trajectory_mutex_.unlock();

  positions = feedback_.desired.positions;
  return true;
}

void TrajectorySampler::actionCb(const control_msgs::FollowJointTrajectoryGoalConstPtr& goal)
{
  control_msgs::FollowJointTrajectoryResult result;

  if (goal->trajectory.points.empty())
  {
    /* Stop execution */
    trajectory_mutex_.lock();
    trajectory_.points.clear();
    trajectory_mutex_.unlock();
    return;
  }

  if (goal->trajectory.joint_names.size() != joints_.size())
  {
    result.error_code = control_msgs::FollowJointTrajectoryResult::INVALID_JOINTS;
    server_->setAborted(result, "Invalid number of joints");
    ROS_ERROR("Invalid number of joints");
    return;
  }

  /* Create trajectory */
  Trajectory t = createTrajectory(goal->trajectory);

  /* Lock and update trajectory */
  trajectory_mutex_.lock();
  trajectory_ = t;
  trajectory_mutex_.unlock();

  /* Deal with feedback */
  while (server_->isActive())
  {
    if (server_->isPreemptRequested())
    {
      control_msgs::FollowJointTrajectoryResult result;
      server_->setPreempted(result, "Preempted");
      ROS_DEBUG("Preempted");
      break;
    }

    /* Publish Feedback */
    feedback_.header.stamp = ros::Time::now();
    for (size_t i = 0; i < feedback_.error.positions.size(); ++i)
      feedback_.error.positions[i] = feedback_.desired.positions[i] - feedback_.actual.positions[i];
    server_->publishFeedback(feedback_);
    ros::Duration(1/25.0).sleep();
  }
}

void TrajectorySampler::messageCb(const trajectory_msgs::JointTrajectoryConstPtr& msg)
{
  if (msg->points.empty())
  {
    /* Stop execution */
    trajectory_mutex_.lock();
    trajectory_.points.clear();
    trajectory_mutex_.unlock();
    return;
  }

  if (msg->joint_names.size() != joints_.size())
  {
    ROS_ERROR("Invalid number of joints");
    return;
  }

  /* Create trajectory */
  Trajectory t = createTrajectory(*msg);

  /* Lock and update trajectory */
  trajectory_mutex_.lock();
  trajectory_ = t;
  trajectory_mutex_.unlock();
}

TrajectorySampler::Trajectory
TrajectorySampler::createTrajectory(const trajectory_msgs::JointTrajectory& msg)
{
  ros::Time time = ros::Time::now();
  Trajectory t;

  if (msg.points.size() < 2)
  {
    /* If we only have one new point, interpolate between present position and the new point */
    t.points.resize(2);
    t.points[0].positions = feedback_.actual.positions;
    t.points[1].positions = msg.points[0].positions;
    t.points[0].time = time;
    t.points[1].time = msg.header.stamp + msg.points[0].time_from_start;
  }
  else
  {
    /* Save old, relevant points */
    for (size_t i = 0; i < trajectory_.points.size(); ++i)
    {
      if ((trajectory_.points[i].time > time) &&
          (trajectory_.points[i].time < msg.header.stamp))
      {
        t.points.push_back(trajectory_.points[i]);
      }
    }

    /* Add new points */
    for (size_t i = 0; i < msg.points.size(); ++i)
    {
      TrajectoryPoint p;
      p.positions = msg.points[i].positions;
      p.time = msg.header.stamp + msg.points[i].time_from_start;
      t.points.push_back(p);
    }
  }

  t.segment = -1;

  return t;
}

}  // namespace smaldog
