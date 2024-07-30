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

#include "smaldog/drivers/trajectory_sampler.h"

namespace smaldog
{

TrajectorySampler::TrajectorySampler(const std::string ns, const std::vector<std::string> joints)
 : namespace_(ns), joints_(joints), initialized_(false)
{
  /* Setup action feedback */
  feedback_ = std::make_shared<FollowJointTrajectoryAction::Feedback>();
  feedback_->joint_names = joints_;
  feedback_->desired.positions.resize(joints_.size());    
  feedback_->actual.positions.resize(joints_.size());    
  feedback_->error.positions.resize(joints_.size());
}

bool TrajectorySampler::init(rclcpp::Node::SharedPtr node)
{
  using namespace std::placeholders;

  /* Store this for logging and timing */
  node_ = node;

  /* Setup action server */
  server_ = rclcpp_action::create_server<FollowJointTrajectoryAction>(
    node->get_node_base_interface(),
    node->get_node_clock_interface(),
    node->get_node_logging_interface(),
    node->get_node_waitables_interface(),
    namespace_ + "/follow_joint_trajectory",
    std::bind(&TrajectorySampler::handleGoal, this, _1, _2),
    std::bind(&TrajectorySampler::handleCancel, this, _1),
    std::bind(&TrajectorySampler::handleAccepted, this, _1)
  );

  feedback_timer_ = node->create_wall_timer(std::chrono::milliseconds(50),
                      std::bind(&TrajectorySampler::feedbackCb, this));

  /* Setup trajectory message callback */
  subscriber_ = node->create_subscription<trajectory_msgs::msg::JointTrajectory>(
    namespace_ + "/trajectory", 5, std::bind(&TrajectorySampler::messageCb, this, _1));

  initialized_ = true;

  return true;
}

bool TrajectorySampler::sample(rclcpp::Time time, std::vector<double>& positions)
{
  /* Attempt to update actual positions */
  if (positions.size() == feedback_->joint_names.size())
  {
    for (size_t j = 0; j < positions.size(); ++j)
      feedback_->actual.positions[j] = positions[j];
  }

  /* If we have no trajectory to sample, stop now */
  if (trajectory_.points.empty())
    return false;

  /* Lock and sample */
  trajectory_mutex_.lock();
  /* Determine which segment we are in */
  while ((static_cast<size_t>(trajectory_.segment + 1) < trajectory_.points.size()) &&
         (trajectory_.points[trajectory_.segment + 1].time < time))
    ++trajectory_.segment;
  /* Sample the segment */
  if (trajectory_.segment == -1)
  {
    /* Time not started, return first point */
    feedback_->desired.positions = trajectory_.points[0].positions;
  }
  else if (static_cast<size_t>(trajectory_.segment + 1) >= trajectory_.points.size())
  {
    /* End of trajectory, return last point */
    feedback_->desired.positions = trajectory_.points[trajectory_.segment].positions;
    /* If we are executing an action, respond */
    if (active_goal_)
    {
      auto result = std::make_shared<FollowJointTrajectoryAction::Result>();
      result->error_code = result->SUCCESSFUL;
      active_goal_->succeed(result);
      active_goal_.reset();
      RCLCPP_DEBUG(node_->get_logger(), "Completed");
    }
  }
  else
  {
    double dt = (trajectory_.points[trajectory_.segment+1].time - trajectory_.points[trajectory_.segment].time).seconds();
    double t = (time - trajectory_.points[trajectory_.segment].time).seconds() / dt;
    for (size_t i = 0; i < positions.size(); ++i)
    {
      feedback_->desired.positions[i] = trajectory_.points[trajectory_.segment].positions[i] +
        (trajectory_.points[trajectory_.segment+1].positions[i] - trajectory_.points[trajectory_.segment].positions[i]) * t;
    }
  }
  trajectory_mutex_.unlock();

  positions = feedback_->desired.positions;
  return true;
}

bool TrajectorySampler::initialized()
{
  return initialized_;
}

rclcpp_action::GoalResponse
TrajectorySampler::handleGoal(const rclcpp_action::GoalUUID &,
  std::shared_ptr<const FollowJointTrajectoryAction::Goal> goal_handle)
{
  if (goal_handle->trajectory.joint_names.size() != joints_.size())
  {
    RCLCPP_ERROR(node_->get_logger(), "Invalid number of joints");
    return rclcpp_action::GoalResponse::REJECT;
  }

  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse
TrajectorySampler::handleCancel(const std::shared_ptr<FollowJointTrajectoryGoal> goal_handle)
{
  if (active_goal_ && active_goal_->get_goal_id() == goal_handle->get_goal_id())
  {
    auto result = std::make_shared<FollowJointTrajectoryAction::Result>();
    active_goal_->canceled(result);
    active_goal_.reset();
  }

  return rclcpp_action::CancelResponse::ACCEPT;
}

void TrajectorySampler::handleAccepted(const std::shared_ptr<FollowJointTrajectoryGoal> goal_handle)
{
  auto result = std::make_shared<FollowJointTrajectoryAction::Result>();
  const auto goal = goal_handle->get_goal();

  if (active_goal_)
  {
    active_goal_->abort(result);
    active_goal_.reset();
    RCLCPP_WARN(node_->get_logger(), "Preempted trajectory");
  }

  if (goal->trajectory.points.empty())
  {
    /* Stop execution */
    trajectory_mutex_.lock();
    trajectory_.points.clear();
    trajectory_mutex_.unlock();
    result->error_code = result->SUCCESSFUL;
    goal_handle->succeed(result);
    RCLCPP_WARN(node_->get_logger(), "Received empty trajectory, stopping execution");
    return;
  }

  /* Create trajectory */
  Trajectory t = createTrajectory(goal->trajectory);

  /* Lock and update trajectory */
  trajectory_mutex_.lock();
  trajectory_ = t;
  active_goal_ = goal_handle;
  trajectory_mutex_.unlock();
}

void TrajectorySampler::feedbackCb()
{
  if (active_goal_)
  {
    /* Publish Feedback */
    feedback_->header.stamp = node_->now();
    for (size_t i = 0; i < feedback_->error.positions.size(); ++i)
      feedback_->error.positions[i] = feedback_->desired.positions[i] - feedback_->actual.positions[i];
    active_goal_->publish_feedback(feedback_);
  }
}

void TrajectorySampler::messageCb(const trajectory_msgs::msg::JointTrajectory::SharedPtr msg)
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
    RCLCPP_ERROR(node_->get_logger(), "Invalid number of joints");
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
TrajectorySampler::createTrajectory(const trajectory_msgs::msg::JointTrajectory& msg)
{
  rclcpp::Time now = node_->now();
  Trajectory t;

  /* Make sure time is filled in */
  rclcpp::Time start = msg.header.stamp;
  if (start.nanoseconds() == 0)
    start = now;

  if (msg.points.size() < 2)
  {
    /* If we only have one new point, interpolate between present position and the new point */
    t.points.resize(2);
    t.points[0].positions = feedback_->actual.positions;
    t.points[1].positions = msg.points[0].positions;
    t.points[0].time = now;
    t.points[1].time = start + msg.points[0].time_from_start;
  }
  else
  {
    /* Save old, relevant points */
    for (size_t i = 0; i < trajectory_.points.size(); ++i)
    {
      if ((trajectory_.points[i].time > now) &&
          (trajectory_.points[i].time < start))
      {
        t.points.push_back(trajectory_.points[i]);
      }
    }

    /* Add new points */
    for (size_t i = 0; i < msg.points.size(); ++i)
    {
      TrajectoryPoint p;
      p.positions = msg.points[i].positions;
      p.time = start + msg.points[i].time_from_start;
      t.points.push_back(p);
    }
  }

  t.segment = -1;

  return t;
}

}  // namespace smaldog
