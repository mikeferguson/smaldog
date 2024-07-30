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

#ifndef SMALDOG_DRIVERS_TRAJECTORY_SAMPLER_H
#define SMALDOG_DRIVERS_TRAJECTORY_SAMPLER_H

#include <memory>
#include <string>
#include <vector>

#include <boost/thread/mutex.hpp>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <control_msgs/action/follow_joint_trajectory.hpp>

namespace smaldog
{

class TrajectorySampler
{
  struct TrajectoryPoint
  {
    rclcpp::Time time;
    std::vector<double> positions;
  };

  struct Trajectory
  {
    std::vector<TrajectoryPoint> points;
    int segment;
  };

  using FollowJointTrajectoryAction = control_msgs::action::FollowJointTrajectory;
  using FollowJointTrajectoryGoal = rclcpp_action::ServerGoalHandle<FollowJointTrajectoryAction>;

public:
  /**
   *  \brief Constructor for a trajectory sampler.
   *  \param ns The namespace for this action.
   *  \param joints Name of joints that should be sampled from.
   */
  TrajectorySampler(const std::string ns, const std::vector<std::string> joints);
  ~TrajectorySampler() {}

  /**
   *  \brief Setup ros publishers and subscribers.
   *  \param node Node-local node handle.
   */
  bool init(rclcpp::Node::SharedPtr node);

  /**
   *  \brief Sample at this time step.
   *  \param time The time to sample at.
   *  \param positions Vector of joint positions. Should be used to pass in
   *         actual joint positions, is then used to return the sampling,
   *         always in same order as joint names passed in.
   *  \returns True if we were able to sample for this time.
   */
  bool sample(rclcpp::Time time, std::vector<double>& positions);

  /**
   *  \brief Returns true if sampler is initialized.
   */
  bool initialized();

private:
  rclcpp_action::GoalResponse handleGoal(const rclcpp_action::GoalUUID & uuid,
    std::shared_ptr<const FollowJointTrajectoryAction::Goal> goal_handle);
  rclcpp_action::CancelResponse handleCancel(const std::shared_ptr<FollowJointTrajectoryGoal> goal_handle);
  void handleAccepted(const std::shared_ptr<FollowJointTrajectoryGoal> goal_handle);

  /**
   * \brief Publish feedback if there is an active goal
   */
  void feedbackCb();

  /**
   *  \brief Callback for a raw trajectory message.
   */
  void messageCb(const trajectory_msgs::msg::JointTrajectory::SharedPtr msg);

  /**
   *  \brief Create a new trajectory.
   */
  Trajectory createTrajectory(const trajectory_msgs::msg::JointTrajectory& msg);


  std::string namespace_;
  std::vector<std::string> joints_;

  // rclcpp action server
  rclcpp::Node::SharedPtr node_;
  rclcpp_action::Server<FollowJointTrajectoryAction>::SharedPtr server_;
  std::shared_ptr<FollowJointTrajectoryGoal> active_goal_;
  std::shared_ptr<FollowJointTrajectoryAction::Feedback> feedback_;
  rclcpp::TimerBase::SharedPtr feedback_timer_;
  rclcpp::Subscription<trajectory_msgs::msg::JointTrajectory>::SharedPtr subscriber_;

  Trajectory trajectory_;
  boost::mutex trajectory_mutex_;

  bool initialized_;
};

}  // namespace smaldog

#endif  // SMALDOG_DRIVERS_TRAJECTORY_SAMPLER_H
