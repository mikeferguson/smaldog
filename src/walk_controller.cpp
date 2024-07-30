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

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

#include <control_msgs/action/follow_joint_trajectory.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <sensor_msgs/msg/joint_state.hpp>

#include <smaldog/kinematics/kinematics_solver.h>
#include <smaldog/kinematics/odometry.h>
#include <smaldog/planners/classical_crawl.h>
#include <smaldog/stability/visualization.h>

namespace smaldog
{

/**
 *  \brief The walking controller. This forms the main node.
 */
class WalkController : public rclcpp::Node
{
  using FollowTrajectory = control_msgs::action::FollowJointTrajectory;
  using GoalHandle = rclcpp_action::ClientGoalHandle<FollowTrajectory>;

public:
  WalkController(const rclcpp::NodeOptions & options) :
    rclcpp::Node("walk_controller", options),
    solver_(0.038, 0.172, 0.050, 0.065, 0.088),
    odom_(&solver_),
    crawl_(&solver_)
  {
    using namespace std::placeholders;
  
    /* Connect to body_controller action */
    RCLCPP_INFO(this->get_logger(), "Waiting for body_controller...");
    client_ = rclcpp_action::create_client<FollowTrajectory>(
      this->get_node_base_interface(),
      this->get_node_graph_interface(),
      this->get_node_logging_interface(),
      this->get_node_waitables_interface(),
      "body_controller/follow_joint_trajectory");
    RCLCPP_INFO(this->get_logger(), "  ...connected");

    /* Subscribe to joint_states */
    state_sub_ = this->create_subscription<sensor_msgs::msg::JointState>("joint_states", 1,
                     std::bind(&WalkController::jointStateCallback, this, _1));

    /* Subscribe to motion command */
    cmd_vel_sub_ = this->create_subscription<geometry_msgs::msg::Twist>("cmd_vel", 1,
                      std::bind(&WalkController::cmdVelCallback, this, _1));

    /* Fake Odometry for now */
    current_state_.odom_transform.p.z(0.1);
    current_state_.leg_contact_likelihood[0] = 1.0;
    current_state_.leg_contact_likelihood[1] = 1.0;
    current_state_.leg_contact_likelihood[2] = 1.0;
    current_state_.leg_contact_likelihood[3] = 1.0;
  }
  // add ability to select planner?

  bool update()
  {
    using namespace std::placeholders;
    rclcpp::Time now = this->now();

    // 1. update planner goals
    crawl_.setForwardVelocity(0.05);

    // 2. plan next step
    FollowTrajectory::Goal goal;
    if (crawl_.getNextStep(current_state_, goal.trajectory, now))
    {
      // 3. execute action
      goal_finished_ = false;
      auto goal_options = rclcpp_action::Client<FollowTrajectory>::SendGoalOptions();
      goal_options.goal_response_callback =
        std::bind(&WalkController::goalResponseCb, this, _1);
      goal_options.feedback_callback =
        std::bind(&WalkController::feedbackCb, this, _1, _2);
      goal_options.result_callback =
        std::bind(&WalkController::resultCb, this, _1);
      auto goal_handle_future = client_->async_send_goal(goal, goal_options);

      rclcpp::Time timeout = this->now() + rclcpp::Duration::from_seconds(5.0);
      while (!goal_finished_)
      {
        if (this->now() > timeout)
        {
          RCLCPP_ERROR(this->get_logger(), "Timed out waiting for action to complete");
          break;
        }
        if (!rclcpp::ok())
        {
          break;
        }
        rclcpp::sleep_for(std::chrono::milliseconds(50));
      }
    }
    else
    {
      std::cerr << "UNABLE TO CRAWL" << std::endl;
    }

    return true;
  }

private:
  /** \brief Callback for joint feedback */
  void jointStateCallback(const sensor_msgs::msg::JointState::SharedPtr msg)
  {
    /* Update joint_positions */
    for (size_t i = 0; i < current_state_.joint_names.size(); ++i)
    {
      for (size_t j = 0; j < msg->name.size(); ++j)
      {
        if (msg->name[j] == current_state_.joint_names[i])
          current_state_.joint_positions[i] = msg->position[j];
      }
    }

    /** Initialize if needed */
    if (!cog_publisher_)
    {
      cog_publisher_ = std::make_shared<CenterOfGravityPublisher>(shared_from_this());
      support_publisher_ = std::make_shared<SupportPolygonPublisher>(shared_from_this());
    }

    /* Publish COG/Support information */
    rclcpp::Time now = this->now();
    cog_publisher_->publish(current_state_, now);
    support_publisher_->publish(current_state_, solver_, now);

    /* Update odometry */
    odom_.update(current_state_, shared_from_this());
  }

  /** \brief Callback for command velocity from higher level planner or controller */
  void cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg)
  {
    // update goals to send to planner
  }

  void goalResponseCb(GoalHandle::SharedPtr goal_handle)
  {
    if (!goal_handle)
    {
      RCLCPP_ERROR(this->get_logger(), "Goal was rejected by server");
      goal_finished_ = true;
    }
  }

  void feedbackCb(GoalHandle::SharedPtr, const std::shared_ptr<const FollowTrajectory::Feedback>)
  {
    // TODO
  }

  void resultCb(const GoalHandle::WrappedResult &)
  {
    goal_finished_ = true;
  }

  bool goal_finished_;
  rclcpp_action::Client<FollowTrajectory>::SharedPtr client_;
  std::shared_ptr<CenterOfGravityPublisher> cog_publisher_;
  std::shared_ptr<SupportPolygonPublisher> support_publisher_;

  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr state_sub_;

  RobotState current_state_;
  KinematicsSolver solver_;

  WalkingOdometry odom_;
  ClassicalCrawl crawl_;
};

}  // namespace smaldog

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);

  rclcpp::NodeOptions options;
  std::shared_ptr<smaldog::WalkController> w;
  w.reset(new smaldog::WalkController(options));

  using rclcpp::executors::SingleThreadedExecutor;
  SingleThreadedExecutor executor;
  executor.add_node(w);
  std::thread executor_thread(std::bind(&SingleThreadedExecutor::spin, &executor));

  while (rclcpp::ok())
  {
    w->update();
  }

  return 0;
}
