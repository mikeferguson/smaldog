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

#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <tf/transform_broadcaster.h>
#include <tf_conversions/tf_kdl.h>

#include <control_msgs/FollowJointTrajectoryAction.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/JointState.h>

#include <smaldog/kinematics/kinematics_solver.h>
#include <smaldog/stability/visualization.h>

namespace smaldog
{

/**
 *  \brief The walking controller. This forms the main node.
 */
class WalkController
{
  typedef actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction> client_t;

public:
  WalkController(ros::NodeHandle& nh) :
    client_("body_controller/follow_joint_trajectory", true),
    cog_publisher_(nh),
    support_publisher_(nh),
    solver_(0.038, 0.172, 0.050, 0.065, 0.088)
  {
    /* Connect to body_controller action */
    ROS_INFO("Waiting for body_controller...");
    client_.waitForServer();
    ROS_INFO("  ...connected");

    /* Subscribe to joint_states */
    state_sub_ = nh.subscribe<sensor_msgs::JointState>("joint_states", 1,
                     boost::bind(&WalkController::jointStateCallback, this, _1));

    /* Subscribe to motion command */
    cmd_vel_sub_ = nh.subscribe<geometry_msgs::Twist>("cmd_vel", 1,
                      boost::bind(&WalkController::cmdVelCallback, this, _1));


    /* Fake Odometry for now */
    current_state_.odom_transform.p.z(0.1);
    current_state_.leg_contact_likelihood[0] = 1.0;
    current_state_.leg_contact_likelihood[1] = 1.0;
    current_state_.leg_contact_likelihood[2] = 1.0;
    current_state_.leg_contact_likelihood[3] = 1.0;

    /* Test Kinematics */
    KDL::Vector lf(0.12, 0.07, -0.05);
    current_state_.leg_contact_likelihood[0] = 0;
    KDL::Vector rr(-0.09, -0.05, -0.1);
    KDL::Vector rf(0.1, -0.05, -0.1);
    KDL::Vector lr(-0.09, 0.07, -0.1);

    RobotState state;

    if (solver_.solveIK(lf, rr, rf, lr, 0.0, -0.1, 0.0, state))
    {
      current_state_.body_transform.M = state.body_transform.M;

      control_msgs::FollowJointTrajectoryGoal goal;
      goal.trajectory.joint_names = state.joint_names;
      goal.trajectory.points.resize(1);
      goal.trajectory.points[0].positions = state.joint_positions;
      goal.trajectory.points[0].time_from_start = ros::Duration(1.0);
      goal.trajectory.header.stamp = ros::Time::now();
      client_.sendGoal(goal);
      client_.waitForResult(ros::Duration(5.0));
    }

    std::cout << "DONE" << std::endl;
  }
  // add ability to select planner?

  bool update(ros::Time time)
  {
    // 1. update planner goals
    // 2. plan next step
    // 3. execute action
  }

private:
  /** \brief Callback for joint feedback */
  void jointStateCallback(const sensor_msgs::JointStateConstPtr& msg)
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

    /* Publish COG/Support information */
    cog_publisher_.publish(current_state_);
    support_publisher_.publish(current_state_, solver_);

    /* Publish odometry */
    tf::Transform t;
    tf::transformKDLToTF(current_state_.odom_transform * current_state_.body_transform, t);
    broadcaster_.sendTransform(tf::StampedTransform(t, ros::Time::now(), "odom", "body_link"));
  }

  /** \brief Callback for command velocity from higher level planner or controller */
  void cmdVelCallback(const geometry_msgs::TwistConstPtr& msg)
  {
    // update goals to send to planner
  }

  client_t client_;
  CenterOfGravityPublisher cog_publisher_;
  SupportPolygonPublisher support_publisher_;

  ros::Subscriber cmd_vel_sub_;
  ros::Subscriber state_sub_;
  tf::TransformBroadcaster broadcaster_;

  RobotState current_state_;
  KinematicsSolver solver_;
};

}  // namespace smaldog

int main(int argc, char **argv)
{
  ros::init(argc, argv, "walk_controller");
  ros::NodeHandle nh;

  smaldog::WalkController* w = new smaldog::WalkController(nh);
  ros::spin();

  return 0;
}
