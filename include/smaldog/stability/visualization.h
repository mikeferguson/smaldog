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

#ifndef SMALDOG_STABILITY_VISUALIZATION_H
#define SMALDOG_STABILITY_VISUALIZATION_H

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <geometry_msgs/msg/polygon_stamped.hpp>

#include <smaldog/robot_state.h>
#include <smaldog/kinematics/kinematics_solver.h>

namespace smaldog
{

/**
 *  \brief Wraps a ros::Publisher which sends a geometry_msgs::msg::PointStamped for visualization.
 */
class CenterOfGravityPublisher
{
public:
  CenterOfGravityPublisher(rclcpp::Node::SharedPtr node)
  {
    point_.header.frame_id = "odom";
    point_.point.z = 0.0;
    publisher_ = node->create_publisher<geometry_msgs::msg::PointStamped>("center_of_gravity", 10);
  }
  ~CenterOfGravityPublisher() {}

  void publish(const RobotState& state, rclcpp::Time& now)
  {
    /* Center of gravity is simply the projection of the body_link frame */
    if (fabs(state.odom_transform.p.x()-point_.point.x) > 0.001 ||
        fabs(state.odom_transform.p.y()-point_.point.y) > 0.001)
    {
      point_.header.stamp = now;
      point_.point.x = state.odom_transform.p.x();
      point_.point.y = state.odom_transform.p.y();
      publisher_->publish(point_);
    }
  }

private:
  rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr publisher_;
  geometry_msgs::msg::PointStamped point_;
};

/**
 *  \brief Wraps a ros::Publisher which sends a geometry_msgs::PolygonStamped representing
 *         the support polygon of the robot state.
 */
class SupportPolygonPublisher
{
public:
  SupportPolygonPublisher(rclcpp::Node::SharedPtr node)
  {
    publisher_ = node->create_publisher<geometry_msgs::msg::PolygonStamped>("support_polygon", 10);
  }
  ~SupportPolygonPublisher() {}
  
  bool publish(const RobotState& state, const KinematicsSolver& solver, rclcpp::Time& now)
  {
    /* Do FK */
    KDL::Vector pose[4];
    if(!solver.solveFK(state, pose[0], pose[1], pose[2], pose[3]))
    {
      //ROS_WARN("Unable to solve FK");
      return false;
    }

    geometry_msgs::msg::PolygonStamped ps;
    ps.header.frame_id = "body_link";
    ps.header.stamp = now;

    /* Need to reorder these for a proper polygon */
    size_t order[] = {0, 2, 1, 3};
    for (size_t i = 0; i < 4; ++i)
    {
      if (state.leg_contact_likelihood[order[i]] > 0.5)
      {
        geometry_msgs::msg::Point32 point;
        point.x = pose[order[i]].x();
        point.y = pose[order[i]].y();
        point.z = pose[order[i]].z();
        ps.polygon.points.push_back(point);
      }
    }

    if (ps.polygon.points.size() < 3)
    {
      //ROS_WARN("Not enough points for support polygon");
      return false;
    }

    publisher_->publish(ps);
    return true;
  }

private:
  rclcpp::Publisher<geometry_msgs::msg::PolygonStamped>::SharedPtr publisher_;
};

}  // namespace smaldog

#endif  // SMALDOG_STABILITY_VISUALIZATION_H
