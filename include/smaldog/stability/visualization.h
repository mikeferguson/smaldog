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

#ifndef SMALDOG_STABILITY_VISUALIZATION_H_
#define SMALDOG_STABILITY_VISUALIZATION_H_

#include <ros/ros.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/PolygonStamped.h>

#include <smaldog/robot_state.h>
#include <smaldog/kinematics/kinematics_solver.h>

namespace smaldog
{

/**
 *  \brief Wraps a ros::Publisher which sends a geometry_msgs::PointStamped for visualization.
 */
class CenterOfGravityPublisher
{
public:
  CenterOfGravityPublisher(ros::NodeHandle nh)
  {
    publisher_ = nh.advertise<geometry_msgs::PointStamped>("center_of_gravity", 10);
  }
  ~CenterOfGravityPublisher() {}

  void publish(const RobotState& state)
  {
    /* Center of gravity is simply the projection of the body_link frame */
    geometry_msgs::PointStamped p;
    p.header.frame_id = "odom";
    p.header.stamp = ros::Time::now();
    p.point.x = state.odom_transform.p.x();
    p.point.y = state.odom_transform.p.y();
    p.point.z = 0.0;
    publisher_.publish(p);
  }

private:
  ros::Publisher publisher_;
};

/**
 *  \brief Wraps a ros::Publisher which sends a geometry_msgs::PolygonStamped representing
 *         the support polygon of the robot state.
 */
class SupportPolygonPublisher
{
public:
  SupportPolygonPublisher(ros::NodeHandle nh)
  {
    publisher_ = nh.advertise<geometry_msgs::PolygonStamped>("support_polygon", 10);
  }
  ~SupportPolygonPublisher() {}
  
  bool publish(const RobotState& state, const KinematicsSolver& solver)
  {
    /* Do FK */
    KDL::Vector pose[4];
    if(!solver.solveFK(state, pose[0], pose[1], pose[2], pose[3]))
    {
      ROS_WARN("Unable to solve FK");
      return false;
    }

    geometry_msgs::PolygonStamped ps;
    ps.header.frame_id = "body_link";
    ps.header.stamp = ros::Time::now();

    /* Need to reorder these for a proper polygon */
    size_t order[] = {0, 2, 1, 3};
    for (size_t i = 0; i < 4; ++i)
    {
      if (state.leg_contact_likelihood[order[i]] > 0.5)
      {
        geometry_msgs::Point32 point;
        point.x = pose[order[i]].x();
        point.y = pose[order[i]].y();
        point.z = pose[order[i]].z();
        ps.polygon.points.push_back(point);
      }
    }

    if (ps.polygon.points.size() < 3)
    {
      ROS_WARN("Not enough points for support polygon");
      return false;
    }

    publisher_.publish(ps);
    return true;
  }

private:
  ros::Publisher publisher_;
};

}  // namespace smaldog

#endif  // SMALDOG_STABILITY_VISUALIZATION_H_
