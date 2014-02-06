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

  void publish(const RobotState& state);

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
  
  void publish(const RobotState& state);

private:
  ros::Publisher publisher_;
};

}  // namespace smaldog

#endif  // SMALDOG_STABILITY_VISUALIZATION_H_
