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

#ifndef SMALDOG_DRIVER_H_
#define SMALDOG_DRIVER_H_

#include <iostream>
#include <string>
#include <boost/array.hpp>
#include <boost/asio.hpp>
#include <boost/date_time/posix_time/posix_time.hpp>

#include <ros/ros.h>
#include <sensor_msgs/JointState.h>

#include "smaldog/State.h"
#include "smaldog/trajectory_sampler.h"

using boost::asio::ip::udp;

namespace smaldog
{

class udp_driver
{
public:
  /** \brief Setup asio/ros interfaces */
  udp_driver(boost::asio::io_service& io_service, ros::NodeHandle nh);

  /** \brief This is the 100hz callback, send updates to the hardware. */
  void updateCallback(const boost::system::error_code& /*e*/);

private:
  /** \brief Start asynchronous recieve of data */
  void start_async_receive();

  /** \brief Callback for boost::asio async reception. */
  void handle_receive(const boost::system::error_code& error, std::size_t bytes_transferred);

  /* 100hz timer for sending updates to hardware */
  long milliseconds_;
  boost::asio::deadline_timer timer_;

  /* Boost::asio components */
  udp::socket socket_;
  udp::endpoint remote_endpoint_;
  boost::array<uint8_t, 255> recv_buffer_;
  std::string ip_address_;

  /* ROS publishers of state */
  ros::Publisher imu_pub_;
  ros::Publisher joint_state_pub_;
  ros::Publisher robot_state_pub_;

  /* ROS messages to be published */
  sensor_msgs::JointState joint_msg_;
  smaldog::State state_msg_;

  /*
   * Specifications for converting from robot servo values to ROS values
   *   radian_value = (servo_value - joint_offset_) * joint_scale_
   */
  std::vector<int> joint_offsets_;
  std::vector<double> joint_scales_;

  /* ROS components for sampling position of whole body. */
  boost::shared_ptr<TrajectorySampler> body_sampler_;

  // TODO: add laser tilt stage trajectory action
};

}  // namespace smaldog

#endif  // SMALDOG_DRIVER_H_
