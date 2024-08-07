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

#include <boost/bind.hpp>
#include "smaldog/drivers/driver.h"

namespace smaldog
{

udp_driver::udp_driver(const rclcpp::NodeOptions & options)
 : rclcpp::Node("smaldog", options),
   milliseconds_(50),
   timer_(io_service_, boost::posix_time::milliseconds(milliseconds_)),
   socket_(io_service_)
{
  /* TODO: Setup IMU publisher */

  /* Setup joint state publisher */
  joint_state_pub_ = this->create_publisher<sensor_msgs::msg::JointState>("joint_states", 10);

  /* Setup robot state publisher */
  robot_state_pub_ = this->create_publisher<smaldog::msg::State>("robot_state", 10);

  joint_msg_.name.resize(12);  // 12 servos
  joint_msg_.position.resize(12);
  joint_offsets_.resize(12);
  joint_scales_.resize(12);

  /*
   * From CAD:
   *  - Shoulder flex is 20.400209 degrees off the 0 when the line between shoulder
   *    flex and knee flex is upright. 20.400209/300 * 1024 = 69.63
   *  - Knee flex is 30.177936 degrees off the 0 when the line between the knee
   *    flex and the foot is upright. 30.177936/300 * 1024 = 204.19
   */
  joint_msg_.name[0] = "rf_pitch_joint";  // id 1
  joint_offsets_[0] = 512;
  joint_scales_[0] = 0.00511326929;

  joint_msg_.name[1] = "lf_pitch_joint";
  joint_offsets_[1] = 512;
  joint_scales_[1] = 0.00511326929;

  joint_msg_.name[2] = "rf_flex_joint";
  joint_offsets_[2] = 442;
  joint_scales_[2] = -0.00511326929;

  joint_msg_.name[3] = "lf_flex_joint";
  joint_offsets_[3] = 582;
  joint_scales_[3] = 0.00511326929;

  joint_msg_.name[4] = "rf_knee_joint";
  joint_offsets_[4] = 308;
  joint_scales_[4] = -0.00511326929;

  joint_msg_.name[5] = "lf_knee_joint";
  joint_offsets_[5] = 716;
  joint_scales_[5] = 0.00511326929;

  joint_msg_.name[6] = "rr_pitch_joint";
  joint_offsets_[6] = 512;
  joint_scales_[6] = -0.00511326929;

  joint_msg_.name[7] = "lr_pitch_joint";
  joint_offsets_[7] = 512;
  joint_scales_[7] = -0.00511326929;

  joint_msg_.name[8] = "rr_flex_joint";
  joint_offsets_[8] = 582;
  joint_scales_[8] = -0.00511326929;

  joint_msg_.name[9] = "lr_flex_joint";
  joint_offsets_[9] = 442;
  joint_scales_[9] = 0.00511326929;

  joint_msg_.name[10] = "rr_knee_joint";
  joint_offsets_[10] = 716;
  joint_scales_[10] = -0.00511326929;

  joint_msg_.name[11] = "lr_knee_joint";  // id 12
  joint_offsets_[11] = 308;
  joint_scales_[11] = 0.00511326929;

  /* Setup whole body trajectory + sampler */
  body_sampler_.reset(new TrajectorySampler("body_controller", joint_msg_.name));

  /* Get ip address parameter */
  ip_address_ = this->declare_parameter<std::string>("ip_address", "192.168.0.42");
  RCLCPP_INFO(this->get_logger(), "IP Address: %s", ip_address_.c_str());

  /* Any socket will do for sending/recieving data */
  socket_.open(udp::v4());
  start_async_receive();

  /* Everything is setup, start internal timer loop */
  timer_.async_wait(std::bind(&udp_driver::updateCallback, this, std::placeholders::_1));
}

void udp_driver::updateCallback(const boost::system::error_code& /*e*/)
{
  /* ROS catches the ctrl-c, but we need to stop the io_service to exit */
  if(!rclcpp::ok())
  {
    io_service_.stop();
    return;
  }

  /* Setup whole body trajectory + sampler */
  if (!body_sampler_->initialized())
  {
    /* Has to be done here, because shared_from_this() does not work in constructor */
    RCLCPP_INFO(this->get_logger(), "Initializing body sampler");
    body_sampler_->init(shared_from_this());
  }

  /* Send updated state */
  udp::endpoint receiver_endpoint = udp::endpoint(boost::asio::ip::address::from_string(ip_address_), 6707);
  try
  {
    /* Get desired positions */
    std::vector<double> positions = joint_msg_.position;
    bool result = body_sampler_->sample(this->now(), positions);

    /* Setup message */
    boost::array<uint8_t, 4 + 4 + 26> send_buf;
    send_buf[0] = 'S';
    send_buf[1] = 'M';
    send_buf[2] = 'A';
    send_buf[3] = 'L';
    send_buf[4] = 0xff;
    send_buf[5] = 0xff;
    send_buf[6] = 253;  // onboard device
    send_buf[7] = 26;
    send_buf[8] = 133;  // FULL_SYNC

    /* Convert positions to servo positions */
    for (size_t s = 0; s < positions.size(); ++s)
    {
      uint16_t val = -1;
      if (result)
        val = (positions[s] / joint_scales_[s]) + joint_offsets_[s];
      send_buf[9 + (2*s)] = val & 0xff;
      send_buf[10 + (2*s)] = (val >> 8) & 0xff;
    }

    /* Send the buffer */
    socket_.send_to(boost::asio::buffer(send_buf), receiver_endpoint);
  } 
  catch (std::exception& e)
  {
    std::cerr << e.what() << std::endl;
  }

  /* need to set a new expiration time before calling async_wait again */
  timer_.expires_at(timer_.expires_at() + boost::posix_time::milliseconds(milliseconds_));
  timer_.async_wait(std::bind(&udp_driver::updateCallback, this, std::placeholders::_1));
}

void udp_driver::start_async_receive()
{
  socket_.async_receive_from(
      boost::asio::buffer(recv_buffer_), remote_endpoint_,
      boost::bind(&udp_driver::handle_receive, this,
        boost::asio::placeholders::error,
        boost::asio::placeholders::bytes_transferred));
}

void udp_driver::handle_receive(const boost::system::error_code& error, std::size_t bytes_transferred)
{
  if (!error || error == boost::asio::error::message_size)
  {
    /* Check header */
    if ((bytes_transferred == 84) &&
        (recv_buffer_[0] == 'S') && (recv_buffer_[1] == 'M') &&
        (recv_buffer_[2] == 'A') && (recv_buffer_[3] == 'L') &&
        /* We only use full_sync in driver, so this is MODEL_L/H */
        (recv_buffer_[4] ==  46) && (recv_buffer_[5] == 1))
    {
      // TODO: extract system time?

      /* Convert current measurements */
      state_msg_.current_total = (recv_buffer_[4+16] + (recv_buffer_[4+17]<<8)) * 0.1;
      state_msg_.current_computer = (recv_buffer_[4+18] + (recv_buffer_[4+19]<<8)) * 0.1;
      state_msg_.current_left_front_leg = (recv_buffer_[4+20] + (recv_buffer_[4+21]<<8)) * 0.1;
      state_msg_.current_right_front_leg = (recv_buffer_[4+22] + (recv_buffer_[4+23]<<8)) * 0.1;
      state_msg_.current_left_rear_leg = (recv_buffer_[4+24] + (recv_buffer_[4+25]<<8)) * 0.1;
      state_msg_.current_right_rear_leg = (recv_buffer_[4+26] + (recv_buffer_[4+27]<<8)) * 0.1;

      /* Convert voltage measurements */
      state_msg_.battery_voltage = recv_buffer_[4+28] * 0.1;

      /* Convert IMU data */
      // TODO

      /* Convert foot sensors */
      state_msg_.status_left_front_foot = recv_buffer_[4+44];
      state_msg_.status_right_front_foot = recv_buffer_[4+45];
      state_msg_.status_left_rear_foot = recv_buffer_[4+46];
      state_msg_.status_right_rear_foot = recv_buffer_[4+47];

      /* Convert servo positions to joint_state messages */
      for (size_t s = 0; s < joint_msg_.position.size(); ++s)
      {
        int16_t val = (int16_t)recv_buffer_[4+48+(2*s)] + ((int16_t)recv_buffer_[5+48+(2*s)] << 8);
        if (val >= 0)
          joint_msg_.position[s] = (val - joint_offsets_[s]) * joint_scales_[s];
        else
          RCLCPP_DEBUG(this->get_logger(), "Read error on servo %i", static_cast<int>(s));
      }

      /* Convert run stop */
      state_msg_.run_stopped = recv_buffer_[4+76];
    }
    else
    {
      RCLCPP_ERROR(this->get_logger(), "Invalid packet recieved");
    }

    if (rclcpp::ok())
    {
      start_async_receive();
    }
  }

  /* Publish new joint_states */
  joint_msg_.header.stamp = this->now();
  joint_state_pub_->publish(joint_msg_);

  /* Publish new robot_state */
  robot_state_pub_->publish(state_msg_);

  /* ROS catches the ctrl-c, but we need to stop the io_service to exit */
  if (!rclcpp::ok())
  {
    io_service_.stop();
  }
}

}  // namespace smaldog


int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);

  try
  {
    /* io_service and driver class do all the heavy lifting */
    rclcpp::NodeOptions options;
    std::shared_ptr<smaldog::udp_driver> driver;
    driver.reset(new smaldog::udp_driver(options));

    /* Use this for RCLCPP_INFO, pub, subs... */
    using rclcpp::executors::SingleThreadedExecutor;
    SingleThreadedExecutor executor;
    executor.add_node(driver);
    std::thread executor_thread(std::bind(&SingleThreadedExecutor::spin, &executor));

    /* Will block here until the end */
    driver->getIoService().run();
  }
  catch (std::exception& e)
  {
    std::cerr << e.what() << std::endl;
  }

  return 0;
}
