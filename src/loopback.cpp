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

#include <iostream>
#include <string>
#include <boost/array.hpp>
#include <boost/asio.hpp>

using boost::asio::ip::udp;

struct RobotData
{
  uint16_t servo_positions[14];
  uint16_t imu_data[6];
  uint16_t current_measurements[6];
  uint16_t voltage;
  uint8_t contact_sensors[4];
  uint8_t run_stop_status;
};

int main(int argc, char **argv)
{
  RobotData data;
  
  // TODO: fill in IMU

  /* Current measurments, 100mA/div.
     Set 3A total, 1A computer, 500mA/leg */
  data.current_measurements[0] = 30;
  data.current_measurements[1] = 10;
  data.current_measurements[2] = 5;
  data.current_measurements[3] = 5;
  data.current_measurements[4] = 5;
  data.current_measurements[5] = 5;

  /* Voltage measurement, 100mV/div.
     Set to 12.1V */
  data.voltage = 121;

  try
  {
    boost::asio::io_service io_service;

    /* Open a upd socket on 6707 */
    udp::socket socket(io_service, udp::endpoint(udp::v4(), 6707));
    std::cout << "Loopback started" << std::endl;

    /* Stow endpoint for multiple returns */
    udp::endpoint remote_endpoint;
      
    while(1)
    {
      /* Get command message */
      boost::array<char, 255> recv_buf;
      boost::system::error_code error;
      socket.receive_from(boost::asio::buffer(recv_buf),
          remote_endpoint, 0, error);

      if (error && error != boost::asio::error::message_size)
        throw boost::system::system_error(error);

      /* Confirm it is good */
      if ((recv_buf[0] != 'S') || (recv_buf[1] != 'M') ||
          (recv_buf[2] != 'A') || (recv_buf[3] != 'L') ||
          (recv_buf[4] != 0x01))
      {
        std::cerr << "Invalid packet." << std::endl;
        continue;
      }
      
      std::cout << "Packet recieved." << std::endl;
      
      /* Copy servo positions */
      std::memcpy(&data, &recv_buf[5], 28);

      /* Send return message */
      boost::array<char, sizeof(RobotData) + 5> send_buf;
      send_buf[0] = 'S';
      send_buf[1] = 'M';
      send_buf[2] = 'A';
      send_buf[3] = 'L';
      send_buf[4] = 0xff;
      std::memcpy(&send_buf[5], &data, sizeof(RobotData));

      socket.send_to(boost::asio::buffer(send_buf), remote_endpoint);
    }
  }
  catch (std::exception& e)
  {
    std::cerr << e.what() << std::endl;
  }

  return 0;
}
