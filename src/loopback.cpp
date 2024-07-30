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
  uint16_t model_number;
  uint8_t  version;
  uint8_t  id;
  uint8_t  baud_rate;
  uint8_t  return_delay;
  uint16_t REG_6;
  uint32_t last_packet;
  uint32_t system_time;

  int16_t current_measurements[6];
  uint8_t voltage;
  uint8_t led;
  int16_t REG_30;

  uint16_t imu_data[6];
  uint8_t contact_sensors[4];

  int16_t servo_positions[14];
  uint8_t run_stop_status;
  uint8_t  REG_77;
  uint16_t REG_78;
};

int main()
{
  RobotData data;
  data.model_number = 302;
  data.id = 253;
  
  /* Initialize servos */
  for (size_t i = 0; i < 14; ++i)
    data.servo_positions[i] = 512;

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
      boost::array<uint8_t, 255> recv_buf;
      boost::system::error_code error;
      socket.receive_from(boost::asio::buffer(recv_buf),
          remote_endpoint, 0, error);

      if (error && error != boost::asio::error::message_size)
        throw boost::system::system_error(error);

      /* Confirm it is good */
      if ((recv_buf[0] != 'S') || (recv_buf[1] != 'M') ||
          (recv_buf[2] != 'A') || (recv_buf[3] != 'L') ||
          (recv_buf[4] != 0xff) || (recv_buf[5] != 0xff) ||
          (recv_buf[6] != 253) || (recv_buf[8] != 133)) // FULL_SYNC
      {
        std::cerr << "Invalid packet." << std::endl;
        continue;
      }
      
      std::cout << "Packet recieved." << std::endl;

      /* Copy servo positions */
      for (size_t i = 0; i < 14; ++i)
      {
        int16_t p = (int16_t)recv_buf[9+(2*i)] + ((int16_t)recv_buf[10+(2*i)] << 8);
        if (p != -1)
          data.servo_positions[i] = p;
      }

      /* Send return message */
      boost::array<uint8_t, sizeof(RobotData) + 4> send_buf;
      send_buf[0] = 'S';
      send_buf[1] = 'M';
      send_buf[2] = 'A';
      send_buf[3] = 'L';
      std::memcpy(&send_buf[4], &data, sizeof(RobotData));

      socket.send_to(boost::asio::buffer(send_buf), remote_endpoint);
    }
  }
  catch (std::exception& e)
  {
    std::cerr << e.what() << std::endl;
  }

  return 0;
}
