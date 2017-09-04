/**
 *
 *  \file
 *  \brief      Reconnecting class for a UDP rosserial session.
 *  \author     Mike Purvis <mpurvis@clearpathrobotics.com>
 *  \copyright  Copyright (c) 2016, Clearpath Robotics, Inc.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of Clearpath Robotics, Inc. nor the
 *       names of its contributors may be used to endorse or promote products
 *       derived from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL CLEARPATH ROBOTICS, INC. BE LIABLE FOR ANY
 * DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 * ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 * Please send comments, questions, or patches to code@clearpathrobotics.com
 *
 */

#ifndef ROSSERIAL_SERVER_UDP_SOCKET_SESSION_H
#define ROSSERIAL_SERVER_UDP_SOCKET_SESSION_H

#include <iostream>
#include <boost/bind.hpp>
#include <boost/asio.hpp>

#include <ros/ros.h>

#include "rosserial_server/session.h"
#include "rosserial_server/udp_stream.h"


namespace rosserial_server
{

using boost::asio::ip::udp;

class UdpSocketSession : public Session<UdpStream>
{
public:
  UdpSocketSession(boost::asio::io_service& io_service,
                   udp::endpoint server_endpoint,
                   udp::endpoint client_endpoint)
    : Session(io_service), timer_(io_service),
      server_endpoint_(server_endpoint), client_endpoint_(client_endpoint)
  {
    ROS_INFO_STREAM("rosserial_server UDP session created between " << server_endpoint << " and " << client_endpoint);
    check_connection();
  }

private:
  void check_connection()
  {
    if (!is_active())
    {
      socket().open(server_endpoint_, client_endpoint_);
      start();
    }

    // Every second, check again if the connection should be reinitialized,
    // if the ROS node is still up.
    if (ros::ok())
    {
      timer_.expires_from_now(boost::posix_time::milliseconds(2000));
      timer_.async_wait(boost::bind(&UdpSocketSession::check_connection, this));
    }
  }

  boost::asio::deadline_timer timer_;
  udp::endpoint server_endpoint_;
  udp::endpoint client_endpoint_;
};

}  // namespace

#endif  // ROSSERIAL_SERVER_UDP_SOCKET_SESSION_H
