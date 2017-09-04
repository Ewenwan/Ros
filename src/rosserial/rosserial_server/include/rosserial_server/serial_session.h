/**
 *
 *  \file
 *  \brief      Single, reconnecting class for a serial rosserial session.
 *  \author     Mike Purvis <mpurvis@clearpathrobotics.com>
 *  \copyright  Copyright (c) 2013, Clearpath Robotics, Inc.
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

#ifndef ROSSERIAL_SERVER_SERIAL_SESSION_H
#define ROSSERIAL_SERVER_SERIAL_SESSION_H

#include <iostream>
#include <boost/bind.hpp>
#include <boost/asio.hpp>

#include <ros/ros.h>

#include "rosserial_server/session.h"

namespace rosserial_server
{

class SerialSession : public Session<boost::asio::serial_port>
{
public:
  SerialSession(boost::asio::io_service& io_service, std::string port, int baud)
    : Session(io_service), port_(port), baud_(baud), timer_(io_service)
  {
    ROS_INFO_STREAM("rosserial_server session configured for " << port_ << " at " << baud << "bps.");

    failed_connection_attempts_ = 0;
    check_connection();
  }

private:
  void check_connection()
  {
    if (!is_active())
    {
      attempt_connection();
    }

    // Every second, check again if the connection should be reinitialized,
    // if the ROS node is still up.
    if (ros::ok())
    {
      timer_.expires_from_now(boost::posix_time::milliseconds(2000));
      timer_.async_wait(boost::bind(&SerialSession::check_connection, this));
    }
  }

  void attempt_connection()
  {
    ROS_DEBUG("Opening serial port.");

    boost::system::error_code ec;
    socket().open(port_, ec);
    if (ec) {
      failed_connection_attempts_++;
      if (failed_connection_attempts_ == 1) {
        ROS_ERROR_STREAM("Unable to open port " << port_ << ": " << ec);
      } else {
        ROS_DEBUG_STREAM("Unable to open port " << port_ << " (" << failed_connection_attempts_ << "): " << ec);
      }
      return;
    }
    ROS_INFO_STREAM("Opened " << port_);
    failed_connection_attempts_ = 0;

    typedef boost::asio::serial_port_base serial;
    socket().set_option(serial::baud_rate(baud_));
    socket().set_option(serial::character_size(8));
    socket().set_option(serial::stop_bits(serial::stop_bits::one));
    socket().set_option(serial::parity(serial::parity::none));
    socket().set_option(serial::flow_control(serial::flow_control::none));

    // Kick off the session.
    start();
  }

  std::string port_;
  int baud_;
  boost::asio::deadline_timer timer_;
  int failed_connection_attempts_;
};

}  // namespace

#endif  // ROSSERIAL_SERVER_SERIAL_SESSION_H
