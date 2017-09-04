/*
 * Software License Agreement (BSD License)
 *
 * Copyright (c) 2011, Yaskawa America, Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *       * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *       * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *       * Neither the name of the Yaskawa America, Inc., nor the names
 *       of its contributors may be used to endorse or promote products derived
 *       from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#ifndef UDP_SOCKET_H
#define UDP_SOCKET_H

#ifndef FLATHEADERS
#include "simple_message/socket/simple_socket.h"
#include "simple_message/shared_types.h"
#include "simple_message/smpl_msg_connection.h"
#else
#include "simple_socket.h"
#include "shared_types.h"
#include "smpl_msg_connection.h"
#endif

#ifdef LINUXSOCKETS
#include "sys/socket.h"
#include "arpa/inet.h"
#include "string.h"
#include "unistd.h"
#endif

#ifdef MOTOPLUS
#include "motoPlus.h"
#endif

namespace industrial
{
namespace udp_socket
{

class UdpSocket : public industrial::simple_socket::SimpleSocket
{
public:

  UdpSocket();
  ~UdpSocket();

protected:

  /**
   * \brief udp socket connect handshake value
   */
  static const char CONNECT_HANDSHAKE = 142;

  char udp_read_buffer_[MAX_BUFFER_SIZE + 1];
  char* udp_read_head_;
  size_t udp_read_len_;

  // Virtual
  int rawSendBytes(char *buffer,
      industrial::shared_types::shared_int num_bytes);
  int rawReceiveBytes(char *buffer,
      industrial::shared_types::shared_int num_bytes);
  bool rawPoll(int timeout, bool & ready, bool & error);

};

} //udp_socket
} //industrial

#endif

