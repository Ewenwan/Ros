/*
 * Software License Agreement (BSD License)
 *
 * Copyright (c) 2011, Southwest Research Institute
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 	* Redistributions of source code must retain the above copyright
 * 	notice, this list of conditions and the following disclaimer.
 * 	* Redistributions in binary form must reproduce the above copyright
 * 	notice, this list of conditions and the following disclaimer in the
 * 	documentation and/or other materials provided with the distribution.
 * 	* Neither the name of the Southwest Research Institute, nor the names
 *	of its contributors may be used to endorse or promote products derived
 *	from this software without specific prior written permission.
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

#ifndef TCP_SOCKET_H
#define TCP_SOCKET_H

#ifndef FLATHEADERS
#include "simple_message/socket/simple_socket.h"
#include "simple_message/shared_types.h"
#else
#include "simple_socket.h"
#include "shared_types.h"
#endif

#ifdef LINUXSOCKETS
#include "sys/socket.h"
#include "netdb.h"
#include "arpa/inet.h"
#include "string.h"
#include "unistd.h"
#endif

#ifdef MOTOPLUS
#include "motoPlus.h"
#endif


namespace industrial
{
namespace tcp_socket
{

class TcpSocket : public industrial::simple_socket::SimpleSocket
{
public:

  TcpSocket();
  virtual ~TcpSocket();

private:

  // Virtual
  int rawSendBytes(char *buffer,
      industrial::shared_types::shared_int num_bytes);
  int rawReceiveBytes(char *buffer,
      industrial::shared_types::shared_int num_bytes);
  bool rawPoll(int timeout, bool & ready, bool & error);

};

} //tcp_socket
} //industrial

#endif /* TCP_SOCKET_H */
