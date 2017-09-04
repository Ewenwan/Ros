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

#ifndef FLATHEADERS
#include "simple_message/socket/udp_server.h"
#include "simple_message/log_wrapper.h"
#else
#include "udp_server.h"
#include "log_wrapper.h"
#endif

using namespace industrial::byte_array;

namespace industrial
{
namespace udp_server
{

UdpServer::UdpServer()
{
  this->setConnected(false);
}

UdpServer::~UdpServer()
{
}



bool UdpServer::init(int port_num)
{
  int rc = this->SOCKET_FAIL;
  bool rtn;
  SOCKLEN_T addrSize = 0;

  /* Create a socket using:
   * AF_INET - IPv4 internet protocol
   * SOCK_DGRAM - UDP type
   * protocol (0) - System chooses
   */
  rc = SOCKET(AF_INET, SOCK_DGRAM, 0);
  if (this->SOCKET_FAIL != rc)
  {
    this->setSockHandle(rc);
    LOG_DEBUG("Socket created, rc: %d", rc);
    LOG_DEBUG("Socket handle: %d", this->getSockHandle());

    // Initialize address data structure
    memset(&this->sockaddr_, 0, sizeof(this->sockaddr_));
    this->sockaddr_.sin_family = AF_INET;
    this->sockaddr_.sin_addr.s_addr = INADDR_ANY;
    this->sockaddr_.sin_port = HTONS(port_num);

    // This set the socket to be non-blocking (NOT SURE I WANT THIS) - sme
    //fcntl(sock_handle, F_SETFL, O_NONBLOCK);

    addrSize = sizeof(this->sockaddr_);
    rc = BIND(this->getSockHandle(), (sockaddr *)&(this->sockaddr_), addrSize);

    if (this->SOCKET_FAIL != rc)
    {
      rtn = true;
      LOG_INFO("Server socket successfully initialized");
    }
    else
    {
      LOG_ERROR("Failed to bind socket, rc: %d", rc);
      CLOSE(this->getSockHandle());
      rtn = false;
    }
  }
  else
  {
    LOG_ERROR("Failed to create socket, rc: %d", rc);
    rtn = false;
  }
  return rtn;
}


bool UdpServer::makeConnect()
{
  ByteArray send;
  char sendHS = this->CONNECT_HANDSHAKE;
  char recvHS = 0;
  int bytesRcvd = 0;
  const int timeout = 1000;  // Time (ms) between handshake sends
  bool rtn = false;
  
  send.load((void*)&sendHS, sizeof(sendHS));
    
  if (!this->isConnected())
  {
    this->setConnected(false);
    
    // Listen for handshake.  Once received, break
    // listen loop.
    do
    {
      ByteArray recv;
      recvHS = 0;
      if (this->isReadyReceive(timeout))
      {
        bytesRcvd = this->rawReceiveBytes(this->buffer_, 0);
        
        if (bytesRcvd > 0)
        {
          LOG_DEBUG("UDP server received %d bytes while waiting for handshake", bytesRcvd);
          recv.init(&this->buffer_[0], bytesRcvd);
          recv.unload((void*)&recvHS, sizeof(recvHS));
        }
      }
      
    }
    while(recvHS != sendHS);
    
    // copy to local array, since ByteArray no longer supports
    // direct pointer-access to data values
    const int sendLen = send.getBufferSize();
    char      localBuffer[sendLen];
    send.unload(localBuffer, sendLen);

    // Send a reply handshake
    this->rawSendBytes(localBuffer, sendLen);
    this->setConnected(true);
    rtn = true;
    
  }
  else
  {
    LOG_WARN("Tried to connect when socket already in connected state");
    rtn = true;
  }

  return rtn;
}


} //udp_server
} //industrial

