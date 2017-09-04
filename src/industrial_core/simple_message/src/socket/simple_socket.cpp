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
#include "simple_message/socket/simple_socket.h"
#include "simple_message/log_wrapper.h"
#else
#include "simple_socket.h"
#include "log_wrapper.h"
#endif

using namespace industrial::byte_array;
using namespace industrial::shared_types;

namespace industrial
{
  namespace simple_socket
  {

    bool SimpleSocket::sendBytes(ByteArray & buffer)
    {
      int rc = this->SOCKET_FAIL;
      bool rtn = false;

      if (this->isConnected())
      {
        // Nothing restricts the ByteArray from being larger than the what the socket
        // can handle.
        if (this->MAX_BUFFER_SIZE > (int)buffer.getBufferSize())
        {

          // copy to local array, since ByteArray no longer supports
          // direct pointer-access to data values
          std::vector<char> localBuffer;
          buffer.copyTo(localBuffer);
          rc = rawSendBytes(&localBuffer[0], localBuffer.size());
          if (this->SOCKET_FAIL != rc)
          {
            rtn = true;
          }
          else
          {
            rtn = false;
            logSocketError("Socket sendBytes failed", rc, errno);
          }

        }
        else
        {
          LOG_ERROR("Buffer size: %u, is greater than max socket size: %u", buffer.getBufferSize(), this->MAX_BUFFER_SIZE);
          rtn = false;
        }

      }
      else
      {
        rtn = false;
        LOG_WARN("Not connected, bytes not sent");
      }

      if (!rtn)
      {
        this->setConnected(false);
      }

      return rtn;

    }

    bool SimpleSocket::receiveBytes(ByteArray & buffer, shared_int num_bytes)
    {
      int rc = this->SOCKET_FAIL;
      bool rtn = false;
      shared_int remainBytes = num_bytes;
      bool ready, error;

      // Reset the buffer (this is not required since the buffer length should
      // ensure that we don't read any of the garbage that may be left over from
      // a previous read), but it is good practice.

      memset(&this->buffer_, 0, sizeof(this->buffer_));

      // Doing a sanity check to determine if the byte array buffer is smaller than
      // what can be received by the socket.
      if (this->MAX_BUFFER_SIZE > buffer.getMaxBufferSize())
      {
        LOG_WARN("Socket buffer max size: %u, is larger than byte array buffer: %u",
            this->MAX_BUFFER_SIZE, buffer.getMaxBufferSize());
      }
      if (this->isConnected())
      {
        buffer.init();
        while (remainBytes > 0)
        {
          // Polling the socket results in an "interruptable" socket read.  This
          // allows Control-C to break out of a socket read.  Without polling,
          // a sig-term is required to kill a program in a socket read function.
          if (this->rawPoll(this->SOCKET_POLL_TO, ready, error))
          {
            if(ready)
            {
              rc = rawReceiveBytes(this->buffer_, remainBytes);
              if (this->SOCKET_FAIL == rc)
              {
                this->logSocketError("Socket received failed", rc, errno);
		        remainBytes = 0;
                rtn = false;
                break;
              }
              else if (0 == rc)
              {
                LOG_WARN("Recieved zero bytes: %u", rc);
		        remainBytes = 0;
                rtn = false;
                break;
              }
              else
              {
                remainBytes = remainBytes - rc;
                LOG_COMM("Byte array receive, bytes read: %u, bytes reqd: %u, bytes left: %u",
                    rc, num_bytes, remainBytes);
                buffer.load(&this->buffer_, rc);
                rtn = true;
              }
            }
            else if(error)
            {
              LOG_ERROR("Socket poll returned an error");
              rtn = false;
              break;
            }
            else
            {
              LOG_ERROR("Uknown error from socket poll");
              rtn = false;
              break;
            }
          }
          else
          {
            LOG_COMM("Socket poll timeout, trying again");
          }
        }
      }
      else
      {
        LOG_WARN("Not connected, bytes not sent");
        rtn = false;
      }

      if (!rtn)
      {
        this->setConnected(false);
      }
      return rtn;
    }

  }  //simple_socket
}  //industrial

