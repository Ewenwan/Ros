/**
Software License Agreement (BSD)

\file      WindowsSocket.cpp
\authors   Kareem Shehata <kshehata@clearpathrobotics.com>
\copyright Copyright (c) 2014, Clearpath Robotics, Inc., All rights reserved.

Redistribution and use in source and binary forms, with or without modification, are permitted provided that
the following conditions are met:
 * Redistributions of source code must retain the above copyright notice, this list of conditions and the
   following disclaimer.
 * Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the 
   following disclaimer in the documentation and/or other materials provided with the distribution.
 * Neither the name of Clearpath Robotics nor the names of its contributors may be used to endorse or promote
   products derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WAR-
RANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, IN-
DIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT
OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

#include "WindowsSocket.h"
#include <string>
#include <iostream>
#include <winsock2.h>
#include <ws2tcpip.h>

#pragma comment(lib, "Ws2_32.lib")

#define DEFAULT_PORT "11411"

using std::string;
using std::cerr;
using std::endl;


class WindowsSocketImpl
{

public:

  WindowsSocketImpl () : mySocket (INVALID_SOCKET)
  { }

  void init (char *server_hostname)
  {
    WSADATA wsaData;
    int result = WSAStartup (MAKEWORD (2, 2), &wsaData);
    if (result)
    {
      // TODO: do something more useful here with the error code
      std::cerr << "Could not initialize windows socket (" << result << ")" << std::endl;
      return;
    }

    struct addrinfo *servers = get_server_addr (server_hostname);

    if (NULL == servers)
    {
      WSACleanup ();
      return;
    }

    connect_to_server (servers);

    freeaddrinfo (servers);

    if (INVALID_SOCKET == mySocket)
    {
      std::cerr << "Could not connect to server" << std::endl;
      WSACleanup ();
    }
  }

  int read ()
  {
    char data;
    int result = recv (mySocket, &data, 1, 0);
    if (result < 0)
    {
      if (WSAEWOULDBLOCK != WSAGetLastError())
      {
        std::cerr << "Failed to receive data from server " << WSAGetLastError() << std::endl;
      }
      return -1;
    }
    else if (result == 0)
    {
      std::cerr << "Connection to server closed" << std::endl;
      return -1;
    }
    return (unsigned char) data;
  }

  void write (const unsigned char *data, int length)
  {
    int result = send (mySocket, (const char *) data, length, 0);
    if (SOCKET_ERROR == result)
    {
      std::cerr << "Send failed with error " << WSAGetLastError () << std::endl;
      closesocket (mySocket);
      WSACleanup ();
    }
  }

  unsigned long time ()
  {
    SYSTEMTIME st_now;
    GetSystemTime (&st_now);
    unsigned long millis = st_now.wHour * 3600000 +
                           st_now.wMinute * 60000 + 
                           st_now.wSecond * 1000 + 
                           st_now.wMilliseconds;
    return millis;
  }

protected:
        /**
	* Helper to get the addrinfo for the server based on a string hostname.
	* NB: you can just pass in a const char* and C++ will automatically
	* create the string instance for you.
	* @param hostname the hostname to connect to. Understands "host:port"
	* @returns pointer to addrinfo from getaddrinfo or NULL on error
	*/
  struct addrinfo *get_server_addr (const string & hostname)
  {
    int result;
    struct addrinfo *ai_output = NULL;
    struct addrinfo ai_input;

    // split off the port number if given
    int c = hostname.find_last_of (':');
    string host = hostname.substr (0, c);
    string port = (c < 0) ? DEFAULT_PORT : hostname.substr (c + 1);

    ZeroMemory (&ai_input, sizeof (ai_input));
    ai_input.ai_family = AF_UNSPEC;
    ai_input.ai_socktype = SOCK_STREAM;
    ai_input.ai_protocol = IPPROTO_TCP;

    // Resolve the server address and port
    result = getaddrinfo (host.c_str (), port.c_str (), &ai_input, &ai_output);
    if (result != 0)
    {
      std::cerr << "Could not resolve server address (" << result << ")" << std::endl;
      return NULL;
    }
    return ai_output;
  }

        /**
	* Helper to connect the socket to a given address.
	* @param server address of the server to connect to, linked list
	*/
  void connect_to_server (struct addrinfo *servers)
  {
    int result;
    for (struct addrinfo * ptr = servers; ptr != NULL; ptr = ptr->ai_next)
    {
      mySocket = socket (ptr->ai_family, ptr->ai_socktype, ptr->ai_protocol);
      if (INVALID_SOCKET == mySocket)
      {
        std::cerr << "Could not great socket " << WSAGetLastError ();
        return;
      }

      result = connect (mySocket, ptr->ai_addr, (int) ptr->ai_addrlen);
      if (SOCKET_ERROR == result)
      {
        closesocket (mySocket);
        mySocket = INVALID_SOCKET;
      }
      else
      {
        break;
      }
    }

    // disable nagle's algorithm
    char value = 1;
    setsockopt (mySocket, IPPROTO_TCP, TCP_NODELAY, &value, sizeof (value));
    // disable blocking
    u_long iMode = 1;
    result = ioctlsocket (mySocket, FIONBIO, &iMode);
    if (result)
    {
      std::cerr << "Could not make socket nonblocking " << result << std::endl;
      closesocket (mySocket);
      mySocket = INVALID_SOCKET;
    }
  }

private:
  SOCKET mySocket;
};

WindowsSocket::WindowsSocket ()
{
  impl = new WindowsSocketImpl ();
}

void WindowsSocket::init (char *server_hostname)
{
  impl->init (server_hostname);
}

int WindowsSocket::read ()
{
  return impl->read ();
}

void WindowsSocket::write (const unsigned char *data, int length)
{
  impl->write (data, length);
}

unsigned long WindowsSocket::time ()
{
  return impl->time ();
}
