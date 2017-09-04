/*
 * Software License Agreement (BSD License)
 *
 * Copyright (c) 2012, Paul Bouchier
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 *  * Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *  * Redistributions in binary form must reproduce the above
 *    copyright notice, this list of conditions and the following
 *    disclaimer in the documentation and/or other materials provided
 *    with the distribution.
 *  * Neither the name of Willow Garage, Inc. nor the names of its
 *    contributors may be used to endorse or promote prducts derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#ifndef ROS_EMBEDDED_LINUX_COMMS_H
#define ROS_EMBEDDED_LINUX_COMMS_H

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <fcntl.h>
#include <errno.h>
#include <termios.h>
#include <string.h>
#include <time.h>
#include <stdint.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <netinet/tcp.h>
#include <netdb.h>
#include <assert.h>

#define DEFAULT_PORTNUM 11411

void error(const char *msg)
{
  perror(msg);
  exit(0);
}

void set_nonblock(int socket)
{
  int flags;
  flags = fcntl(socket, F_GETFL, 0);
  assert(flags != -1);
  fcntl(socket, F_SETFL, flags | O_NONBLOCK);
}

int elCommInit(const char *portName, int baud)
{
  struct termios options;
  int fd;
  int sockfd;
  struct sockaddr_in serv_addr;
  struct hostent *server;
  int rv;

  if (*portName == '/')       // linux serial port names always begin with /dev
  {
    printf("Opening serial port %s\n", portName);

    fd = open(portName, O_RDWR | O_NOCTTY | O_NDELAY);

    if (fd == -1)
    {
      // Could not open the port.
      perror("init(): Unable to open serial port - ");
    }
    else
    {
      // Sets the read() function to return NOW and not wait for data to enter
      // buffer if there isn't anything there.
      fcntl(fd, F_SETFL, FNDELAY);

      // Configure port for 8N1 transmission, 57600 baud, SW flow control.
      tcgetattr(fd, &options);
      cfsetispeed(&options, B57600);
      cfsetospeed(&options, B57600);
      options.c_cflag |= (CLOCAL | CREAD);
      options.c_cflag &= ~PARENB;
      options.c_cflag &= ~CSTOPB;
      options.c_cflag &= ~CSIZE;
      options.c_cflag |= CS8;
      options.c_cflag &= ~CRTSCTS;
      options.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);
      options.c_iflag &= ~(IXON | IXOFF | IXANY);
      options.c_oflag &= ~OPOST;

      // Set the new options for the port "NOW"
      tcsetattr(fd, TCSANOW, &options);
    }
    return fd;
  }
  else
  {
    // Split connection string into IP address and port.
    const char* tcpPortNumString = strchr(portName, ':');
    long int tcpPortNum;
    char ip[16];
    if (!tcpPortNumString)
    {
      tcpPortNum = DEFAULT_PORTNUM;
      strncpy(ip, portName, 16);
    }
    else
    {
      tcpPortNum = strtol(tcpPortNumString + 1, NULL, 10);
      strncpy(ip, portName, tcpPortNumString - portName);
    }

    printf("Connecting to TCP server at %s:%ld....\n", ip, tcpPortNum);

    // Create the socket.
    sockfd = socket(AF_INET, SOCK_STREAM, 0);
    if (sockfd < 0)
    {
      error("ERROR opening socket");
      exit(-1);
    }

    // Disable the Nagle (TCP No Delay) algorithm.
    int flag = 1;
    rv = setsockopt(sockfd, IPPROTO_TCP, TCP_NODELAY, (char *)&flag, sizeof(flag));
    if (rv == -1)
    {
      printf("Couldn't setsockopt(TCP_NODELAY)\n");
      exit(-1);
    }

    // Connect to the server
    server = gethostbyname(ip);
    if (server == NULL)
    {
      fprintf(stderr, "ERROR, no such host\n");
      exit(0);
    }
    bzero((char *) &serv_addr, sizeof(serv_addr));
    serv_addr.sin_family = AF_INET;
    bcopy((char *)server->h_addr,
          (char *)&serv_addr.sin_addr.s_addr,
          server->h_length);
    serv_addr.sin_port = htons(tcpPortNum);
    if (connect(sockfd, (struct sockaddr *) &serv_addr, sizeof(serv_addr)) < 0)
      error("ERROR connecting");
    set_nonblock(sockfd);
    printf("connected to server\n");
    return sockfd;
  }
  return -1;
}

int elCommRead(int fd)
{
  unsigned char c;
  unsigned int i;
  int rv;
  rv = read(fd, &c, 1); // read one byte
  i = c;          // convert byte to an int for return
  if (rv > 0)
    return i;     // return the character read
  if (rv < 0)
  {
    if ((errno != EAGAIN) && (errno != EWOULDBLOCK))
      perror("elCommRead() error:");
  }

  // return -1 or 0 either if we read nothing, or if read returned negative
  return rv;
}

int elCommWrite(int fd, uint8_t* data, int len)
{
  int rv;
  int length = len;
  int totalsent = 0;
  while (totalsent < length)
  {
    rv = write(fd, data + totalsent, length);
    if (rv < 0)
      perror("write(): error writing - trying again - ");
    else
      totalsent += rv;
  }
  return rv;
}

#endif
