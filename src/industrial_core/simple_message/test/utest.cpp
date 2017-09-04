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

#include "simple_message/simple_message.h"
#include "simple_message/byte_array.h"
#include "simple_message/shared_types.h"
#include "simple_message/smpl_msg_connection.h"
#include "simple_message/socket/udp_client.h"
#include "simple_message/socket/udp_server.h"
#include "simple_message/socket/tcp_client.h"
#include "simple_message/socket/tcp_server.h"
#include "simple_message/ping_message.h"
#include "simple_message/ping_handler.h"
#include "simple_message/messages/joint_message.h"
#include "simple_message/joint_data.h"
#include "simple_message/message_manager.h"
#include "simple_message/simple_comms_fault_handler.h"
#include "simple_message/joint_traj_pt.h"
#include "simple_message/messages/joint_traj_pt_message.h"
#include "simple_message/typed_message.h"
#include "simple_message/joint_traj.h"

#include <gtest/gtest.h>
// Use pthread instead of boost::thread so we can cancel the TCP/UDP server
// threads 
//#include <boost/thread/thread.hpp>
#include <pthread.h>
#include <limits>

using namespace industrial::simple_message;
using namespace industrial::byte_array;
using namespace industrial::shared_types;
using namespace industrial::smpl_msg_connection;
using namespace industrial::udp_socket;
using namespace industrial::udp_client;
using namespace industrial::udp_server;
using namespace industrial::tcp_socket;
using namespace industrial::tcp_client;
using namespace industrial::tcp_server;
using namespace industrial::ping_message;
using namespace industrial::ping_handler;
using namespace industrial::joint_data;
using namespace industrial::joint_message;
using namespace industrial::message_manager;
using namespace industrial::simple_comms_fault_handler;
using namespace industrial::joint_traj_pt;
using namespace industrial::joint_traj_pt_message;
using namespace industrial::typed_message;
using namespace industrial::joint_traj;

// Multiple tests require TEST_PORT_BASE to be defined.  This is defined
// by the make file at compile time.
//#define TEST_PORT_BASE 11000

TEST(ByteArraySuite, init)
{

  const shared_int SIZE = 100;

  ByteArray bytes;
  char buffer[SIZE];

  // Valid byte arrays
  EXPECT_TRUE(bytes.init(&buffer[0], SIZE));
  EXPECT_EQ((shared_int)bytes.getBufferSize(), SIZE);

  // Invalid init (too big)
  if (bytes.getMaxBufferSize() < std::numeric_limits<shared_int>::max())
  {
      shared_int TOO_BIG = bytes.getMaxBufferSize()+1;
      char bigBuffer[TOO_BIG];
      EXPECT_FALSE(bytes.init(&bigBuffer[0], TOO_BIG));
  }
  else
      std::cout << std::string(15, ' ')
                << "ByteArray.MaxSize==INT_MAX.  Skipping TOO_BIG tests" << std::endl;

}

TEST(ByteArraySuite, loading)
{
  const shared_int SIZE = 100;
  char buffer[SIZE];

  ByteArray bytes;
  ByteArray empty;

  ASSERT_TRUE(bytes.init(&buffer[0], SIZE));

  shared_bool bIN = true, bOUT = false;
  shared_int iIN = 999, iOUT = 0;
  shared_real rIN = 9999.9999, rOUT = 0;

  // Boolean loading
  EXPECT_TRUE(bytes.load(bIN));
  EXPECT_EQ(bytes.getBufferSize(), SIZE+sizeof(shared_bool));
  EXPECT_TRUE(bytes.unload(bOUT));
  EXPECT_EQ((shared_int)bytes.getBufferSize(), SIZE);
  EXPECT_EQ(bOUT, bIN);

  // Integer loading
  EXPECT_TRUE(bytes.load(iIN));
  EXPECT_EQ(bytes.getBufferSize(), SIZE+sizeof(shared_int));
  EXPECT_TRUE(bytes.unload(iOUT));
  EXPECT_EQ((shared_int)bytes.getBufferSize(), SIZE);
  EXPECT_EQ(iOUT, iIN);

  // Real loading
  EXPECT_TRUE(bytes.load(rIN));
  EXPECT_EQ(bytes.getBufferSize(), SIZE+sizeof(shared_real));
  EXPECT_TRUE(bytes.unload(rOUT));
  EXPECT_EQ((shared_int)bytes.getBufferSize(), SIZE);
  EXPECT_EQ(rOUT, rIN);

  // Unloading a single member (down to an empty buffer size)
  EXPECT_TRUE(empty.load(bIN));
  EXPECT_EQ(empty.getBufferSize(), sizeof(shared_bool));
  EXPECT_TRUE(empty.unload(bOUT));
  EXPECT_EQ((int)empty.getBufferSize(), 0);
  EXPECT_EQ(bOUT, bIN);

  // Loading two members (unloading the first) and then checking the value of the second
  rOUT = 0.0;
  iOUT = 0;
  EXPECT_TRUE(empty.load(rIN));
  EXPECT_EQ(empty.getBufferSize(), sizeof(shared_real));
  EXPECT_TRUE(empty.load(iIN));
  EXPECT_EQ(empty.getBufferSize(), sizeof(shared_real)+sizeof(shared_int));
  EXPECT_TRUE(empty.unloadFront(rOUT));
  EXPECT_EQ(rOUT, rIN);
  EXPECT_TRUE(empty.unload(iOUT));
  EXPECT_EQ((int)empty.getBufferSize(), 0);
  EXPECT_EQ(iOUT, iIN);
}

TEST(ByteArraySuite, byteSwapping)
{
  if(ByteArray::isByteSwapEnabled())
  {
    ASSERT_TRUE(ByteArray::isByteSwapEnabled());

    ByteArray swapped;
    unsigned char buffer[] = {
        0x00, 0x00, 0x00, 0x38,   // be: 56
        0x00, 0x00, 0x00, 0x0a,   // be: 10
        0x00, 0x00, 0x00, 0x01,   // be:  1

        0x3e, 0x81, 0x32, 0x64,   // be:  0.25233757495880127
        0x3f, 0x30, 0x4b, 0x75,   // be:  0.68865138292312622
        0x3f, 0xa8, 0x9d, 0xd2,   // be:  1.3173162937164307
        0x3f, 0x85, 0x93, 0xdd,   // be:  1.0435749292373657
        0xbf, 0xf4, 0x8c, 0xc5,   // be: -1.9105459451675415

    };
    const unsigned int bufferLength = 32;
    shared_int tempInt;
    shared_real tempReal;

    swapped.init((const char*) buffer, bufferLength);
    ASSERT_EQ(swapped.getBufferSize(), bufferLength);

    ASSERT_TRUE(swapped.unload(tempReal));
    EXPECT_FLOAT_EQ(tempReal, -1.9105459451675415);

    ASSERT_TRUE(swapped.unload(tempReal));
    EXPECT_FLOAT_EQ(tempReal, 1.0435749292373657);

    ASSERT_TRUE(swapped.unload(tempReal));
    EXPECT_FLOAT_EQ(tempReal, 1.3173162937164307);

    ASSERT_TRUE(swapped.unload(tempReal));
    EXPECT_FLOAT_EQ(tempReal, 0.68865138292312622);

    ASSERT_TRUE(swapped.unload(tempReal));
    EXPECT_FLOAT_EQ(tempReal, 0.25233757495880127);

    ASSERT_TRUE(swapped.unload(tempInt));
    EXPECT_EQ(tempInt, 1);

    ASSERT_TRUE(swapped.unload(tempInt));
    EXPECT_EQ(tempInt, 10);

    ASSERT_TRUE(swapped.unload(tempInt));
    EXPECT_EQ(tempInt, 56);

    ASSERT_EQ(swapped.getBufferSize(), 0);
  }

}

TEST(ByteArraySuite, copy)
{

  const shared_int SIZE = 100;
  char buffer[SIZE];

  // Copy
  ByteArray copyFrom;
  ByteArray copyTo;

  EXPECT_TRUE(copyFrom.init(&buffer[0], SIZE));
  EXPECT_TRUE(copyTo.load(copyFrom));
  EXPECT_EQ((shared_int)copyTo.getBufferSize(), SIZE);
  EXPECT_TRUE(copyTo.load(copyFrom));
  EXPECT_EQ((shared_int)copyTo.getBufferSize(), 2*SIZE);

  // Copy too large
  ByteArray tooBig;
  if (tooBig.getMaxBufferSize()-1 <= std::numeric_limits<shared_int>::max())
  {
      shared_int TOO_BIG = tooBig.getMaxBufferSize()-1;
      char bigBuffer[TOO_BIG];

      EXPECT_TRUE(tooBig.init(&bigBuffer[0], TOO_BIG));
      EXPECT_FALSE(copyTo.load(tooBig));
      // A failed load should not change the buffer.
      EXPECT_EQ((shared_int)copyTo.getBufferSize(), 2*SIZE);
  }
  else
      std::cout << std::string(15, ' ')
                << "ByteArray.MaxSize==INT_MAX.  Skipping TOO_BIG tests" << std::endl;
}

// Need access to protected members for testing
#ifndef UDP_TEST
class TestClient : public TcpClient
{
  public:
  bool sendBytes(ByteArray & buffer)
  {
    return TcpClient::sendBytes(buffer);
  };
};
class TestServer : public TcpServer
{
  public:
  bool receiveBytes(ByteArray & buffer, shared_int num_bytes)
  {
    return TcpServer::receiveBytes(buffer, num_bytes);
  }
};
#else
class TestClient : public UdpClient
{
  public:
  bool sendBytes(ByteArray & buffer)
  {
    return UdpClient::sendBytes(buffer);
  };
};
class TestServer : public UdpServer
{
  public:
  bool receiveBytes(ByteArray & buffer, shared_int num_bytes)
  {
    return UdpServer::receiveBytes(buffer, num_bytes);
  }
};
#endif

void*
connectServerFunc(void* arg)
{
  TestServer* server = (TestServer*)arg;  
  server->makeConnect();
  return NULL;
}

TEST(SocketSuite, read)
{
  const int port = TEST_PORT_BASE;
  char ipAddr[] = "127.0.0.1";

  TestClient client;
  TestServer server;
  ByteArray send, recv;
  shared_int DATA = 99;
  shared_int TWO_INTS = 2 * sizeof(shared_int);
  shared_int ONE_INTS = 1 * sizeof(shared_int);

  // Construct server
  ASSERT_TRUE(server.init(port));

  // Construct a client
  ASSERT_TRUE(client.init(&ipAddr[0], port));
  pthread_t serverConnectThrd;
  pthread_create(&serverConnectThrd, NULL, connectServerFunc, &server);

  ASSERT_TRUE(client.makeConnect());
  pthread_join(serverConnectThrd, NULL);

  ASSERT_TRUE(send.load(DATA));

  // Send just right amount
  ASSERT_TRUE(client.sendBytes(send));
  ASSERT_TRUE(client.sendBytes(send));
  sleep(2);
  ASSERT_TRUE(server.receiveBytes(recv, TWO_INTS));
  ASSERT_EQ(TWO_INTS, recv.getBufferSize());

  // Send too many bytes
  ASSERT_TRUE(client.sendBytes(send));
  ASSERT_TRUE(client.sendBytes(send));
  ASSERT_TRUE(client.sendBytes(send));
  ASSERT_TRUE(server.receiveBytes(recv, TWO_INTS));
  ASSERT_EQ(TWO_INTS, recv.getBufferSize());
  ASSERT_TRUE(server.receiveBytes(recv, ONE_INTS));
  ASSERT_EQ(ONE_INTS, recv.getBufferSize());
}


// Utility for running tcp client in sending loop
void*
spinSender(void* arg)
{
  TestClient* client = (TestClient*)arg;  
  ByteArray send;
  const int DATA = 256;

  send.load(DATA);

  while(true)
  {
    client->sendBytes(send);
    sleep(0.1);
  }
}

TEST(SocketSuite, splitPackets)
{
  const int port = TEST_PORT_BASE + 1;
  char ipAddr[] = "127.0.0.1";
  const int RECV_LENGTH = 64;

  TestClient client;
  TestServer server;
  ByteArray recv;
// Construct server
  ASSERT_TRUE(server.init(port));

  // Construct a client
  ASSERT_TRUE(client.init(&ipAddr[0], port));
  pthread_t serverConnectThrd;
  pthread_create(&serverConnectThrd, NULL, connectServerFunc, &server);

  ASSERT_TRUE(client.makeConnect());
  pthread_join(serverConnectThrd, NULL);

  pthread_t senderThrd;
  pthread_create(&senderThrd, NULL, spinSender, &client);

  ASSERT_TRUE(server.receiveBytes(recv, RECV_LENGTH));
  ASSERT_EQ(RECV_LENGTH, recv.getBufferSize());

  pthread_cancel(senderThrd);
  pthread_join(senderThrd, NULL);
}


TEST(SimpleMessageSuite, init)
{
  SimpleMessage msg;
  ByteArray bytes;

  // Valid messages
  EXPECT_TRUE(msg.init(StandardMsgTypes::PING, CommTypes::TOPIC, ReplyTypes::INVALID, bytes));
  EXPECT_TRUE(msg.init(StandardMsgTypes::PING, CommTypes::SERVICE_REQUEST,ReplyTypes::INVALID, bytes));
  EXPECT_TRUE(msg.init(StandardMsgTypes::PING, CommTypes::SERVICE_REPLY,ReplyTypes::SUCCESS, bytes));
  EXPECT_TRUE(msg.init(StandardMsgTypes::PING, CommTypes::SERVICE_REPLY,ReplyTypes::FAILURE, bytes));

  // Unused command
  EXPECT_FALSE(msg.init(StandardMsgTypes::INVALID, CommTypes::INVALID,ReplyTypes::INVALID, bytes));

  // Service request with a reply
  EXPECT_FALSE(msg.init(StandardMsgTypes::PING, CommTypes::SERVICE_REQUEST,ReplyTypes::SUCCESS, bytes));
  EXPECT_FALSE(msg.init(StandardMsgTypes::PING, CommTypes::SERVICE_REQUEST,ReplyTypes::FAILURE, bytes));
}

TEST(PingMessageSuite, init)
{
  PingMessage ping;
  SimpleMessage msg;

  EXPECT_FALSE(ping.init(msg));
  ping.init();
  EXPECT_EQ(StandardMsgTypes::PING, ping.getMessageType());

  ping = PingMessage();
  ASSERT_TRUE(msg.init(StandardMsgTypes::PING, CommTypes::SERVICE_REQUEST, ReplyTypes::INVALID));
  EXPECT_TRUE(ping.init(msg));
  EXPECT_EQ(StandardMsgTypes::PING, ping.getMessageType());
}

TEST(PingMessageSuite, toMessage)
{
  PingMessage ping;
  SimpleMessage msg;

  ping.init();

  ASSERT_TRUE(ping.toReply(msg, ReplyTypes::SUCCESS));
  EXPECT_EQ(StandardMsgTypes::PING, msg.getMessageType());
  EXPECT_EQ(CommTypes::SERVICE_REPLY, msg.getCommType());
  EXPECT_EQ(ReplyTypes::SUCCESS, msg.getReplyCode());

  ASSERT_TRUE(ping.toRequest(msg));
  EXPECT_EQ(StandardMsgTypes::PING, msg.getMessageType());
  EXPECT_EQ(CommTypes::SERVICE_REQUEST, msg.getCommType());
  EXPECT_EQ(ReplyTypes::INVALID, msg.getReplyCode());

  EXPECT_FALSE(ping.toTopic(msg));

}

TEST(PingHandlerSuite, init)
{
  PingHandler handler;
  TestClient client;

  ASSERT_TRUE(handler.init(&client));
  EXPECT_EQ(StandardMsgTypes::PING, handler.getMsgType());

  EXPECT_FALSE(handler.init(NULL));

}

TEST(MessageManagerSuite, init)
{
  MessageManager manager;
  TestClient client;

  EXPECT_TRUE(manager.init(&client));
  EXPECT_FALSE(manager.init(NULL));

}

TEST(MessageManagerSuite, addHandler)
{
  MessageManager manager;
  TestClient client;
  PingHandler handler;

  EXPECT_EQ(0, (int)manager.getNumHandlers());

  ASSERT_TRUE(manager.init(&client));
  EXPECT_EQ(1, (int)manager.getNumHandlers());
  EXPECT_FALSE(manager.add(NULL));

  ASSERT_TRUE(handler.init(&client));
  EXPECT_FALSE(manager.add(&handler));
}

// wrapper around MessageManager::spin() that can be passed to
// pthread_create()
void*
spinFunc(void* arg)
{
  MessageManager* mgr = (MessageManager*)arg;
  mgr->spin();
  return NULL;
}

/*  Commenting out this test because build shows "unstable" with disabled tests
// See https://github.com/ros-industrial/industrial_core/issues/149 for details
TEST(DISABLED_MessageManagerSuite, tcp)
{
  const int port = TEST_PORT_BASE + 201;
  char ipAddr[] = "127.0.0.1";

  TestClient* client = new TestClient();
  TestServer server;
  SimpleMessage pingRequest, pingReply;
  MessageManager msgManager;

  // MessageManager uses ros::ok, which needs ros spinner
  ros::AsyncSpinner spinner(0);
  spinner.start();

  ASSERT_TRUE(pingRequest.init(StandardMsgTypes::PING, CommTypes::SERVICE_REQUEST, ReplyTypes::INVALID));

  // TCP Socket testing

  // Construct server
  ASSERT_TRUE(server.init(port));

  // Construct a client
  ASSERT_TRUE(client->init(&ipAddr[0], port));

  // Connect server and client
  pthread_t serverConnectThrd;
  pthread_create(&serverConnectThrd, NULL, connectServerFunc, &server);

  ASSERT_TRUE(client->makeConnect());
  pthread_join(serverConnectThrd, NULL);

  // Listen for client connection, init manager and start thread
  ASSERT_TRUE(msgManager.init(&server));

  // TODO: The message manager is not thread safe (threads are used for testing,
  // but running the message manager in a thread results in errors when the
  // underlying connection is deconstructed before the manager
  //boost::thread spinSrvThrd(boost::bind(&MessageManager::spin, &msgManager));
  pthread_t spinSrvThrd;
  pthread_create(&spinSrvThrd, NULL, spinFunc, &msgManager);

  // Ping the server
  ASSERT_TRUE(client->sendMsg(pingRequest));
  ASSERT_TRUE(client->receiveMsg(pingReply));
  ASSERT_TRUE(client->sendAndReceiveMsg(pingRequest, pingReply));

  // Delete client and try to reconnect

  delete client;
  sleep(10); //Allow time for client to destruct and free up port
  client = new TestClient();

  ASSERT_TRUE(client->init(&ipAddr[0], port));
  ASSERT_TRUE(client->makeConnect());
  ASSERT_TRUE(client->sendAndReceiveMsg(pingRequest, pingReply));

  pthread_cancel(spinSrvThrd);
  pthread_join(spinSrvThrd, NULL);

  delete client;
}
*/

// Run all the tests that were declared with TEST()
int main(int argc, char **argv)
{
  ros::init(argc, argv, "test");  // some tests need ROS framework
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}

