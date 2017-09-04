/*
 * Software License Agreement (BSD License)
 *
 * Copyright (c) 2012, Southwest Research Institute
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
#include "simple_message/socket/tcp_client.h"
#include "simple_message/socket/tcp_server.h"
#include "simple_message/messages/joint_message.h"
#include "simple_message/joint_data.h"
#include "simple_message/joint_traj_pt.h"
#include "simple_message/messages/joint_traj_pt_message.h"
#include "simple_message/typed_message.h"
#include "simple_message/joint_traj.h"
#include "simple_message/robot_status.h"
#include "simple_message/messages/robot_status_message.h"

#include <gtest/gtest.h>

using namespace industrial::simple_message;
using namespace industrial::tcp_socket;
using namespace industrial::tcp_client;
using namespace industrial::tcp_server;
using namespace industrial::joint_data;
using namespace industrial::joint_message;
using namespace industrial::joint_traj_pt;
using namespace industrial::joint_traj_pt_message;
using namespace industrial::typed_message;
using namespace industrial::joint_traj;
using namespace industrial::robot_status;
using namespace industrial::robot_status_message;

// Message passing routine, used to send and receive a typed message
// Useful for checking the packing and unpacking of message data.
void messagePassing(TypedMessage &send, TypedMessage &recv)
{
  const int tcpPort = TEST_PORT_BASE+401;
  char ipAddr[] = "127.0.0.1";

  TcpClient tcpClient;
  TcpServer tcpServer;
  SimpleMessage msgSend, msgRecv;

  ASSERT_TRUE(send.toTopic(msgSend));

  // Construct server

  ASSERT_TRUE(tcpServer.init(tcpPort));

  // Construct a client
  ASSERT_TRUE(tcpClient.init(&ipAddr[0], tcpPort));
  ASSERT_TRUE(tcpClient.makeConnect());

  ASSERT_TRUE(tcpServer.makeConnect());

  ASSERT_TRUE(tcpClient.sendMsg(msgSend));
  ASSERT_TRUE(tcpServer.receiveMsg(msgRecv));
  ASSERT_TRUE(recv.init(msgRecv));
}

TEST(JointMessage, init)
{
  JointData joint;

  joint.init();
  EXPECT_TRUE(joint.setJoint(0, 1.0));
  EXPECT_TRUE(joint.setJoint(1, 2.0));
  EXPECT_TRUE(joint.setJoint(2, 3.0));
  EXPECT_TRUE(joint.setJoint(3, 4.0));
  EXPECT_TRUE(joint.setJoint(4, 5.0));
  EXPECT_TRUE(joint.setJoint(5, 6.0));
  EXPECT_TRUE(joint.setJoint(6, 7.0));
  EXPECT_TRUE(joint.setJoint(7, 8.0));
  EXPECT_TRUE(joint.setJoint(8, 9.0));
  EXPECT_TRUE(joint.setJoint(9, 10.0));

  EXPECT_FALSE(joint.setJoint(10, 11.0));

}

TEST(JointMessage, equal)
{
  JointData jointlhs, jointrhs;

  jointrhs.init();
  jointlhs.init();
  jointlhs.setJoint(0, -1.0);
  jointlhs.setJoint(9, 1.0);

  EXPECT_FALSE(jointlhs==jointrhs);

  jointrhs.setJoint(0, -1.0);
  jointrhs.setJoint(9, 1.0);

  EXPECT_TRUE(jointlhs==jointrhs);

}

TEST(JointMessage, toMessage)
{
  JointData toMessage, fromMessage;
  JointMessage msg;

  toMessage.init();
  toMessage.setJoint(4, 44.44);

  msg.init(1, toMessage);

  fromMessage.copyFrom(msg.getJoints());

  EXPECT_TRUE(toMessage==fromMessage);

}

TEST(JointMessage, Comms)
{

  JointMessage jointSend, jointRecv;
  JointData posSend, posRecv;

  posSend.init();
  posSend.setJoint(0, 1.0);
  posSend.setJoint(1, 2.0);
  posSend.setJoint(2, 3.0);
  posSend.setJoint(3, 4.0);
  posSend.setJoint(4, 5.0);
  posSend.setJoint(5, 6.0);
  posSend.setJoint(6, 7.0);
  posSend.setJoint(7, 8.0);
  posSend.setJoint(8, 9.0);
  posSend.setJoint(9, 10.0);

  jointSend.init(1, posSend);

  messagePassing(jointSend, jointRecv);

  posRecv.copyFrom(jointRecv.getJoints());
  ASSERT_TRUE(posRecv==posSend);
}

TEST(JointTrajPt, equal)
{
  JointTrajPt lhs, rhs;
  JointData joint;

  joint.init();
  ASSERT_TRUE(joint.setJoint(0, 1.0));
  ASSERT_TRUE(joint.setJoint(1, 2.0));
  ASSERT_TRUE(joint.setJoint(2, 3.0));
  ASSERT_TRUE(joint.setJoint(3, 4.0));
  ASSERT_TRUE(joint.setJoint(4, 5.0));
  ASSERT_TRUE(joint.setJoint(5, 6.0));
  ASSERT_TRUE(joint.setJoint(6, 7.0));
  ASSERT_TRUE(joint.setJoint(7, 8.0));
  ASSERT_TRUE(joint.setJoint(8, 9.0));
  ASSERT_TRUE(joint.setJoint(9, 10.0));

  rhs.init(1.0, joint, 50.0, 100);
  EXPECT_FALSE(lhs==rhs);

  lhs.init(0, joint, 0, 0);
  EXPECT_FALSE(lhs==rhs);

  lhs.copyFrom(rhs);
  EXPECT_TRUE(lhs==rhs);

}

TEST(JointTrajPt, toMessage)
{
  JointTrajPt toMessage, fromMessage;
  JointTrajPtMessage msg;

  toMessage.init();
  toMessage.setSequence(99);
  msg.init(toMessage);

  fromMessage.copyFrom(msg.point_);

  EXPECT_TRUE(toMessage==fromMessage);

}

TEST(JointTrajPt, Comms)
{

  JointTrajPtMessage jointSend, jointRecv;
  JointData data;
  JointTrajPt posSend, posRecv;

  data.init();
  data.setJoint(0, 1.0);
  data.setJoint(1, 2.0);
  data.setJoint(2, 3.0);
  data.setJoint(3, 4.0);
  data.setJoint(4, 5.0);
  data.setJoint(5, 6.0);
  data.setJoint(6, 7.0);
  data.setJoint(7, 8.0);
  data.setJoint(8, 9.0);
  data.setJoint(9, 10.0);
  posSend.init(1, data, 99, 100);

  jointSend.init(posSend);

  messagePassing(jointSend, jointRecv);

  posRecv.copyFrom(jointRecv.point_);
  ASSERT_TRUE(posRecv==posSend);
}

TEST(JointTraj, equal)
{
  JointTraj lhs, rhs;
  JointData joint;
  JointTrajPt point;

  joint.init();
  ASSERT_TRUE(joint.setJoint(0, 1.0));
  ASSERT_TRUE(joint.setJoint(1, 2.0));
  ASSERT_TRUE(joint.setJoint(2, 3.0));
  ASSERT_TRUE(joint.setJoint(3, 4.0));
  ASSERT_TRUE(joint.setJoint(4, 5.0));
  ASSERT_TRUE(joint.setJoint(5, 6.0));
  ASSERT_TRUE(joint.setJoint(6, 7.0));
  ASSERT_TRUE(joint.setJoint(7, 8.0));
  ASSERT_TRUE(joint.setJoint(8, 9.0));
  ASSERT_TRUE(joint.setJoint(9, 10.0));

  point.init(1.0, joint, 50.0, 100);
  rhs.addPoint(point);
  EXPECT_FALSE(lhs==rhs);

  lhs.addPoint(point);
  EXPECT_TRUE(lhs==rhs);

  lhs.addPoint(point);
  EXPECT_FALSE(lhs==rhs);

  lhs.copyFrom(rhs);
  EXPECT_TRUE(lhs==rhs);

}

TEST(RobotStatus, enumerations)
{
  // Verifying the disabled state and aliases match
  EXPECT_EQ(TriStates::TS_DISABLED, TriStates::TS_FALSE);
  EXPECT_EQ(TriStates::TS_DISABLED, TriStates::TS_LOW);
  EXPECT_EQ(TriStates::TS_DISABLED, TriStates::TS_OFF);

  // Verifying the enabled state and aliases values match
  EXPECT_EQ(TriStates::TS_ENABLED, TriStates::TS_TRUE);
  EXPECT_EQ(TriStates::TS_ENABLED, TriStates::TS_HIGH);
  EXPECT_EQ(TriStates::TS_ENABLED, TriStates::TS_ON);

  // Verifying the unknown values match (this isn't reqd, but makes sense)
  EXPECT_EQ(TriStates::TS_UNKNOWN, RobotModes::UNKNOWN);
}

TEST(RobotStatus, init)
{
  RobotStatus status;
  RobotStatus empty;
  status.init();
  // An empty (non-initted) status should be initialized in the constructor.
  EXPECT_TRUE(status==empty);
  EXPECT_EQ(status.getDrivesPowered(), TriStates::TS_UNKNOWN);
  EXPECT_EQ(status.getEStopped(), TriStates::TS_UNKNOWN);
  EXPECT_EQ(status.getErrorCode(), 0);
  EXPECT_EQ(status.getInError(), TriStates::TS_UNKNOWN);
  EXPECT_EQ(status.getInMotion(), TriStates::TS_UNKNOWN);
  EXPECT_EQ(status.getMode(), RobotModes::UNKNOWN);
  EXPECT_EQ(status.getMotionPossible(), TriStates::TS_UNKNOWN);
}

TEST(RobotStatus, equal)
{
  RobotStatus lhs, rhs;

  EXPECT_TRUE(lhs==rhs);
  lhs.setDrivesPowered(TriStates::TS_ENABLED);
  EXPECT_FALSE(lhs==rhs);
  rhs.setDrivesPowered(TriStates::TS_ENABLED);
  EXPECT_TRUE(lhs==rhs);

  lhs.setEStopped(TriStates::TS_ENABLED);
  EXPECT_FALSE(lhs==rhs);
  rhs.setEStopped(TriStates::TS_ENABLED);
  EXPECT_TRUE(lhs==rhs);

  lhs.setErrorCode(99);
  EXPECT_FALSE(lhs==rhs);
  rhs.setErrorCode(99);
  EXPECT_TRUE(lhs==rhs);

  lhs.setInError(TriStates::TS_ENABLED);
  EXPECT_FALSE(lhs==rhs);
  rhs.setInError(TriStates::TS_ENABLED);
  EXPECT_TRUE(lhs==rhs);

  lhs.setInMotion(TriStates::TS_ENABLED);
  EXPECT_FALSE(lhs==rhs);
  rhs.setInMotion(TriStates::TS_ENABLED);
  EXPECT_TRUE(lhs==rhs);

  lhs.setMode(RobotModes::AUTO);
  EXPECT_FALSE(lhs==rhs);
  rhs.setMode(RobotModes::AUTO);
  EXPECT_TRUE(lhs==rhs);

  lhs.setMotionPossible(TriStates::TS_ENABLED);
  EXPECT_FALSE(lhs==rhs);
  rhs.setMotionPossible(TriStates::TS_ENABLED);
  EXPECT_TRUE(lhs==rhs);

}

TEST(RobotStatus, toMessage)
{
  RobotStatus toMessage, fromMessage;
  RobotStatusMessage msg;

  toMessage.init(TriStates::TS_ENABLED, TriStates::TS_FALSE, 99, TriStates::TS_TRUE, TriStates::TS_TRUE, RobotModes::MANUAL,
                 TriStates::TS_DISABLED);
  msg.init(toMessage);

  fromMessage.copyFrom(msg.status_);

  EXPECT_TRUE(toMessage==fromMessage);

}

TEST(RobotStatus, Comms)
{
  RobotStatusMessage statusMsgSend, statusMsgRecv;
  RobotStatus statusSend, statusRecv;

  statusSend.init(TriStates::TS_ENABLED, TriStates::TS_FALSE, 99, TriStates::TS_TRUE, TriStates::TS_TRUE, RobotModes::MANUAL,
                   TriStates::TS_DISABLED);

  statusMsgSend.init(statusSend);

  messagePassing(statusMsgSend, statusMsgRecv);

  statusRecv.copyFrom(statusMsgRecv.status_);
  ASSERT_TRUE(statusRecv==statusSend);
}

