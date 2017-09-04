
#include <gtest/gtest.h>

#include <sys/socket.h>
#include <sys/fcntl.h>
#include <netinet/in.h>
#include <arpa/inet.h>

#include "ros/ros.h"

namespace rosserial {
#include "rosserial_test/ros.h"
}

class AbstractSetup {
public:
  virtual void SetUp()=0;
  virtual void TearDown()=0;
  int fd;
};

class SerialSetup : public AbstractSetup {
public:
  virtual void SetUp() {
    ASSERT_NE(-1, fd = posix_openpt( O_RDWR | O_NOCTTY | O_NDELAY ));
    ASSERT_NE(-1, grantpt(fd));
    ASSERT_NE(-1, unlockpt(fd));

    char* pty_name;
    ASSERT_TRUE((pty_name = ptsname(fd)) != NULL);

    ros::param::get("~port", symlink_name);
    symlink(pty_name, symlink_name.c_str());
  }
  virtual void TearDown() {
    unlink(symlink_name.c_str());
    close(fd);
  }
  std::string symlink_name;
};

class SocketSetup : public AbstractSetup {
public:
  virtual void SetUp() {
    ros::param::get("~tcp_port", tcp_port);
    memset(&serv_addr, 0, sizeof(serv_addr));
    serv_addr.sin_family = AF_INET;
    serv_addr.sin_port = htons(tcp_port);
    ASSERT_GE(inet_pton(AF_INET, "127.0.0.1", &serv_addr.sin_addr), 0);

    // Try a bunch of times; we don't know how long it will take for the
    // server to come up.
    for (int attempt = 0; attempt < 10; attempt++)
    {
      fd = socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);
      ASSERT_GE(fd, 0);
      if (connect(fd, (struct sockaddr *)&serv_addr, sizeof(serv_addr)) >= 0)
      {
        // Connection successful, set nonblocking and return.
        fcntl(fd, F_SETFL, O_NONBLOCK);
        return;
      }
      close(fd);
      ros::Duration(0.5).sleep();
    }
    FAIL() << "Unable to connect to rosserial socket server.";
  }
  virtual void TearDown() {
    close(fd);
  }
  struct sockaddr_in serv_addr;
  int tcp_port;
};

class SingleClientFixture : public ::testing::Test {
protected:
  static void SetModeFromParam() {
    std::string mode;
    ros::param::get("~mode", mode);
    ROS_INFO_STREAM("Using test mode [" << mode << "]");
    if (mode == "socket") {
      setup = new SocketSetup();
    } else if (mode == "serial") {
      setup = new SerialSetup();
    } else {
      FAIL() << "Mode specified other than 'serial' or 'socket'.";
    }
  }
  virtual void SetUp() {
    if (setup == NULL) SetModeFromParam();
    setup->SetUp();
    rosserial::ClientComms::fd = setup->fd;
  }
  virtual void TearDown() {
    setup->TearDown();
  }

  rosserial::ros::NodeHandle client_nh;
  ros::NodeHandle nh;
  static AbstractSetup* setup;
};
AbstractSetup* SingleClientFixture::setup = NULL;


