// Copyright [2015] Takashi Ogura<t.ogura@gmail.com>

#include "cv_camera/driver.h"

int main(int argc, char**argv)
{
  ros::init(argc, argv, "cv_camera");
  ros::NodeHandle private_node("~");
  cv_camera::Driver driver(private_node, private_node);

  try
  {
    driver.setup();
    while (ros::ok())
    {
      driver.proceed();
      ros::spinOnce();
    }
  }
  catch (cv_camera::DeviceError &e)
  {
    ROS_ERROR_STREAM("cv camera open failed: " << e.what());
    return 1;
  }

  return 0;
}
