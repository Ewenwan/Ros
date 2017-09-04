// Copyright [2015] Takashi Ogura<t.ogura@gmail.com>

#include <ros/ros.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/Image.h>
#include <gtest/gtest.h>

sensor_msgs::Image g_image;
sensor_msgs::CameraInfo g_camera_info;

void ImageCallback(const sensor_msgs::Image::ConstPtr& image)
{
  g_image = *image;
}

void InfoCallback(const sensor_msgs::CameraInfo::ConstPtr& info)
{
  g_camera_info = *info;
}

TEST(CvCameraNode, getImage)
{
  ros::NodeHandle node;
  ros::Subscriber sub = node.subscribe("/cv_camera_node/image_raw",
                                       1,
                                       &ImageCallback);
  ros::Rate r(10.0);
  while (sub.getNumPublishers() == 0) {
    r.sleep();
  }
  while (g_image.header.frame_id == "") {
    ros::spinOnce();
    r.sleep();
  }
  EXPECT_EQ("camera1", g_image.header.frame_id);
  EXPECT_EQ(480, g_image.height);
  EXPECT_EQ(640, g_image.width);
  EXPECT_EQ("bgr8", g_image.encoding);
}

TEST(CvCameraNode, getCameraInfo)
{
  ros::NodeHandle node;
  ros::Subscriber sub = node.subscribe("/cv_camera_node/camera_info",
                                       1,
                                       &InfoCallback);
  ros::Rate r(10.0);
  while (sub.getNumPublishers() == 0) {
    r.sleep();
  }
  while (g_camera_info.header.frame_id == "") {
    ros::spinOnce();
    r.sleep();
  }
  EXPECT_EQ("camera1", g_camera_info.header.frame_id);
  // K
  EXPECT_EQ(9, g_camera_info.K.size());
  EXPECT_NEAR(4827.94, g_camera_info.K.at(0), 0.001);
  EXPECT_NEAR(0.0, g_camera_info.K.at(1), 0.001);
  EXPECT_NEAR(1223.5, g_camera_info.K.at(2), 0.001);
  // D
  EXPECT_EQ(5, g_camera_info.D.size());
  EXPECT_NEAR(-0.41527, g_camera_info.D.at(0), 0.001);
  EXPECT_NEAR(0.31874, g_camera_info.D.at(1), 0.001);
  EXPECT_NEAR(-0.00197, g_camera_info.D.at(2), 0.001);
  EXPECT_NEAR(0.00071, g_camera_info.D.at(3), 0.001);
  EXPECT_NEAR(0.0, g_camera_info.D.at(4), 0.001);

  EXPECT_EQ("plumb_bob", g_camera_info.distortion_model);

  // width
  EXPECT_EQ(2448, g_camera_info.width);
  EXPECT_EQ(2050, g_camera_info.height);
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "test_cv_camera");
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
