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
  ros::Subscriber sub = node.subscribe("/cv_camera_no_yaml/image_raw",
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
  EXPECT_EQ("camera2", g_image.header.frame_id);
  EXPECT_EQ(480, g_image.height);
  EXPECT_EQ(640, g_image.width);
  EXPECT_EQ("bgr8", g_image.encoding);
}

TEST(CvCameraNode, getCameraInfo)
{
  ros::NodeHandle node;
  ros::Subscriber sub = node.subscribe("/cv_camera_no_yaml/camera_info",
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
  EXPECT_EQ("camera2", g_camera_info.header.frame_id);
  // K
  EXPECT_EQ(9, g_camera_info.K.size());
  EXPECT_NEAR(0.0, g_camera_info.K.at(0), 0.001);
  EXPECT_NEAR(0.0, g_camera_info.K.at(1), 0.001);
  EXPECT_NEAR(0.0, g_camera_info.K.at(2), 0.001);
  // D
  EXPECT_EQ(0, g_camera_info.D.size());

  EXPECT_EQ("", g_camera_info.distortion_model);

  // width
  EXPECT_EQ(640, g_camera_info.width);
  EXPECT_EQ(480, g_camera_info.height);
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "test_cv_camera_no_yaml");
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
