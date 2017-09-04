// Copyright [2015] Takashi Ogura<t.ogura@gmail.com>

#ifndef CV_CAMERA_EXCEPTION_H
#define CV_CAMERA_EXCEPTION_H

#include <stdexcept>
#include <string>

namespace cv_camera
{

/**
 * @brief ROS cv camera device exception.
 *
 */
class DeviceError : public std::runtime_error
{
 public:
  explicit DeviceError(const std::string &cause):
      std::runtime_error(cause) {}
};

}  // end namespace cv_camera

#endif  // CV_CAMERA_EXCEPTION_H
