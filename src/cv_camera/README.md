ROS OpenCV camera driver
========================
It is very easy to capture video device if we use cv::VideoCapture of OpenCV.

cv_camera_node
------------------
This node uses [camera_info_manager](http://wiki.ros.org/camera_info_manager) for dealing with camera_info.
If no calibration data is set, it has dummy values except for width and height.

### Publish ###

* ~image_raw (sensor_msgs/Image)
* ~camera_info (sensor_msgs/CameraInfo)

### Service ###

* ~set_camera_info (sensor_msgs/SetCameraInfo)

### Parameters ###

* ~rate (double: default 30.0) publish rate [Hz].
* ~device_id (int: default 0) capture device id.
* ~frame_id (string: default "camera") frame_id of message header.
* ~image_width (int) try to set capture image width.
* ~image_height (int) try to set capture image height.
* ~camera_info_url (string) url of camera info yaml.
* ~file (string: default "") if not "" then use movie file instead of device.

supports CV_CAP_PROP_*, by below params.

* ~cv_cap_prop_pos_msec (double)
* ~cv_cap_prop_pos_avi_ratio (double)
* ~cv_cap_prop_frame_width (double)
* ~cv_cap_prop_frame_height (double)
* ~cv_cap_prop_fps (double)
* ~cv_cap_prop_fourcc (double)
* ~cv_cap_prop_frame_count (double)
* ~cv_cap_prop_format (double)
* ~cv_cap_prop_mode (double)
* ~cv_cap_prop_brightness (double)
* ~cv_cap_prop_contrast (double)
* ~cv_cap_prop_saturation (double)
* ~cv_cap_prop_hue (double)
* ~cv_cap_prop_gain (double)
* ~cv_cap_prop_exposure (double)
* ~cv_cap_prop_convert_rgb (double)
* ~cv_cap_prop_rectification (double)
* ~cv_cap_prop_iso_speed (double)

And supports any props. Thanks to Hernan Badino!

* ~property_$(i)_code (int) : set this code property using ~property_$(i)_value, $(i) must start from 0.
* ~property_$(i)_value (double) : the value to be set to ~property_$(i)_code

If you want to set the property which code is 404 as 1,

    $ rosrun cv_camera cv_camera_node _property_0_code:=404 _property_0_code:=1

If you want to set more, use ~property_1_code and ~property_1_code.


Nodelet
-------------------

This node works as nodelet (cv_camera/CvCameraNodelet).
