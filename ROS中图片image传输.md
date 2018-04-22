# 1. 摄像头图像采集

      有2个思路，一个是使用ROS现有的包（主流方法）；
      一个是利用OpenCV提供的VideoCapture方法，然后通过ROS提供的cv_bridge转为sensor_msgs/Image消息（cv_bridge的例程）。
      对于第一个思路，ROS有个官方提供的驱动包：
      libuvc_camera，但《ROS By Example》一书出于使用便利性推荐第三方包：usb_cam。
      具体使用方法参考“ROS中使用摄像头的问题”。



# 2. 图像信息在ROS中的流动

      使用ROS提供的image_transport空间发布和订阅图像话题。
      一个细节是，image_transport提供了两组类：Publisher, Subscriber 和 CameraPublisher, CameraSubscriber，
      前者仅发送图像消息（Sensor_msgs/Image），后者同时提供摄像机信息（sensor_msgs/CameraInfo）。

##  2.1 Sensor_msgs/Image
          # This message contains an uncompressed image  
          # (0, 0) is at top-left corner of image  
          std_msgs/Header header  
          uint32 height  
          uint32 width  
          string encoding  
          uint8 is_bigendian  
          uint32 step  
          uint8[] data 

##  2.2 sensor_msgs/CameraInfo
      主要包含了相机分辨率、内参矩阵、畸变向量、旋转矩阵（针对双目相机）,投影矩阵 等。

      派生到我的代码片

          std_msgs/Header header  
          uint32 height  
          uint32 width  
          stringdistortion_model  
          float64[] D  
          float64[9] K  
          float64[9] R  
          float64[12] P  
          uint32 binning_x  
          uint32 binning_y  
          sensor_msgs/RegionOfInterestroi  

## 2.3 插件
      Compressed_image_transport是image_transport的一个插件包，可以将图像压缩为JPEG或PNG。
      Threora_image_transport是image_transport的一个插件包，可以对图像消息做视频流压缩处理。

# 3. 视觉算法前的预处理
      从摄像头采集的图像一般需要去畸变等步骤才可为高阶视觉算法使用。
      ROS的image_pipline主要实现5个功能：相机标定、单相机去畸变（image_proc类实现）、
      双目相机去畸变、深度相机处理（Kinect）和可视化（image_view）。
      实际上这些功能大多使用OpenCV的函数实现的，因此如果嫌麻烦了解这些包，
      完全可以自己用OpenCV搞定标定、去畸变这些工作。

      如图，Image_proc输出4种图像，分别为灰度畸变、彩色畸变、灰度去畸变、彩色去畸变图像。


      如图，Image_proc输出4种图像，分别为灰度畸变、彩色畸变、灰度去畸变、彩色去畸变图像。
      image_mono[image]
      image_color[image]
      image_rect[image]
      image_rect_color[image]


#  4. OpenCV在ROS里的使用
      参考官方文档。主要包含两个包：cv_bridge和image_geometry。
      前者主要实现OpenCV的Mat/IplImage向ROS消息的转换；
      http://wiki.ros.org/cv_bridge/Tutorials/UsingCvBridgeToConvertBetweenROSImagesAndOpenCVImages

      后者提供了提供了一系列关于图像几何处理的方法。
      http://wiki.ros.org/image_geometry/Tutorials/ProjectTfFrameToImage


      Image_geometry预设了两种相机模型：针孔模型（PinholeCameraModel）和双目模型（StereoCameraModel）。
      类里面实现了图像坐标系（已做去畸变处理）与相机坐标系的转换方法（project3dToPixel / projectPixelTo3dRay，
      [u v 1] = K * (x, y, z)

      可以参考其源代码，就是简单的应用了图像坐标系到相机坐标系的转换关系
      http://docs.ros.org/api/image_geometry/html/c++/pinhole__camera__model_8cpp_source.html#l00234
      所以需要提前做rectify 矫正去畸变）。

      以上所有的应用都是基于已知CameraInfo的前提,
      camera_info_manager（函数源代码）通过load标定结果设置camerainfo，
      其中解析标定YML文件的函数由camera_calibrate提供（看它的源代码更好理解）。
      camera_info_manager的使用在camera1394里有应用。
      https://github.com/ros-drivers/camera1394/blob/master/src/nodes/driver1394.cpp#L80

#    5. 坐标转换
      Tf采用树的结构组织每个frame的转换关系（旋转和平移，一个细节是这里用四元数Quaternion或 欧拉角 RPY的方式描述旋转，
      有别于OpenCV里的3*3旋转矩阵或罗德里格斯变换。）
      每个frame可以发布和订阅指定的转换（同时还有时间维度，因为转换关系可能随时间变化）。
      一个简单的TF解释及例子：ROS探索总结（二十二）——设置机器人的tf变换
      http://www.guyuehome.com/355

