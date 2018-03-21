/*
点云滤波分割聚类识别
pipeline 流水线

【1】点云算法头文件
【2】初始化 ROS节点
【3】私有节点获取参数
【4】发布处理识别后点云消息topic
【5】监听 源电云 话题数据获取点云
【6】 将相机坐标系下的点云 转换到 世界坐标系下
【7】ROS点云格式 转换到 PCL点云格式 
【8】体素滤波下采样  + 去除 NAN点   
     直通滤波保存特点范围内的点
     PLANE平面分割 
     欧式距离分类
     统计学滤波剔除外点
     多边形分割
【10】 PCL点云格式 转换到 ROS点云格式     
      PCL->ROS方便发布点云数据 提供RVIZ可视化显示
【11】广播坐标变换 (可选)
*/

#include <ros/ros.h>
#include <tf/transform_datatypes.h>//坐标变换数据类型
#include <tf/transform_listener.h>//坐标变换监听
#include <tf/transform_broadcaster.h>//坐标变换广播
#include <tf_conversions/tf_eigen.h>//tf::transformTFToEigen(part_transform,eigen3d);
#include <sensor_msgs/PointCloud2.h> //hydro  ROS下的PCL消息类型
#include <sensor_msgs/point_cloud_conversion.h>//ROS PCL


// PCL specific includes
#include <pcl_conversions/pcl_conversions.h> //hydro   pcl类型转换
#include "pcl_ros/transforms.h"
#include <pcl/conversions.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
//=========================================================================
//=======================【1】点云算法头文件 ===============================
#include <pcl/filters/voxel_grid.h>// 体素格下采样滤波器
#include <pcl/filters/passthrough.h>//直通滤波器
#include <pcl/filters/crop_box.h>//立方体滤波
#include <pcl/sample_consensus/method_types.h>// 采样方法
#include <pcl/sample_consensus/model_types.h>//采样模型
#include <pcl/segmentation/sac_segmentation.h>//点云分割算法
#include <pcl/segmentation/extract_polygonal_prism_data.h>//多边形分割
#include <pcl/filters/extract_indices.h>//滤波器提取索引  从一个点云中提取索引
#include <pcl/segmentation/extract_clusters.h>//分割提取 点云
#include <pcl/kdtree/kdtree_flann.h>//最近临 存储
#include <pcl/filters/statistical_outlier_removal.h>// 统计学滤波提出 外点 噪点 离群点

int main(int argc, char *argv[])
{
//===========================================================================
//========================【2】初始化 ROS节点================================
// 一个主节点 一个私有节点获取参数
	ros::init(argc, argv, "perception_node");
	ros::NodeHandle nh;
	ros::NodeHandle priv_nh_("~");//私有节点获取参数
//==============================================================================
//========================【3】私有节点获取参数==============================
//可来自 launch文件 以及 terminal终端设置
// priv_nh_.getParam 私有节点获取参数   点云处理相关参数较多 需要放在launch文件里 
	std::string cloud_topic, world_frame, camera_frame;
	//world_frame="kinect_link";//世界坐标系
	//camera_frame="kinect_link";//相机坐标系
	//cloud_topic="kinect/depth_registered/points";//发布源点云数据的话题

	//priv_nh_.getParam("cloud_topic", cloud_topic);//
	//priv_nh_.getParam("world_frame", world_frame);
	//priv_nh_.getParam("camera_frame", camera_frame);
	//priv_nh_.getParam("voxel_leaf_size", voxel_leaf_size);
	//priv_nh_.getParam("x_filter_min", x_filter_min);
	//priv_nh_.getParam("x_filter_max", x_filter_max);
	//priv_nh_.getParam("y_filter_min", y_filter_min);
	//priv_nh_.getParam("y_filter_max", y_filter_max);
	//priv_nh_.getParam("z_filter_min", z_filter_min);
	//priv_nh_.getParam("z_filter_max", z_filter_max);
	//priv_nh_.getParamCached("plane_max_iterations", plane_max_iter);
	//priv_nh_.getParamCached("plane_distance_threshold", plane_dist_thresh);
	//priv_nh_.getParam("cluster_tolerance", cluster_tol);
	//priv_nh_.getParam("cluster_min_size", cluster_min_size);
	//priv_nh_.getParam("cluster_max_size", cluster_max_size);
        float x_filter_min, x_filter_max, y_filter_min, y_filter_max,z_filter_min, z_filter_max;
	int plane_max_iter, cluster_min_size;
 	double cluster_max_size;
	float voxel_leaf_size, plane_dist_thresh, cluster_tol;
	//发布源点云数据的话题
	priv_nh_.param<std::string>("cloud_topic", cloud_topic, "kinect/depth_registered/points");
	priv_nh_.param<std::string>("world_frame", world_frame, "kinect_link");
	priv_nh_.param<std::string>("camera_frame", camera_frame, "kinect_link");
	priv_nh_.param<float>("voxel_leaf_size", voxel_leaf_size, 0.001);// 体素滤波 格子的大小
	priv_nh_.param<float>("x_filter_min", x_filter_min, -2.5);// m
	priv_nh_.param<float>("x_filter_max", x_filter_max, 2.5);//   立方体滤波
	priv_nh_.param<float>("y_filter_min", y_filter_min, -2.5);// m
	priv_nh_.param<float>("y_filter_max", y_filter_max, 2.5);//
	priv_nh_.param<float>("z_filter_min", z_filter_min, -2.5);// m
	priv_nh_.param<float>("z_filter_max", z_filter_max, 2.5);//
	priv_nh_.param<float>("z_filter_max", z_filter_max, 2.5);//
	priv_nh_.param<int>("plane_max_iterations", plane_max_iter, 100);// PLANE平面分割 最大迭代次数 
	priv_nh_.param<float>("plane_distance_threshold", plane_dist_thresh, 0.05);//距离阈值 
	priv_nh_.param<float>("cluster_tolerance", cluster_tol, 0.01);//距离聚类容差 2cm
	priv_nh_.param<double>("cluster_max_size", cluster_max_size, 50000);// max number
	priv_nh_.param<int>("cluster_min_size", cluster_min_size, 100);// minimam number

std::cout << world_frame << std::endl;
//============================================================================
//========================【4】发布处理识别后点云消息========================
//用于可视化识别结果
	ros::Publisher object_pub, cluster_pub, pose_pub, VoxelG_fil_pub, pt_filter_pub, CBox_fil_pub, plane_pub;
	ros::Publisher one_object_pub, one_object_outlier_pub, prism_fil_pub;
	object_pub = nh.advertise<sensor_msgs::PointCloud2>("object_cluster", 1);// 发布点云话题  物体点云
	cluster_pub = nh.advertise<sensor_msgs::PointCloud2>("primary_cluster", 1);// 主要点云
	VoxelG_fil_pub = nh.advertise<sensor_msgs::PointCloud2>("VoxelGrid_down_sam", 1);// VoxelGrid_down_sam
	pt_filter_pub  = nh.advertise<sensor_msgs::PointCloud2>("PassThrough_filter", 1);// PassThrough_filter
	CBox_fil_pub   = nh.advertise<sensor_msgs::PointCloud2>("CropBox_filter", 1);// CropBox filter
	plane_pub      = nh.advertise<sensor_msgs::PointCloud2>("plane", 1);// plane
	one_object_pub = nh.advertise<sensor_msgs::PointCloud2>("one_object", 1);// one_object
	one_object_outlier_pub = nh.advertise<sensor_msgs::PointCloud2>("one_object_outlier", 1);// one_object_outlier
	prism_fil_pub = nh.advertise<sensor_msgs::PointCloud2>("prism_fil", 1);//

 while (ros::ok())
 {
//=========================================================================
//=======================【5】监听 源电云 话题数据获取点云===================
//  可以使用 订阅话题消息  回调函数处理消息 
// 参考http://wiki.ros.org/pcl/Tutorials
// 简单可以使用 ros::topic::waitForMessage

	std::string topic = nh.resolveName(cloud_topic);
	ROS_INFO_STREAM("Cloud service called; waiting for a PointCloud2 on topic "<< topic);
	sensor_msgs::PointCloud2::ConstPtr recent_cloud =
	       ros::topic::waitForMessage<sensor_msgs::PointCloud2>(topic, nh);

//==========================================================================
//======================【6】 将相机坐标系下的点云 转换到 世界坐标系下=======
//方便处理
//需要监听 两个坐标系之间的变换 

	tf::TransformListener listener;//坐标变换监听
	tf::StampedTransform stransform;//时间戳
	try
	{
	// 等待坐标变换 转换
	listener.waitForTransform(world_frame, recent_cloud->header.frame_id,  ros::Time::now(), ros::Duration(6.0));
	// 得到坐标系之间的变换
	listener.lookupTransform(world_frame, recent_cloud->header.frame_id,  ros::Time(0), stransform);
	}
	catch (tf::TransformException ex)
	{
	ROS_ERROR("%s",ex.what());
	}
	sensor_msgs::PointCloud2 transformed_cloud;//转换后的点云
	//  sensor_msgs::PointCloud2::ConstPtr recent_cloud =
	//               ros::topic::waitForMessage<sensor_msgs::PointCloud2>(topic, nh);
	// 转换
	pcl_ros::transformPointCloud(world_frame, stransform, *recent_cloud, transformed_cloud);


 //================================================================ 
//====================【7】ROS点云格式 转换到 PCL点云格式  ================
// ROS->PCL方便使用pcl库函数处理点云

	pcl::PointCloud<pcl::PointXYZ> cloud;//PCL点云格式
	pcl::fromROSMsg (transformed_cloud, cloud);

//========================================
//Fill Code: VOXEL GRID 体素滤波下采样  + 去除 NAN点
//========================================
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_ptr (new pcl::PointCloud<pcl::PointXYZ> (cloud));
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_voxel_filtered (new pcl::PointCloud<pcl::PointXYZ> ());
	pcl::VoxelGrid<pcl::PointXYZ> voxel_filter;//滤波器
	voxel_filter.setInputCloud (cloud_ptr);//指针
	voxel_filter.setLeafSize (voxel_leaf_size, voxel_leaf_size, voxel_leaf_size);//2cm大小的格子
	voxel_filter.filter (*cloud_voxel_filtered);



  //ROS_INFO_STREAM("Original cloud  had " << cloud_ptr->size() << " points");
  //ROS_INFO_STREAM("Downsampled cloud  with " << cloud_voxel_filtered->size() << " points");

//==============================================================
//===================【8】 直通滤波保存特点范围内的点==============
//===============================================================
	// x 轴滤波
	pcl::PointCloud<pcl::PointXYZ> xf_cloud, yf_cloud, zf_cloud;//非 指针
	pcl::PassThrough<pcl::PointXYZ> pass_x;//滤波器
	pass_x.setInputCloud(cloud_voxel_filtered);//指针
	pass_x.setFilterFieldName("x");
	pass_x.setFilterLimits(-1.0,1.0);//保留 -1m 到 +1m范围内的点
	pass_x.filter(xf_cloud);
	// y 轴滤波
	pcl::PointCloud<pcl::PointXYZ>::Ptr xf_cloud_ptr(new pcl::PointCloud<pcl::PointXYZ>(xf_cloud));
	pcl::PassThrough<pcl::PointXYZ> pass_y;//滤波器
	pass_y.setInputCloud(xf_cloud_ptr);//指针
	pass_y.setFilterFieldName("y");
	pass_y.setFilterLimits(-1.0, 1.0);//保留 -1m 到 +1m范围内的点
	pass_y.filter(yf_cloud);
	// z 轴滤波
	pcl::PointCloud<pcl::PointXYZ>::Ptr yf_cloud_ptr(new pcl::PointCloud<pcl::PointXYZ>(yf_cloud));
	pcl::PassThrough<pcl::PointXYZ> pass_z;
	pass_z.setInputCloud(yf_cloud_ptr);
	pass_z.setFilterFieldName("z");
	pass_z.setFilterLimits(-1.0, 1.0);
	pass_z.filter(zf_cloud);

//========================================
//======过滤掉在用户给定立方体内的点云数据 
//=============【9】立方体滤波 pcl::CropBox< PointT>    
//过滤掉在用户给定立方体内的点云数据
//========================================*/
	pcl::PointCloud<pcl::PointXYZ> xyz_filtered_cloud;
	pcl::CropBox<pcl::PointXYZ> crop;
	crop.setInputCloud(cloud_voxel_filtered);
//	float x_filter_min = -1.0, y_filter_min = -1.0, z_filter_min = -1.0;
//	float x_filter_max = 1.0, y_filter_max = 1.0,  z_filter_max = 1.0;
	Eigen::Vector4f min_point = Eigen::Vector4f(x_filter_min, y_filter_min, z_filter_min, 0);
	Eigen::Vector4f max_point = Eigen::Vector4f(x_filter_max, y_filter_max, z_filter_max, 0);
	crop.setMin(min_point);
	crop.setMax(max_point);

	crop.filter(xyz_filtered_cloud);



//========================================
//===================【10】PLANE平面分割 SEGEMENTATION 
//============物体在平面上 分割出平面后 就可以分割出物体  
//========================================*/
        //pcl::PointCloud<pcl::PointXYZ>::Ptr cropped_cloud(new pcl::PointCloud<pcl::PointXYZ>(xyz_filtered_cloud));// CropBox
	pcl::PointCloud<pcl::PointXYZ>::Ptr cropped_cloud(new pcl::PointCloud<pcl::PointXYZ>(zf_cloud));//PassThrough 源 滤波后的点云
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_f (new pcl::PointCloud<pcl::PointXYZ>);// 平面上的点
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_plane (new pcl::PointCloud<pcl::PointXYZ> ());//分割出的平面点云
	// 创建分割模型 初始化参数
	pcl::SACSegmentation<pcl::PointXYZ> seg;// RANSAC segmentation model 随机序列采样分割模型
	pcl::PointIndices::Ptr inliers (new pcl::PointIndices);//分割出的点云的 索引初始化
	pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);//平面模型参数
	seg.setOptimizeCoefficients (true);// 也同时优化模型参数
	seg.setModelType (pcl::SACMODEL_PLANE);//分割模型为 平面
	seg.setMethodType (pcl::SAC_RANSAC);// 方法为 随机序列采样算法
	seg.setMaxIterations (plane_max_iter);//最大迭代次数    100
	seg.setDistanceThreshold (plane_dist_thresh);//距离阈值  0.01  这两个参数 确定平面切合程度
	// Segment the largest planar component from the cropped cloud
	seg.setInputCloud (cropped_cloud);//输入点云
	seg.segment (*inliers, *coefficients);//进行分割，存储分割后的点云 的 索引
	if (inliers->indices.size () == 0)//未找到符合平面的点云
	{
	  ROS_WARN_STREAM ("Could not estimate a planar model for the given dataset.") ;
	  //break;
	}

	// 从输入点云中按照索引提取 平面的内点
	pcl::ExtractIndices<pcl::PointXYZ> extract;
	extract.setInputCloud (cropped_cloud);//源 滤波后的点云
	extract.setIndices(inliers);//索引
	extract.setNegative (false);
	// Get the points associated with the planar surface
	extract.filter (*cloud_plane);//得到 平面内的点云
	ROS_INFO_STREAM("PointCloud representing the planar component: " << cloud_plane->points.size () << " data points." );

	// 去除平面点云, 提取剩余的点云 平面桌子上的 物品
	extract.setNegative (true);
	extract.filter (*cloud_f);

  /* ========================================
   * Fill Code: 发布平面可视化 PLANE MARKER (OPTIONAL)
   * ========================================*/


//===================【11】=====================
//Fill Code: EUCLIDEAN CLUSTER EXTRACTION (OPTIONAL/RECOMMENDED)
// 欧式距离分类EuclideanClusterExtraction
// 区域生长算法实现的分割    分割不同群体点云的物体
// https://www.cnblogs.com/yhlx125/p/5161186.html
// 欧几里得与区域生长算法
//http://blog.csdn.net/fandq1223/article/details/53176101?locationNum=10&fps=1
// 基于欧式距离的分割和基于区域生长的分割本质上都是用区分邻里关系远近来完成的。
// 欧几里得算法使用邻居之间距离作为判定标准，
// 而区域生长算法则利用了法线，曲率，颜色（RGB点云）等信息来判断点云是否应该聚成一类。
// 都是从一个点出发，最终占领整个被分割区域。毛主席说：“星星之火，可以燎原” 就是这个意思
//========================================

// Creating the KdTree object for the search method of the extraction
	pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);//存储结构 搜索树
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZ>);//需要分类分类的源点云
	*cloud_filtered = *cloud_f;//桌子上的物体点云 多个物体
	tree->setInputCloud (cloud_filtered);//搜索这些点云

	std::vector<pcl::PointIndices> cluster_indices;//索引容器
	pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;// 欧式距离分类 算法模型
	ec.setClusterTolerance (cluster_tol); // 距离聚类容差 2cm
	ec.setMinClusterSize (cluster_min_size);// minimam number
	ec.setMaxClusterSize (cluster_max_size);// maxmam number
	ec.setSearchMethod (tree);//search method
	ec.setInputCloud (cloud_filtered);//
	ec.extract (cluster_indices);

	std::vector<sensor_msgs::PointCloud2::Ptr> pc2_clusters;
	std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr > clusters;
	for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin (); it != cluster_indices.end (); ++it)
	{
	  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cluster (new pcl::PointCloud<pcl::PointXYZ>);
	  for (std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); pit++)
	    cloud_cluster->points.push_back(cloud_filtered->points[*pit]);
	  cloud_cluster->width = cloud_cluster->points.size ();
	  cloud_cluster->height = 1;
	  cloud_cluster->is_dense = true;
	  std::cout << "Cluster has " << cloud_cluster->points.size() << " points.\n";
	  clusters.push_back(cloud_cluster);
	  sensor_msgs::PointCloud2::Ptr tempROSMsg(new sensor_msgs::PointCloud2);
	  pcl::toROSMsg(*cloud_cluster, *tempROSMsg);
	  pc2_clusters.push_back(tempROSMsg);
	}


//========================================
//====================【12】统计学滤波剔除外点========== (可选)
//========= 球半径滤波也可以剔除离群点=======
//========================================*/
	pcl::PointCloud<pcl::PointXYZ>::Ptr cluster_cloud_ptr= clusters.at(0);
	pcl::PointCloud<pcl::PointXYZ>::Ptr sor_cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
	sor.setInputCloud (cluster_cloud_ptr);
	sor.setMeanK (50);
	sor.setStddevMulThresh (1.0);
	sor.filter(*sor_cloud_filtered);



//================================================================
//===================【13】 PCL点云格式 转换到 ROS点云格式 ==========
//  PCL->ROS方便发布点云数据 提供RVIZ可视化显示

	sensor_msgs::PointCloud2::Ptr pc2_cloud (new sensor_msgs::PointCloud2);//ROS点云格式
	// pcl::toROSMsg(*cloud_ptr, );
	//pcl::toROSMsg(*cloud_voxel_filtered,*pc2_cloud);//体素滤波后
	//pcl::toROSMsg(zf_cloud,*pc2_cloud);//体素 + xyz轴直通 滤波后
	//pcl::toROSMsg(xyz_filtered_cloud,*pc2_cloud);//体素 + Box 滤波后 equal to xyz轴直通
	pcl::toROSMsg(*cloud_f,*pc2_cloud);//体素 + xyz轴直通滤波 + 平面分割后得到的桌子上的all物体
	//pcl::toROSMsg(*cloud_plane,*pc2_cloud);//体素 + xyz轴直通滤波 + 平面分割后得到的桌子平面
	pc2_cloud->header.frame_id=world_frame;//坐标系
	pc2_cloud->header.stamp=ros::Time::now();//时间戳
	object_pub.publish(pc2_cloud);//发布点云

	pcl::toROSMsg(*cloud_voxel_filtered,*pc2_cloud);//体素滤波后
	pc2_cloud->header.frame_id=world_frame;//坐标系
	pc2_cloud->header.stamp=ros::Time::now();//时间戳
	VoxelG_fil_pub.publish(pc2_cloud);

	pcl::toROSMsg(zf_cloud,*pc2_cloud);//体素 + xyz轴直通 滤波后
	pc2_cloud->header.frame_id=world_frame;//坐标系
	pc2_cloud->header.stamp=ros::Time::now();//时间戳
	pt_filter_pub.publish(pc2_cloud);

	pcl::toROSMsg(*cloud_plane,*pc2_cloud);//体素 + xyz轴直通滤波 + 平面分割后得到的桌子平面
	pc2_cloud->header.frame_id=world_frame;//坐标系
	pc2_cloud->header.stamp=ros::Time::now();//时间戳
	plane_pub.publish(pc2_cloud);

	pcl::toROSMsg(*(clusters.at(0)),*pc2_cloud);//体素 + xyz轴直通滤波 + 平面分割后得到的桌子 one obj
	pc2_cloud->header.frame_id=world_frame;//坐标系
	pc2_cloud->header.stamp=ros::Time::now();//时间戳
	one_object_pub.publish(pc2_cloud);

	pcl::toROSMsg(*sor_cloud_filtered,*pc2_cloud);//体素 + xyz轴直通滤波 + 平面分割后得到的桌子 上的一个物体 滤出外点后
	pc2_cloud->header.frame_id=world_frame;//坐标系
	pc2_cloud->header.stamp=ros::Time::now();//时间戳
	one_object_pub.publish(pc2_cloud);

//================================================================
//===================【14】发布其他可视化MARKERS (可选)==============
//=================================================================


//=================================================================
//====================【15】广播坐标变换 (可选)======================
//=================================================================
	static tf::TransformBroadcaster br;//坐标变换广播
	tf::Transform part_transform;//

	//Here in the tf::Vector3(x,y,z) x,y, and z should be calculated based on the pointcloud filtering results
	part_transform.setOrigin( tf::Vector3(sor_cloud_filtered->at(1).x, sor_cloud_filtered->at(1).y, sor_cloud_filtered->at(1).z) );
	tf::Quaternion q;
	q.setRPY(0, 0, 0);
	part_transform.setRotation(q);
	// 广播这个坐标变换
	br.sendTransform(tf::StampedTransform(part_transform, ros::Time::now(), world_frame, "part"));
//============================================================
//=========【16】多边形分割 POLYGONAL SEGMENTATION (OPTIONAL)============
//==============================================================

	pcl::PointCloud<pcl::PointXYZ>::Ptr sensor_cloud_ptr (new pcl::PointCloud<pcl::PointXYZ>(cloud));//源传感器发布的点云
	pcl::PointCloud<pcl::PointXYZ>::Ptr prism_filtered_cloud (new pcl::PointCloud<pcl::PointXYZ>);//多边形滤波后的点云
	pcl::PointCloud<pcl::PointXYZ>::Ptr pick_surface_cloud_ptr(new pcl::PointCloud<pcl::PointXYZ>);//表面点云
	// 多边形滤波器
	pcl::ExtractPolygonalPrismData<pcl::PointXYZ> prism;
	pcl::ExtractIndices<pcl::PointXYZ> extract_ind;//提取的 索引
	prism.setInputCloud(sensor_cloud_ptr);//输入点云
	pcl::PointIndices::Ptr pt_inliers (new pcl::PointIndices());//

	// create prism 多边形 surface
	double box_length=0.25;
	double box_width=0.25;
	pick_surface_cloud_ptr->width = 5;
	pick_surface_cloud_ptr->height = 1;
	pick_surface_cloud_ptr->points.resize(5);

	pick_surface_cloud_ptr->points[0].x = 0.5f*box_length;
	pick_surface_cloud_ptr->points[0].y = 0.5f*box_width;
	pick_surface_cloud_ptr->points[0].z = 0;

	pick_surface_cloud_ptr->points[1].x = -0.5f*box_length;
	pick_surface_cloud_ptr->points[1].y = 0.5f*box_width;
	pick_surface_cloud_ptr->points[1].z = 0;

	pick_surface_cloud_ptr->points[2].x = -0.5f*box_length;
	pick_surface_cloud_ptr->points[2].y = -0.5f*box_width;
	pick_surface_cloud_ptr->points[2].z = 0;

	pick_surface_cloud_ptr->points[3].x = 0.5f*box_length;
	pick_surface_cloud_ptr->points[3].y = -0.5f*box_width;
	pick_surface_cloud_ptr->points[3].z = 0;

	pick_surface_cloud_ptr->points[4].x = 0.5f*box_length;
	pick_surface_cloud_ptr->points[4].y = 0.5f*box_width;
	pick_surface_cloud_ptr->points[4].z = 0;

	Eigen::Affine3d eigen3d;
	tf::transformTFToEigen(part_transform,eigen3d);
	pcl::transformPointCloud(*pick_surface_cloud_ptr,*pick_surface_cloud_ptr,Eigen::Affine3f(eigen3d));

	prism.setInputPlanarHull( pick_surface_cloud_ptr);
	prism.setHeightLimits(-10,10);

	prism.segment(*pt_inliers);//Segment:
	extract_ind.setInputCloud(sensor_cloud_ptr);
	extract_ind.setIndices(pt_inliers);

	extract_ind.setNegative(true);

	extract_ind.filter(*prism_filtered_cloud);

	pcl::toROSMsg(*prism_filtered_cloud,*pc2_cloud);//体素 + xyz轴直通滤波 + 平面分割后得到的桌子 上的一个物体 滤出外点后
	pc2_cloud->header.frame_id=world_frame;//坐标系
	pc2_cloud->header.stamp=ros::Time::now();//时间戳
	prism_fil_pub.publish(pc2_cloud);

  }
  return 0;
}
