/*
目标识别服务节点
*/
#ifdef __i386__
  #pragma message("i386 Architecture detected, disabling EIGEN VECTORIZATION")
  #define EIGEN_DONT_VECTORIZE
  #define EIGEN_DISABLE_UNALIGNED_ARRAY_ASSERT
#else
  #pragma message("64bit Architecture detected, enabling EIGEN VECTORIZATION")
#endif



#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud_conversion.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/conversions.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/passthrough.h>//　直通滤波器 PassThrough　　
#include <pcl/filters/statistical_outlier_removal.h>// 统计滤波器
#include <pcl/filters/radius_outlier_removal.h>// 球半径滤波器
#include <pcl/filters/extract_indices.h>// 从一个点云中提取索引
#include <pcl/common/centroid.h>
#include <pcl_ros/transforms.h>
#include <pcl/segmentation/extract_polygonal_prism_data.h>
#include <pcl/surface/convex_hull.h>
#include <pcl/common/common.h>
#include <boost/make_shared.hpp>
#include <tf_conversions/tf_eigen.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include <collision_avoidance_pick_and_place/GetTargetPose.h>
#include <math.h>

// alias
typedef pcl::PointCloud<pcl::PointXYZ> Cloud;
typedef boost::shared_ptr<tf::TransformListener> TransformListenerPtr;

// constants
const std::string SENSOR_CLOUD_TOPIC = "sensor_cloud";//源点云　发布话题
const std::string FILTERED_CLOUD_TOPIC = "filtered_cloud";//　统计学滤波后点云　发布话题
const std::string TARGET_RECOGNITION_SERVICE = "target_recognition";//目标识别服务器名字

// defaults
const double BOX_SCALE = 1.2f;
const double ANGLE_TOLERANCE = 0.1f* (M_PI/180.0f);

using namespace std;
class TargetRecognition
{
public:
	// 默认构造函数
	TargetRecognition():
		box_filter_scale_(BOX_SCALE),
		angle_tolerance_(ANGLE_TOLERANCE)
	{

	}
	// 默认析构函数
	~TargetRecognition()
	{

	}
        // 初始化函数	
	bool init()
	{
		ros::NodeHandle nh;
		ros::NodeHandle ph("~");//私有节点　获取点云滤波参数
		///*
		// read parameters 滤波器参数
		if(ph.getParam("box_filter_scale",box_filter_scale_) &&
				ph.getParam("angle_tolerance",angle_tolerance_))
		{
			ROS_INFO_STREAM("target recognition read parameters successfully");
		}
		else
		{
			ROS_WARN_STREAM("target recognition did not find one or more parameters, using defaults");
		}
                
		// 初始化　目标识别　服务器initializing service server
		target_detection_server = nh.advertiseService(TARGET_RECOGNITION_SERVICE,&TargetRecognition::target_recognition_callback,this);
//　服务名子　　　　　　　　　　　　服务回调函数                     类this指针

		// 初始化　点云滤波　发布话题　initializing publisher
		filtered_cloud_publisher = nh.advertise<sensor_msgs::PointCloud2>(FILTERED_CLOUD_TOPIC,1);

		// 初始化　传感器点云订阅话题　initializing subscriber
		point_cloud_subscriber = nh.subscribe(SENSOR_CLOUD_TOPIC,1,&TargetRecognition::point_cloud_callback,this);
                // 要订阅的话题名           订阅回调函数			　类this指针

		// 初始化坐标变换监听　initializing transform listener
		 transform_listener_ptr = TransformListenerPtr(new tf::TransformListener(nh,ros::Duration(1.0f)));

		// 初始化　发布点云消息　ros接口　滤波点云　initializing cloud messages
		filtered_cloud_msg_ = sensor_msgs::PointCloud2();
		
		return true;

	}

	void run()
	{
		cout << "target_recognition_running ５hz ..." << endl;
		while(ros::ok())
		{
			ros::Duration(0.2f).sleep();
			ros::spinOnce();
		}
	}

protected:

	bool grab_sensor_snapshot(sensor_msgs::PointCloud2& msg)
	{
		// grab sensor data snapshot
		ros::NodeHandle nh;
		sensor_msgs::PointCloud2ConstPtr msg_ptr =
				ros::topic::waitForMessage<sensor_msgs::PointCloud2>(SENSOR_CLOUD_TOPIC,nh,
						ros::Duration(5.0f));

		// check for empty message
		if(msg_ptr != sensor_msgs::PointCloud2ConstPtr())
		{

			msg = *msg_ptr;
		}

		return msg_ptr != sensor_msgs::PointCloud2ConstPtr();
	}
// 订阅传感器点云发布话题的　回调函数
	void point_cloud_callback(const sensor_msgs::PointCloud2ConstPtr msg)
	{
		sensor_cloud_msg_ = sensor_msgs::PointCloud2(*msg);
	}

// 服务回调函数　
	bool target_recognition_callback(
		collision_avoidance_pick_and_place::GetTargetPose::Request& req,// 请求
		collision_avoidance_pick_and_place::GetTargetPose::Response& res)//　响应
	{
		// transforms 坐标变换
		tf::StampedTransform world_to_sensor_tf;// 世界到　点云捕获的相机的位姿变换
		tf::Transform        world_to_box_pick_tf;//　世界到　目标点云(箱子)　的位姿变换
		tf::Transform        world_to_ar_tf;//世界到假的AR块　的位姿变换
		tf::Vector3          box_pick_position;//　目标点云(箱子)的位置

		// 更新全局变量　updating global variables
		box_length_ = req.shape.dimensions[0];//　目标块长度
		box_width_  = req.shape.dimensions[1];//　目标块宽度
		box_height_ = req.shape.dimensions[2];//　目标块高度
		world_frame_id_ = req.world_frame_id;//　世界坐标系id 
		ar_frame_id_    = req.ar_tag_frame_id;//　ar块坐标id

		// 得到源传感器点云消息　get point cloud message
		sensor_msgs::PointCloud2 msg = sensor_cloud_msg_;
		if(msg.data.size() == 0)//点云大小为0
		{
		   ROS_ERROR_STREAM("Cloud message is invalid, returning detection failure");
		   res.succeeded = false;
		   return true;
		}

		// 得到坐标变换　looking up transforms
		if(get_transform(world_frame_id_ , ar_frame_id_,        world_to_ar_tf) &&
		   get_transform(world_frame_id_,  msg.header.frame_id, world_to_sensor_tf))
		{

			// 转换 ros pcl消息　到　pcl 点云　convert from message to point cloud
			Cloud::Ptr sensor_cloud_ptr(new Cloud());
			//pcl::PointCloud<pcl::PointXYZ>::Ptr sensor_cloud_ptr (new pcl::PointCloud<pcl::PointXYZ>);
			pcl::fromROSMsg<pcl::PointXYZ>(msg, *sensor_cloud_ptr);//　获取目标点云

			// 统计学滤波去除外点　离群点　applying statistical removal　
			// 在这之前如果点云较密集　可先进行　体素格滤波下采样　
			// 在降低数量的同时　保证　点云的形状
			pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
		        // 创建滤波器对象　一创建就段错误 pcl版本没有支持c++11 
			// pcl::RadiusOutlierRemoval<pcl::PointXYZ> outrem; // 也发生段错误
			sor.setInputCloud (sensor_cloud_ptr);//设置待滤波的点云 　指针
			sor.setMeanK (50);
                        //设置在进行统计时考虑查询点邻近点数 每个点分析的临近点个数设为50
			sor.setStddevMulThresh (1.0);//设置判断是否为离群点的阈值
// 将标准差倍数设为1，如一个点的距离超出平均距离一个标准差以上，则该点被标记为离群点，并将被移除。
			sor.filter(*sensor_cloud_ptr);


			// creating cloud message for publisher
			Cloud::Ptr filtered_cloud_ptr(new Cloud());// 滤波后的点云

			// 转换到世界坐标系（点云原先在相机传感器坐标系下）
			Eigen::Affine3d eigen_3d;// 仿射变换
			tf::transformTFToEigen(world_to_sensor_tf, eigen_3d);
			Eigen::Affine3f eigen_3f(eigen_3d);
			pcl::transformPointCloud(*sensor_cloud_ptr, *sensor_cloud_ptr, eigen_3f);

			// find height of box top surface
			// 检测 盒子 上表面的 高度
			double height;
			if(!detect_box_height(*sensor_cloud_ptr, world_to_ar_tf, height) )
			{
				ROS_ERROR_STREAM("Target height detection failed");
				res.succeeded = false;
				return true;
			}

			// 更新抓取位置信息 updating box pick transform
			world_to_box_pick_tf = world_to_ar_tf;
			box_pick_position    = world_to_box_pick_tf.getOrigin();
			box_pick_position.setZ(height);
			world_to_box_pick_tf.setOrigin(box_pick_position);

			// 执行盒子滤波 分割平面上的点云  filtering box from sensor cloud
			filter_box(world_to_sensor_tf,
					world_to_box_pick_tf,*sensor_cloud_ptr,*filtered_cloud_ptr);

			// 多次滤波  filter box at requested locations
			for(unsigned int i =0;i < req.remove_at_poses.size();i++)
			{
				tf::Transform world_to_box;
				tf::poseMsgToTF(req.remove_at_poses[i],world_to_box);

				// copying last filter cloud
				pcl::copyPointCloud(*filtered_cloud_ptr, *sensor_cloud_ptr);

				// filter box from last computed cloud
				filter_box(world_to_sensor_tf,world_to_box, *sensor_cloud_ptr, *filtered_cloud_ptr);
			}

			// transforming to world frame
		        pcl::transformPointCloud(*filtered_cloud_ptr, *filtered_cloud_ptr, eigen_3f.inverse());

			// 转换点云消息类型 converting to message
			filtered_cloud_msg_ = sensor_msgs::PointCloud2();
			pcl::toROSMsg(*filtered_cloud_ptr, filtered_cloud_msg_);

			// 位姿 populating response
			tf::poseTFToMsg(world_to_box_pick_tf, res.target_pose);
			res.succeeded = true ;

			// 发布点云 publishing cloud
			filtered_cloud_msg_.header.stamp = ros::Time::now()-ros::Duration(0.5f);
			filtered_cloud_publisher.publish(filtered_cloud_msg_);

		}
		else
		{
			res.succeeded = false;
		}


		return true;
	}

//　获取两个坐标系的　坐标变换
	bool get_transform(std::string target,std::string source,tf::Transform& trg_to_src_tf)
	{
		// create tf listener and broadcaster
		//static tf::TransformListener tf_listener;
		tf::StampedTransform trg_to_src_stamped;

		// find ar tag transform
		try
		{       // 等待坐标时间戳同步进行转换
			transform_listener_ptr->waitForTransform(target,
								 source,
								 ros::Time::now(),
							         ros::Duration(4.0f));
		        // 得到坐标变换
			transform_listener_ptr->lookupTransform(target,
								source,
								ros::Time::now()-ros::Duration(0.2f),
								trg_to_src_stamped);
			// 复制坐标变换消息　copying transform data
			trg_to_src_tf.setRotation( trg_to_src_stamped.getRotation());//选择矩阵
			trg_to_src_tf.setOrigin(trg_to_src_stamped.getOrigin());// 平移向量

		}
		catch(tf::LookupException &e)
		{
			ROS_ERROR_STREAM("transform lookup for '"<<ar_frame_id_<<"' failed");
			return false;
		}
		catch(tf::ExtrapolationException &e)
		{
			ROS_ERROR_STREAM("transform lookup for '"<<ar_frame_id_<<"' failed");
			return false;
		}
		catch(tf::TransformException &e)
		{
			ROS_ERROR_STREAM("transform lookup for '"<<ar_frame_id_<<"' failed");
			return false;
		}

		return true;
	}

// 检测点云的　高度等尺寸  计算点云质心 
// 检测 盒子 上表面的 高度 （引用形参返回）
	bool detect_box_height(const Cloud& cloud,
			const tf::Transform &world_to_ar_tf,double& height )
	{
		// 点云对象　cloud objects
		Cloud::Ptr cloud_ptr= boost::make_shared<Cloud>(cloud);//拷贝初始化
		Cloud::Ptr filtered_cloud_ptr(new Cloud);//滤波后的对象

		// x轴滤波　applying filter in x axis
		float min = world_to_ar_tf.getOrigin().x() - 0.2f*box_length_;//ｘ轴上保留的范围
		float max = world_to_ar_tf.getOrigin().x() + 0.2f*box_length_;
		pcl::PassThrough<pcl::PointXYZ> filter;//直通滤波器　保留指定轴上知道范围内的点云
		filter.setInputCloud(cloud_ptr);
		filter.setFilterFieldName("x");
		filter.setFilterLimits(min,max);
		filter.filter(*filtered_cloud_ptr);

		//  y轴滤波 applying filter in y axis
		min = world_to_ar_tf.getOrigin().y()- 0.2f*box_width_;
		max = world_to_ar_tf.getOrigin().y() + 0.2f*box_width_;;
		filter.setInputCloud(filtered_cloud_ptr);
		filter.setFilterFieldName("y");
		filter.setFilterLimits(min,max);
		filter.filter(*filtered_cloud_ptr);

		// 计算点云质心computing centroid
		Eigen::Vector4f centroid;//点云质心坐标
		int count = pcl::compute3DCentroid(*filtered_cloud_ptr,centroid);
		height = centroid[2];

		ROS_INFO_STREAM("Detected height is: "<<centroid[2]);

		// return z value
		return count != 0;
	}

// 盒子滤波器
// 执行盒子滤波 
	void filter_box(const tf::Transform& world_to_sensor_tf,
		        const tf::Transform& world_to_box_pick_tf,
 			const Cloud &sensor_cloud,
			Cloud& filtered_cloud)
        {
		// creating surface with center at box pick location
		Cloud::Ptr pick_surface_cloud_ptr(new Cloud());

		// 表面点云 用于分割　adding surface points to cloud
		pick_surface_cloud_ptr->width = 5;//5个点
		pick_surface_cloud_ptr->height = 1;
		pick_surface_cloud_ptr->points.resize(5);

		pick_surface_cloud_ptr->points[0].x = 0.5f*box_filter_scale_*box_length_;
		pick_surface_cloud_ptr->points[0].y = 0.5f*box_filter_scale_*box_width_;
		pick_surface_cloud_ptr->points[0].z = 0;

		pick_surface_cloud_ptr->points[1].x = -0.5f*box_filter_scale_*box_length_;
		pick_surface_cloud_ptr->points[1].y = 0.5f*box_filter_scale_*box_width_;
		pick_surface_cloud_ptr->points[1].z = 0;

		pick_surface_cloud_ptr->points[2].x = -0.5f*box_filter_scale_*box_length_;
		pick_surface_cloud_ptr->points[2].y = -0.5f*box_filter_scale_*box_width_;
		pick_surface_cloud_ptr->points[2].z = 0;

		pick_surface_cloud_ptr->points[3].x = 0.5f*box_filter_scale_*box_length_;
		pick_surface_cloud_ptr->points[3].y = -0.5f*box_filter_scale_*box_width_;
		pick_surface_cloud_ptr->points[3].z = 0;

		pick_surface_cloud_ptr->points[4].x = 0.5f*box_filter_scale_*box_length_;
		pick_surface_cloud_ptr->points[4].y = 0.5f*box_filter_scale_*box_width_;
		pick_surface_cloud_ptr->points[4].z = 0;

		ROS_INFO_STREAM("Points in surface: "<<pick_surface_cloud_ptr->points.size());

		// 箱子z轴和世界坐标系Z轴夹角　finding angle between world z and box z vectors
		tf::Vector3 z_world_vect(0,0,1);//z轴向量
		tf::Vector3 z_box_vect = world_to_box_pick_tf.getBasis().getColumn(2);//箱子z轴向量
		double angle  = z_world_vect.angle(z_box_vect);
		ROS_INFO_STREAM("Angle between z vectors : "<<angle);


		// transforming cloud to match orientation of box (rectify in the z direction)
		tf::Vector3 axis = z_world_vect.cross(z_box_vect);//向量叉乘
		tf::Transform world_to_rectified_box_tf = world_to_box_pick_tf;
		Eigen::Affine3d eigen3d;
		if(std::abs(angle) > angle_tolerance_)
		{
			ROS_INFO_STREAM("Rectifying pick pose z direction");
			world_to_rectified_box_tf.setRotation(world_to_box_pick_tf.getRotation() * tf::Quaternion(axis,-angle)) ;
		}

		tf::transformTFToEigen(world_to_rectified_box_tf,eigen3d);
		pcl::transformPointCloud(*pick_surface_cloud_ptr,*pick_surface_cloud_ptr,Eigen::Affine3f(eigen3d));


		// extracting points in extruded volume
		Cloud::Ptr sensor_cloud_ptr = boost::make_shared<Cloud>(sensor_cloud);
		// 该类用于分割出棱柱模型内部的点集
		pcl::ExtractPolygonalPrismData<pcl::PointXYZ> prism;//分割
		pcl::ExtractIndices<pcl::PointXYZ> extract;//提取
		pcl::PointIndices::Ptr inliers(new pcl::PointIndices());

		ROS_INFO_STREAM("Sensor cloud points: "<<sensor_cloud_ptr->points.size());

		tf::Vector3 viewpoint = world_to_sensor_tf.getOrigin();
		prism.setInputCloud(sensor_cloud_ptr);
		prism.setInputPlanarHull( pick_surface_cloud_ptr);// 设置平面模型的点集
		prism.setHeightLimits(-10,10);// 设置高度范围
		//prism.setViewPoint(viewpoint.x(),viewpoint.y(),viewpoint.z());
		prism.segment(*inliers);// 分割 得到 index

		//pcl::copyPointCloud(*sensor_cloud_ptr,indices.indices,filtered_cloud);
		// extracting remaining points
		extract.setInputCloud(sensor_cloud_ptr);
		extract.setIndices(inliers);// 按照index 提取点云
		extract.setNegative(true);
		extract.filter(filtered_cloud);

		ROS_INFO_STREAM("Filtered cloud points: "<<filtered_cloud.points.size());

        }

protected:

	// members
	std::string ar_frame_id_;
	std::string world_frame_id_;
	float box_width_;
	float box_length_;
	float box_height_;
	sensor_msgs::PointCloud2 filtered_cloud_msg_;
	sensor_msgs::PointCloud2 sensor_cloud_msg_;

        //pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;// 创建滤波器对象 已创建就　段错误？？

	// ros parameter
	double box_filter_scale_;
	double angle_tolerance_;

	// ros service server
	ros::ServiceServer target_detection_server;

	// ros subscriber
	ros::Subscriber point_cloud_subscriber;

	// ros publishers and subscribers
	ros::Publisher filtered_cloud_publisher;

	// transform listener
	TransformListenerPtr transform_listener_ptr;

};

// main program
int main(int argc,char** argv)
{
	cout << "init target_recognition_node" << endl;
	ros::init(argc,argv,"target_recognition_node");
	cout << "creat TargetRecognition  obj" << endl;
	TargetRecognition tg;
        //pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
	//pcl::PassThrough<pcl::PointXYZ> pass;
	if(tg.init());
	{
                cout << "target_recognition obj init ok" << endl;
		tg.run();
	}

	return 0;
}
