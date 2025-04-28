#include <omp.h>
#include <mutex>
#include <condition_variable>
#include <math.h>
#include <thread>
#include <fstream>
#include <csignal>
#include <unistd.h>
#include <queue>
// #include <so3_math.h>
#include <ros/ros.h>
#include <Eigen/Core>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <visualization_msgs/Marker.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/passthrough.h>
#include <pcl/registration/gicp.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/filters/passthrough.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_broadcaster.h>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/Transform.h>
#include <stdio.h>
#include <stdlib.h>
#include "backward.hpp"
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <sensor_msgs/NavSatFix.h>
#include <GeographicLib/Geocentric.hpp> 
#include <GeographicLib/LocalCartesian.hpp>
#include <octomap/octomap.h>
#include <octomap_msgs/Octomap.h>
#include <octomap_msgs/conversions.h>
#include <std_msgs/Int32MultiArray.h>
#include <std_msgs/MultiArrayDimension.h>
#include <cmath> 

#define BACKWARD_HAS_DW 1
#define MAP_DIR std::getenv("HOME") + std::string("/uav_ws/output")

// 三维数组类型定义 (x, y, z -> 0=Free, 1=Occupied)
typedef std::vector<std::vector<std::vector<int>>> Grid3D;

namespace backward {
	backward::SignalHandling sh;
}

std::queue<sensor_msgs::NavSatFix> gps_buf;
std::queue<sensor_msgs::PointCloud2> pointCloud_buf; 
// std::queue<std::pair<sensor_msgs::PointCloud2, sensor_msgs::NavSatFix>> sensors_buf; 
std::mutex m_buf;

struct Pose
{
  float x;
	float y;
	float z;
  float roll;
  float pitch;
  float yaw;
	float q_w;
	float q_x;
	float q_y;
	float q_z;
};

class LocalMap
{
  public:

  LocalMap();
  
  // void sensorsCallback(const sensor_msgs::PointCloud2::ConstPtr &pcl_msg, const sensor_msgs::NavSatFix::ConstPtr &gps_msg);
  
  void pointCloudCallback(const sensor_msgs::PointCloud2ConstPtr &msg);

  void gpsCallBack(const sensor_msgs::NavSatFixConstPtr &msg);

  void process();

  void saveLocalMap();

  void alignByIcp(pcl::PointCloud<pcl::PointXYZI>::Ptr &target_cloud, pcl::PointCloud<pcl::PointXYZI>::Ptr &source_cloud, Eigen::Affine3f &T);
  
  void publishGrid3D(const Grid3D& grid, int size_x, int size_y, int size_z);
  
  void saveGrid3D(const Grid3D& grid, const std::string& path);

  ros::NodeHandle nh;
  ros::Publisher pub_localMap;
  ros::Publisher pub_Grid;
  ros::Publisher pub_gps;
  ros::Subscriber sub_pcl;
  ros::Subscriber sub_gps;

  pcl::PointCloud<pcl::PointXYZI>::Ptr localMap;
  Eigen::Vector3d init_lla;
  bool first_gps_flag = 1;
  bool first_pcl_flag = 1;
  Eigen::Affine3f last_pose;
  nav_msgs::Path gps_path;

  // 配置参数
  float fitness_score_threshold_;
  float max_Correspondence_Distance_;
  double blindSpot_;
  bool save_to_file_;
  std::string file_path_;
  double octomap_resolution_;
  double octomap_size_x_;
  double octomap_size_y_;
  double octomap_size_z_;
  // **********************滤波器的参数***********************
  // 统计滤波参数调节建议：
  // ​​MeanK（邻域点数）​​
  // ​​场景依赖​​：室内场景建议20-50，室外稀疏点云可增至50-100
  // ​​过小​​（<20）：可能误判密集区域的正常点为噪声
  // ​​过大​​（>100）：计算量激增，适合高密度激光雷达数据（如128线雷达）
  // ​​调试技巧​​：通过可视化观察噪声分布，选择覆盖典型噪声区域的最小邻域
  // ​​StddevMulThresh（标准差乘数）​​
  // ​​典型范围​​：1.0-3.0（当前0.5偏严格，可能过度删除有效点）
  // ​​计算公式​​：阈值 = 平均距离 + 标准差 × 乘数
  // ​​动态调整​​：建议从2.0开始，逐步降低至1.0直到噪声点消失
  // ​​特殊场景​​：针对传感器噪声特性（如TOF相机脉冲噪声）可设为3.0+

  double MeanK; //​​（邻域点数）​​
  double StddevMulThresh; //（标准差乘数）​​

  // 半径滤波参数调节建议：
  // ​​RadiusSearch（搜索半径）​​
  // ​​尺度匹配​​：室内场景0.3-1.0m，城市道路2-5m，森林环境3-8m
  // ​​传感器关联​​：16线雷达建议1.5-3m，固态激光雷达可缩小至0.5-1m
  // ​​动态调整​​：取点云平均间距的3-5倍（可通过KDTree计算局部密度）
  // ​​MinNeighbors（最小邻居数）​​
  // ​​密度基准​​：典型值=半径内预期点数×安全系数（建议0.7-0.9）
  // ​​场景示例​​：
  // 城市道路：5-10（当前参数偏小）
  // 室内场景：3-5
  // 植被区域：8-15
  // ​​特殊处理​​：对移动物体区域可适当放宽，避免滤除动态目标

  double RadiusSearch; // 搜索半径
  double MinNeighbors; // 最小邻居数

  // 参数调节建议：
  // ​​LeafSize（体素尺寸）​​
  // ​​分辨率平衡​​：
  // 高精度需求（SLAM建图）：0.1-0.3m
  // 实时处理（自动驾驶）：0.3-0.5m
  // 大规模场景可视化：0.5-2.0m
  // ​​各向异性调节​​：
  // 地面车辆：可设Z轴0.1m保持高程细节
  // 无人机：XYZ均匀设置
  // ​​传感器适配​​：
  // 机械式雷达：建议0.2-0.4m
  // 固态激光雷达：0.1-0.2m
  // ​​降采样策略​​：
  // ​​分层处理​​：先粗采样（0.5m）去噪，后细采样（0.2m）保特征
  // ​​动态调整​​：根据处理阶段调整（前端匹配用细采样，后端优化用粗采样）
  // 体素滤波（降采样）

  double LeafSize; // 体素尺寸


};

LocalMap::LocalMap()
{
  // icp
  nh.param<float>("/fitness_score_threshold", fitness_score_threshold_, 0.4);
  nh.param<float>("/max_Correspondence_Distance", max_Correspondence_Distance_, 0.5);
  // octomap
  nh.param<double>("octomap/blindSpot", blindSpot_, 2.5);
  nh.param<double>("octomap/octomap_resolution", octomap_resolution_, 1.0);
  nh.param<double>("octomap/octomap_size_x", octomap_size_x_, 40.0);
  nh.param<double>("octomap/octomap_size_y", octomap_size_y_, 40.0);
  nh.param<double>("octomap/octomap_size_z", octomap_size_z_, 20.0);
  // 3dGrid
  nh.param<bool>("Grid/save_to_file", save_to_file_, true);
  nh.param<std::string>("Grid/file_path", file_path_, std::string("grid3d.bin"));

  // 滤波参数
  nh.param<double>("Filter/MeanK", MeanK, 50);
  nh.param<double>("Filter/StddevMulThresh", StddevMulThresh, 2.0);
  nh.param<double>("Filter/RadiusSearch", RadiusSearch, 1);
  nh.param<double>("Filter/MinNeighbors", MinNeighbors, 8);
  nh.param<double>("Filter/LeafSize", LeafSize, 0.1);


  // message_filters::Subscriber<sensor_msgs::PointCloud2> sub_pcl(nh, "/livox/lidar", 1);
  // message_filters::Subscriber<sensor_msgs::NavSatFix> sub_gps(nh, "/gps_topic", 1);
  // typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::PointCloud2, sensor_msgs::NavSatFix> MySyncPolicy;
  // message_filters::Synchronizer<MySyncPolicy> sync(MySyncPolicy(10), sub_pcl, sub_gps);
  // // 设置年龄惩罚因子（值越大越宽松）
  // sync.setAgePenalty(1.0); // 示例值，需根据实际测试调整
  // // 设置第一个话题（PointCloud2）的最小消息间隔为10ms（0.01秒）
  // sync.setInterMessageLowerBound(0,ros::Duration(0.01));     
  // sync.setMaxIntervalDuration(ros::Duration(0.02));
  // sync.registerCallback(boost::bind(&LocalMap::sensorsCallback, this, _1, _2));
  
  sub_pcl = nh.subscribe<sensor_msgs::PointCloud2>("/livox/lidar", 10, &LocalMap::pointCloudCallback, this);
  sub_gps = nh.subscribe<sensor_msgs::NavSatFix>("/gps_topic", 10, &LocalMap::gpsCallBack, this);
  pub_localMap = nh.advertise<sensor_msgs::PointCloud2>("/localMap_", 10 );
  pub_Grid = nh.advertise<std_msgs::Int32MultiArray>("/grid3d_", 10); // 3D栅格
  pub_gps = nh.advertise<nav_msgs::Path>("gps_path", 10);

  localMap = pcl::PointCloud<pcl::PointXYZI>::Ptr(new pcl::PointCloud<pcl::PointXYZI>());
}

// void LocalMap::sensorsCallback(const sensor_msgs::PointCloud2::ConstPtr &pcl_msg, const sensor_msgs::NavSatFix::ConstPtr &gps_msg)
// {
//   std::cout << "sensor callback" << std::endl;
//   if(first_gps_flag)
//   {
//     init_lla[0] = gps_msg->latitude;
//     init_lla[1] = gps_msg->longitude;
//     init_lla[2] = gps_msg->altitude;

//     pcl::fromROSMsg(*pcl_msg, *localMap);

//     first_gps_flag = 0;
//     return;
//   }

//   static uint32_t last_seq = 0;
//   static double last_t = 0.;
//   if (pcl_msg->header.seq <= last_seq)
//   {
//     ROS_WARN("sensor discontinue! stamp:%f %f, seq:%d %d",
//       pcl_msg->header.stamp.toSec(), last_t,
//       pcl_msg->header.seq, last_seq);
//     return;
//   }
//   last_seq = pcl_msg->header.seq;
//   last_t = pcl_msg->header.stamp.toSec();
  
//   std::pair<sensor_msgs::PointCloud2, sensor_msgs::NavSatFix> pair;
//   pair.first = *pcl_msg;
//   pair.second = *gps_msg;

//   std::unique_lock<std::mutex> lock(m_buf);
//   if(sensors_buf.size() < 100) {  // 防止内存溢出
//     sensors_buf.push(pair);
//   }else{
//     ROS_WARN("Queue overflow! Dropping oldest message");
//     sensors_buf.pop();
//     sensors_buf.push(pair);
//   }
//   lock.unlock();
// }

void LocalMap::pointCloudCallback(const sensor_msgs::PointCloud2ConstPtr &msg)
{
  static uint32_t last_seq = 0;
  static double last_pcl_t = 0.;
  if (msg->header.seq <= last_seq)
  {
      ROS_WARN("gps discontinue! stamp:%f %f, seq:%d %d",
               msg->header.stamp.toSec(), last_pcl_t,
               msg->header.seq, last_seq);
      return;
  }
  last_seq = msg->header.seq;
  last_pcl_t = msg->header.stamp.toSec();

  std::unique_lock<std::mutex> lock(m_buf);
  pointCloud_buf.push(*msg);
  lock.unlock();
}

void LocalMap::gpsCallBack(const sensor_msgs::NavSatFixConstPtr &msg)
{
  if(first_gps_flag)
  {
    init_lla[0] = msg->latitude;
    init_lla[1] = msg->longitude;
    init_lla[2] = msg->altitude;
    first_gps_flag = 0;
  }
  
  static uint32_t last_seq = 0;
  static double last_gps_t = 0.;
  if (msg->header.seq <= last_seq)
  {
      ROS_WARN("gps discontinue! stamp:%f %f, seq:%d %d",
               msg->header.stamp.toSec(), last_gps_t,
               msg->header.seq, last_seq);
      return;
  }
  last_seq = msg->header.seq;
  last_gps_t = msg->header.stamp.toSec();

  std::unique_lock<std::mutex> lock(m_buf);
  gps_buf.push(*msg);
  lock.unlock();
}

void LocalMap::alignByIcp(pcl::PointCloud<pcl::PointXYZI>::Ptr &target_cloud, pcl::PointCloud<pcl::PointXYZI>::Ptr &source_cloud, Eigen::Affine3f &T)
{
  Eigen::Matrix4f tf = T.matrix();
  // 精匹配
  pcl::GeneralizedIterativeClosestPoint<pcl::PointXYZI, pcl::PointXYZI> gicp;
	// pcl::IterativeClosestPoint<pcl::PointXYZI, pcl::PointXYZI> icp;
	// 对应点对之间的最大距离
	gicp.setMaxCorrespondenceDistance(max_Correspondence_Distance_);
	// 最大迭代次数
	gicp.setMaximumIterations(50);
	// 两次变化矩阵之间的最小差值	
	gicp.setTransformationEpsilon(1e-8);
	// 欧几里得拟合度阈值
  gicp.setEuclideanFitnessEpsilon(1e-6);
	gicp.setRANSACIterations(2000);
	gicp.setRANSACOutlierRejectionThreshold(0.35);

	// Align clouds
	gicp.setInputSource(source_cloud);
	gicp.setInputTarget(target_cloud);
	pcl::PointCloud<pcl::PointXYZI>::Ptr aligned_cloud(new pcl::PointCloud<pcl::PointXYZI>());
  gicp.align(*aligned_cloud, tf);

  if(gicp.hasConverged())
  {
    std::cout << "ICP converged with score: " << gicp.getFitnessScore() << std::endl;
    // std::cout << "Transformation matrix:\n" << gicp.getFinalTransformation() << std::endl;

    if(gicp.getFitnessScore() < fitness_score_threshold_)
    {
      // 记录上次icp的变换矩阵
      T = gicp.getFinalTransformation();
    }   
  }else{
    std::cerr << "ICP did not converge!" << std::endl;
  }
}

void LocalMap::saveLocalMap()
{
	std::string filename = std::string(MAP_DIR) + std::string("/local_map") + std::string(".pcd");
  int result = pcl::io::savePCDFileASCII(filename, *localMap);
  if (result == -1)
  {
    PCL_ERROR("Couldn't write the PCD file \n");
  }

	std::cout << "Saved " << localMap->points.size() << " data points to " << filename << std::endl;
}

void LocalMap::publishGrid3D(const Grid3D& grid, int size_x, int size_y, int size_z) {
  std_msgs::Int32MultiArray msg;
      
  // 设置x/y/z三个维度 
  msg.layout.dim.push_back(std_msgs::MultiArrayDimension());  
  msg.layout.dim[0].label  = "x";
  msg.layout.dim[0].size  = size_x;
  msg.layout.dim[0].stride  = size_x * size_y * size_z;  // 总元素数 
  
  msg.layout.dim.push_back(std_msgs::MultiArrayDimension()); 
  msg.layout.dim[1].label  = "y";
  msg.layout.dim[1].size  = size_y;
  msg.layout.dim[1].stride  = size_y * size_z;  // 每x层的元素数 
  
  msg.layout.dim.push_back(std_msgs::MultiArrayDimension()); 
  msg.layout.dim[2].label  = "z";
  msg.layout.dim[2].size  = size_z;
  msg.layout.dim[2].stride  = size_z;  // 每行的元素数 

  // 展开三维数组为一维数据 
  for (const auto& slice : grid) {
      for (const auto& row : slice) {
              msg.data.insert(msg.data.end(),  row.begin(),  row.end()); 
          }
      }
  pub_Grid.publish(msg);
}

void LocalMap::saveGrid3D(const Grid3D& grid, const std::string& path) {
  std::ofstream outfile(path, std::ios::binary);
  for (const auto& slice : grid) {
      for (const auto& row : slice) {
          outfile.write(reinterpret_cast<const  char*>(row.data()),  row.size()  * sizeof(int));
      }
  }
  outfile.close(); 
}

void LocalMap::process()
{
  while(ros::ok())
  {
    sensor_msgs::PointCloud2 msg_1;
    sensor_msgs::NavSatFix msg_2;
    std_msgs::Header header;
    double time = 0;
    std::unique_lock<std::mutex> lock(m_buf);
    if(!pointCloud_buf.empty() && !gps_buf.empty())
    {
      double time0 = pointCloud_buf.front().header.stamp.toSec();
      double time1 = gps_buf.front().header.stamp.toSec();
      if(time0 < time1 - 0.1)
      {
        pointCloud_buf.pop();
        // printf("throw pcl\n");
        // std::cout<< "pcl time: " << time0 <<std::endl;
        continue;
      }
      else if(time0 > time1 + 0.1)
      {
        gps_buf.pop();
        // printf("throw gps\n");
        // std::cout<< "gps time: " << time1 <<std::endl;
        continue;
      }
      else
      {
        time = pointCloud_buf.front().header.stamp.toSec();
        header = pointCloud_buf.front().header;
        msg_1 = pointCloud_buf.front();
        pointCloud_buf.pop();
        msg_2 = gps_buf.front();
        gps_buf.pop();
        // printf("find pcl and gps\n");
      }
    }else{
      continue;
    }
    lock.unlock();
    
    pcl::PointCloud<pcl::PointXYZI>::Ptr local_pcl(new pcl::PointCloud<pcl::PointXYZI>());
    pcl::fromROSMsg(msg_1, *local_pcl);
    
    Eigen::Vector3d G_p_Gps;
    static GeographicLib::LocalCartesian local_cartesian;

    // // 设置局部坐标系原点（初始化经纬高）
    // local_cartesian.Reset(init_lla[0], init_lla[1], init_lla[2]);
    // // 输出的是ENU
    // local_cartesian.Forward(msg_2.latitude, msg_2.longitude, msg_2.altitude,
    //                     G_p_Gps.data()[0], G_p_Gps.data()[1], G_p_Gps.data()[2]); // ENU

    // lidar->base_link
    Eigen::Affine3f T_b_l = pcl::getTransformation(-0.27, 0, 0, 0, 10/180*M_PI, 0);
    // base_link->uav_base(NED)
    Eigen::Affine3f T_uav_b = pcl::getTransformation(0, 0, 0, M_PI, 0, 0);
    // uav_base->NED RPY 在GPS的基础下
    // Eigen::Affine3f T_NED_GPS_uav = pcl::getTransformation(G_p_Gps(0), G_p_Gps(1), G_p_Gps(2),
    //                                                    msg_2.position_covariance[0], msg_2.position_covariance[1], msg_2.position_covariance[2]);
    // uav_base->NED RPY
    Eigen::Affine3f T_NED_uav = pcl::getTransformation(0,0,0,
                                                       msg_2.position_covariance[0], msg_2.position_covariance[1], msg_2.position_covariance[2]);

    Eigen::Affine3f T_NED_X_uav = pcl::getTransformation(0,0,0,
                                                       msg_2.position_covariance[0], 0,0);
    Eigen::Affine3f T_NED_Y_uav = pcl::getTransformation(0,0,0,
                                                       0, msg_2.position_covariance[1], 0);
    Eigen::Affine3f T_NED_Z_uav = pcl::getTransformation(0,0,0,
                                                       0, 0, msg_2.position_covariance[2]);

    // uav_base->ENU RPY
    // Eigen::Affine3f T_ENU_uav = pcl::getTransformation(G_p_Gps(0), G_p_Gps(1), G_p_Gps(2),
    //                                                    0,0,0);

    Eigen::Affine3f T_ENU_X_uav = pcl::getTransformation(0,0,0,
                                                       msg_2.position_covariance[1], 0,0);
    Eigen::Affine3f T_ENU_Y_uav = pcl::getTransformation(0,0,0,
                                                       0, msg_2.position_covariance[0], 0);
    Eigen::Affine3f T_ENU_Z_uav = pcl::getTransformation(0,0,0,
                                                       0, 0, msg_2.position_covariance[2]);
    // ENU -> NED
    Eigen::Affine3f T_NED_ENU = pcl::getTransformation(
                                                      0, 0, 0,         // 平移：ENU原点直接映射到NED原点（若需要平移偏移需调整前三个参数）
                                                      M_PI, 0, M_PI/2  // 旋转：绕X轴180°（补偿Z轴方向），绕Z轴90°（对齐北向轴）
                                                      );

    // lidar->NED
    Eigen::Affine3f T_NED_l = T_NED_uav*T_uav_b*T_b_l;
    
    // NED->ENU
    Eigen::Affine3f T_ENU_X_NED = pcl::getTransformation(0, 0, 0, M_PI, 0, 0);

    Eigen::Affine3f T_ENU_Z_NED = pcl::getTransformation(0, 0, 0, 0, 0, -M_PI/2);

    // 根据NED的结果进行调节
    Eigen::Affine3f T_END_ENU = pcl::getTransformation(0, 0, 0,0, 0, M_PI);

    // 最终的坐标系转换(不可更改)
    Eigen::Affine3f T_END = T_END_ENU*T_ENU_Z_NED*T_ENU_X_NED*T_NED_l;

    // // 去除盲区点云，并转到uav系下
    pcl::PointCloud<pcl::PointXYZI>::Ptr localMap_filtered(new pcl::PointCloud<pcl::PointXYZI>) ;
    for (const auto& point : local_pcl->points) {
      if(std::sqrt(point.x*point.x + point.y*point.y + point.z*point.z) > blindSpot_) //blindSpot_
      {
        pcl::PointXYZI pointCur;
        // lidar->base_link
        pointCur = point;
        pointCur.x = T_END(0, 0) * point.x + T_END(0, 1) * point.y + T_END(0, 2) * point.z + T_END(0, 3);
        pointCur.y = T_END(1, 0) * point.x + T_END(1, 1) * point.y + T_END(1, 2) * point.z + T_END(1, 3);
        pointCur.z = T_END(2, 0) * point.x + T_END(2, 1) * point.y + T_END(2, 2) * point.z + T_END(2, 3);

        localMap_filtered->push_back(pointCur);
      }else{
        continue;
      }
    }
    
    // ##########################以上代码先进行注释掉##############################

    local_pcl = localMap_filtered;
    // icp初值
    // Eigen::Affine3f relativeTransform = pose.inverse() * last_pose;

    // 计算icp，将局部地图对齐到最新帧
    // double t_icp_1 = ros::Time::now().toSec();
    // alignByIcp(local_pcl, localMap, relativeTransform);
    // double t_icp_2 = ros::Time::now().toSec();
    // std::cout << "icp cost time: " << t_icp_2-t_icp_1 << std::endl;
    
    // pcl::transformPointCloud(*localMap, *localMap, relativeTransform);
    localMap = local_pcl;


    // 设置直通滤波器（原代码）
    pcl::PassThrough<pcl::PointXYZI> pass_filter;
    pass_filter.setFilterFieldName("x");
    pass_filter.setFilterLimits(-octomap_size_x_, octomap_size_x_);
    pass_filter.setInputCloud(localMap);
    pass_filter.filter(*localMap);

    pass_filter.setFilterFieldName("y");
    pass_filter.setFilterLimits(-octomap_size_y_, octomap_size_y_);
    pass_filter.setInputCloud(localMap);
    pass_filter.filter(*localMap);

    pass_filter.setFilterFieldName("z");
    pass_filter.setFilterLimits(-octomap_size_z_, octomap_size_z_);
    pass_filter.setInputCloud(localMap);
    pass_filter.filter(*localMap);


    // 1. 统计滤波（去除全局噪声）
    // pcl::StatisticalOutlierRemoval<pcl::PointXYZI> stat_filter;
    // stat_filter.setInputCloud(localMap);
    // stat_filter.setMeanK(MeanK);                // 每个点分析的邻近点数
    // stat_filter.setStddevMulThresh(StddevMulThresh);     // 标准差阈值（大于均值+1σ的点视为离群）
    // stat_filter.filter(*localMap);           // 覆盖原始点云
    // 2. 半径滤波（处理局部离群点）
    pcl::RadiusOutlierRemoval<pcl::PointXYZI> radius_filter;
    radius_filter.setInputCloud(localMap);
    radius_filter.setRadiusSearch(RadiusSearch);       // 搜索半径（单位：米）
    radius_filter.setMinNeighborsInRadius(MinNeighbors); // 半径内最少邻居点数
    radius_filter.filter(*localMap);          // 执行滤波
    // 体素滤波（降采样）
    pcl::VoxelGrid<pcl::PointXYZI> VoxelGrid; 
    VoxelGrid.setLeafSize(LeafSize, LeafSize, LeafSize); 
    VoxelGrid.setInputCloud(localMap);
    VoxelGrid.filter(*localMap);

    sensor_msgs::PointCloud2 local_map_msg;
    pcl::toROSMsg(*localMap, local_map_msg);
    local_map_msg.header = header;
    local_map_msg.header.frame_id = "map";
    pub_localMap.publish(local_map_msg);
    
    

    // 创建OctoMap树
    octomap::OcTree tree(octomap_resolution_);
    // 插入点云到OctoMap,设置盲区
    for (const auto& point : localMap->points) {
      if(std::sqrt(point.x*point.x + point.y*point.y + point.z*point.z) > blindSpot_)
      {
        pcl::PointXYZI pointCur;
        pointCur = point;
        tree.updateNode(octomap::point3d(pointCur.x, pointCur.y, pointCur.z), true); // 标记为占据
      }else{
          continue;
      }
    }
    tree.updateInnerOccupancy(); // 更新内部占据状态
    
    // 计算地图边界和分辨率 
    double min_x, min_y, min_z, max_x, max_y, max_z;
    min_x = -octomap_size_x_;
    min_y = -octomap_size_y_;
    min_z = -octomap_size_z_;
    max_x = octomap_size_x_;
    max_y = octomap_size_y_;
    max_z = octomap_size_z_;
    double resolution = tree.getResolution();


    // 初始化三维数组 
    Grid3D grid;
    int size_x = static_cast<int>((max_x - min_x) / resolution);
    int size_y = static_cast<int>((max_y - min_y) / resolution);
    int size_z = static_cast<int>((max_z - min_z) / resolution);
  
    grid.resize(size_x,  std::vector<std::vector<int>>(size_y, std::vector<int>(size_z, 0)));

    // 填充占用体素 
    for (auto it = tree.begin_leafs(); it != tree.end_leafs(); ++it) {
      if (tree.isNodeOccupied(*it)) {
          int x = static_cast<int>((it.getX()  - min_x) / resolution);
          int y = static_cast<int>((it.getY()  - min_y) / resolution);
          int z = static_cast<int>((it.getZ()  - min_z) / resolution);
          if (x >= 0 && x < size_x && y >= 0 && y < size_y && z >= 0 && z < size_z) {
              grid[x][y][z] = 1;
          }
      }
    }

    // 发布或保存数据 
    publishGrid3D(grid, size_x, size_y, size_z);
    if (save_to_file_) {
      saveGrid3D(grid, file_path_);
      ROS_INFO("Grid saved to %s", file_path_.c_str());
      save_to_file_ = false;
      ROS_INFO("Map bounds: [%.2f, %.2f, %.2f] to [%.2f, %.2f, %.2f], grid [%d, %d, %d],res=%.3f",
              min_x, min_y, min_z, max_x, max_y, max_z,size_x, size_y, size_z, resolution);
    }
    
    gps_path.header.stamp = header.stamp;
    gps_path.header.frame_id = "map";
    geometry_msgs::PoseStamped curr_path;

    curr_path.header.stamp = header.stamp;
    curr_path.header.frame_id = "map";
    
    // NED
    curr_path.pose.position.x = G_p_Gps(0);
    curr_path.pose.position.y = G_p_Gps(1);
    curr_path.pose.position.z = G_p_Gps(2);

    curr_path.pose.orientation.x = 0;
    curr_path.pose.orientation.y = 0;
    curr_path.pose.orientation.z = 0;
    curr_path.pose.orientation.w = 1;

    gps_path.poses.push_back(curr_path);

    // pub_gps.publish(gps_path);

    // last_pose = pose;
  }
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "LocalMapping_node");

  LocalMap lm;

  std::thread processThread;

  processThread = std::thread(&LocalMap::process, &lm);
      
  ros::spin();    

  processThread.join();

  // lm.saveLocalMap();

  return 0;
}