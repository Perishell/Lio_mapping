#ifndef PREPROCESS_H
#define PREPROCESS_H

#include <ros/ros.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/common/transforms.h>
#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/PointStamped.h>
#ifdef ENABLE_LIVOX
#include <livox_ros_driver/CustomMsg.h>
#endif

// #include "lio/common_lib.h"

using namespace std;

#define IS_VALID(a)  ((abs(a)>1e8) ? true : false)

typedef pcl::PointXYZINormal PointType;
typedef pcl::PointCloud<PointType> PointCloudXYZI;

enum LID_TYPE{AVIA = 1, VELO16, OUST64, LS16, RS32, RS32_DENSE, MID360, TANWAY48}; //{1, 2, 3, 4, 5, 6, 7, 8}
enum TIME_UNIT{SEC = 0, MS = 1, US = 2, NS = 3};
enum Feature{Nor, Poss_Plane, Real_Plane, Edge_Jump, Edge_Plane, Wire, ZeroPoint};
enum Surround{Prev, Next};
enum E_jump{Nr_nor, Nr_zero, Nr_180, Nr_inf, Nr_blind};

const bool time_list_cut_frame(PointType &x, PointType &y);

struct orgtype
{
  double range;
  double dista; 
  double angle[2];
  double intersect;
  E_jump edj[2];
  Feature ftype;
  orgtype()
  {
    range = 0;
    edj[Prev] = Nr_nor;
    edj[Next] = Nr_nor;
    ftype = Nor;
    intersect = 2;
  }
};

namespace ls_ros {
  struct EIGEN_ALIGN16 Point {
      PCL_ADD_POINT4D;
      float intensity;
      uint16_t ring;
      double time;
      EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  };
}  // namespace velodyne_ros
POINT_CLOUD_REGISTER_POINT_STRUCT(ls_ros::Point,
    (float, x, x)
    (float, y, y)
    (float, z, z)
    (float, intensity, intensity)
    (uint16_t, ring, ring)
    (double, time, time)
)

// rslidar的点云格式
namespace rslidar_ros {
  struct EIGEN_ALIGN16 Point {
      PCL_ADD_POINT4D;
      // uint8_t intensity; // v1.3
      float intensity; // v1.5
      uint16_t ring;
      double timestamp;
      EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  };
}  // namespace velodyne_ros
POINT_CLOUD_REGISTER_POINT_STRUCT(rslidar_ros::Point,
    (float, x, x)
    (float, y, y)
    (float, z, z)
    (float, intensity, intensity)
    (uint16_t, ring, ring)
    (double, timestamp, timestamp)
)

// tanway的点云格式
namespace tanway_ros {
  struct EIGEN_ALIGN16 Point {
      PCL_ADD_POINT4D;
      // uint8_t intensity; // v1.3
      float intensity; // v1.5
      int32_t channel;
      float angle;
      int32_t echo;
      int32_t block;
      uint32_t t_sec;
      uint32_t t_usec;
      EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  };
}  // namespace velodyne_ros
POINT_CLOUD_REGISTER_POINT_STRUCT(tanway_ros::Point,
    (float, x, x)
    (float, y, y)
    (float, z, z)
    (float, intensity, intensity)
    (int32_t, channel, channel)
    (float, angle, angle)
    (int32_t, echo, echo)
    (int32_t, block, block)
    (uint32_t, t_sec, t_sec)
    (uint32_t, t_usec, t_usec)
)

namespace livox_ros
{
  struct EIGEN_ALIGN16 Point
  {
    PCL_ADD_POINT4D;
    float intensity;
    uint8_t tag;
    uint8_t line;
    double timestamp;
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  };
} // namespace velodyne_ros
POINT_CLOUD_REGISTER_POINT_STRUCT(livox_ros::Point,
                                  (float, x, x)(float, y, y)(float, z, z)(float, intensity, intensity)(uint8_t, tag, tag)(uint8_t, line, line)(double, timestamp, timestamp))


namespace velodyne_ros {
  struct EIGEN_ALIGN16 Point {
      PCL_ADD_POINT4D;
      float intensity;
      float time;
      uint16_t ring;
      EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  };
}  // namespace velodyne_ros
POINT_CLOUD_REGISTER_POINT_STRUCT(velodyne_ros::Point,
    (float, x, x)
    (float, y, y)
    (float, z, z)
    (float, intensity, intensity)
    (float, time, time)
    (uint16_t, ring, ring)
)

namespace ouster_ros {
  struct EIGEN_ALIGN16 Point {
      PCL_ADD_POINT4D;
      float intensity;
      uint32_t t;
      uint16_t reflectivity;
      uint8_t  ring;
      uint16_t ambient;
      uint32_t range;
      EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  };
}  // namespace ouster_ros

// clang-format off
POINT_CLOUD_REGISTER_POINT_STRUCT(ouster_ros::Point,
    (float, x, x)
    (float, y, y)
    (float, z, z)
    (float, intensity, intensity)
    // use std::uint32_t to avoid conflicting with pcl::uint32_t
    (std::uint32_t, t, t)
    (std::uint16_t, reflectivity, reflectivity)
    (std::uint8_t, ring, ring)
    (std::uint16_t, ambient, ambient)
    (std::uint32_t, range, range)
)

class Preprocess
{
  public:
//   EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  Preprocess();
  ~Preprocess();
  
#ifdef ENABLE_LIVOX
  void process(const livox_ros_driver::CustomMsg::ConstPtr &msg, PointCloudXYZI::Ptr &pcl_out);
#endif
  void process(const sensor_msgs::PointCloud2::ConstPtr &msg, PointCloudXYZI::Ptr &pcl_out);

  // 雷达分帧
  void process_cut_frame_pcl2(const sensor_msgs::PointCloud2::ConstPtr &msg, deque<PointCloudXYZI::Ptr> &pcl_out, deque<double> &time_lidar, const int required_frame_num, int scan_count);

  // 设置是否使用特征提取，雷达类型，盲区，采样间距（不提取特征时用于降低点云密度）
  void set(bool feat_en, int lid_type, double bld, int pfilt_num);

  // sensor_msgs::PointCloud2::ConstPtr pointcloud;
  PointCloudXYZI pl_full, pl_corn, pl_surf; // 全部点、边缘点、平面点
  PointCloudXYZI pl_buff[128]; //maximum 128 line lidar
  vector<orgtype> typess[128]; //maximum 128 line lidar
  float time_unit_scale;
  int lidar_type, point_filter_num, N_SCANS, SCAN_RATE, time_unit; // 雷达类型、采样间隔、扫描线数
  double blind; // 最小距离阈值
  double blind_far; //最大距离阈值
  bool feature_enabled, given_offset_time; // 是否提取特征、是否进行时间偏移
  std::vector<float> lidarRot; // 旋转矫正
  ros::Publisher pub_full, pub_surf, pub_corn; // 发布全部点、发布平面点、发布边缘点 

  geometry_msgs::PointStamped target_point;
  bool target_avail = false;

  private:
  // 雷达数据处理
#ifdef ENABLE_LIVOX
  void avia_handler(const livox_ros_driver::CustomMsg::ConstPtr &msg);
#endif
  void oust64_handler(const sensor_msgs::PointCloud2::ConstPtr &msg);
  void velodyne_handler(const sensor_msgs::PointCloud2::ConstPtr &msg);
  void ls16_handler(const sensor_msgs::PointCloud2::ConstPtr &msg);
  void rs32_handler(const sensor_msgs::PointCloud2::ConstPtr &msg);
  void mid360_handler(const sensor_msgs::PointCloud2::ConstPtr &msg);
  void tanway48_handler(const sensor_msgs::PointCloud2::ConstPtr &msg);
  void removeNanPoint(pcl::PointCloud<rslidar_ros::Point> cloudIn, pcl::PointCloud<rslidar_ros::Point>& cloudOut);
  void give_feature(PointCloudXYZI &pl, vector<orgtype> &types);
  void pub_func(PointCloudXYZI &pl, const ros::Time &ct);
  int  plane_judge(const PointCloudXYZI &pl, vector<orgtype> &types, uint i, uint &i_nex, Eigen::Vector3d &curr_direct);
  bool small_plane(const PointCloudXYZI &pl, vector<orgtype> &types, uint i_cur, uint &i_nex, Eigen::Vector3d &curr_direct);
  // 判断是否是有跨越的边
  bool edge_jump_judge(const PointCloudXYZI &pl, vector<orgtype> &types, uint i, Surround nor_dir);
  // 矫正雷达安装倾斜
  void rotateLidar(pcl::PointCloud<rslidar_ros::Point>& cloud);
  
  int group_size;
  double disA, disB, inf_bound;
  double limit_maxmid, limit_midmin, limit_maxmin;
  double p2l_ratio;
  double jump_up_limit, jump_down_limit;
  double cos160;
  double edgea, edgeb;
  double smallp_intersect, smallp_ratio;
  double vx, vy, vz;
};

#endif