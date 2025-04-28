/*
 * @Author: dyhan
 * @Date: 2022-11-03 10:00:41
 * @LastEditors: dyhan
 * @LastEditTime: 2024-08-22 18:50:34
 * @Description: 
 */
#ifndef COMMON_OP_H
#define COMMON_OP_H

#include <omp.h>
#include <mutex>
#include <math.h>
#include <thread>
#include <algorithm>
#include <sstream>
#include <fstream>
#include <iomanip>
#include <csignal>
#include <unistd.h>
#include <unordered_map>
#include <Python.h>
#include <so3_math.h>
#include <ros/ros.h>
#include <Eigen/Core>
#include <std_msgs/UInt8.h>
#include <std_msgs/Header.h>
#include <std_msgs/Float64MultiArray.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/NavSatFix.h>
#include <sensor_msgs/PointCloud2.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

#include <tf/transform_datatypes.h>
#include <tf/transform_broadcaster.h>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>

#include <pcl/search/impl/search.hpp>
#include <pcl/range_image/range_image.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/common/common.h>
#include <pcl/common/transforms.h>
#include <pcl/registration/icp.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/io/obj_io.h>
#include <pcl/PolygonMesh.h>
#include <pcl/filters/filter.h>
#include <pcl/filters/crop_box.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>

// gstam
#include <gtsam/geometry/Rot3.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/slam/PriorFactor.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/navigation/GPSFactor.h>
#include <gtsam/navigation/ImuFactor.h>
#include <gtsam/navigation/CombinedImuFactor.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/nonlinear/Marginals.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/nonlinear/ISAM2.h>

#define GROUND 0
#define AMBIGUOUS 1
#define OBSTACLE 2

/**
 * 6D位姿点云结构定义
*/
struct PointXYZIRPYT
{
    PCL_ADD_POINT4D     
    PCL_ADD_INTENSITY;  
    float roll;         
    float pitch;
    float yaw;
    double time;
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW   
} EIGEN_ALIGN16;                    

POINT_CLOUD_REGISTER_POINT_STRUCT (PointXYZIRPYT,
                                   (float, x, x) (float, y, y)
                                   (float, z, z) (float, intensity, intensity)
                                   (float, roll, roll) (float, pitch, pitch) (float, yaw, yaw)
                                   (double, time, time))
struct PointXYZQUAT
{
    PCL_ADD_POINT4D
    PCL_ADD_INTENSITY;
    float qx;         
    float qy;
    float qz;
    float qw;
    double time;
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW   
} EIGEN_ALIGN16;                    

POINT_CLOUD_REGISTER_POINT_STRUCT (PointXYZQUAT,
                                   (float, x, x) (float, y, y) (float, z, z) (float, intensity, intensity)
                                   (float, qx, qx) (float, qy, qy) (float, qz, qz) (float, qw, qw)
                                   (double, time, time))

typedef pcl::PointXYZINormal PointType;
typedef PointXYZQUAT  PointTypePose;

/**
 * 位姿格式变换
 */
gtsam::Pose3 pclPointTogtsamPose3(PointTypePose thisPoint)
{
    // return gtsam::Pose3(gtsam::Rot3(Eigen::Quaterniond(thisPoint.qw, thisPoint.qx, thisPoint.qy, thisPoint.qz).normalized().toRotationMatrix()),
    //                     gtsam::Point3(double(thisPoint.x), double(thisPoint.y), double(thisPoint.z)));
    return gtsam::Pose3(gtsam::Rot3::Quaternion(double(thisPoint.qw),double(thisPoint.qx),double(thisPoint.qy),double(thisPoint.qz)),
                        gtsam::Point3(double(thisPoint.x), double(thisPoint.y), double(thisPoint.z)));
    // return gtsam::Pose3(gtsam::Rot3::Quaternion(double(thisPoint.qw), double(thisPoint.qx), double(thisPoint.qy), double(thisPoint.qz)),
    //                     gtsam::Point3(double(thisPoint.x), double(thisPoint.y), double(thisPoint.z)));
}

/**
 * 位姿格式变换
 */
gtsam::Pose3 trans2gtsamPose(Eigen::Isometry3d transformIn)
{
    Eigen::Quaterniond quat(transformIn.rotation());
    return gtsam::Pose3(gtsam::Rot3::Quaternion(quat.w(),quat.x(),quat.y(),quat.z()),
                        gtsam::Point3(transformIn.translation()));
    // return gtsam::Pose3(gtsam::Rot3(transformIn.rotation()),
    //                     gtsam::Point3(transformIn.translation()));
}

/**
 * Eigen格式的位姿变换
 */
Eigen::Isometry3d pclPointToIsometry3d(PointTypePose thisPoint)
{ 
    Eigen::Isometry3d transform = Eigen::Isometry3d::Identity();
    transform.rotate (Eigen::Quaterniond(thisPoint.qw,thisPoint.qx,thisPoint.qy,thisPoint.qz));
    transform.pretranslate(Eigen::Vector3d(thisPoint.x, thisPoint.y, thisPoint.z));
    return transform;
}

Eigen::Isometry3f pclPointToIsometry3f(PointTypePose thisPoint)
{ 
    Eigen::Isometry3f transform = Eigen::Isometry3f::Identity();
    transform.rotate (Eigen::Quaternionf(thisPoint.qw,thisPoint.qx,thisPoint.qy,thisPoint.qz));
    transform.pretranslate(Eigen::Vector3f(thisPoint.x, thisPoint.y, thisPoint.z));
    return transform;
}

/**
 * 位姿格式变换
 */
PointTypePose trans2PointTypePose(Eigen::Isometry3d transformIn)
{
    PointTypePose thisPose6D;
    Eigen::Quaterniond transformIn_quat(transformIn.rotation());
    thisPose6D.qx = transformIn_quat.x();
    thisPose6D.qy = transformIn_quat.y();
    thisPose6D.qz = transformIn_quat.z();
    thisPose6D.qw = transformIn_quat.w();
    thisPose6D.x = transformIn.translation()[0];
    thisPose6D.y = transformIn.translation()[1];
    thisPose6D.z = transformIn.translation()[2];
    return thisPose6D;
}

/**
 * 发布thisCloud，返回thisCloud对应msg格式
 */
sensor_msgs::PointCloud2 publishCloud(const ros::Publisher &thisPub, pcl::PointCloud<PointType>::Ptr thisCloud, ros::Time thisStamp, std::string thisFrame)
{
    sensor_msgs::PointCloud2 tempCloud;
    pcl::toROSMsg(*thisCloud, tempCloud);
    tempCloud.header.stamp = thisStamp;
    tempCloud.header.frame_id = thisFrame;
    if (thisPub.getNumSubscribers() != 0)
        thisPub.publish(tempCloud);
    return tempCloud;
}

/**
 * 点到坐标系原点距离
 */
float pointDistance(PointType p)
{
    return sqrt(p.x * p.x + p.y * p.y + p.z * p.z);
}

/**
 * 两点之间距离
 */
float pointDistance(PointType p1, PointType p2)
{
    return sqrt((p1.x - p2.x) * (p1.x - p2.x) + (p1.y - p2.y) * (p1.y - p2.y) + (p1.z - p2.z) * (p1.z - p2.z));
}

/******save map and pose*******/
bool CreateFile(std::ofstream& ofs, std::string file_path) {
    ofs.open(file_path, std::ios::out);                          //  使用std::ios::out 可实现覆盖
    if(!ofs)
    {
        // std::cout << "open csv file error " << std::endl;
        // PrintInfo("open csv file error");
        return  false;
    }
    return true;
}

/*定义pose结构体*/
struct pose
{
    Eigen::Vector3d  t ;
    Eigen::Matrix3d  R;
};
/**
 * @brief write2txt KITTI format
 * 
 * @param ofs 
 * @param data 
 */
void WriteTextKITTI(std::ofstream& ofs, pose data){
    ofs << std::fixed  <<  data.R(0,0)  << " " << data.R(0,1)   << " "<<   data.R(0,2)  << " "  <<    data.t[0]  <<  " "
                                      <<  data.R(1,0)  << " "  << data.R(1,1)  <<" " <<   data.R(1,2)   << " "  <<   data.t[1]  <<  " "
                                      <<  data.R(2,0)  << " "  << data.R(2,1)  <<" " <<   data.R(2,2)   << " "  <<   data.t[2]  <<  std::endl;
}

/**
 * @brief write2txt TUM format
 * 
 * @param ofs 
 * @param timestamp 
 * @param data 
 */
void WriteTextTUM(std::ofstream& ofs, double timestamp, pose data){
    Eigen::Quaterniond data_quat(data.R);

    ofs << std::fixed  << timestamp << " " << data.t[0] << " " << data.t[1] << " " << data.t[2] << " "
                       << data_quat.x() << " " << data_quat.y() << " " << data_quat.z() << " " << data_quat.w() << std::endl;
}

//tools 
template <typename T>
T square(T a)
{
    return a * a;
}
Eigen::Vector3d quatToRPY(const Eigen::Quaterniond &q)
{
    Eigen::Vector3d rpy;
    double as = std::min(-2. * (q.x() * q.z() - q.w() * q.y()), .99999);
    rpy(2) =
        std::atan2(2 * (q.x() * q.y() + q.w() * q.z()),
                square(q.w()) + square(q.x()) - square(q.y()) - square(q.z()));
    rpy(1) = std::asin(as);
    rpy(0) =
        std::atan2(2 * (q.y() * q.z() + q.w() * q.x()),
                square(q.w()) - square(q.x()) - square(q.y()) + square(q.z()));
    return rpy;
}

void groundExtraction(const pcl::PointCloud<PointType>::Ptr in_pcl_cloud_ptr,
                        pcl::PointCloud<PointType>::Ptr pcl_cloud_no_ground_ptr)
{
    int m_m = 180, m_n = 120;
    double m_r_h = 60, m_r_l = 0.1, m_pho_gnd = 1.0, m_pho_s = 0.5, m_pho_asc = 0.5, m_pho_h = 0.2;
    // double m_car_width, m_car_length, m_car_chassis_height, m_car_roll_threhold, m_car_pitch_threhold;
    if(in_pcl_cloud_ptr->points.size() == 0)
    {
        ROS_ERROR("Ground Extraction Error\n");
        return;
    }

    std::vector<double> grid_height_h_vec(m_m * m_n, -DBL_MAX);
    std::vector<double> grid_lowest_h_vec(m_m * m_n, DBL_MAX);
    std::vector<double> grid_height_r_vec(m_m * m_n, 0.0);
    std::vector<double> grid_lowest_r_vec(m_m * m_n, 0.0);
    std::vector<int> grid_count_vec(m_m * m_n, 0);
    std::vector<double> grid_slope_inside_cell_vec(m_m * m_n, 0.0);
    std::vector<double> grid_slope_between_cell_vec(m_m * m_n, 0.0);
    std::vector<int> grid_result_vec(m_m * m_n, GROUND);
    std::vector<int> cloud_grid_index_vec(in_pcl_cloud_ptr->size(), -1);
    double delta_theta = 2 * M_PI / m_m;
    double delta_r = (m_r_h - m_r_l) / m_n;

    double r, theta, h;
    int grid_i, grid_j;
    int grid_index;
    for(int cloud_index = 0; cloud_index < in_pcl_cloud_ptr->size(); cloud_index++)
    {
        r = sqrt(pow(in_pcl_cloud_ptr->points[cloud_index].x, 2) + pow(in_pcl_cloud_ptr->points[cloud_index].y, 2));
        theta = atan2(in_pcl_cloud_ptr->points[cloud_index].y, in_pcl_cloud_ptr->points[cloud_index].x) + M_PI;
        h = in_pcl_cloud_ptr->points[cloud_index].z;
        if(r < m_r_h && r >= m_r_l)
        {
            grid_i = floor(theta / delta_theta) + 1;
            grid_j = floor((r - m_r_l) / delta_r) + 1;
            grid_index = (grid_j - 1) * m_m + (grid_i - 1);
            if(grid_height_h_vec[grid_index] < h)
            {
                grid_height_h_vec[grid_index] = h;
                grid_height_r_vec[grid_index] = r;
            }
            if(grid_lowest_h_vec[grid_index] > h)
            {
                grid_lowest_h_vec[grid_index] = h;
                grid_lowest_r_vec[grid_index] = r;
            }
            grid_count_vec[grid_index]++;
            cloud_grid_index_vec[cloud_index] = grid_index;
        }
    }

    int grid_between_index = 0;
    bool grid_between_empty = true;
    for(int grid_index = 0; grid_index < m_m * m_n; grid_index++)
    {
        if(grid_count_vec[grid_index] == 0)
        {
            continue;
        }

        grid_i = grid_index % m_m + 1;
        grid_j = grid_index / m_m + 1;
        grid_slope_inside_cell_vec[grid_index] = fabs((grid_height_h_vec[grid_index] - grid_lowest_h_vec[grid_index]) / (grid_height_r_vec[grid_index] - grid_lowest_r_vec[grid_index] + 1e-5));
        if(grid_j > 1)
        {
            grid_between_index = (grid_j - 2) * m_m + (grid_i - 1);
            if(grid_count_vec[grid_between_index] > 0)
            {
                grid_between_empty = false;
            }
        }

        if(!grid_between_empty)
        {
            grid_between_index = (grid_j - 2) * m_m + (grid_i - 1);
            grid_slope_between_cell_vec[grid_index] = fabs((grid_lowest_h_vec[grid_index] - grid_lowest_h_vec[grid_between_index]) / (grid_lowest_r_vec[grid_index] - grid_lowest_r_vec[grid_between_index]));
            if(grid_slope_inside_cell_vec[grid_index] < m_pho_s && grid_slope_between_cell_vec[grid_index] < m_pho_asc && grid_result_vec[grid_between_index] != OBSTACLE)
            {
                grid_result_vec[grid_index] = GROUND;
            }
            else if(grid_slope_inside_cell_vec[grid_index] >= m_pho_s && grid_slope_between_cell_vec[grid_index] < m_pho_asc && grid_result_vec[grid_between_index] != OBSTACLE)
            {
                grid_result_vec[grid_index] = AMBIGUOUS;
            }
            else if(grid_lowest_h_vec[grid_index] < m_pho_gnd && grid_slope_inside_cell_vec[grid_index] >= m_pho_s && grid_result_vec[grid_between_index] != OBSTACLE)
            {
                grid_result_vec[grid_index] = AMBIGUOUS;
            }
            else if(grid_lowest_h_vec[grid_index] >= m_pho_gnd && grid_slope_between_cell_vec[grid_index] >= m_pho_asc && grid_result_vec[grid_between_index] != OBSTACLE)
            {
                grid_result_vec[grid_index] = OBSTACLE;
            }
            else if(grid_lowest_h_vec[grid_index] >= m_pho_gnd && grid_result_vec[grid_between_index] == OBSTACLE)
            {
                grid_result_vec[grid_index] = OBSTACLE;
            }
            else if(grid_lowest_h_vec[grid_index] < m_pho_gnd && grid_result_vec[grid_between_index] == OBSTACLE)
            {
                grid_result_vec[grid_index] = AMBIGUOUS;
            }
        }
        else
        {
            if(grid_lowest_h_vec[grid_index] < m_pho_gnd && grid_slope_inside_cell_vec[grid_index] < m_pho_s)
            {
                grid_result_vec[grid_index] = GROUND;
            }
            else if(grid_lowest_h_vec[grid_index] < m_pho_gnd && grid_slope_inside_cell_vec[grid_index] >= m_pho_s)
            {
                grid_result_vec[grid_index] = AMBIGUOUS;
            }
            else
            {
                grid_result_vec[grid_index] = OBSTACLE;
            }
        }
    }

    pcl_cloud_no_ground_ptr->points.clear();
    PointType point;
    for(int cloud_index = 0; cloud_index < in_pcl_cloud_ptr->size(); cloud_index++)
    {
        if(cloud_grid_index_vec[cloud_index] == -1)
        {
            continue;
        }
        point.x = in_pcl_cloud_ptr->points[cloud_index].x;
        point.y = in_pcl_cloud_ptr->points[cloud_index].y;
        point.z = 0;//in_pcl_cloud_ptr->points[cloud_index].z;
        point.intensity = in_pcl_cloud_ptr->points[cloud_index].intensity;
        if(grid_result_vec[cloud_grid_index_vec[cloud_index]] == AMBIGUOUS)
        {
            r = sqrt(pow(point.x, 2) + pow(point.y, 2));
            theta = atan2(point.y, point.x) + M_PI;
            h = point.z;
            grid_i = floor(theta / delta_theta) + 1;
            grid_j = floor((r - m_r_l) / delta_r) + 1;
            grid_index = (grid_j - 1) * m_m + (grid_i - 1);
            if(h > (grid_lowest_h_vec[grid_index] + m_pho_h) && grid_count_vec[grid_index] > 0)
            {
                pcl_cloud_no_ground_ptr->points.push_back(point);
            }
        }
        else if(grid_result_vec[cloud_grid_index_vec[cloud_index]] == OBSTACLE)
        {
            pcl_cloud_no_ground_ptr->points.push_back(point);
        }
    }
    pcl_cloud_no_ground_ptr->width = 1;
    pcl_cloud_no_ground_ptr->height = pcl_cloud_no_ground_ptr->points.size();
}

#endif