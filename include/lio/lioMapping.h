/*
 * @Author: dyhan
 * @Date: 2022-10-12 09:26:04
 * @LastEditors: dyhan
 * @LastEditTime: 2024-10-25 13:29:11
 * @Description: 
 */
#ifndef LIOMAPPING_H
#define LIOMAPPING_H

#include <omp.h>
#include <mutex>
#include <math.h>
#include <thread>
#include <fstream>
#include <sstream>
#include <iomanip>
#include <csignal>
#include <unistd.h>
#include <Python.h>
#include <so3_math.h>
#include <ros/ros.h>
#include <Eigen/Core>
#include <condition_variable>

#include <std_msgs/UInt8.h>
#include <std_msgs/Header.h>
#include <std_msgs/Float64MultiArray.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/NavSatFix.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_broadcaster.h>
#include <geometry_msgs/Vector3.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#ifdef ENABLE_LIVOX
#include <livox_ros_driver/CustomMsg.h>
#endif

#include <ikd-Tree/ikd_Tree.h>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/search/impl/search.hpp>
#include <pcl/range_image/range_image.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/common/common.h>
#include <pcl/common/transforms.h>
#include <pcl/registration/icp.h>
#include <pcl/registration/ndt.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/filter.h>
#include <pcl/filters/crop_box.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/statistical_outlier_removal.h>

#include "lio/imuProcessing.hpp"
#include "lio/preprocess.h"
#include "Scancontext/Scancontext.h"

#include "slog/UserPrint.h"

#define INIT_TIME           (0.1)
#define LASER_POINT_COV     (0.001)
#define MAXN                (720000)
#define PUBFRAME_PERIOD     (20)

using namespace std;

namespace fusion_slam{
class lioMapping
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    
    lioMapping();
    ~lioMapping(){};

    //init 
    bool initROS(ros::NodeHandle* nh, ros::NodeHandle* priNh);
    void initVariable();
    void subAndPubToROS(ros::NodeHandle *nh);
    void loadParams(ros::NodeHandle *priNh);

    // main 
    bool mainRun();

    void imuCompensation(state_ikfom &state);
    void h_share_model(state_ikfom &s, esekfom::dyn_share_datastruct<double> &ekfom_data);
    void lasermap_fov_segment();
    void map_incremental();

    void imu_cbk(const sensor_msgs::Imu::ConstPtr &msg_in);
    #ifdef ENABLE_LIVOX
    void livox_pcl_cbk(const livox_ros_driver::CustomMsg::ConstPtr &msg);
    #endif
    void standard_pcl_cbk(const sensor_msgs::PointCloud2::ConstPtr &msg);
    void mode_cbk(const std_msgs::UInt8::ConstPtr& modeMsg);
    void wheel_cbk(const geometry_msgs::Twist::ConstPtr &msg_vel);
    void target_cbk(const geometry_msgs::PoseStampedConstPtr& targetMsg);
    bool sync_packages(MeasureGroup &meas);

    void mainShutDown();

    // 将更新的pose赋值到 transformTobeMapped
    bool getOdomAndFrame(Eigen::Isometry3d& transformTobeMapped, PointCloudXYZI::Ptr curFrame, double& frameEndTime);
    void updateMap(const PointCloudXYZI::Ptr mapKeyFramesDS);
    void updateState(const Eigen::Isometry3d& poseOp);

    // save and pub 
    void publish_path(const ros::Publisher pubPath);
    void publish_odometry(const ros::Publisher & pubOdomAftMapped);
    void publish_frame_body(const ros::Publisher & pubLaserCloudFull_body);
    void publish_effect_world(const ros::Publisher & pubLaserCloudEffect);
    void publish_map(const ros::Publisher & pubLaserCloudMap);
    void publish_frame_world(const ros::Publisher & pubLaserCloudFull);

    void saveAndPub();

    //localization
    bool loadPriorMap(const std::string& mapPath);
    bool initIkdTree();
    bool initPriorPose(pcl::PointCloud<PointType>::Ptr laserCloudIn);
    void globalLocalization();
    void initPoseReLocalization();
    void scReLocalization();
    void initPoseHandler(const geometry_msgs::PoseStamped::ConstPtr &initPose);

    void dump_lio_state_to_log(FILE *fp);

private:
    /*** Time Log Variables ***/
    double kdtree_incremental_time = 0.0, kdtree_search_time = 0.0, kdtree_delete_time = 0.0;
    double T1[MAXN], s_plot[MAXN], s_plot2[MAXN], s_plot3[MAXN], s_plot4[MAXN], s_plot5[MAXN], s_plot6[MAXN], s_plot7[MAXN], s_plot8[MAXN], s_plot9[MAXN], s_plot10[MAXN], s_plot11[MAXN];
    double match_time = 0, solve_time = 0, solve_const_H_time = 0;
    int    kdtree_size_st = 0, kdtree_size_end = 0, add_point_size = 0, kdtree_delete_counter = 0;
    bool   runtime_pos_log = false, pcd_save_en = false, time_sync_en = false, extrinsic_est_en = true, path_en = true;
    /**************************/

    float res_last[100000] = {0.0};
    float DET_RANGE = 300.0f;
    const float MOV_THRESHOLD = 1.5f;

    mutex mtx_buffer;
    condition_variable sig_buffer;

    string root_dir = ROOT_DIR;
    string map_file_path, lid_topic, imu_topic, wheel_topic, target_topic;

    double res_mean_last = 0.05, total_residual = 0.0;
    double last_timestamp_lidar = 0, last_timestamp_imu = -1.0;
    double gyr_cov = 0.1, acc_cov = 0.1, b_gyr_cov = 0.0001, b_acc_cov = 0.0001;
    double filter_size_corner_min = 0, filter_size_surf_min = 0, filter_size_map_min = 0, fov_deg = 0;
    double cube_len = 0, HALF_FOV_COS = 0, FOV_DEG = 0, total_distance = 0, lidar_end_time = 0, first_lidar_time = 0.0;
    int    effct_feat_num = 0, time_log_counter = 0, scan_count = 0, publish_count = 0;
    int    iterCount = 0, feats_down_size = 0, NUM_MAX_ITERATIONS = 0, laserCloudValidNum = 0, pcd_save_interval = -1, pcd_index = 0;
    bool   point_selected_surf[100000] = {0};
    bool   lidar_pushed, flg_first_scan = true, flg_exit = false, flg_EKF_inited;
    bool   scan_pub_en = false, dense_pub_en = false, scan_body_pub_en = false;

    double t0,t1,t2,t3,t4,t5,match_start, solve_start, svd_time;

    vector<vector<int>>  pointSearchInd_surf; 
    vector<BoxPointType> cub_needrm; // ikd-tree中，地图需要移除的包围盒序列
    vector<PointVector>  Nearest_Points; 
    vector<double>       extrinT{3, 0.0};
    vector<double>       extrinR{9, 0.0};
    deque<double>                     time_buffer; // 记录lidar时间
    deque<PointCloudXYZI::Ptr>        lidar_buffer; //记录特征提取或间隔采样后的lidar（特征）数据
    deque<sensor_msgs::Imu::ConstPtr> imu_buffer;

    vector<float>       lidarRot{3, 0.0};
    int cut_frame_num = 0;

    PointCloudXYZI::Ptr featsFromMap{new PointCloudXYZI()};
    PointCloudXYZI::Ptr feats_undistort{new PointCloudXYZI()};
    PointCloudXYZI::Ptr feats_down_body{new PointCloudXYZI()};  //畸变纠正后降采样的单帧点云，lidar系
    PointCloudXYZI::Ptr feats_down_world{new PointCloudXYZI()}; //畸变纠正后降采样的单帧点云，w系
    PointCloudXYZI::Ptr normvec{new PointCloudXYZI(100000, 1)}; //特征点在地图中对应点的，局部平面参数,w系
    PointCloudXYZI::Ptr laserCloudOri{new PointCloudXYZI(100000, 1)};
    PointCloudXYZI::Ptr corr_normvect{new PointCloudXYZI(100000, 1)};
    PointCloudXYZI::Ptr _featsArray{new PointCloudXYZI()};                            // ikd-tree中，map需要移除的点云

    PointCloudXYZI::Ptr pcl_wait_save{new PointCloudXYZI()};
    
    KD_TREE<PointType> ikdtree;

    V3F XAxisPoint_body{LIDAR_SP_LEN, 0.0, 0.0};  // T lidar to imu (imu = r * lidar + t)
    V3F XAxisPoint_world{LIDAR_SP_LEN, 0.0, 0.0}; // R lidar to imu (imu = r * lidar + t)
    V3D euler_cur;
    V3D position_last{Zero3d};
    V3D Lidar_T_wrt_IMU{Zero3d};
    M3D Lidar_R_wrt_IMU{Eye3d};

    V3D Wheel_T_wrt_IMU{Zero3d};
    M3D Wheel_R_wrt_IMU{Eye3d};
    vector<double>       extrinT_wheel{3, 0.0};
    vector<double>       extrinR_wheel{9, 0.0};
    bool useWheelVelParam = false;
    bool useWheelVel = false;
    bool twistAvail = false;
    Eigen::Vector3d curTwist{Zero3d};
    bool bTargetReceived = false;
    geometry_msgs::PointStamped targetPointCur;

    /*** EKF inputs and output ***/
    MeasureGroup Measures;
    esekfom::esekf<state_ikfom, 12, input_ikfom> kf; // 状态，噪声维度，输入
    state_ikfom state_point;
    vect3 pos_lid; // world系下lidar坐标

    BoxPointType LocalMap_Points; // 用于计算Box大小
    bool Localmap_Initialized = false;

    double epsi[23] = {0.001};
    pcl::VoxelGrid<PointType> downSizeFilterSurf;               //单帧内降采样使用voxel grid

    // sync
    double lidar_mean_scantime = 0.0;
    int    scan_num = 0;

    // standard_pcl_cbk
    int cnt = 0;
    PointCloudXYZI::Ptr  cloudPtr{new PointCloudXYZI()};

    // livox_pcl_cbk
    double timediff_lidar_wrt_imu = 0.0;
    bool   timediff_set_flg = false;

    nav_msgs::Path path;
    nav_msgs::Odometry odomAftMapped;
    geometry_msgs::Quaternion geoQuat;
    geometry_msgs::PoseStamped msg_body_pose;

    shared_ptr<Preprocess> p_pre{new Preprocess()};
    shared_ptr<ImuProcess> p_imu{new ImuProcess()};

    // Frame
    string lidarFrame, baselinkFrame, odometryFrame, mapFrame;

    // 融合imuRPY权重
    bool imuRPYEnable;
    std::vector<double>imuRPYWeight{3,0.0};
    Eigen::Vector3d imu_init;
    int imuCnt = 0;
    double angleDiff;
    bool updateLio;

    /*** variables definition ***/
    int effect_feat_num = 0, frame_num = 0;
    double deltaT, deltaR, aver_time_consu = 0, aver_time_icp = 0, aver_time_match = 0, aver_time_incre = 0, aver_time_solve = 0, aver_time_const_H_time = 0;
    bool flg_EKF_converged, EKF_stop_flg = 0;
    FILE *fp;

    //vel 
    Eigen::Isometry3d lastPose3d = Eigen::Isometry3d::Identity();
    double last_lidar_end_time = 0;

    Eigen::Isometry3d rotateLidarToBody = Eigen::Isometry3d::Identity();
    bool lidarRpyInit = false;

    // localization
    bool isLocalization = false;
    bool isPriorPoseInit = false;
    pcl::PointCloud<PointType>::Ptr priorMap{new pcl::PointCloud<PointType>()};  
    pcl::PointCloud<PointType>::Ptr priorMapFilter{new pcl::PointCloud<PointType>()};
    std::string mapDirectory;   

    //relocalization
    // scmanager
    SCManager scManager;
    // std::vector<pcl::PointCloud<PointType>::Ptr> priorMapKeyFrames;
    pcl::PointCloud<PointType>::Ptr priorMapKeyPose{new pcl::PointCloud<PointType>()};         // 历史关键帧位姿（位置）
    pcl::KdTreeFLANN<PointType>::Ptr priorMapKeyPoseKdtree{new pcl::KdTreeFLANN<PointType>()};  
    pcl::PointCloud<PointType>::Ptr priorMapKdtree{new pcl::PointCloud<PointType>()};
    std::mutex prior_map_mutex;  
    // init cloud vector
    Eigen::Isometry3d initPose3d = Eigen::Isometry3d::Identity();
    bool initPoseHeading = false;
    int reLocalizationMode = 0;
    std::mutex init_feats_down_body_mutex;
    std::queue<std::pair<int, PointCloudXYZI::Ptr>> init_feats_down_bodys;
    int init_count = 0;
    std::pair<int, Eigen::Matrix4d> init_result;
    std::mutex global_localization_finish_state_mutex;
    bool global_localization_finish = false;
    bool global_update = false;
    std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d>> position_map;
    std::vector<Eigen::Quaterniond, Eigen::aligned_allocator<Eigen::Quaterniond>> pose_map;
    std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d>> position_init;
    std::vector<Eigen::Quaterniond, Eigen::aligned_allocator<Eigen::Quaterniond>> pose_init; 
    KD_TREE<PointType> ikdtree_global;
    std::thread *globalLocalizationThread;

    ofstream fout_pre, fout_out, fout_dbg;

    ros::Subscriber subPcl;
    ros::Subscriber subImu;
    ros::Subscriber subWheelVel;
    ros::Subscriber subTargetPose;
    ros::Subscriber subMode;
    ros::Subscriber subInitPose;

    ros::Publisher pubLaserCloudFull; //  world系下稠密点云
    ros::Publisher pubLaserCloudFull_body; // body系下稠密点云
    ros::Publisher pubOdomAftMapped;
    ros::Publisher pubPath;
    ros::Publisher pubPosUpdate;
    ros::Publisher pubLaserCloudMap;
    ros::Publisher pubTwist;
    ros::Publisher pubPriorMap;

    ros::Timer mainLoopTimer;
    std::thread *mainThread;

public:
    // tools
    template<typename T>
    void set_posestamp(T & out)
    {
        out.pose.position.x = state_point.pos(0);
        out.pose.position.y = state_point.pos(1);
        out.pose.position.z = state_point.pos(2);
        out.pose.orientation.x = geoQuat.x;
        out.pose.orientation.y = geoQuat.y;
        out.pose.orientation.z = geoQuat.z;
        out.pose.orientation.w = geoQuat.w;
    }
    template<typename T>
    void set_twistamp(T & out)
    {
        out.twist.linear.x = state_point.vel(0);
        out.twist.linear.y = state_point.vel(1);
        out.twist.linear.z = state_point.vel(2);
    }
    
    //按当前body(lidar)的状态，将局部点转换到世界系下
    void pointBodyToWorld(PointType const * const pi, PointType * const po)
    {
        V3D p_body(pi->x, pi->y, pi->z);
        V3D p_global(state_point.rot * (state_point.offset_R_L_I*p_body + state_point.offset_T_L_I) + state_point.pos);

        po->x = p_global(0);
        po->y = p_global(1);
        po->z = p_global(2);
        po->intensity = pi->intensity;
    }
    template<typename T>
    void pointBodyToWorld(const Matrix<T, 3, 1> &pi, Matrix<T, 3, 1> &po)
    {
        V3D p_body(pi[0], pi[1], pi[2]);
        V3D p_global(state_point.rot * (state_point.offset_R_L_I*p_body + state_point.offset_T_L_I) + state_point.pos);

        po[0] = p_global(0);
        po[1] = p_global(1);
        po[2] = p_global(2);
    }
    void RGBpointBodyToWorld(PointType const * const pi, PointType * const po)
    {
        V3D p_body(pi->x, pi->y, pi->z);
        V3D p_global(state_point.rot * (state_point.offset_R_L_I*p_body + state_point.offset_T_L_I) + state_point.pos);

        po->x = p_global(0);
        po->y = p_global(1);
        po->z = p_global(2);
        po->intensity = pi->intensity;
    }
    void RGBpointBodyLidarToIMU(PointType const * const pi, PointType * const po)
    {
        V3D p_body_lidar(pi->x, pi->y, pi->z);
        V3D p_body_imu(state_point.offset_R_L_I*p_body_lidar + state_point.offset_T_L_I);

        po->x = p_body_imu(0);
        po->y = p_body_imu(1);
        po->z = p_body_imu(2);
        po->intensity = pi->intensity;
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
};
}
#endif