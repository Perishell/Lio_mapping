/*
 * @Author: dyhan
 * @Date: 2022-10-12 09:26:31
 * @LastEditors: dyhan
 * @LastEditTime: 2024-10-18 15:11:25
 * @Description: 
 */
#ifndef OPTIMIZATION_H
#define OPTIMIZATION_H

#include "lio/common_op.h"
#include "conversions.h"
// save map
#include "fusion_slam/save_map.h"
#include "fusion_slam/save_pose.h"
#include "fusion_slam/r_map_info_msg.h"
#include "fusion_slam/gps_to_map.h"
#include "fusion_slam/map_to_gps.h"
#include <std_msgs/Int32MultiArray.h>
#include <std_msgs/MultiArrayDimension.h>
// visual loop
// #include <cv_bridge/cv_bridge.h>
// #include <image_transport/image_transport.h>
// #include <sensor_msgs/Image.h>
// #include <sensor_msgs/CompressedImage.h>
// #include "orb-slam/keyframedb.h"

// #include "glog/logging.h"
#include "slog/UserPrint.h"

using namespace std;

// 三维数组类型定义 (x, y, z -> 0=Free, 1=Occupied)
typedef std::vector<std::vector<std::vector<int>>> Grid3D;

namespace fusion_slam{
class optimization
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    optimization(){};
    ~optimization(){};

private:
    /*back end*/
    bool backEndEnable = 0;
    // vector<pcl::PointCloud<PointType>::Ptr> cornerCloudKeyFrames; // 历史所有关键帧的角点集合（降采样）
    vector<pcl::PointCloud<PointType>::Ptr> surfCloudKeyFrames;   // 历史所有关键帧的平面点集合（降采样）
    vector<pcl::PointCloud<PointType>::Ptr> cloudKeyFramesForVisual;   // 历史所有关键帧的平面点集合（降采样）

    pcl::PointCloud<PointType>::Ptr cloudKeyPoses3D{new pcl::PointCloud<PointType>()};         // 历史关键帧位姿（位置）
    pcl::PointCloud<PointTypePose>::Ptr cloudKeyPoses6D{new pcl::PointCloud<PointTypePose>()}; // 历史关键帧位姿
    pcl::PointCloud<PointType>::Ptr copy_cloudKeyPoses3D{new pcl::PointCloud<PointType>()};
    pcl::PointCloud<PointTypePose>::Ptr copy_cloudKeyPoses6D{new pcl::PointCloud<PointTypePose>()};
    // pcl::PointCloud<PointType>::Ptr save_cloudKeyPoses3D{new pcl::PointCloud<PointType>()};
    // pcl::PointCloud<PointTypePose>::Ptr save_cloudKeyPoses6D{new pcl::PointCloud<PointTypePose>()};

    pcl::PointCloud<PointTypePose>::Ptr lio_cloudKeyPoses6D{new pcl::PointCloud<PointTypePose>()}; //  存储fastlio 未优化的位姿
    pcl::PointCloud<PointTypePose>::Ptr gnss_cloudKeyPoses6D{new pcl::PointCloud<PointTypePose>()}; //  gnss 轨迹
    pcl::PointCloud<PointTypePose>::Ptr old_cloudKeyPoses6D{new pcl::PointCloud<PointTypePose>()};

    //保存地图文件
    pcl::PointCloud<PointType>::Ptr globalMapCloudAll{new pcl::PointCloud<PointType>()};
    pcl::PointCloud<PointType>::Ptr globalMapCloud2d{new pcl::PointCloud<PointType>()};

    // 局部稠密地图
    // std::queue<PointTypePose> local_cloudPoses6D;
    std::queue<pcl::PointCloud<PointType>::Ptr> localCloudFrames;   // 局部点云帧的平面点集合（降采样）
    // pcl::PointCloud<PointType>::Ptr localMapCloud{new pcl::PointCloud<PointType>()};
    pcl::PointCloud<PointType>::Ptr localMapCloudDS{new pcl::PointCloud<PointType>()};

    pcl::PointCloud<PointType>::Ptr curFrame{new pcl::PointCloud<PointType>()};
    
    double lidarEndTime;

    // voxel filter paprams
    float odometrySurfLeafSize;
    // float mappingCornerLeafSize;
    float mappingSurfLeafSize;

    float z_tollerance;
    float rotation_tollerance;

    // CPU Params
    int numberOfCores = 8;
    double mappingProcessInterval;

    /*loop clousre*/
    bool startFlag = true;
    bool loopClosureEnableFlag;
    float loopClosureFrequency; //   回环检测频率
    int surroundingKeyframeSize;
    float historyKeyframeSearchRadius;   // 回环检测 radius kdtree搜索半径
    float historyKeyframeSearchTimeDiff; //  帧间时间阈值
    int historyKeyframeSearchNum;        //   回环时多少个keyframe拼成submap
    float historyKeyframeFitnessScore;   // icp 匹配阈值
    bool potentialLoopFlag = false;

    ros::Publisher pubHistoryKeyFrames; //  发布 loop history keyframe submap
    ros::Publisher pubIcpKeyFrames;
    ros::Publisher pubRecentKeyFrames;
    ros::Publisher pubRecentKeyFrame;
    ros::Publisher pubCloudRegisteredRaw;
    ros::Publisher pubLoopConstraintEdge;
    ros::Publisher pubPathUpdate;
    ros::Publisher pubGpsConstraintEdge;
    ros::Publisher pubGpsPath;
    ros::Publisher pubMapPos, pubGpsPos;
    ros::Publisher pubOdomUpdate, pubOdomLidar;
    ros::Publisher pubLocalMap;
    ros::Publisher pubOctoMap;
    ros::Publisher pubGrid;
    ros::Publisher pubTwist;
    ros::Publisher pubSlamPose;

    ros::Subscriber subMode;
    ros::Subscriber subArmMode;
    ros::Subscriber subSegmentSignal;
    ros::Subscriber subGPS;
    ros::Subscriber subRPY;
    ros::Subscriber subUAV;
    ros::Subscriber subIMU;

    std::string gpsTopic;
    std::string rpyTopic;
    std::string uavTopic;
    std::string imuTopic;

    // gps
    bool bUseGps = 0;
    bool addGps;
    bool gpsInitOrientation;
    double gpsCovThreshold;
    bool useGpsElevation = false;
    bool debugGps;
    double gpsDistance;
    int gpsFrequence;
    int gpsInitPointCnt;
    bool updateLioOnce;
    std::vector<double> RPYGPS2Imu{0,0,0};

    pcl::PointCloud<PointType>::Ptr cloudKeyGPSPoses3D{new pcl::PointCloud<PointType>()};
    std::vector<gtsam::GPSFactor> keyframeGPSfactor;
    std::queue<sensor_msgs::NavSatFix::ConstPtr>gpsBuff;
    std::queue<sensor_msgs::Imu>imuBuff;
    std::queue<geometry_msgs::Vector3Stamped>bdRPYBuff;
    std::queue<geometry_msgs::Vector3Stamped>UAVBuff;
    std::queue<nav_msgs::Odometry>gpsOdomBuff;
    map<int, int> gpsIndexContainer;   // from new to old
    nav_msgs::Path gpsPath;

    bool bFirstGPS = true;
    bool bUseGnssYaw;
    bool bRPYInit = false;
    bool gtSAMgraphMade = false;
    bool gpsInitialized = false;
    bool updateLioFlag = 0;
    int rpyInitMode;
    std::string UTMZone;
    Eigen::Vector3d zeroUTM;
    double yawInitRPY;
    Eigen::AngleAxisd rotationVector = Eigen::AngleAxisd::Identity();
    PointType lastGPSPoint;
    int gpsToKeyPoseIdx;
    double gpsNoiseDefault;
    double gpsNoiseScale;
    double blindSpot;
    bool save_to_file;
    std::string file_path;
    double octomap_resolution;
    double octomap_size_x;
    double octomap_size_y;
    double octomap_size_z;
    double curGpsTime = 0.0;
    double lastGpsTime = 0.0;

    // 坐标转换服务
    ros::ServiceServer srvGpsToMap;
    ros::ServiceServer srvMapToGps;

    // gps yaw init
    std::vector<Eigen::Vector3d> lidarOdomPoses, GNSSPoses;
    std::vector<double>lidarOdomTime, GNSSPosesTime;
    Eigen::Isometry3d T_lw;

    Eigen::Isometry3d lastLioPose;
    Eigen::Isometry3d lastOptPose;

    // 噪声定义
    double gpsAltitudeNoiseScore, gpsLatitudeNoiseScore;
    gtsam::noiseModel::Diagonal::shared_ptr priorNoise;
    gtsam::noiseModel::Diagonal::shared_ptr odomNoise;
    gtsam::noiseModel::Base::shared_ptr robustGPSNoise;
    // 保存
    std::fstream pgGpsSaveStream;
    std::fstream pgOdomSaveStream;
    std::fstream pgOdomOpSaveStream;

    bool loopMode = 0;
    bool aLoopIsClosed = false;
    map<int, int> loopIndexContainer; // from new to old
    vector<pair<int, int>> loopIndexQueue;
    vector<gtsam::Pose3> loopPoseQueue;
    // vector<gtsam::noiseModel::Diagonal::shared_ptr> loopNoiseQueue;
    vector<gtsam::SharedNoiseModel> loopNoiseQueue;
    deque<std_msgs::Float64MultiArray> loopInfoVec;

    nav_msgs::Path globalPath;

    pcl::KdTreeFLANN<PointType>::Ptr kdtreeHistoryKeyPoses{new pcl::KdTreeFLANN<PointType>()};

    // 降采样
    pcl::VoxelGrid<PointType> downSizeFilterICP;
    pcl::VoxelGrid<PointType> downSizeFilterSurroundingKeyPoses; // for surrounding key poses of scan-to-map optimization
    pcl::VoxelGrid<PointType> downSizeFilterMap;               //单帧内降采样使用voxel grid

    Eigen::Isometry3d transformTobeMapped; // 当前帧的位姿(world系下)

    std::mutex mtx;
    std::mutex mtxLoopInfo;
    std::mutex mtxFloorInfo;
    std::mutex mtxBuff;
    std::mutex mtxGps;

    // Surrounding map
    float surroundingkeyframeAddingDistThreshold;  //  判断是否为关键帧的距离阈值
    float surroundingkeyframeAddingAngleThreshold; //  判断是否为关键帧的角度阈值
    float surroundingKeyframeDensity;
    float surroundingKeyframeSearchRadius;

    // gtsam
    gtsam::NonlinearFactorGraph gtSAMgraph;
    gtsam::Values initialEstimate;
    gtsam::ISAM2 *isam;
    gtsam::Values isamCurrentEstimate;
    Eigen::MatrixXd poseCovariance;

    ros::Publisher pubLaserCloudSurround;
    ros::Publisher pubOptimizedGlobalMap ;           //   发布最后优化的地图

    bool recontructKdTree = false;
    int updateKdtreeCount = 0 ;        //  每100次更新一次
    bool visulize_IkdtreeMap = false;            //  visual iktree submap

    double angleThreshold;
    double transThreshold;

    // global map visualization radius
    float globalMapVisualizationSearchRadius;
    float globalMapVisualizationPoseDensity;
    float globalMapVisualizationLeafSize;

    // saveMap
    bool isFullMap;
    ros::ServiceServer srvSaveMap;
    ros::ServiceServer srvSavePose;
    bool savePCD;               // 是否保存地图
    bool savePose;
    string savePCDDirectory;    // 保存路径
    ros::Publisher pubPosePath;//发布轨迹用于返航
    ros::Publisher pubMapInfo; //发布地图信息to代理
    
    // 分层保存
    std::vector<int> segmentList;
    // 当前楼层
    int floorNum = 0;
    // 当前楼层的部分关键帧起始id
    struct FloorFrames
    {
        int startIdx;
        int endIdx;
    };
    // 当前楼层的所有关键帧起始id
    using FloorState = std::vector<FloorFrames>;
    // 建筑物楼层和关键帧记录
    unordered_map<int, FloorState> floorStates;
    int lastFloorFrame = 0;
    bool is_seg_map_ = 0;
    int saveFrameInterval; //发布xyz点云地图的关键帧间隔

    // local map
    bool isNeedLocalMap = 0;
    Eigen::Isometry3d lastLioPoseLocalMap = Eigen::Isometry3d::Identity();

    // tf
    double carRadius;
    Eigen::Vector3d initLidarRPY = Eigen::Vector3d::Zero();
    Eigen::Quaterniond lidarToBaselink = Eigen::Quaterniond(1, 0, 0, 0);
    bool lidarInit = false;

    // vel
    Eigen::Isometry3d lastBasePose3d = Eigen::Isometry3d::Identity();
    double lastLidarTime = 0;

    // Frame
    string lidarFrame, baselinkFrame, odometryFrame, mapFrame;

    std::thread *loopthread;
    std::thread *savemapthread;

    double roll = 0.0;  // 绕X轴旋转的角度（弧度）
    double pitch = 0.0; // 绕Y轴旋转的角度（弧度）
    double yaw = 0.0;   // 绕Z轴旋转的角度（弧度）
    
public:
    // 初始化
    void initROS(ros::NodeHandle* nh, ros::NodeHandle* priNh);
    void initVariable();
    void subAndPubToROS(ros::NodeHandle *nh);
    void loadParams(ros::NodeHandle *priNh);
    void initNoises();

    // rviz展示闭环边
    void visualizeLoopClosure();
    // 间隔发布轨迹显示
    void publishPathUpdate(const ros::Publisher pubPath);
    // 发布所有关键点位姿
    void publishAllPath(const ros::Publisher pubPath);
    // gps边
    void visualGPSConstraint();

    // 更新里程计轨迹
    void updatePath(const PointTypePose &pose_in);
    // 计算当前帧与前一帧位姿变换，如果变化太小，不设为关键帧，反之设为关键帧
    bool saveFrame();

    pcl::PointCloud<PointType>::Ptr transformPointCloud(pcl::PointCloud<PointType>::Ptr cloudIn, PointTypePose *transformIn);
    void transformPointCloudNew(pcl::PointCloud<PointType>::Ptr cloudIn, PointTypePose *transformIn);

    // 添加激光里程计因子
    void addOdomFactor();
    // 添加闭环因子
    void addLoopFactor();
    // 添加gps因子
    void addGpsFactor();

    bool syncGps(nav_msgs::Odometry& curGpsOdom, const double& eps);
    bool gpsFactorInitial();
    bool initRPY();
    void umeyamaAlignment();

    bool syncImuLidar(sensor_msgs::Imu& imuSync);

    // 点云投影到地面
    void projectToGround(pcl::PointCloud<PointType>::Ptr thisKeyFrame);

    void saveCloudKeyFramesForVisual(pcl::PointCloud<PointType>::Ptr thisKeyFrame, PointType thisPose3D);
    // 主程序
    bool mainRun();
    // 更新优化后的位姿
    void correctPoses();
    // 获取优化后的当前帧位姿
    bool getOpPose(Eigen::Isometry3d& poseOp);
    // 判断是否需要重构ikdtree
    bool isNeedReconstruct();
    // 获取优化后的点云
    bool getIKdTreeCloud(pcl::PointCloud<PointType>::Ptr mapCloudFrames);
    // 插入当前帧
    void setOdomAndFrame(const Eigen::Isometry3d& poseFromLio, const pcl::PointCloud<PointType>::Ptr frameFromLio, const double& frameEndTime);
    // 发布局部地图
    void sendLocalMap(const Eigen::Isometry3d& poseFromLio, const double& frameEndTime, pcl::PointCloud<PointType>::Ptr frameFromLio);
    
    //回环检测线程
    void loopClosureThread();
    void performLoopClosure();
    void loopFindNearKeyframes(pcl::PointCloud<PointType>::Ptr &nearKeyframes, const int &key, const int &searchNum);
    bool detectLoopClosureDistance(int *latestID, int *closestID);
    void getCurFloorcloudKeyPoses3D(pcl::PointCloud<PointType>::Ptr curFloorcloudKeyPoses3D, 
        pcl::PointCloud<PointTypePose>::Ptr curFloorcloudKeyPoses6D, 
        std::vector<int>& idxCloudKeyPoses);

    bool saveSegmentMap(int startIdx, int endIdx, double resolution, std::string saveMapPath, bool saveMap, bool visualMap);
    void segmentSignalHandler(const std_msgs::UInt8ConstPtr& signal_msg);

    void mainShutDown();
    
    void savePclToXYZ(pcl::PointCloud<PointType>::Ptr cloudIn, std::string xyzFilePath);
    void saveAndPublishMapInfo();
    void saveAndPublishPose();
    void saveMapThread();

    bool saveMapService(fusion_slam::save_mapRequest& req, fusion_slam::save_mapResponse& res);
    bool savePoseService(fusion_slam::save_poseRequest& req, fusion_slam::save_poseResponse& res);

    bool gpsToMapService(fusion_slam::gps_to_mapRequest& req, fusion_slam::gps_to_mapResponse& res);
    bool mapToGpsService(fusion_slam::map_to_gpsRequest& req, fusion_slam::map_to_gpsResponse& res);

    void modeHandler(const std_msgs::UInt8::ConstPtr& modeMsg);
    void armModeHandler(const std_msgs::UInt8::ConstPtr& armModeMsg);

    void gpsHandler(const sensor_msgs::NavSatFix::ConstPtr &gpsMsg);
    void rpyHandler(const geometry_msgs::Vector3Stamped::ConstPtr &rpyMsg);
    void uavHandler(const geometry_msgs::Vector3Stamped::ConstPtr &uavMsg);
    void imuHandler(const sensor_msgs::Imu::ConstPtr &imuMsg);

    void publishGrid3D(const Grid3D& grid, int size_x, int size_y, int size_z);
    void saveGrid3D(const Grid3D& grid, const std::string& path);
};
}

#endif