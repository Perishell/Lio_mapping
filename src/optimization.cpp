/*
 * @Author: dyhan
 * @Date: 2022-10-12 09:23:06
 * @LastEditors: dyhan
 * @LastEditTime: 2024-10-30 10:07:56
 * @Description: 
 */
#include "lio/optimization.h"
#include <pcl/filters/passthrough.h>
#include <octomap/octomap.h>
#include <octomap_msgs/Octomap.h>
#include <octomap_msgs/conversions.h>



namespace fusion_slam
{
void optimization::initROS(ros::NodeHandle* nh, ros::NodeHandle* priNh)
{
    LogMgmt::GetInstance();

    loadParams(nh);

    initVariable();
    initNoises();

    subAndPubToROS(nh);

    // 回环检测线程
    loopthread = new std::thread(std::bind(&optimization::loopClosureThread, this));

    // 地图保存发布线程
    savemapthread = new std::thread(std::bind(&optimization::saveMapThread, this));
}

void optimization::loadParams(ros::NodeHandle *priNh)
{
    // Frame
    priNh->param<std::string>("common/lidarFrame", lidarFrame, "base_link"); 
    priNh->param<std::string>("common/baselinkFrame", baselinkFrame, "base_link");
    priNh->param<std::string>("common/odometryFrame", odometryFrame, "odom");
    priNh->param<std::string>("common/mapFrame", mapFrame, "map");  

    priNh->param<float>("lio_sam/odometrySurfLeafSize", odometrySurfLeafSize, 0.2);
    // priNh->param<float>("lio_sam/mappingCornerLeafSize", mappingCornerLeafSize, 0.2);
    priNh->param<float>("lio_sam/mappingSurfLeafSize", mappingSurfLeafSize, 0.2);
    priNh->param<float>("lio_sam/z_tollerance", z_tollerance, FLT_MAX);
    priNh->param<float>("lio_sam/rotation_tollerance", rotation_tollerance, FLT_MAX);
    priNh->param<int>("lio_sam/numberOfCores", numberOfCores, 2);
    priNh->param<double>("lio_sam/mappingProcessInterval", mappingProcessInterval, 0.15);
    // save keyframes
    priNh->param<float>("lio_sam/surroundingkeyframeAddingDistThreshold", surroundingkeyframeAddingDistThreshold, 20.0);
    priNh->param<float>("lio_sam/surroundingkeyframeAddingAngleThreshold", surroundingkeyframeAddingAngleThreshold, 0.2);
    priNh->param<float>("lio_sam/surroundingKeyframeDensity", surroundingKeyframeDensity, 1.0);
    priNh->param<float>("lio_sam/surroundingKeyframeSearchRadius", surroundingKeyframeSearchRadius, 50.0);
    // loop clousre
    priNh->param<bool>("lio_sam/loopClosureEnableFlag", loopClosureEnableFlag, false);
    priNh->param<float>("lio_sam/loopClosureFrequency", loopClosureFrequency, 1.0);
    priNh->param<int>("lio_sam/surroundingKeyframeSize", surroundingKeyframeSize, 50);
    priNh->param<float>("lio_sam/historyKeyframeSearchRadius", historyKeyframeSearchRadius, 10.0);
    priNh->param<float>("lio_sam/historyKeyframeSearchTimeDiff", historyKeyframeSearchTimeDiff, 30.0);
    priNh->param<int>("lio_sam/historyKeyframeSearchNum", historyKeyframeSearchNum, 25);
    priNh->param<float>("lio_sam/historyKeyframeFitnessScore", historyKeyframeFitnessScore, 0.3);
    // Visualization
    priNh->param<float>("lio_sam/globalMapVisualizationSearchRadius", globalMapVisualizationSearchRadius, 1e3);
    priNh->param<float>("lio_sam/globalMapVisualizationPoseDensity", globalMapVisualizationPoseDensity, 10.0);
    priNh->param<float>("lio_sam/globalMapVisualizationLeafSize", globalMapVisualizationLeafSize, 1.0);
    // visual ikdtree map
    priNh->param<bool>("lio_sam/visulize_IkdtreeMap", visulize_IkdtreeMap, false);
    // reconstruct ikdtree 
    priNh->param<bool>("lio_sam/recontructKdTree", recontructKdTree, false);
    priNh->param<bool>("lio_sam/backEndEnable", backEndEnable, false);
    priNh->param<double>("lio_sam/angleThreshold", angleThreshold, 0.2);
    priNh->param<double>("lio_sam/transThreshold", transThreshold, 0.2);
    priNh->param<bool>("lio_sam/isNeedLocalMap", isNeedLocalMap, false);

    // savMap
    priNh->param<bool>("lio_sam/isFullMap", isFullMap, false);
    priNh->param<bool>("lio_sam/savePCD", savePCD, false);
    priNh->param<std::string>("lio_sam/savePCDDirectory", savePCDDirectory, "/Downloads/LOAM/"); 

    priNh->param<int>("lio_sam/saveFrameInterval", saveFrameInterval, 20);

    priNh->param<double>("lio_sam/carRadius", carRadius, 0.33);

    priNh->param<bool>("gps/bUseGps", bUseGps, false);
    priNh->param<bool>("gps/debugGps", debugGps, false);
    priNh->param<bool>("gps/addGps", addGps, false);
    priNh->param<bool>("gps/gpsInitOrientation", gpsInitOrientation, false);
    priNh->param<bool>("gps/updateLioOnce", updateLioOnce, false);
    priNh->param<bool>("gps/bUseGnssYaw", bUseGnssYaw, false);
    priNh->param<std::string>("gps/gpsTopic", gpsTopic, "/gps/fixed");  
    priNh->param<std::string>("gps/rpyTopic", rpyTopic, "/Euler_Angles");
    priNh->param<std::string>("uav/uavTopic", uavTopic, "/bd_rpy");
    priNh->param<std::string>("gps/imuTopic", imuTopic, "/imu_raw");
    priNh->param<double>("gps/gpsAltitudeNoiseScore", gpsAltitudeNoiseScore, 10.0);
    priNh->param<double>("gps/gpsLatitudeNoiseScore", gpsLatitudeNoiseScore, 100.0);
    priNh->param<std::vector<double> >("gps/RPYGPS2Imu", RPYGPS2Imu, std::vector<double>());
    priNh->param<int>("gps/rpyInitMode", rpyInitMode, 0);
    priNh->param<int>("gps/gpsFrequence", gpsFrequence, 10);
    priNh->param<int>("gps/gpsInitPointCnt", gpsInitPointCnt, 20);
    priNh->param<bool>("gps/useGpsElevation", useGpsElevation, false);
    priNh->param<double>("gps/gpsCovThreshold", gpsCovThreshold, 2.0);
    priNh->param<double>("gps/gpsDistance", gpsDistance, 0.5);
    priNh->param<double>("gps/gpsNoiseDefault", gpsNoiseDefault, 50);
    priNh->param<double>("gps/gpsNoiseScale", gpsNoiseScale, 10);

    // octomap
    priNh->param<double>("octomap/blindSpot", blindSpot, 2.0);
    priNh->param<double>("octomap/octomap_resolution", octomap_resolution, 1.0);
    priNh->param<double>("octomap/octomap_size_x", octomap_size_x, 40.0);
    priNh->param<double>("octomap/octomap_size_y", octomap_size_y, 40.0);
    priNh->param<double>("octomap/octomap_size_z", octomap_size_z, 20.0);
    
    // 3dGrid
    priNh->param<bool>("Grid/save_to_file", save_to_file, true);
    priNh->param<std::string>("Grid/file_path", file_path, std::string("grid3d.bin")); 
}

void optimization::subAndPubToROS(ros::NodeHandle *nh)
{
    pubPathUpdate = nh->advertise<nav_msgs::Path>("fusion_slam/path_update", 10);                   //  isam更新后的path
    pubLaserCloudSurround = nh->advertise<sensor_msgs::PointCloud2>("fusion_slam/mapping/keyframe_submap", 1); // 发布局部关键帧map的特征点云
    pubOptimizedGlobalMap = nh->advertise<sensor_msgs::PointCloud2>("fusion_slam/mapping/map_global_optimized", 1); // 发布局部关键帧map的特征点云

    pubMapInfo = nh->advertise<fusion_slam::r_map_info_msg>("/remote_map_info_report_topic", 10); 

    // loop clousre
    // 发布闭环匹配关键帧局部map
    pubHistoryKeyFrames = nh->advertise<sensor_msgs::PointCloud2>("fusion_slam/mapping/icp_loop_closure_history_cloud", 1);
    // 发布当前关键帧经过闭环优化后的位姿变换之后的特征点云
    pubIcpKeyFrames = nh->advertise<sensor_msgs::PointCloud2>("fusion_slam/mapping/icp_loop_closure_corrected_cloud", 1);
    // 发布闭环边，rviz中表现为闭环帧之间的连线
    pubLoopConstraintEdge = nh->advertise<visualization_msgs::MarkerArray>("/fusion_slam/mapping/loop_closure_constraints", 1);

    // saveMap  发布地图保存服务
    srvSaveMap  = nh->advertiseService("/save_map" ,  &optimization::saveMapService, this);
    // savePose  发布轨迹保存服务
    srvSavePose = nh->advertiseService("/save_pose" ,  &optimization::savePoseService, this);
    pubPosePath = nh->advertise<nav_msgs::Path>("/pose_path_topic", 10);

    subSegmentSignal = nh->subscribe<std_msgs::UInt8> ("/segmentation_signal_topic", 10, &optimization::segmentSignalHandler, this);
    subMode = nh->subscribe<std_msgs::UInt8>("/planMode", 5, &optimization::modeHandler, this);
    // subArmMode = nh->subscribe<std_msgs::UInt8>("/remote_arm_mode_topic", 5, &optimization::armModeHandler, this);

    subRPY = nh->subscribe<geometry_msgs::Vector3Stamped>(rpyTopic, 100, &optimization::rpyHandler, this);
    subUAV = nh->subscribe<geometry_msgs::Vector3Stamped>(uavTopic, 100, &optimization::uavHandler, this);
    subGPS = nh->subscribe<sensor_msgs::NavSatFix>(gpsTopic, 10, &optimization::gpsHandler, this);
    subIMU = nh->subscribe<sensor_msgs::Imu>(imuTopic, 20, &optimization::imuHandler, this);

    pubGpsConstraintEdge = nh->advertise<visualization_msgs::MarkerArray>("/fusion_slam/mapping/gps_constraints", 1);
    pubGpsPath = nh->advertise<nav_msgs::Path>("/fusion_slam/mapping/gps_path", 100);

    pubMapPos = nh->advertise<nav_msgs::Odometry>("/robot_pose_topic", 10);
    pubGpsPos = nh->advertise<geometry_msgs::Point>("/robot_gps_topic", 10);
    pubOdomUpdate = nh->advertise<nav_msgs::Odometry>("/odom", 10); //odometry_update
    pubOdomLidar = nh->advertise<nav_msgs::Odometry>("/odom_lidar", 10); //odometry_lidar
    
    pubOctoMap = nh->advertise<octomap_msgs::Octomap>("/octomap", 10); //body系下局部Octomap
    pubGrid = nh->advertise<std_msgs::Int32MultiArray>("/grid3d", 10); // 3D栅格
    pubLocalMap = nh->advertise<sensor_msgs::PointCloud2>("/localMap", 10); // body系下局部点云

    pubTwist = nh->advertise<geometry_msgs::Twist>("/slam_twist", 10);

    pubSlamPose = nh->advertise<geometry_msgs::PoseWithCovarianceStamped>("/slam_pose", 10);

    //　发布坐标转换服务
    srvGpsToMap = nh->advertiseService("/gps_to_map", &optimization::gpsToMapService, this);
    srvMapToGps = nh->advertiseService("/map_to_gps", &optimization::mapToGpsService, this);

}

void optimization::initVariable()
{
    downSizeFilterICP.setLeafSize(mappingSurfLeafSize, mappingSurfLeafSize, mappingSurfLeafSize);
    downSizeFilterSurroundingKeyPoses.setLeafSize(surroundingKeyframeDensity, surroundingKeyframeDensity, surroundingKeyframeDensity); // for surrounding key poses of scan-to-map optimization

    loopMode = loopClosureEnableFlag;

    // ISAM2参数
    gtsam::ISAM2Params parameters;
    parameters.relinearizeThreshold = 0.01;
    parameters.relinearizeSkip = 1;
    isam = new gtsam::ISAM2(parameters);  

    // std::string posePath = std::getenv("HOME") + savePCDDirectory + "tum/";
    savePCDDirectory = std::getenv("HOME") + savePCDDirectory;
    std::string posePath = savePCDDirectory + "tum/";

    pgGpsSaveStream = std::fstream(posePath + "GpsTUM.txt", std::fstream::out);
    pgGpsSaveStream.precision(std::numeric_limits<double>::max_digits10);

    pgOdomOpSaveStream = std::fstream(posePath + "OdomOpTUM.txt", std::fstream::out);
    pgOdomOpSaveStream.precision(std::numeric_limits<double>::max_digits10);

    pgOdomSaveStream = std::fstream(posePath + "OdomTUM.txt", std::fstream::out);
    pgOdomSaveStream.precision(std::numeric_limits<double>::max_digits10);  

    if(rpyInitMode == 2)//set rpy
    {
        Eigen::AngleAxisd rollAngle(Eigen::AngleAxisd(RPYGPS2Imu[0], Eigen::Vector3d::UnitX()));
        Eigen::AngleAxisd pitchAngle(Eigen::AngleAxisd(RPYGPS2Imu[1], Eigen::Vector3d::UnitY()));
        Eigen::AngleAxisd yawAngle(Eigen::AngleAxisd(RPYGPS2Imu[2], Eigen::Vector3d::UnitZ()));//yaw_gps2utm        
        rotationVector = yawAngle*pitchAngle*rollAngle;
        bRPYInit = true;
    }

    T_lw = Eigen::Isometry3d::Identity();

}

// 噪声定义
void optimization::initNoises()
{
    gtsam::Vector priorNoiseVector6(6);
    priorNoiseVector6 << 1e-12, 1e-12, 1e-12, 1e-12, 1e-12, 1e-12;
    priorNoise = gtsam::noiseModel::Diagonal::Variances(priorNoiseVector6);

    gtsam::Vector odomNoiseVector6(6);
    odomNoiseVector6 << 1e-6, 1e-6, 1e-6, 1e-4, 1e-4, 1e-4;
    odomNoise = gtsam::noiseModel::Diagonal::Variances(odomNoiseVector6);
    
    gtsam::Vector robustNoiseVector3(3);  
    robustNoiseVector3 << gpsLatitudeNoiseScore, gpsLatitudeNoiseScore, gpsAltitudeNoiseScore; // gps factor has 3 elements (xyz)
    robustGPSNoise = gtsam::noiseModel::Robust::Create(
        gtsam::noiseModel::mEstimator::Cauchy::Create(1), // optional: replacing Cauchy by DCS or GemanMcClure is okay but Cauchy is empirically good.
        gtsam::noiseModel::Diagonal::Variances(robustNoiseVector3));

} // initNoises

/**
 * 更新里程计轨迹
 */
void optimization::updatePath(const PointTypePose &pose_in)
{
    // string odometryFrame = mapFrame;//"camera_init";
    geometry_msgs::PoseStamped pose_stamped;
    pose_stamped.header.stamp = ros::Time().fromSec(pose_in.time);

    pose_stamped.header.frame_id = odometryFrame;
    pose_stamped.pose.position.x = pose_in.x;
    pose_stamped.pose.position.y = pose_in.y;
    pose_stamped.pose.position.z = pose_in.z;
    pose_stamped.pose.orientation.x = pose_in.qx;
    pose_stamped.pose.orientation.y = pose_in.qy;
    pose_stamped.pose.orientation.z = pose_in.qz;
    pose_stamped.pose.orientation.w = pose_in.qw;

    globalPath.poses.push_back(pose_stamped);
}

//间隔发布优化后的轨迹
void optimization::publishPathUpdate(const ros::Publisher pubPath)
{
    ros::Time timeLaserInfoStamp = ros::Time().fromSec(lidarEndTime); //  时间戳
    // string odometryFrame = mapFrame;//"camera_init";
    if (pubPath.getNumSubscribers() != 0)
    {
        /*** if path is too large, the rvis will crash ***/
        static int kkk = 0;
        kkk++;
        // if (kkk % 10 == 0)
        {
            // path.poses.push_back(globalPath);
            globalPath.header.stamp = timeLaserInfoStamp;
            globalPath.header.frame_id = odometryFrame;
            pubPath.publish(globalPath);
        }
    }
}

//发布优化后的轨迹
void optimization::publishAllPath(const ros::Publisher pubPath)
{
    ros::Time timeLaserInfoStamp = ros::Time().fromSec(lidarEndTime); //  时间戳
    // string odometryFrame = mapFrame;//"camera_init";
    if (pubPath.getNumSubscribers() != 0)
    {
        // path.poses.push_back(globalPath);
        globalPath.header.stamp = timeLaserInfoStamp;
        globalPath.header.frame_id = odometryFrame;
        pubPath.publish(globalPath);
    }
}

/**
 * 对点云cloudIn进行变换transformIn，返回结果点云， 修改liosam, 考虑到外参的表示
 */
pcl::PointCloud<PointType>::Ptr optimization::transformPointCloud(pcl::PointCloud<PointType>::Ptr cloudIn, PointTypePose *transformIn)
{
    pcl::PointCloud<PointType>::Ptr cloudOut(new pcl::PointCloud<PointType>());

    int cloudSize = cloudIn->size();
    if(cloudSize == 0)
    {
        PrintWarn("transformPointCloud::cloudSize == 0");
        // return cloudOut;
    }
    cloudOut->resize(cloudSize);
    
    // 注意：lio_sam 中的姿态用的euler表示，而fastlio存的姿态角是旋转矢量。而 pcl::getTransformation是将euler_angle 转换到rotation_matrix 不合适，注释
    // Eigen::Affine3f transCur = pcl::getTransformation(transformIn->x, transformIn->y, transformIn->z, transformIn->roll, transformIn->pitch, transformIn->yaw);
    // Eigen::Isometry3d T_b_lidar(state_point.offset_R_L_I);       //  获取  body2lidar  外参
    // T_b_lidar.pretranslate(state_point.offset_T_L_I);        
    Eigen::Isometry3d T_b_lidar = Eigen::Isometry3d::Identity();

    Eigen::Isometry3d T_w_b = pclPointToIsometry3d(*transformIn); // world2body 
    Eigen::Isometry3d T_w_lidar  =  T_w_b * T_b_lidar;            // T_w_lidar  转换矩阵
    Eigen::Isometry3d transCur = T_w_lidar;        

    #pragma omp parallel for num_threads(numberOfCores)
    for (int i = 0; i < cloudSize; ++i)
    {
        const auto &pointFrom = cloudIn->points[i];
        cloudOut->points[i].x = transCur(0, 0) * pointFrom.x + transCur(0, 1) * pointFrom.y + transCur(0, 2) * pointFrom.z + transCur(0, 3);
        cloudOut->points[i].y = transCur(1, 0) * pointFrom.x + transCur(1, 1) * pointFrom.y + transCur(1, 2) * pointFrom.z + transCur(1, 3);
        cloudOut->points[i].z = transCur(2, 0) * pointFrom.x + transCur(2, 1) * pointFrom.y + transCur(2, 2) * pointFrom.z + transCur(2, 3);
        cloudOut->points[i].intensity = pointFrom.intensity;
    }
    return cloudOut;
}

/**
 * 对点云cloudIn进行变换transformIn，返回结果点云， 修改liosam, 考虑到外参的表示
 */
void optimization::transformPointCloudNew(pcl::PointCloud<PointType>::Ptr cloudIn, PointTypePose *transformIn)
{
    // pcl::PointCloud<PointType>::Ptr cloudOut(new pcl::PointCloud<PointType>());

    int cloudSize = cloudIn->size();
    if(cloudSize == 0)
    {
        PrintWarn("transformPointCloud::cloudSize == 0");
        return;
    }
    // cloudOut->resize(cloudSize);
    
    // 注意：lio_sam 中的姿态用的euler表示，而fastlio存的姿态角是旋转矢量。而 pcl::getTransformation是将euler_angle 转换到rotation_matrix 不合适，注释
    // Eigen::Affine3f transCur = pcl::getTransformation(transformIn->x, transformIn->y, transformIn->z, transformIn->roll, transformIn->pitch, transformIn->yaw);
    // Eigen::Isometry3d T_b_lidar(state_point.offset_R_L_I);       //  获取  body2lidar  外参
    // T_b_lidar.pretranslate(state_point.offset_T_L_I);        
    Eigen::Isometry3d T_b_lidar = Eigen::Isometry3d::Identity();

    Eigen::Isometry3d T_w_b = pclPointToIsometry3d(*transformIn); // world2body 
    Eigen::Isometry3d T_w_lidar  =  T_w_b * T_b_lidar;            // T_w_lidar  转换矩阵
    Eigen::Isometry3d transCur = T_w_lidar;        

    // #pragma omp parallel for num_threads(numberOfCores)
    for (int i = 0; i < cloudSize; ++i)
    {
        const auto &pointFrom = cloudIn->points[i];
        PointType curPoint;
        curPoint.x = transCur(0, 0) * pointFrom.x + transCur(0, 1) * pointFrom.y + transCur(0, 2) * pointFrom.z + transCur(0, 3);
        curPoint.y = transCur(1, 0) * pointFrom.x + transCur(1, 1) * pointFrom.y + transCur(1, 2) * pointFrom.z + transCur(1, 3);
        curPoint.z = transCur(2, 0) * pointFrom.x + transCur(2, 1) * pointFrom.y + transCur(2, 2) * pointFrom.z + transCur(2, 3);
        curPoint.intensity = pointFrom.intensity;
        globalMapCloudAll->push_back(curPoint);

        // if(pointFrom.z < 1.2 && pointFrom.z > 0.5)
        // {
        //     globalMapCloud2d->push_back(curPoint);
        // }
    }
    // return cloudOut;
}

void optimization::visualGPSConstraint() 
{
    if (gpsIndexContainer.empty()) return;

    visualization_msgs::MarkerArray markerArray;
    // gps nodes
    visualization_msgs::Marker markerNode;
    markerNode.header.frame_id = odometryFrame;
    markerNode.header.stamp = ros::Time().fromSec(lidarEndTime);
    markerNode.action = visualization_msgs::Marker::ADD;
    markerNode.type = visualization_msgs::Marker::SPHERE_LIST;
    markerNode.ns = "gps_nodes";
    markerNode.id = 0;
    markerNode.pose.orientation.w = 1;
    markerNode.scale.x = 0.3;
    markerNode.scale.y = 0.3;
    markerNode.scale.z = 0.3;
    markerNode.color.r = 0.8;
    markerNode.color.g = 0;
    markerNode.color.b = 1;
    markerNode.color.a = 1;

    // loop edges
    visualization_msgs::Marker markerEdge;
    markerEdge.header.frame_id = odometryFrame;
    markerEdge.header.stamp = ros::Time().fromSec(lidarEndTime);
    markerEdge.action = visualization_msgs::Marker::ADD;
    markerEdge.type = visualization_msgs::Marker::LINE_LIST;
    markerEdge.ns = "gps_edges";
    markerEdge.id = 1;
    markerEdge.pose.orientation.w = 1;
    markerEdge.scale.x = 0.2;
    markerEdge.color.r = 0.9;
    markerEdge.color.g = 0;
    markerEdge.color.b = 0.1;
    markerEdge.color.a = 1;

    for (auto it = gpsIndexContainer.begin(); it != gpsIndexContainer.end();++it) 
    {
        int key_cur = it->first;
        int key_pre = it->second;

        geometry_msgs::Point p;
        p.x = cloudKeyPoses6D->points[key_cur].x;
        p.y = cloudKeyPoses6D->points[key_cur].y;
        p.z = cloudKeyPoses6D->points[key_cur].z;
        markerNode.points.push_back(p);
        markerEdge.points.push_back(p);

        p.x = cloudKeyGPSPoses3D->points[key_pre].x;
        p.y = cloudKeyGPSPoses3D->points[key_pre].y;
        p.z = cloudKeyGPSPoses3D->points[key_pre].z;
        markerNode.points.push_back(p);
        markerEdge.points.push_back(p);
    }

    markerArray.markers.push_back(markerNode);
    markerArray.markers.push_back(markerEdge);
    pubGpsConstraintEdge.publish(markerArray);
}

/**
 * rviz展示闭环边
 */
void optimization::visualizeLoopClosure()
{
    ros::Time timeLaserInfoStamp = ros::Time().fromSec(lidarEndTime); //  时间戳
    // string odometryFrame = mapFrame;//"camera_init";

    if (loopIndexContainer.empty())
        return;

    visualization_msgs::MarkerArray markerArray;
    // 闭环顶点
    visualization_msgs::Marker markerNode;
    markerNode.header.frame_id = odometryFrame;
    markerNode.header.stamp = timeLaserInfoStamp;
    markerNode.action = visualization_msgs::Marker::ADD;
    markerNode.type = visualization_msgs::Marker::SPHERE_LIST;
    markerNode.ns = "loop_nodes";
    markerNode.id = 0;
    markerNode.pose.orientation.w = 1;
    markerNode.scale.x = 0.3;
    markerNode.scale.y = 0.3;
    markerNode.scale.z = 0.3;
    markerNode.color.r = 0;
    markerNode.color.g = 0.8;
    markerNode.color.b = 1;
    markerNode.color.a = 1;
    // 闭环边
    visualization_msgs::Marker markerEdge;
    markerEdge.header.frame_id = odometryFrame;
    markerEdge.header.stamp = timeLaserInfoStamp;
    markerEdge.action = visualization_msgs::Marker::ADD;
    markerEdge.type = visualization_msgs::Marker::LINE_LIST;
    markerEdge.ns = "loop_edges";
    markerEdge.id = 1;
    markerEdge.pose.orientation.w = 1;
    markerEdge.scale.x = 0.1;
    markerEdge.color.r = 0.9;
    markerEdge.color.g = 0.9;
    markerEdge.color.b = 0;
    markerEdge.color.a = 1;

    // 遍历闭环
    for (auto it = loopIndexContainer.begin(); it != loopIndexContainer.end(); ++it)
    {
        int key_cur = it->first;
        int key_pre = it->second;
        geometry_msgs::Point p;
        p.x = copy_cloudKeyPoses6D->points[key_cur].x;
        p.y = copy_cloudKeyPoses6D->points[key_cur].y;
        p.z = copy_cloudKeyPoses6D->points[key_cur].z;
        markerNode.points.push_back(p);
        markerEdge.points.push_back(p);
        p.x = copy_cloudKeyPoses6D->points[key_pre].x;
        p.y = copy_cloudKeyPoses6D->points[key_pre].y;
        p.z = copy_cloudKeyPoses6D->points[key_pre].z;
        markerNode.points.push_back(p);
        markerEdge.points.push_back(p);
    }

    markerArray.markers.push_back(markerNode);
    markerArray.markers.push_back(markerEdge);
    pubLoopConstraintEdge.publish(markerArray);
}

/**
 * 计算当前帧与前一帧位姿变换，如果变化太小，不设为关键帧，反之设为关键帧
 */
bool optimization::saveFrame()
{
    if (cloudKeyPoses3D->points.empty())
        return true;

    // 前一帧位姿
    Eigen::Isometry3d transStart = pclPointToIsometry3d(cloudKeyPoses6D->back());
    // 当前帧位姿
    Eigen::Isometry3d transFinal = transformTobeMapped;

    // 位姿变换增量
    Eigen::Isometry3d transBetween = transStart.inverse() * transFinal;
    Eigen::Vector3d transBetween_tran = transBetween.translation();
    Eigen::Vector3d transBetween_rot = quatToRPY(Eigen::Quaterniond(transBetween.rotation()));

    float tran_distance = transBetween_tran[0]*transBetween_tran[0]+
                            transBetween_tran[1]*transBetween_tran[1]+
                            transBetween_tran[2]*transBetween_tran[2];

    // 旋转和平移量都较小，当前帧不设为关键帧
    if (abs(transBetween_rot[0]) < surroundingkeyframeAddingAngleThreshold &&
        abs(transBetween_rot[1]) < surroundingkeyframeAddingAngleThreshold &&
        abs(transBetween_rot[2]) < surroundingkeyframeAddingAngleThreshold &&
        sqrt(tran_distance) < surroundingkeyframeAddingDistThreshold)
        return false;

    return true;
}

/**
 * 添加激光里程计因子
 */
void optimization::addOdomFactor()
{
    if (cloudKeyPoses3D->points.empty())
    {
        // 第一帧初始化先验因子
        // 第一个noise会让第一个关键帧位姿也发生平移和旋转，特殊情况考虑使用。
        gtsam::noiseModel::Diagonal::shared_ptr priorNoise;
        // if(gpsInitOrientation)
        //     priorNoise = gtsam::noiseModel::Diagonal::Variances((gtsam::Vector(6) << 1e-2, 1e-2, M_PI*M_PI, 1e8, 1e8, 1e8).finished()); // rad*rad, meter*meter
        // else
            priorNoise = gtsam::noiseModel::Diagonal::Variances((gtsam::Vector(6) <<1e-12, 1e-12, 1e-12, 1e-12, 1e-12, 1e-12).finished()); // rad*rad, meter*meter 
        gtSAMgraph.add(gtsam::PriorFactor<gtsam::Pose3>(0, trans2gtsamPose(transformTobeMapped), priorNoise));
        // 变量节点设置初始值
        initialEstimate.insert(0, trans2gtsamPose(transformTobeMapped));
        gtSAMgraphMade = true;
    }
    else
    {
        // 添加激光里程计因子
        gtsam::noiseModel::Diagonal::shared_ptr odometryNoise = gtsam::noiseModel::Diagonal::Variances((gtsam::Vector(6) << 1e-6, 1e-6, 1e-6, 1e-4, 1e-4, 1e-4).finished());
        gtsam::Pose3 poseFrom = pclPointTogtsamPose3(cloudKeyPoses6D->points.back()); /// pre
        gtsam::Pose3 poseTo = trans2gtsamPose(transformTobeMapped);                   // cur
        // 参数：前一帧id，当前帧id，前一帧与当前帧的位姿变换（作为观测值），噪声协方差
        gtSAMgraph.add(gtsam::BetweenFactor<gtsam::Pose3>(cloudKeyPoses3D->size() - 1, cloudKeyPoses3D->size(), poseFrom.between(poseTo), odometryNoise));
        // 变量节点设置初始值
        initialEstimate.insert(cloudKeyPoses3D->size(), poseTo);
    }
}

/**
 * 添加闭环因子
 */
void optimization::addLoopFactor()
{
    mtxLoopInfo.lock();
    if (loopIndexQueue.empty())
    {
        mtxLoopInfo.unlock();
        return;
    }

    // 闭环队列
    for (int i = 0; i < (int)loopIndexQueue.size(); ++i)
    {
        // 闭环边对应两帧的索引
        int indexFrom = loopIndexQueue[i].first; //   cur
        int indexTo = loopIndexQueue[i].second;  //    pre
        // 闭环边的位姿变换
        gtsam::Pose3 poseBetween = loopPoseQueue[i];
        // gtsam::noiseModel::Diagonal::shared_ptr noiseBetween = loopNoiseQueue[i];
        auto noiseBetween = loopNoiseQueue[i];
        gtSAMgraph.add(gtsam::BetweenFactor<gtsam::Pose3>(indexFrom, indexTo, poseBetween, noiseBetween));
    }

    loopIndexQueue.clear();
    loopPoseQueue.clear();
    loopNoiseQueue.clear();
    aLoopIsClosed = true;
    mtxLoopInfo.unlock();
}

bool optimization::syncGps(nav_msgs::Odometry& curGpsOdom, const double& eps)
{
    bool hasGPSforThisKF = false;
    while (!gpsOdomBuff.empty())
    {
        auto thisGPS = gpsOdomBuff.front();
        auto thisGPSTime = thisGPS.header.stamp.toSec();
        if (abs(thisGPSTime - lidarEndTime) < eps)
        {
            curGpsOdom = thisGPS;
            hasGPSforThisKF = true;
            gpsOdomBuff.pop();
            // PrintInfo("-%s-%d-:thisGPSTime = %.8f, lidarEndTime = %.8f", __func__, __LINE__, thisGPSTime, lidarEndTime);
            break;
        }
        else if(thisGPSTime - lidarEndTime <= -eps)
        {
            hasGPSforThisKF = false;
            gpsOdomBuff.pop();
            // PrintInfo("-%s-%d-:thisGPSTime - lidarEndTime <= -eps: thisGPSTime = %.8f, lidarEndTime = %.8f", __func__, __LINE__, thisGPSTime, lidarEndTime);
        }
        else if(thisGPSTime - lidarEndTime >= eps)
        {
            hasGPSforThisKF = false;
            // PrintInfo("-%s-%d-:thisGPSTime - lidarEndTime >= -eps: thisGPSTime = %.8f, lidarEndTime = %.8f", __func__, __LINE__, thisGPSTime, lidarEndTime);
            break;
        }  
    }
    return hasGPSforThisKF;
}

void optimization::addGpsFactor()
{
    if(!bUseGps || gpsOdomBuff.empty())return;
    if(gpsInitOrientation && !gpsInitialized)return;
    if(cloudKeyPoses3D->size() <= 0)return;

    // find nearest gps
    double eps = 1.0/gpsFrequence;//0.1; // find a gps topic arrived within eps second
    
    sensor_msgs::NavSatFix::ConstPtr curGPS;
    nav_msgs::Odometry curGpsOdom;
    if(!syncGps(curGpsOdom, eps))
    {
        PrintInfo("sync gps and lidar failed");
        return;
    }

    Eigen::Vector3d curUTM;
    curUTM(0) = curGpsOdom.pose.pose.position.x;
    curUTM(1) = curGpsOdom.pose.pose.position.y;
    curUTM(2) = curGpsOdom.pose.pose.position.z;
    curUTM = T_lw*curUTM;
    // add first gps in the processing of slam 
    curUTM(0) += cloudKeyPoses3D->points[gpsToKeyPoseIdx].x;
    curUTM(1) += cloudKeyPoses3D->points[gpsToKeyPoseIdx].y;
    // PrintInfo("gps pose x:%f, y:%f, z:%f", curUTM(0), curUTM(1), curUTM(2));

    // update zeroUTM
    // Eigen::Vector3d curUTM = zeroUTM;
    // gps_common::LLtoUTM(curGpsOdom.pose.covariance[1], curGpsOdom.pose.covariance[2], curUTM(1), curUTM(0), UTMZone);
    // zeroUTM = curUTM;   

    double gps_x = curUTM(0);
    double gps_y = curUTM(1);
    double gps_z = curUTM(2);

    // GPS too noisy, skip
    double noise_x = curGpsOdom.pose.covariance[0];
    double noise_y = curGpsOdom.pose.covariance[0]; //curGpsOdom.pose.covariance[7];
    double noise_z = curGpsOdom.pose.covariance[7]; //curGpsOdom.pose.covariance[14];

    // 发送和记录gps轨迹
    Eigen::Vector3d odom = transformTobeMapped.translation();
    if (abs(noise_x) < 0.05 && abs(noise_y) < 0.05 && abs(noise_z) < 0.1)
    {
        const int curNodeIdx = cloudKeyPoses3D->size() - 1;
        auto pGps = cloudKeyPoses6D->at(curNodeIdx);
        pgGpsSaveStream << lidarEndTime << " " << gps_x << " " << gps_y << " " << odom(2) << " " 
        << pGps.qx << " " << pGps.qy << " " << pGps.qz << " " << pGps.qw << std::endl;
    }
    
    // publish path
    gpsPath.header.frame_id = odometryFrame;
    gpsPath.header.stamp = curGpsOdom.header.stamp;
    geometry_msgs::PoseStamped pose;
    pose.header = gpsPath.header;
    pose.pose.position.x = curUTM(0);
    pose.pose.position.y = curUTM(1);
    pose.pose.position.z = curUTM(2);
    pose.pose.orientation.x = 0.0;
    pose.pose.orientation.y = 0.0;
    pose.pose.orientation.z = 0.0;
    pose.pose.orientation.w = 1.0;
    gpsPath.poses.push_back(pose);
    pubGpsPath.publish(gpsPath);

    // make sure the gps data is stable encough
    if (abs(noise_x) > gpsCovThreshold || abs(noise_y) > gpsCovThreshold || abs(noise_z) > 5*gpsCovThreshold)
    {
        PrintInfo("gps noise is too big");
        return;
    }

    noise_x = noise_x < 0.000001 ? gpsNoiseDefault : noise_x;
    noise_y = noise_y < 0.000001 ? gpsNoiseDefault : noise_y;
    noise_z = noise_z < 0.000001 ? gpsNoiseDefault : noise_z;

    // noise_x = noise_x > 0.02 ? 100*noise_x : noise_x;
    // noise_y = noise_y > 0.02 ? 100*noise_y : noise_y;
    // noise_z = noise_z > 0.1 ? 100*noise_z : noise_zp;

    if (!useGpsElevation) 
    {
        gps_z = odom(2);
        noise_z = 0.01;
    }

    // 判断gps是否稳定
    // double gpsDistThreshold = 5.0;
    // if(abs(gps_x - odom(0)) > gpsDistThreshold || abs(gps_y - odom(1)) > gpsDistThreshold || abs(gps_z - odom(2)) > gpsDistThreshold)
    //     return;

    // GPS not properly initialized (0,0,0)
    if (abs(gps_x) < 1e-6 && abs(gps_y) < 1e-6) return;

    curGpsTime = curGpsOdom.header.stamp.toSec();

    // Add GPS every a few meters
    PointType curGPSPoint;
    curGPSPoint.x = gps_x;
    curGPSPoint.y = gps_y;
    curGPSPoint.z = gps_z;

    double distGps = pointDistance(curGPSPoint, lastGPSPoint);
    // 通过速度判断gps是否正常
    if(lastGpsTime <= 0.000001)
    {
        lastGpsTime = curGpsTime;
    }
    else
    {
        double deltaT  = curGpsTime - lastGpsTime;
        deltaT = deltaT <= 0 ? 0.02 : deltaT;
        if(distGps/deltaT > 5)
        {
            PrintInfo("gps velocity is too fast!!");
            lastGpsTime = curGpsTime;
            return;
        }
        lastGpsTime = curGpsTime;
    }

    if (distGps < gpsDistance)
        return;
    else
        lastGPSPoint = curGPSPoint;

    if (debugGps) 
    {
        PrintInfo("curr gps pose: %f, %f , %f", gps_x, gps_y, gps_z);
        PrintInfo("curr gps cov: %f, %f , %f", curGpsOdom.pose.covariance[0],
                    curGpsOdom.pose.covariance[7], curGpsOdom.pose.covariance[14]);
    }

    gtsam::Vector Vector3(3);
    // Vector3 << noise_x*noise_x, noise_y*noise_y, noise_z;
    Vector3 << gpsNoiseScale*noise_x, gpsNoiseScale*noise_y, gpsNoiseScale*noise_z;
    // Vector3 << std::max(noise_x, 10.0), std::max(noise_y, 10.0), std::max(noise_z, 10.0);
    // gtsam::noiseModel::Diagonal::shared_ptr gps_noise = gtsam::noiseModel::Diagonal::Variances(Vector3);
    gtsam::noiseModel::Base::shared_ptr gps_noise = gtsam::noiseModel::Robust::Create(
        gtsam::noiseModel::mEstimator::Cauchy::Create(1), //核函数.
        gtsam::noiseModel::Diagonal::Variances(Vector3)); // 噪声模型    
    gtsam::GPSFactor gps_factor(cloudKeyPoses3D->size(), gtsam::Point3(gps_x, gps_y, gps_z), gps_noise);
    keyframeGPSfactor.push_back(gps_factor);
    cloudKeyGPSPoses3D->points.push_back(curGPSPoint);

    // debug
    static int gpsCnt = 0;
    if(addGps)
    {
        // gpsCnt ++;
        // // if((gpsCnt > 50 && gpsCnt < 70) || (gpsCnt > 110 && gpsCnt < 140) || (gpsCnt > 180 && gpsCnt < 200))
        // if(gpsCnt > 1)
        // {
        //     PrintInfo("cloudKeyPoses3D->size():%d", cloudKeyPoses3D->size());
        //     PrintInfo("skip gps:%d", gpsCnt);
        //     return;
        // }
        gtSAMgraph.add(gps_factor);
        gpsIndexContainer[cloudKeyPoses3D->size()] = cloudKeyGPSPoses3D->size() - 1;
    }

    aLoopIsClosed = true;

    if(debugGps)visualGPSConstraint();
}

/*
utm : x:northing, y:easting
*/
void optimization::gpsHandler(const sensor_msgs::NavSatFix::ConstPtr &gpsMsg)
{
    if(std::isnan(gpsMsg->latitude + gpsMsg->longitude + gpsMsg->altitude))
    {
        PrintInfo("gps is nan-------");
        return;
    }
    if(gpsMsg->latitude > 90 || gpsMsg->latitude < 4
        || gpsMsg->longitude > 140 || gpsMsg->longitude < 70
        || gpsMsg->altitude > 6000 || gpsMsg->altitude < -1000)
        return;
    // if(bUseGnssYaw && !bRPYInit)
    //     return;

    double UTMNorthing;
    double UTMEasting;
    double latitudeGPS = gpsMsg->latitude;
    double longitudeGPS = gpsMsg->longitude;
    double altitudeGPS = gpsMsg->altitude;
    gps_common::LLtoUTM(latitudeGPS, longitudeGPS, UTMNorthing, UTMEasting, UTMZone);
    Eigen::Vector3d curUTM(UTMEasting, UTMNorthing, altitudeGPS);

    if(bFirstGPS)
    {
        if(gpsMsg->position_covariance[0] > gpsCovThreshold || gpsMsg->position_covariance[1] > gpsCovThreshold)
        {
            PrintInfo("gps is too noise!!");
            return;
        }            
        zeroUTM[0] = curUTM(0);
        zeroUTM[1] = curUTM(1);
        zeroUTM[2] = curUTM(2);
        bFirstGPS = 0;
        PrintInfo("first gps--------");
        return;
    }
    curUTM = curUTM - zeroUTM;

    if(bUseGnssYaw)
    {
        curUTM = rotationVector*curUTM;
    }

    // odom
    nav_msgs::Odometry curGpsOdom;
    curGpsOdom.header.stamp = gpsMsg->header.stamp;
    curGpsOdom.header.frame_id = odometryFrame;//mapFrame;
    curGpsOdom.pose.pose.position.x  = curUTM(0);
    curGpsOdom.pose.pose.position.y  = curUTM(1);
    curGpsOdom.pose.pose.position.z  = curUTM(2);
    curGpsOdom.pose.pose.orientation = tf::createQuaternionMsgFromYaw(0);

    // 0.4.8 -> 0.1.2
    curGpsOdom.pose.covariance[0]  = gpsMsg->position_covariance[0];
    curGpsOdom.pose.covariance[7]  = gpsMsg->position_covariance[1];
    curGpsOdom.pose.covariance[14] = gpsMsg->position_covariance[2];
    curGpsOdom.pose.covariance[1]  = latitudeGPS;
    curGpsOdom.pose.covariance[2]  = longitudeGPS;
    curGpsOdom.pose.covariance[3]  = altitudeGPS;
    curGpsOdom.pose.covariance[4]  = gpsMsg->status.status;

    if (bUseGps)
    {
        mtxBuff.lock();
        // gpsBuff.push(gpsMsg);
        gpsOdomBuff.push(curGpsOdom);
        while(gpsOdomBuff.size() > 500)
            gpsOdomBuff.pop();
        // PrintInfo("-%s-%d-:_gps_time = %.8f", __func__, __LINE__, gpsMsg->header.stamp.toSec());
        mtxBuff.unlock();

        //gps yaw init
        if(gpsInitOrientation && !gpsInitialized)
        {
            if(gpsMsg->position_covariance[0] > gpsCovThreshold || gpsMsg->position_covariance[1] > gpsCovThreshold)
            {
                PrintInfo("gps is too noise!!");
                return;
            }  
            mtxGps.lock();
            GNSSPoses.push_back(Eigen::Vector3d(curUTM(0), curUTM[1], curUTM[2]));
            GNSSPosesTime.push_back(gpsMsg->header.stamp.toSec());
            mtxGps.unlock();
        }
    }

} // gpsHandler

//rpyHandler
void optimization::rpyHandler(const geometry_msgs::Vector3Stamped::ConstPtr &rpyMsg)
{
    if(!bUseGps || rpyInitMode != 1 || bRPYInit)return;
    {
        mtxBuff.lock();
        bdRPYBuff.push(*rpyMsg);
        while(bdRPYBuff.size() > 500)
            bdRPYBuff.pop();
        mtxBuff.unlock();        
    }
  // PrintInfo("-%s-%d-:", __func__, __LINE__);
} //rpyHandler

//uavHandler
void optimization::uavHandler(const geometry_msgs::Vector3Stamped::ConstPtr &uavMsg)
{
    mtxBuff.lock();
    UAVBuff.push(*uavMsg);
    while(UAVBuff.size() > 500)
        UAVBuff.pop();
    mtxBuff.unlock();        

} //uavHandler

//imuHandler
void optimization::imuHandler(const sensor_msgs::Imu::ConstPtr &imuMsg)
{
    // if(!bUseGps || (rpyInitMode != 0) || bRPYInit)return;
    {
        mtxBuff.lock();
        imuBuff.push(*imuMsg);
        while(imuBuff.size() > 500)
            imuBuff.pop();
        mtxBuff.unlock();
    }

    // if(bUseGnssYaw && !bRPYInit)
    // {
    //     Eigen::Vector3d imu_rpy = quatToRPY(Eigen::Quaterniond(imuMsg->orientation.w,
    //         imuMsg->orientation.x,imuMsg->orientation.y,imuMsg->orientation.z));
        
    //     double yawInit = -imu_rpy(2) + M_PI_2 + M_PI + RPYGPS2Imu[2];
    //     if(yawInit > M_PI)yawInit = yawInit - 2*M_PI;
    //     if(yawInit < -M_PI)yawInit = yawInit + 2*M_PI;
    //     Eigen::AngleAxisd rollAngle(Eigen::AngleAxisd(RPYGPS2Imu[0], Eigen::Vector3d::UnitX()));
    //     Eigen::AngleAxisd pitchAngle(Eigen::AngleAxisd(RPYGPS2Imu[1], Eigen::Vector3d::UnitY()));
    //     Eigen::AngleAxisd yawAngle(Eigen::AngleAxisd(yawInit, Eigen::Vector3d::UnitZ()));//yaw_gps2utm
    //     rotationVector = yawAngle*pitchAngle*rollAngle;
    //     PrintInfo("yawInit:%f", yawInit);
    //     bRPYInit = true;     
    // }
    
} //imuHandler

void optimization::projectToGround(pcl::PointCloud<PointType>::Ptr thisKeyFrame)
{
    if(thisKeyFrame->points.size() <= 0)return;

    for(int i = 0; i < thisKeyFrame->points.size(); i ++)
    {
        thisKeyFrame->points[i].z = 0;
    }
}

void optimization::saveCloudKeyFramesForVisual(pcl::PointCloud<PointType>::Ptr thisKeyFrame, PointType thisPose3D)
{   
    pcl::PointCloud<PointType>::Ptr thisKeyFrameDS(new pcl::PointCloud<PointType>());
    // pcl::PointCloud<PointType>::Ptr noGroundKeyFrame(new pcl::PointCloud<PointType>());
    // pcl::PassThrough<PointType> PassThroughZ;
    // PassThroughZ.setInputCloud(thisKeyFrame);
    // PassThroughZ.setFilterFieldName("z");
    // if(isFullMap)
    //     // PassThroughZ.setFilterLimits(-5.0, 50);
    //     PassThroughZ.setFilterLimits(-0.5, 50);
    // else
    //     PassThroughZ.setFilterLimits(0.5, 1.2);
    // PassThroughZ.filter(*noGroundKeyFrame);

    // // pcl::PointCloud<PointType>::Ptr noGroundKeyFrameDS(new pcl::PointCloud<PointType>());
    // // groundExtraction(thisKeyFrameDS, noGroundKeyFrame);
    // if(!isFullMap)
    //     projectToGround(noGroundKeyFrame);

    pcl::VoxelGrid<PointType> VoxelGridKeyFrame; // for global map visualization
    VoxelGridKeyFrame.setLeafSize(0.1, 0.1, 0.1); // for global map visualization
    VoxelGridKeyFrame.setInputCloud(thisKeyFrame);
    VoxelGridKeyFrame.filter(*thisKeyFrameDS);
    
    cloudKeyFramesForVisual.push_back(thisKeyFrameDS);
    // cloudKeyFramesForVisual.push_back(noGroundKeyFrame);
}

bool optimization::gpsFactorInitial()
{
    // initialization the first frame
    if (cloudKeyPoses3D->points.empty())
    {
        nav_msgs::Odometry curGpsOdom;
        double eps = 1.0/gpsFrequence;
        if(syncGps(curGpsOdom, eps))
        {
            Eigen::Vector3d curUTM;
            curUTM(0) = curGpsOdom.pose.pose.position.x;
            curUTM(1) = curGpsOdom.pose.pose.position.y;
            curUTM(2) = curGpsOdom.pose.pose.position.z;
            if(bFirstGPS)
            {
                zeroUTM[0] = curUTM(0);
                zeroUTM[1] = curUTM(1);
                zeroUTM[2] = curUTM(2);
                bFirstGPS = 0;
                gpsToKeyPoseIdx = 0;
                PrintInfo("first gps--------");
                // return;
            }
            curUTM = curUTM - zeroUTM;
            curUTM = rotationVector*curUTM;
            // PrintInfo("gps pose x:%f, y:%f, z:%f", curUTM(0), curUTM(1), curUTM(2));

            // update zeroUTM
            // Eigen::Vector3d curUTM = zeroUTM;
            // gps_common::LLtoUTM(curGpsOdom.pose.covariance[1], curGpsOdom.pose.covariance[2], curUTM(1), curUTM(0), UTMZone);
            // zeroUTM = curUTM;      

            // add first factor
            // we need this origin GPS point for prior map based localization
            // but we need to optimize its value by pose graph if the origin gps
            // data is not fixed.
            PointType gnssPoint;
            gnssPoint.x = curUTM(0);
            gnssPoint.y = curUTM(1);
            gnssPoint.z = curUTM(2);

            double noise_x = curGpsOdom.pose.covariance[0] < 0.000001 ? gpsNoiseDefault : curGpsOdom.pose.covariance[0];
            double noise_y = curGpsOdom.pose.covariance[0] < 0.000001 ? gpsNoiseDefault : curGpsOdom.pose.covariance[0]; //curGpsOdom.pose.covariance[7];
            double noise_z = curGpsOdom.pose.covariance[7] < 0.000001 ? gpsNoiseDefault : curGpsOdom.pose.covariance[7]; //curGpsOdom.pose.covariance[14];

            if (!useGpsElevation) 
            {
                Eigen::Vector3d odom = transformTobeMapped.translation();
                gnssPoint.z = odom(2);
                noise_z = 0.01;
            }

            gtsam::Vector Vector3(3);
            Vector3 << noise_x, noise_y, noise_z;
            // Vector3 << std::max(noise_x, 10.0), std::max(noise_y, 10.0), std::max(noise_z, 10.0);
            gtsam::noiseModel::Diagonal::shared_ptr gps_noise = gtsam::noiseModel::Diagonal::Variances(Vector3);
            gtsam::GPSFactor gps_factor(0, gtsam::Point3(gnssPoint.x, gnssPoint.y, gnssPoint.z), gps_noise);
            keyframeGPSfactor.push_back(gps_factor);
            cloudKeyGPSPoses3D->points.push_back(gnssPoint);
        }
        else
        {
            if(!gpsInitialized && gpsInitOrientation)
            {
                ROS_WARN("sysyem need to be initialized");
                return false;
            }
        }
    }
    gpsInitialized = true;
    return true;
}

bool optimization::initRPY()
{
    if(rpyInitMode == 0){ //imu rpy to init 
        double eps = 1.0/100;
        while(!imuBuff.empty())
        {
            auto curImu = imuBuff.front();
            double imuTime = curImu.header.stamp.toSec();
            if(imuTime - lidarEndTime > eps)
            {
                // PrintInfo("imuTime - lidarEndTime > eps");
                // break;
                return false;
            }
            else if(imuTime - lidarEndTime < -eps)
            {
                // PrintInfo("imuTime - lidarEndTime < -eps");
                imuBuff.pop();
            }
            else
            {
                Eigen::Vector3d imu_rpy = quatToRPY(Eigen::Quaterniond(curImu.orientation.w,
                    curImu.orientation.x,curImu.orientation.y,curImu.orientation.z));
                
                double yawInit = -imu_rpy(2) + M_PI_2 + M_PI + RPYGPS2Imu[2];
                // if(imu_rpy(2) >= -M_PI && imu_rpy(2) < -M_PI_2)yawInit = imu_rpy(2) - 3*M_PI_2;
                // else yawInit = -imu_rpy(2) + M_PI_2;
                if(yawInit > M_PI)yawInit = yawInit - 2*M_PI;
                if(yawInit < -M_PI)yawInit = yawInit + 2*M_PI;
                Eigen::AngleAxisd rollAngle(Eigen::AngleAxisd(RPYGPS2Imu[0], Eigen::Vector3d::UnitX()));
                Eigen::AngleAxisd pitchAngle(Eigen::AngleAxisd(RPYGPS2Imu[1], Eigen::Vector3d::UnitY()));
                Eigen::AngleAxisd yawAngle(Eigen::AngleAxisd(yawInit, Eigen::Vector3d::UnitZ()));//yaw_gps2utm
                // Eigen::AngleAxisd rotation_vector;
                rotationVector = yawAngle*pitchAngle*rollAngle;
                PrintInfo("yawInit:%f", yawInit);
                bRPYInit = true;
                break;  
            }
        }
    }
    else if(rpyInitMode == 1)// bd rpy to init
    {
        double eps = 1.0/5.0;
        while(!bdRPYBuff.empty())
        {
            auto curbdRPY = bdRPYBuff.front();
            double bdRPYTime = curbdRPY.header.stamp.toSec();
            if(bdRPYTime - lidarEndTime > eps)
            {
                // PrintInfo("bdRPYTime - lidarEndTime > eps bdRPYTime:%.8f, lidarTime:%.8f", bdRPYTime, lidarEndTime);
                return false;
            }
            else if(bdRPYTime - lidarEndTime < -eps)
            {
                // PrintInfo("bdRPYTime - lidarEndTime < -eps bdRPYTime:%.8f, lidarTime:%.8f", bdRPYTime, lidarEndTime);
                bdRPYBuff.pop();
                // break;
            }
            else
            {
                yawInitRPY = curbdRPY.vector.z;
                // double yaw_init = (-90-yawInitRPY)*M_PI/180;
                double yawInit =  -M_PI_2-yawInitRPY*M_PI/180 + RPYGPS2Imu[2];//
                Eigen::AngleAxisd rollAngle(Eigen::AngleAxisd(RPYGPS2Imu[0], Eigen::Vector3d::UnitX()));
                Eigen::AngleAxisd pitchAngle(Eigen::AngleAxisd(RPYGPS2Imu[1], Eigen::Vector3d::UnitY()));
                Eigen::AngleAxisd yawAngle(Eigen::AngleAxisd(yawInit, Eigen::Vector3d::UnitZ()));//yaw_gps2utm
                // Eigen::AngleAxisd rotation_vector;
                rotationVector = yawAngle*pitchAngle*rollAngle;
                bRPYInit = true;  
                PrintInfo("beidou rpy init success");
                break;
            }          
        }
    }
    return true;
}

bool optimization::mainRun()
{
    // update 
    if(bUseGps && !bRPYInit && !bFirstGPS)
        initRPY();

    // gps init orientation
    // if(bUseGps && gpsInitOrientation && !gpsInitialized)
    // {
    //     if(!gpsFactorInitial())
    //         return false;
    // }
        
    // 计算当前帧与前一帧位姿变换，如果变化太小，不设为关键帧，反之设为关键帧
    if (saveFrame() == false)
        return false; 

    // 激光里程计因子(from fast-lio),  输入的是frame_relative pose  帧间位姿(body 系下)
    addOdomFactor();

    // GPS因子 (UTM -> WGS84)
    addGpsFactor();

    // 闭环因子 (rs-loop-detect)  基于欧氏距离的检测
    addLoopFactor();

    // 执行优化
    isam->update(gtSAMgraph, initialEstimate);
    isam->update();
    if (aLoopIsClosed == true) // 有回环因子，多update几次
    {
        isam->update();
        isam->update();
        isam->update();
        isam->update();
        isam->update();
    }
    // update之后要清空一下保存的因子图，注：历史数据不会清掉，ISAM保存起来了
    gtSAMgraph.resize(0);
    initialEstimate.clear();

    PointType thisPose3D;
    PointTypePose thisPose6D;
    gtsam::Pose3 latestEstimate;

    // 优化结果
    isamCurrentEstimate = isam->calculateBestEstimate();
    // 当前帧位姿结果
    latestEstimate = isamCurrentEstimate.at<gtsam::Pose3>(isamCurrentEstimate.size() - 1);

    // cloudKeyPoses3D加入当前帧位置
    thisPose3D.x = latestEstimate.translation().x();
    thisPose3D.y = latestEstimate.translation().y();
    thisPose3D.z = latestEstimate.translation().z();
    // 索引
    thisPose3D.intensity = cloudKeyPoses3D->size(); //  使用intensity作为该帧点云的index
    cloudKeyPoses3D->push_back(thisPose3D);         //  新关键帧帧放入队列中

    // cloudKeyPoses6D加入当前帧位姿
    thisPose6D.x = thisPose3D.x;
    thisPose6D.y = thisPose3D.y;
    thisPose6D.z = thisPose3D.z;
    thisPose6D.intensity = thisPose3D.intensity;
    thisPose6D.qx = latestEstimate.rotation().toQuaternion().x();
    thisPose6D.qy = latestEstimate.rotation().toQuaternion().y();
    thisPose6D.qz = latestEstimate.rotation().toQuaternion().z();
    thisPose6D.qw = latestEstimate.rotation().toQuaternion().w();
    thisPose6D.time = lidarEndTime;
    cloudKeyPoses6D->push_back(thisPose6D);

    // 位姿协方差
    poseCovariance = isam->marginalCovariance(isamCurrentEstimate.size() - 1);

    // 当前帧激光点，降采样集合
    // pcl::PointCloud<PointType>::Ptr thisKeyFrame(new pcl::PointCloud<PointType>());
    // pcl::copyPointCloud(*curFrame, *thisKeyFrame); // 存储关键帧,没有降采样的点云

    // save odom
    pgOdomOpSaveStream << lidarEndTime << " " << thisPose6D.x << " " << thisPose6D.y << " " << thisPose6D.z << " " << thisPose6D.qx << " " 
    << thisPose6D.qy << " " << thisPose6D.qz << " " << thisPose6D.qw << std::endl;

    //  可视化update后的path
    updatePath(thisPose6D);

    // 保存特征点降采样集合
    pcl::PointCloud<PointType>::Ptr curFrameDS(new pcl::PointCloud<PointType>());
    pcl::VoxelGrid<PointType> VoxelGridKeyFrame;
    // VoxelGridKeyFrame.setLeafSize(0.1, 0.1, 0.1);
    VoxelGridKeyFrame.setLeafSize(0.1, 0.1, 0.1);
    VoxelGridKeyFrame.setInputCloud(curFrame);
    VoxelGridKeyFrame.filter(*curFrameDS);
    surfCloudKeyFrames.push_back(curFrameDS);
    // surfCloudKeyFrames.push_back(curFrame);
    // 去除地面和天花板并滤波后保存
    saveCloudKeyFramesForVisual(curFrame, thisPose3D);
 
    // 更新因子图中所有变量节点的位姿，也就是所有历史关键帧的位姿，更新里程计轨迹
    correctPoses();

    // 更新位姿
    transformTobeMapped = pclPointToIsometry3d(thisPose6D);

    if(pubPathUpdate.getNumSubscribers() != 0)
    {
        publishPathUpdate(pubPathUpdate);
    }

    return true;
}

// 获取最新位姿
bool optimization::getOpPose(Eigen::Isometry3d& poseOp)
{
    if(cloudKeyPoses6D->size() <= 0)
        return false;

    // 使用gps，不在更新lio的信息;或者仅更新高度？从室内到室外如何处理？
    // if(!bFirstGPS)
    // {
    //     return false;
    // }

    // Eigen::Isometry3d firstPose = pclPointToIsometry3d(cloudKeyPoses6D->points[0]);
    // Eigen::Isometry3d currPose  = pclPointToIsometry3d(cloudKeyPoses6D->back());
    // poseOp = firstPose.inverse() * currPose;
    poseOp = pclPointToIsometry3d(cloudKeyPoses6D->back());
    updateLioFlag = 1;
    return true;
}

/**
 * 判断是否需要重构ikdtree
 */
bool optimization::isNeedReconstruct()
{
    if (cloudKeyPoses3D->points.empty())
        return false;

    // 使用gps，不在更新lio的信息;从室内到室外如何处理？
    // if(!bFirstGPS)
    // {
    //     return false;
    // }

    // 优化后位姿
    Eigen::Isometry3d transNewPose = pclPointToIsometry3d(cloudKeyPoses6D->back());
    // 优化前位姿
    Eigen::Isometry3d transOldPose = lastLioPose;//lastOptPose;

    // 位姿变换增量
    Eigen::Isometry3d transBetween = transNewPose.inverse() * transOldPose;
    Eigen::Vector3d transBetween_tran = transBetween.translation();
    Eigen::Vector3d transBetween_rot = quatToRPY(Eigen::Quaterniond(transBetween.rotation()));

    float tran_distance = transBetween_tran[0]*transBetween_tran[0]+
                            transBetween_tran[1]*transBetween_tran[1]+
                            transBetween_tran[2]*transBetween_tran[2];

    if(gpsInitOrientation)
        PrintInfo("rpy:%f, %f, %f, %f", transBetween_rot[0], transBetween_rot[1], transBetween_rot[2], tran_distance);

    // 旋转和平移量都较小，不重构ikdtree
    if (abs(transBetween_rot[0]) < angleThreshold &&
        abs(transBetween_rot[1]) < angleThreshold &&
        abs(transBetween_rot[2]) < angleThreshold &&
        sqrt(tran_distance) < transThreshold)
        return false;

    return true;
}

bool optimization::getIKdTreeCloud(pcl::PointCloud<PointType>::Ptr mapCloudFrames)
{
    if(!isNeedReconstruct() || !recontructKdTree )
    {
        return false;
    }

    // if(updateKdtreeCount <= 0)
    // {
    //     updateKdtreeCount ++ ;
    //     return false;
    // }

    pcl::KdTreeFLANN<PointType>::Ptr kdtreeGlobalMapPoses(new pcl::KdTreeFLANN<PointType>());
    pcl::PointCloud<PointType>::Ptr subMapKeyPoses(new pcl::PointCloud<PointType>());
    pcl::PointCloud<PointType>::Ptr subMapKeyPosesDS(new pcl::PointCloud<PointType>());
    pcl::PointCloud<PointType>::Ptr subMapKeyFrames(new pcl::PointCloud<PointType>());
    // pcl::PointCloud<PointType>::Ptr subMapKeyFramesDS(new pcl::PointCloud<PointType>());

    // kdtree查找最近一帧关键帧相邻的关键帧集合
    std::vector<int> pointSearchIndGlobalMap;
    std::vector<float> pointSearchSqDisGlobalMap;
    mtx.lock();
    kdtreeGlobalMapPoses->setInputCloud(cloudKeyPoses3D);
    kdtreeGlobalMapPoses->radiusSearch(cloudKeyPoses3D->back(), globalMapVisualizationSearchRadius, pointSearchIndGlobalMap, pointSearchSqDisGlobalMap, 0);
    mtx.unlock();

    for (int i = 0; i < (int)pointSearchIndGlobalMap.size(); ++i)
        subMapKeyPoses->push_back(cloudKeyPoses3D->points[pointSearchIndGlobalMap[i]]);     //  subMap的pose集合
    // 降采样
    pcl::VoxelGrid<PointType> downSizeFilterSubMapKeyPoses;
    downSizeFilterSubMapKeyPoses.setLeafSize(globalMapVisualizationPoseDensity, globalMapVisualizationPoseDensity, globalMapVisualizationPoseDensity); // for global map visualization
    downSizeFilterSubMapKeyPoses.setInputCloud(subMapKeyPoses);
    downSizeFilterSubMapKeyPoses.filter(*subMapKeyPosesDS);         //  subMap poses  downsample
    // 提取局部相邻关键帧对应的特征点云
    for (int i = 0; i < (int)subMapKeyPosesDS->size(); ++i)
    {
        // 距离过大
        if (pointDistance(subMapKeyPosesDS->points[i], cloudKeyPoses3D->back()) > globalMapVisualizationSearchRadius)
                continue;
        int thisKeyInd = (int)subMapKeyPosesDS->points[i].intensity;
        *subMapKeyFrames += *transformPointCloud(surfCloudKeyFrames[thisKeyInd], &cloudKeyPoses6D->points[thisKeyInd]); //  fast_lio only use  surfCloud
    }
    // 降采样，发布
    pcl::VoxelGrid<PointType> downSizeFilterGlobalMapKeyFrames;                                                                                   // for global map visualization
    downSizeFilterGlobalMapKeyFrames.setLeafSize(globalMapVisualizationLeafSize, globalMapVisualizationLeafSize, globalMapVisualizationLeafSize); // for global map visualization
    downSizeFilterGlobalMapKeyFrames.setInputCloud(subMapKeyFrames);
    downSizeFilterGlobalMapKeyFrames.filter(*mapCloudFrames);
    PrintInfo("need reconstruct ikd tree");
    return true;
 
}

void optimization::sendLocalMap(const Eigen::Isometry3d& poseFromLio, const double& frameEndTime, pcl::PointCloud<PointType>::Ptr frameFromLio)
{
    // 发布局部稠密地图
    if (!isNeedLocalMap || cloudKeyPoses3D->points.empty() || !lidarInit)
        return;

    // clock_t start_ms = clock();
    pcl::PointCloud<PointType>::Ptr frameFromLioDS(new pcl::PointCloud<PointType>());
    pcl::VoxelGrid<PointType> VoxelGridKeyFrame;
    VoxelGridKeyFrame.setLeafSize(0.1, 0.1, 0.1);
    VoxelGridKeyFrame.setInputCloud(frameFromLio);
    VoxelGridKeyFrame.filter(*frameFromLioDS);
    // pcl::PointCloud<PointType>::Ptr frameFromLioDS(frameFromLio);

    pcl::PointCloud<PointType>::Ptr curCloud(new pcl::PointCloud<PointType>());
    pcl::PointCloud<PointType>::Ptr curLocalCloud(new pcl::PointCloud<PointType>());
        
    Eigen::Isometry3d T_b_lidar = Eigen::Isometry3d::Identity();
    Eigen::Isometry3d T_w_b = transformTobeMapped; // world2body 

    if(lidarInit)
    {
        Eigen::Isometry3d RtLidarToBody = Eigen::Isometry3d::Identity();
        RtLidarToBody.rotate(lidarToBaselink);
        RtLidarToBody.pretranslate(Eigen::Vector3d(-carRadius, 0.0, 0.0));  
        T_w_b = T_w_b*RtLidarToBody;
        RtLidarToBody = RtLidarToBody.inverse();      
        pcl::transformPointCloud(*frameFromLioDS, *frameFromLioDS, RtLidarToBody.matrix());
    }

    Eigen::Isometry3d T_w_lidar  =  T_w_b * T_b_lidar; // T_w_lidar  转换矩阵
    Eigen::Isometry3d transCur = T_w_lidar;   

    int cloudSize = frameFromLioDS->size();     

    // clock_t start_ms = clock();
    // #pragma omp parallel for num_threads(numberOfCores) //numberOfCores
    for (int i = 0; i < cloudSize; ++i)
    {
        auto &pointFrom = frameFromLioDS->points[i];
        // if(fabs(pointFrom.z) > 1.5 || fabs(pointFrom.y) > 2.0 || fabs(pointFrom.x) > 10.0)
        // if(pointFrom.z > 1.5 || pointFrom.z < -4.0 || fabs(pointFrom.y) > 2.0 || fabs(pointFrom.x) > 10.0)
        //     continue;
        curLocalCloud->push_back(pointFrom);
        
        PointType pointCur;
        pointCur.x = transCur(0, 0) * pointFrom.x + transCur(0, 1) * pointFrom.y + transCur(0, 2) * pointFrom.z + transCur(0, 3);
        pointCur.y = transCur(1, 0) * pointFrom.x + transCur(1, 1) * pointFrom.y + transCur(1, 2) * pointFrom.z + transCur(1, 3);
        pointCur.z = transCur(2, 0) * pointFrom.x + transCur(2, 1) * pointFrom.y + transCur(2, 2) * pointFrom.z + transCur(2, 3);
        pointCur.intensity = pointFrom.intensity;
        curCloud->push_back(pointCur);
    }

    // 转到车身坐标后发布
    pcl::PointCloud<PointType>::Ptr localMapCloud(new pcl::PointCloud<PointType>());

    transCur = transCur.inverse();     

    for (int i = 0; i < localCloudFrames.size(); i++) 
    {
        pcl::PointCloud<PointType>::Ptr curCloudFrame = localCloudFrames.front();
        pcl::PointCloud<PointType>::Ptr curCloudLidar(new pcl::PointCloud<PointType>());
        
        for(int j = 0; j < curCloudFrame->size(); j++)
        {
            const auto &pointFrom = curCloudFrame->points[j];
            PointType pointCur;
            pointCur.x = transCur(0, 0) * pointFrom.x + transCur(0, 1) * pointFrom.y + transCur(0, 2) * pointFrom.z + transCur(0, 3);
            pointCur.y = transCur(1, 0) * pointFrom.x + transCur(1, 1) * pointFrom.y + transCur(1, 2) * pointFrom.z + transCur(1, 3);
            pointCur.z = transCur(2, 0) * pointFrom.x + transCur(2, 1) * pointFrom.y + transCur(2, 2) * pointFrom.z + transCur(2, 3);
            pointCur.intensity = pointFrom.intensity;

            // if(pointCur.z > 1.5 || pointCur.z < -4.0 || fabs(pointCur.y) > 2.0 || pointCur.x > 10.0 || pointCur.x < -2.0)
            //     continue;
            curCloudLidar->push_back(pointCur);
        }

        *localMapCloud   += *curCloudLidar;

        localCloudFrames.pop();
        localCloudFrames.push(curCloudFrame);
    }       
    *localMapCloud   += *curLocalCloud;         
    
    sensor_msgs::PointCloud2 localCloudmsg;
    pcl::toROSMsg(*localMapCloud, localCloudmsg);
    localCloudmsg.header.stamp = ros::Time().fromSec(frameEndTime);
    localCloudmsg.header.frame_id = baselinkFrame; //lidarFrame;
    pubLocalMap.publish(localCloudmsg);

    pcl::PassThrough<PointType> pass_filter;
    // 设置直通滤波器,地图大小40*40*20
    pass_filter.setFilterFieldName("x");
    pass_filter.setFilterLimits(-octomap_size_x, octomap_size_x);
    pass_filter.setInputCloud(localMapCloud);
    pass_filter.filter(*localMapCloud);

    pass_filter.setFilterFieldName("y");
    pass_filter.setFilterLimits(-octomap_size_y, octomap_size_y);
    pass_filter.setInputCloud(localMapCloud);
    pass_filter.filter(*localMapCloud);
    
    pass_filter.setFilterFieldName("z");
    pass_filter.setFilterLimits(-octomap_size_z, octomap_size_z);
    pass_filter.setInputCloud(localMapCloud);
    pass_filter.filter(*localMapCloud);

    double eps = 1.0/10.0;
    mtxBuff.lock();
    while(!UAVBuff.empty())
    {
        auto curRPY = UAVBuff.front();
        double RPYTime = curRPY.header.stamp.toSec();
        if(RPYTime - lidarEndTime > eps)
        {
            PrintInfo("RPYTime - lidarEndTime > eps RPYTime:%.8f, lidarTime:%.8f", RPYTime, lidarEndTime);
            continue;
        }
        else if(RPYTime - lidarEndTime < -eps)
        {
            PrintInfo("RPYTime - lidarEndTime < -eps RPYTime:%.8f, lidarTime:%.8f", RPYTime, lidarEndTime);
            UAVBuff.pop();
            // break;
        }
        else
        {
            roll = curRPY.vector.x;
            pitch = curRPY.vector.y;
            yaw = curRPY.vector.z;
            PrintInfo("UAV rpy init success");
            break;
        }          
    }
    mtxBuff.unlock();

    // 创建绕X轴旋转的旋转矩阵
    Eigen::AngleAxisd rotX(roll, Eigen::Vector3d::UnitX());
    // 创建绕Y轴旋转的旋转矩阵
    Eigen::AngleAxisd rotY(pitch, Eigen::Vector3d::UnitY());
    // 创建绕Z轴旋转的旋转矩阵
    Eigen::AngleAxisd rotZ(yaw, Eigen::Vector3d::UnitZ());

    // 将三个旋转矩阵相乘得到总的旋转矩阵
    Eigen::AngleAxisd rotationMatrix;
    rotationMatrix = rotZ * rotY * rotX;

    // 将旋转矩阵应用到Isometry3d对象中
    Eigen::Isometry3d R_uav_b = Eigen::Isometry3d::Identity();
    R_uav_b.rotate(rotationMatrix); 
    // 创建OctoMap树
    octomap::OcTree tree(octomap_resolution);
    // 插入点云到OctoMap,设置盲区
    for (const auto& point : localMapCloud->points) {
        if(std::sqrt(point.x*point.x + point.y*point.y + point.z*point.z) > blindSpot)
        {
            PointType pointCur;

            // base_link to NED
            pointCur.y = -1*point.y;
            pointCur.z = -1*point.z;

            //NED to uav
            pointCur.x = R_uav_b(0, 0) * point.x + R_uav_b(0, 1) * point.y + R_uav_b(0, 2) * point.z + R_uav_b(0, 3);
            pointCur.y = R_uav_b(1, 0) * point.x + R_uav_b(1, 1) * point.y + R_uav_b(1, 2) * point.z + R_uav_b(1, 3);
            pointCur.z = R_uav_b(2, 0) * point.x + R_uav_b(2, 1) * point.y + R_uav_b(2, 2) * point.z + R_uav_b(2, 3);

            tree.updateNode(octomap::point3d(pointCur.x, pointCur.y, pointCur.z), true); // 标记为占据
        }else{
            continue;
        }
    }
    tree.updateInnerOccupancy(); // 更新内部占据状态

    // 计算地图边界和分辨率 
    double min_x, min_y, min_z, max_x, max_y, max_z;
    min_x = -octomap_size_x;
    min_y = -octomap_size_y;
    min_z = -octomap_size_z;
    max_x = octomap_size_x;
    max_y = octomap_size_y;
    max_z = octomap_size_z;
    // tree.getMetricMin(min_x, min_y, min_z);
    // tree.getMetricMax(max_x, max_y, max_z);
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
    if (save_to_file) {
        saveGrid3D(grid, file_path);
        ROS_INFO("Grid saved to %s", file_path.c_str());
        save_to_file = false;
        ROS_INFO("Map bounds: [%.2f, %.2f, %.2f] to [%.2f, %.2f, %.2f], grid [%d, %d, %d],res=%.3f",
                min_x, min_y, min_z, max_x, max_y, max_z,size_x, size_y, size_z, resolution);
    }

    // 创建ROS消息容器
    octomap_msgs::Octomap octomap_msg;
    octomap_msg.header.frame_id = baselinkFrame;  // 坐标系需与RViz一致
    octomap_msg.header.stamp = ros::Time().fromSec(frameEndTime);

    // 转换为ROS消息（需选择full/binary编码）
    octomap_msgs::fullMapToMsg(tree, octomap_msg); // 包含颜色信息必须用full
    pubOctoMap.publish(octomap_msg);

    Eigen::Isometry3d RtBetween = lastLioPoseLocalMap.inverse()*poseFromLio;
    Eigen::Vector3d transRtBetween = RtBetween.translation();
    Eigen::Vector3d rotRtBetween = quatToRPY(Eigen::Quaterniond(RtBetween.rotation()));   
    float tran_distance = sqrt(transRtBetween[0]*transRtBetween[0]+
                        transRtBetween[1]*transRtBetween[1]+
                        transRtBetween[2]*transRtBetween[2]);

    // if (abs(rotRtBetween[0]) > 0.2 || abs(rotRtBetween[1]) > 0.2 || abs(rotRtBetween[2]) > 0.2 || sqrt(tran_distance) > 0.5)
    if (abs(rotRtBetween[0]) > 0.2 || abs(rotRtBetween[1]) > 0.2 || abs(rotRtBetween[2]) > 0.2 || sqrt(tran_distance) > 0.5)
    {           
        if(curCloud->size() > 0)
        {
            localCloudFrames.push(curCloud);
            lastLioPoseLocalMap = poseFromLio;
        }
        // *localMapCloud   += *curCloud;
    }
    while (localCloudFrames.size() > 15)
    {
        localCloudFrames.pop();
    }

    // clock_t end_ms = clock();
    // std::cout << "end: " << double(end_ms - start_ms) / CLOCKS_PER_SEC << " s"  << std::endl;
    
}

bool optimization::syncImuLidar(sensor_msgs::Imu& imuSync)
{
        double eps = 1.0/100;
        while(!imuBuff.empty())
        {
            auto curImu = imuBuff.front();
            double imuTime = curImu.header.stamp.toSec();
            if(imuTime - lidarEndTime > eps)
            {
                return false;
            }
            else if(imuTime - lidarEndTime < -eps)
            {
                imuBuff.pop();
            }
            else
            {
                imuSync = curImu;
                return true;
            }
        }
        return false;
}

void optimization::setOdomAndFrame(const Eigen::Isometry3d& poseFromLio, const pcl::PointCloud<PointType>::Ptr frameFromLio, const double& frameEndTime)
{
    pcl::copyPointCloud(*frameFromLio, *curFrame);
    if (cloudKeyPoses3D->points.empty())
    {
        // Eigen::Vector3d odom = poseFromLio.translation();
        // geometry_msgs::Quaternion quat = tf::createQuaternionMsgFromYaw(0);
        // transformTobeMapped.rotate (Eigen::Quaterniond(quat.w, quat.x, quat.y, quat.z));
        // transformTobeMapped.pretranslate(poseFromLio.translation());
        transformTobeMapped = poseFromLio;
        // lastLioPose = poseFromLio;
        // lastOptPose = transformTobeMapped;
    }
    else
    {
        //　更新了lio的位姿，直接采用lio的位姿
        if(updateLioFlag)
        {
            transformTobeMapped = poseFromLio;
            // lastLioPose = poseFromLio;
            // lastOptPose = transformTobeMapped;
            updateLioFlag = false;            
        }
        //　未更新lio的位姿，采用相对位姿
        else
        {
            Eigen::Isometry3d RtBetween = lastLioPose.inverse()*poseFromLio;
            transformTobeMapped = transformTobeMapped*RtBetween;
            // lastLioPose = poseFromLio;
            // lastOptPose = transformTobeMapped;
        }
    }

    sendLocalMap(poseFromLio, frameEndTime, frameFromLio);

    lastLioPose = poseFromLio;
    lastOptPose = transformTobeMapped;

    lidarEndTime = frameEndTime;

    // save odom
    Eigen::Vector3d odomLio = transformTobeMapped.translation();
    Eigen::Quaterniond odomQ = Eigen::Quaterniond(transformTobeMapped.rotation());
    pgOdomSaveStream << lidarEndTime << " " << odomLio[0] << " " << odomLio[1] << " " << odomLio[2] << " " << odomQ.x() << " " 
    << odomQ.y() << " " << odomQ.z() << " " << odomQ.w() << std::endl;

    // gps init
    if(!bFirstGPS && gpsInitOrientation && !gpsInitialized)
    {
        double x = odomLio[0];
        double y = odomLio[1];
        double distance = 0.0;
        if(lidarOdomPoses.size() == 0)
        {
            distance = 1.0;
            // if(sqrt(x * x + y * y) > 10.0)distance = sqrt(x * x + y * y);
        }
        else
        {
            distance = sqrt((x - lidarOdomPoses.back()[0]) * (x - lidarOdomPoses.back()[0]) 
                            + (y - lidarOdomPoses.back()[1]) * (y - lidarOdomPoses.back()[1]));
        }
        
        if (distance > 0.2)
        {
            lidarOdomPoses.push_back(Eigen::Vector3d(odomLio[0], odomLio[1], odomLio[2]));
            lidarOdomTime.push_back(frameEndTime);
        }    


        if(lidarOdomPoses.size() > gpsInitPointCnt)
        {
            umeyamaAlignment();
        }    
    }

    // publish tf and pose
    static tf::TransformBroadcaster br;
    tf::Transform                   transform;
    tf::Quaternion                  q;
    transform.setOrigin(tf::Vector3(odomLio[0],
                                    odomLio[1],
                                    odomLio[2]));
    q.setW(odomQ.w());
    q.setX(odomQ.x());
    q.setY(odomQ.y());
    q.setZ(odomQ.z());
    transform.setRotation( q );
    br.sendTransform( tf::StampedTransform( transform, ros::Time().fromSec(lidarEndTime), odometryFrame, lidarFrame ) );
    // br.sendTransform( tf::StampedTransform( transform, ros::Time().now(), odometryFrame, lidarFrame ) );

    if(!lidarInit)
    {
        double rollLidar, pitchLidar, yawLidar;
        tf::Matrix3x3(q).getRPY(rollLidar, pitchLidar, yawLidar);
        initLidarRPY(0) = rollLidar;
        initLidarRPY(1) = pitchLidar;
        PrintInfo("lidar init roll:%f, pitch::%f", rollLidar, pitchLidar);
        lidarToBaselink = 
            Eigen::Quaterniond(Eigen::AngleAxisd(0.0, Eigen::Vector3d::UnitZ()) 
            * Eigen::AngleAxisd(-pitchLidar, Eigen::Vector3d::UnitY()) 
            * Eigen::AngleAxisd(-rollLidar, Eigen::Vector3d::UnitX()));
        lidarInit = true;
    }

    if(odometryFrame != mapFrame)
    {
        tf::Quaternion qMapOdom;
        transform.setOrigin(tf::Vector3(0, 0, 0));
        qMapOdom.setW(1.0);
        qMapOdom.setX(0.0);
        qMapOdom.setY(0.0);
        qMapOdom.setZ(0.0);
        transform.setRotation( qMapOdom );
        br.sendTransform( tf::StampedTransform( transform, ros::Time().fromSec(lidarEndTime), mapFrame, odometryFrame ) );        
    }

    nav_msgs::Odometry odomUpdate;
    odomUpdate.header.frame_id = odometryFrame;//mapFrame;
    odomUpdate.header.stamp = ros::Time().fromSec(frameEndTime); //ros::Time().now();//
    // odomUpdate.child_frame_id = lidarFrame;//"body";
    odomUpdate.pose.pose.position.x = odomLio[0];
    odomUpdate.pose.pose.position.y = odomLio[1];
    odomUpdate.pose.pose.position.z = odomLio[2];
    odomUpdate.pose.pose.orientation.x = odomQ.x();
    odomUpdate.pose.pose.orientation.y = odomQ.y();
    odomUpdate.pose.pose.orientation.z = odomQ.z();
    odomUpdate.pose.pose.orientation.w = odomQ.w();
    pubOdomLidar.publish(odomUpdate);

    // 转到车身中心坐标
    Eigen::Isometry3d RtLidarToBody = Eigen::Isometry3d::Identity();
    RtLidarToBody.rotate(lidarToBaselink);
    // RtLidarToBody.rotate(Eigen::Quaterniond(1.0, 0.0, 0.0, 0.0));
    RtLidarToBody.pretranslate(Eigen::Vector3d(-carRadius, 0.0, 0.0));
    Eigen::Isometry3d bodyPose = transformTobeMapped*RtLidarToBody;
    odomLio = bodyPose.translation();
    odomQ = Eigen::Quaterniond(bodyPose.rotation());

    //vel
    if(pubTwist.getNumSubscribers() > 0)
    {
        if(lastLidarTime == 0)
        {
            lastLidarTime = lidarEndTime;
            lastBasePose3d = bodyPose;
        }
        else
        {
            Eigen::Isometry3d diffBasePose = lastBasePose3d.inverse() * bodyPose;
            Eigen::Vector3d transDiff = diffBasePose.translation();
            Eigen::Vector3d rpyDiff = quatToRPY(Eigen::Quaterniond(diffBasePose.rotation()));
            Eigen::Vector3d curVelTrans = transDiff/(lidarEndTime - lastLidarTime);
            Eigen::Vector3d curVelRPY = rpyDiff/(lidarEndTime - lastLidarTime);
            // PrintWarn("lidar pose vel: %f, %f, %f !!!", curVelTrans(0), curVelTrans(1), curVelTrans(2));
            // PrintWarn("lidar rpy vel: %f, %f, %f !!!", curVelRPY(0), curVelRPY(1), curVelRPY(2));
            lastLidarTime = lidarEndTime;
            lastBasePose3d = bodyPose;
            geometry_msgs::Twist curSlamTwist;
            curSlamTwist.linear.x = curVelTrans(0);
            curSlamTwist.linear.y = curVelTrans(1);
            curSlamTwist.linear.z = curVelTrans(2);
            curSlamTwist.angular.x = curVelRPY(0);
            curSlamTwist.angular.y = curVelRPY(1);
            curSlamTwist.angular.z = curVelRPY(2);
            pubTwist.publish(curSlamTwist);
        }
    }

    q.setW(odomQ.w());
    q.setX(odomQ.x());
    q.setY(odomQ.y());
    q.setZ(odomQ.z());

    odomUpdate.pose.pose.orientation.x = odomQ.x();
    odomUpdate.pose.pose.orientation.y = odomQ.y();
    odomUpdate.pose.pose.orientation.z = odomQ.z();
    odomUpdate.pose.pose.orientation.w = odomQ.w();
    odomUpdate.pose.pose.position.x = odomLio[0];
    odomUpdate.pose.pose.position.y = odomLio[1];
    odomUpdate.pose.pose.position.z = odomLio[2];
    pubOdomUpdate.publish(odomUpdate);

    geometry_msgs::PoseWithCovarianceStamped slamPose;
    slamPose.pose = odomUpdate.pose;
    slamPose.header = odomUpdate.header;
    pubSlamPose.publish(slamPose);

    /******* Publish pose to operation ********/
    nav_msgs::Odometry robotMapPos;
    robotMapPos.pose.pose.position.x = odomLio[0];
    robotMapPos.pose.pose.position.y = odomLio[1];
    robotMapPos.pose.pose.position.z = odomLio[2];
    double rollP, pitchP, yawP;
    tf::Matrix3x3(q).getRPY(rollP, pitchP, yawP);

    // 使用imu的roll, pitch
    // sensor_msgs::Imu curImu;
    // if(syncImuLidar(curImu))
    // {
    //     Eigen::Vector3d imu_rpy = quatToRPY(Eigen::Quaterniond(curImu.orientation.w,
    //                           curImu.orientation.x,curImu.orientation.y,curImu.orientation.z));
    //     rollP = imu_rpy[0];
    //     pitchP = imu_rpy[1];
    //     // odomQ = Eigen::Quaterniond(curImu.orientation.w, curImu.orientation.x, curImu.orientation.y, curImu.orientation.z);
    // }

    robotMapPos.pose.pose.orientation.x = rollP;
    robotMapPos.pose.pose.orientation.y = pitchP;
    robotMapPos.pose.pose.orientation.z = yawP;
    robotMapPos.pose.pose.orientation.w = floorNum;
    pubMapPos.publish(robotMapPos);

    if(lidarFrame != baselinkFrame)
    {
        transform.setOrigin(tf::Vector3(odomLio[0],
                                        odomLio[1],
                                        odomLio[2]));
        q.setW(odomQ.w());
        q.setX(odomQ.x());
        q.setY(odomQ.y());
        q.setZ(odomQ.z());
        transform.setRotation( q );
        br.sendTransform( tf::StampedTransform( transform, ros::Time().fromSec(lidarEndTime), odometryFrame, baselinkFrame ) );        
    }

    if(!bFirstGPS)
    {
        geometry_msgs::Point robotGpsPoint;
        Eigen::Vector3d curUTM(odomLio[0], odomLio[1], odomLio[2]);
        curUTM = rotationVector.inverse()*T_lw.inverse()*curUTM;
        curUTM = curUTM + zeroUTM;

        double latitude, longitude;
        gps_common::UTMtoLL(curUTM(1), curUTM(0), UTMZone, latitude, longitude);
        robotGpsPoint.x  = latitude;
        robotGpsPoint.y = longitude;
        robotGpsPoint.z  = curUTM(2);
        pubGpsPos.publish(robotGpsPoint);
    }
}

// gps orientation init
void optimization::umeyamaAlignment()
{
    if (lidarOdomPoses.size() < gpsInitPointCnt)
        return;

    Eigen::VectorXd lidar_timestamp(lidarOdomPoses.size());
    std::vector<Eigen::Vector3d> lidar_pos(lidarOdomPoses.size());
    for (int i = 0; i < lidarOdomPoses.size(); i++)
    {
        lidar_timestamp[i] = lidarOdomTime[i];
        lidar_pos[i][0] = lidarOdomPoses[i][0];
        lidar_pos[i][1] = lidarOdomPoses[i][1];
        lidar_pos[i][2] = lidarOdomPoses[i][2];
    }
    mtxGps.lock();
    Eigen::VectorXd gnss_timestamp(GNSSPoses.size());
    std::vector<Eigen::Vector3d> gnss_pos(GNSSPoses.size());
    for (int i = 0; i < GNSSPoses.size(); i++)
    {
        gnss_timestamp[i] = GNSSPosesTime[i];
        gnss_pos[i][0] = GNSSPoses[i][0];
        gnss_pos[i][1] = GNSSPoses[i][1];
        gnss_pos[i][2] = GNSSPoses[i][2];
    }
    mtxGps.unlock();
    if(gnss_pos.size() < 10)
        return;

    // time aligen
    double max_tiff = 0.05;
    std::vector<Eigen::Vector3d> lidar_pos_align, gnss_pos_align;
    for (int i = 1; i < lidar_timestamp.size(); i++)
    {

        Eigen::VectorXd gnss_diff = (gnss_timestamp.array() - lidar_timestamp[i]).array().abs();
        Eigen::MatrixXd::Index minRow, minCol;
        double min_diff = gnss_diff.minCoeff(&minRow, &minCol);

        Eigen::Vector3d align_pose;
        if (min_diff < max_tiff)
        {
            // align pose
            if (gnss_timestamp[minRow] - lidar_timestamp[i] > 0)
            {
                int j = 1;
                while (gnss_timestamp[minRow - j] > lidar_timestamp[i])
                {
                    if (j == minRow)
                        break;
                    j++;
                }

                if (j == minRow)
                    continue;

                double t =
                    (lidar_timestamp[i] - gnss_timestamp[minRow - j]) / (gnss_timestamp[minRow] - gnss_timestamp[minRow - j]);
                assert(t >= 0.0);
                assert(t <= 1.0);

                Eigen::Vector3d pre_pose = gnss_pos[minRow - j];
                Eigen::Vector3d cur_pose = gnss_pos[minRow];

                align_pose[0] = pre_pose[0] + (cur_pose[0] - pre_pose[0]) * t;
                align_pose[1] = pre_pose[1] + (cur_pose[1] - pre_pose[1]) * t;
                align_pose[2] = pre_pose[2] + (cur_pose[2] - pre_pose[2]) * t;
            }
            else
            {
                int j = 0;
                while (gnss_timestamp[minRow + j] < lidar_timestamp[i])
                {
                    if (minRow + j == (gnss_timestamp.size() - 1))
                        break;
                    j++;
                }

                if (minRow + j == (gnss_timestamp.size() - 1))
                    continue;

                double t =
                    (lidar_timestamp[i] - gnss_timestamp[minRow]) / (gnss_timestamp[minRow + j] - gnss_timestamp[minRow]);
                assert(t >= 0.0);
                assert(t <= 1.0);

                Eigen::Vector3d pre_pose = gnss_pos[minRow];
                Eigen::Vector3d cur_pose = gnss_pos[minRow + j];

                align_pose[0] = pre_pose[0] + (cur_pose[0] - pre_pose[0]) * t;
                align_pose[1] = pre_pose[1] + (cur_pose[1] - pre_pose[1]) * t;
                align_pose[2] = pre_pose[2] + (cur_pose[2] - pre_pose[2]) * t;
            }

            lidar_pos_align.push_back(lidar_pos[i]);
            gnss_pos_align.push_back(align_pose);
        }
    }

    if (lidar_pos_align.size() < gpsInitPointCnt)
    {
        std::cout << "lidar_pos_align : " << lidar_pos_align.size() << std::endl;
        return;
    }

    // umeyama ignore scale
    Eigen::Matrix<double, 2, Eigen::Dynamic> points_tgt(2, lidar_pos_align.size());
    Eigen::Matrix<double, 2, Eigen::Dynamic> points_src(2, gnss_pos_align.size());
    double heightAver = 0.0;
    for (int i = 0; i < lidar_pos_align.size(); i++)
    {
        points_tgt(0, i) = lidar_pos_align[i][0];
        points_tgt(1, i) = lidar_pos_align[i][1];
        points_src(0, i) = gnss_pos_align[i][0];
        points_src(1, i) = gnss_pos_align[i][1];

        heightAver += lidar_pos_align[i][2] - gnss_pos_align[i][2];
    }
    if(lidar_pos_align.size() > 0)
        heightAver = heightAver/lidar_pos_align.size();
    PrintInfo("heightAver:%f", heightAver);

    Eigen::Matrix3d rt = Eigen::umeyama(points_src, points_tgt, false);
    // std::cout << rt << std::endl;
    PrintInfo("rt:%f %f %f %f", rt(0,0), rt(0,1), rt(1,0), rt(1,1));

    // result
    T_lw = Eigen::Isometry3d::Identity();
    Eigen::Matrix3d r;
    r << rt(0,0),rt(0,1),0,
         rt(1,0),rt(1,1),0,
         0,0,1;
    T_lw.rotate(r);
    // T_lw.pretranslate(Eigen::Vector3d(rt(0, 3), rt(1, 3), rt(2, 3)));
    T_lw.pretranslate(Eigen::Vector3d(rt(0, 2), rt(1, 2), heightAver));

    gpsInitialized = true;
}

/**
 * 更新因子图中所有变量节点的位姿，也就是所有历史关键帧的位姿，更新里程计轨迹
 */
void optimization::correctPoses()
{
    if (cloudKeyPoses3D->points.empty())
        return;

    if (aLoopIsClosed == true)
    {
        // 清空里程计轨迹
        globalPath.poses.clear();
        // 更新因子图中所有变量节点的位姿，也就是所有历史关键帧的位姿
        int numPoses = isamCurrentEstimate.size();
        for (int i = 0; i < numPoses; ++i)
        {
            cloudKeyPoses3D->points[i].x = isamCurrentEstimate.at<gtsam::Pose3>(i).translation().x();
            cloudKeyPoses3D->points[i].y = isamCurrentEstimate.at<gtsam::Pose3>(i).translation().y();
            cloudKeyPoses3D->points[i].z = isamCurrentEstimate.at<gtsam::Pose3>(i).translation().z();

            cloudKeyPoses6D->points[i].x = cloudKeyPoses3D->points[i].x;
            cloudKeyPoses6D->points[i].y = cloudKeyPoses3D->points[i].y;
            cloudKeyPoses6D->points[i].z = cloudKeyPoses3D->points[i].z;
            cloudKeyPoses6D->points[i].qx = isamCurrentEstimate.at<gtsam::Pose3>(i).rotation().toQuaternion().x();
            cloudKeyPoses6D->points[i].qy = isamCurrentEstimate.at<gtsam::Pose3>(i).rotation().toQuaternion().y();
            cloudKeyPoses6D->points[i].qz = isamCurrentEstimate.at<gtsam::Pose3>(i).rotation().toQuaternion().z();
            cloudKeyPoses6D->points[i].qw = isamCurrentEstimate.at<gtsam::Pose3>(i).rotation().toQuaternion().w();

            // 更新里程计轨迹
            updatePath(cloudKeyPoses6D->points[i]);
        }

        // we save optimized origin gps point, maybe the altitude value need to
        // be fixes
        // if (bUseGps && gpsIndexContainer.size() / 200 == 0) {
        //     Eigen::Vector3d firstPoint(cloudKeyPoses6D->at(0).x,
        //                                 cloudKeyPoses6D->at(0).y,
        //                                 cloudKeyPoses6D->at(0).z);

        //     std::cout << std::setprecision(9)
        //             << "origin LLA: " << zeroUTM.transpose() << std::endl;

        //     zeroUTM = firstPoint + zeroUTM;

        //     std::cout << std::setprecision(9)
        //             << "update LLA: " << zeroUTM.transpose() << std::endl;

        //     ROS_WARN("UPDATE ORGIN LLA");
        // }

        // PrintInfo("ISMA2 Update");
        aLoopIsClosed = false;
    }
}

void optimization::getCurFloorcloudKeyPoses3D(pcl::PointCloud<PointType>::Ptr curFloorcloudKeyPoses3D, 
    pcl::PointCloud<PointTypePose>::Ptr curFloorcloudKeyPoses6D, 
    std::vector<int>& idxCloudKeyPoses)
{
    mtxFloorInfo.lock();
    int curFloorNum;
    FloorState curFloorState;
    curFloorNum = floorNum;
    if(floorStates.find(curFloorNum) != floorStates.end())
    {
        curFloorState = floorStates.at(curFloorNum);
    }
    // 分层保存的数据
    for(int i = 0; i < curFloorState.size(); i++)
    {
        int startIdx = curFloorState[i].startIdx;
        int endIdx = curFloorState[i].endIdx;
        // std::cout << "startIdx:" << startIdx << "  endIdx:" << endIdx << std::endl;
        if(endIdx - startIdx < 1)continue;
        for(int j = startIdx; j < endIdx; j++)
        {
            curFloorcloudKeyPoses3D->push_back(copy_cloudKeyPoses3D->points[j]);
            curFloorcloudKeyPoses6D->push_back(copy_cloudKeyPoses6D->points[j]);
            idxCloudKeyPoses.push_back(j);
        }
    }

    for (int i = lastFloorFrame; i < copy_cloudKeyPoses3D->size(); i++) 
    {
        curFloorcloudKeyPoses3D->push_back(copy_cloudKeyPoses3D->points[i]);
        curFloorcloudKeyPoses6D->push_back(copy_cloudKeyPoses6D->points[i]);
        idxCloudKeyPoses.push_back(i);
    }
    mtxFloorInfo.unlock();
}

//回环检测三大要素
// 1.设置最小时间差，太近没必要
// 2.控制回环的频率，避免频繁检测，每检测一次，就做一次等待
// 3.根据当前最小距离重新计算等待时间
bool optimization::detectLoopClosureDistance(int *latestID, int *closestID)
{
    // 当前关键帧帧
    int loopKeyCur = copy_cloudKeyPoses3D->size() - 1; //  当前关键帧索引
    int loopKeyPre = -1;

    // 当前帧已经添加过闭环对应关系，不再继续添加
    auto it = loopIndexContainer.find(loopKeyCur);
    if (it != loopIndexContainer.end())
        return false;
    // 在历史关键帧中查找与当前关键帧距离最近的关键帧集合
    std::vector<int> pointSearchIndLoop;                        //  候选关键帧索引
    std::vector<float> pointSearchSqDisLoop;                    //  候选关键帧距离
    pcl::PointCloud<PointType>::Ptr curFloorcloudKeyPoses3D(new pcl::PointCloud<PointType>());
    pcl::PointCloud<PointTypePose>::Ptr curFloorcloudKeyPoses6D(new pcl::PointCloud<PointTypePose>());
    std::vector<int>idxCloudKeyPoses;
    getCurFloorcloudKeyPoses3D(curFloorcloudKeyPoses3D, curFloorcloudKeyPoses6D, idxCloudKeyPoses);
    if(curFloorcloudKeyPoses3D->size() <= 0)return false;
    kdtreeHistoryKeyPoses->setInputCloud(curFloorcloudKeyPoses3D); //  历史帧构建kdtree
    kdtreeHistoryKeyPoses->radiusSearch(copy_cloudKeyPoses3D->back(), historyKeyframeSearchRadius, pointSearchIndLoop, pointSearchSqDisLoop, 0);
    // kdtreeHistoryKeyPoses->setInputCloud(copy_cloudKeyPoses3D); //  历史帧构建kdtree
    // kdtreeHistoryKeyPoses->radiusSearch(copy_cloudKeyPoses3D->back(), historyKeyframeSearchRadius, pointSearchIndLoop, pointSearchSqDisLoop, 0);
    // 在候选关键帧集合中，找到与当前帧时间相隔较远的帧，设为候选匹配帧
    for (int i = 0; i < (int)pointSearchIndLoop.size(); ++i)
    {
        int idFloorCloud = pointSearchIndLoop[i];
        int id = idxCloudKeyPoses[idFloorCloud];
        double time_diff = fabs(curFloorcloudKeyPoses6D->points[idFloorCloud].time - lidarEndTime);
        double distance  = pointDistance(curFloorcloudKeyPoses3D->points[idFloorCloud], copy_cloudKeyPoses3D->back());
        // int id = pointSearchIndLoop[i];
        // double time_diff = fabs(copy_cloudKeyPoses6D->points[id].time - lidarEndTime);
        // double distance  = pointDistance(copy_cloudKeyPoses3D->points[id], copy_cloudKeyPoses3D->back());
        if(copy_cloudKeyPoses3D->size() - id < 20)
        {
            continue;
        }
        //　楼梯限制回环
        if(fabs(curFloorcloudKeyPoses3D->points[idFloorCloud].z-curFloorcloudKeyPoses3D->back().z) > 3.0)
        {
            PrintInfo("loop frame z diff is too large:%f", curFloorcloudKeyPoses3D->points[idFloorCloud].z-curFloorcloudKeyPoses3D->back().z);
            continue;
        }
        //根据时间和距离选择候选关键帧，时间越长，距离越长
        int numIdx = 4;
        for(int j = 1; j <= numIdx; j++)
        {
            if(time_diff > j*historyKeyframeSearchTimeDiff && distance <= historyKeyframeSearchRadius*0.25*j)
            {
                loopKeyPre = id;
                break;                   
            }
        }
    }
    if (loopKeyPre == -1 || loopKeyCur == loopKeyPre)
        return false;
    *latestID = loopKeyCur;
    *closestID = loopKeyPre;

    PrintInfo("Find loop clousre frame ");
    return true;
}

/**
 * 提取key索引的关键帧前后相邻若干帧的关键帧特征点集合，降采样
 */
void optimization::loopFindNearKeyframes(pcl::PointCloud<PointType>::Ptr &nearKeyframes, const int &key, const int &searchNum)
{
    // 提取key索引的关键帧前后相邻若干帧的关键帧特征点集合
    nearKeyframes->clear();
    // int cloudSize = copy_cloudKeyPoses6D->size();
    int cloudSize = surfCloudKeyFrames.size() < copy_cloudKeyPoses6D->size() ? surfCloudKeyFrames.size() : copy_cloudKeyPoses6D->size();
    if(surfCloudKeyFrames.size() != copy_cloudKeyPoses6D->size())
    {
        PrintWarn("surfCloudKeyFrames.size: %d, copy_cloudKeyPoses6D.size:%d", surfCloudKeyFrames.size(), copy_cloudKeyPoses6D->size());
    }
    for (int i = -searchNum; i <= searchNum; ++i)
    {
        int keyNear = key + i;
        if (keyNear < 0 || keyNear >= cloudSize)
            continue;
        // 注意：cloudKeyPoses6D 存储的是 T_w_b , 而点云是lidar系下的，构建icp的submap时，需要通过外参数T_b_lidar 转换 , 参考pointBodyToWorld 的转换
        *nearKeyframes += *transformPointCloud(surfCloudKeyFrames[keyNear], &copy_cloudKeyPoses6D->points[keyNear]); //  fast-lio 没有进行特征提取，默认点云就是surf
    }

    if (nearKeyframes->empty())
        return;

    // 降采样
    pcl::PointCloud<PointType>::Ptr cloud_temp(new pcl::PointCloud<PointType>());
    downSizeFilterICP.setInputCloud(nearKeyframes);
    downSizeFilterICP.filter(*cloud_temp);
    *nearKeyframes = *cloud_temp;
}

void optimization::performLoopClosure()
{
    if (cloudKeyPoses3D->points.empty() == true)
    {
        return;
    }
    ros::Time timeLaserInfoStamp = ros::Time().fromSec(lidarEndTime); //  时间戳
    // string odometryFrame = mapFrame;//"camera_init";

    mtx.lock();
    if(!copy_cloudKeyPoses3D->empty() && copy_cloudKeyPoses3D->size() == cloudKeyPoses3D->size())
    {
        mtx.unlock();
        return;
    }
    *copy_cloudKeyPoses3D = *cloudKeyPoses3D;
    *copy_cloudKeyPoses6D = *cloudKeyPoses6D;
    mtx.unlock();

    // 当前关键帧索引，候选闭环匹配帧索引
    int loopKeyCur;
    int loopKeyPre;
    // if(visual_loop_enable)
    // {   
    //     // 通过视觉查找回环
    //     if(detectLoopClosureVisual(&loopKeyCur, &loopKeyPre) == false)
    //         return;
    // }
    // else
    {
        // 在历史关键帧中查找与当前关键帧距离最近的关键帧集合，选择时间相隔较远的一帧作为候选闭环帧
        if (detectLoopClosureDistance(&loopKeyCur, &loopKeyPre) == false)
            return;
    }

    // 提取
    pcl::PointCloud<PointType>::Ptr cureKeyframeCloud(new pcl::PointCloud<PointType>()); //  cue keyframe
    pcl::PointCloud<PointType>::Ptr prevKeyframeCloud(new pcl::PointCloud<PointType>()); //   history keyframe submap
    {
        // 提取当前关键帧特征点集合，降采样
        loopFindNearKeyframes(cureKeyframeCloud, loopKeyCur, 0); //  将cur keyframe 转换到world系下
        // 提取闭环匹配关键帧前后相邻若干帧的关键帧特征点集合，降采样
        loopFindNearKeyframes(prevKeyframeCloud, loopKeyPre, historyKeyframeSearchNum); //  选取historyKeyframeSearchNum个keyframe拼成submap
        // 如果特征点较少，返回
        if (cureKeyframeCloud->size() < 300 || prevKeyframeCloud->size() < 1000)
            return;
        // 发布闭环匹配关键帧局部map
        if (pubHistoryKeyFrames.getNumSubscribers() != 0)
            publishCloud(pubHistoryKeyFrames, prevKeyframeCloud, timeLaserInfoStamp, odometryFrame);
    }

    // ICP Settings
    pcl::IterativeClosestPoint<PointType, PointType> icp;
    icp.setMaxCorrespondenceDistance(2*historyKeyframeSearchRadius); //150// giseop , use a value can cover 2*historyKeyframeSearchNum range in meter
    icp.setMaximumIterations(100);
    icp.setTransformationEpsilon(1e-6);
    icp.setEuclideanFitnessEpsilon(1e-6);
    icp.setRANSACIterations(0);

    // scan-to-map，调用icp匹配
    icp.setInputSource(cureKeyframeCloud);
    icp.setInputTarget(prevKeyframeCloud);
    pcl::PointCloud<PointType>::Ptr unused_result(new pcl::PointCloud<PointType>());
    icp.align(*unused_result);

    // 未收敛，或者匹配不够好
    if (icp.hasConverged() == false || icp.getFitnessScore() > historyKeyframeFitnessScore)
        return;

    // std::cout << "icp  success  " << std::endl;
    PrintInfo("icp  success");

    // 发布当前关键帧经过闭环优化后的位姿变换之后的特征点云
    if (pubIcpKeyFrames.getNumSubscribers() != 0)
    {
        pcl::PointCloud<PointType>::Ptr closed_cloud(new pcl::PointCloud<PointType>());
        pcl::transformPointCloud(*cureKeyframeCloud, *closed_cloud, icp.getFinalTransformation());
        publishCloud(pubIcpKeyFrames, closed_cloud, timeLaserInfoStamp, odometryFrame);
    }

    // 闭环优化得到的当前关键帧与闭环关键帧之间的位姿变换
    Eigen::Isometry3f correctionLidarFrame = Eigen::Isometry3f::Identity();
    correctionLidarFrame = icp.getFinalTransformation();

    // 闭环优化前当前帧位姿
    Eigen::Isometry3f tWrong = pclPointToIsometry3f(copy_cloudKeyPoses6D->points[loopKeyCur]);
    // 闭环优化后当前帧位姿
    Eigen::Isometry3f tCorrect = correctionLidarFrame * tWrong; // 获取上一帧 相对 当前帧的 位姿
    Eigen::Isometry3d tCorrect_d = tCorrect.cast <double> ();
    gtsam::Pose3 poseFrom = gtsam::Pose3(gtsam::Rot3(tCorrect_d.rotation()), 
                                         gtsam::Point3(tCorrect_d.translation()));
    // 闭环匹配帧的位姿
    gtsam::Pose3 poseTo = pclPointTogtsamPose3(copy_cloudKeyPoses6D->points[loopKeyPre]);
    // gtsam::Vector Vector6(6);
    float noiseScore = icp.getFitnessScore() ; //  loop_clousre  noise from icp
    // Vector6 << noiseScore, noiseScore, noiseScore, noiseScore, noiseScore, noiseScore;
    // gtsam::noiseModel::Diagonal::shared_ptr constraintNoise = gtsam::noiseModel::Diagonal::Variances(Vector6);
    // std::cout << "loopNoiseQueue   =   " << noiseScore << std::endl;
    gtsam::Vector noiseVector6(6);
    gtsam::noiseModel::Base::shared_ptr constraintNoise; 
    noiseVector6 << noiseScore*10, noiseScore*10, noiseScore*10, noiseScore*10, noiseScore*10, noiseScore*10;
    constraintNoise = gtsam::noiseModel::Robust::Create(
    gtsam::noiseModel::mEstimator::Cauchy::Create(1), //核函数，可选: replacing Cauchy by DCS or GemanMcClure, but with a good front-end loop detector, Cauchy is empirically enough.
    gtsam::noiseModel::Diagonal::Variances(noiseVector6)); // 噪声模型
    // constraintNoise = gtsam::noiseModel::Diagonal::Variances(noiseVector6);

    // 添加闭环因子需要的数据
    mtxLoopInfo.lock();
    loopIndexQueue.push_back(make_pair(loopKeyCur, loopKeyPre));
    loopPoseQueue.push_back(poseFrom.between(poseTo));
    loopNoiseQueue.push_back(constraintNoise);
    mtxLoopInfo.unlock();

    loopIndexContainer[loopKeyCur] = loopKeyPre; //   使用hash map 存储回环对
}

//回环检测线程
void optimization::loopClosureThread()
{
    if (loopClosureEnableFlag == false)
    {
        // std::cout << "loopClosureEnable is false " << endl;
        PrintInfo("loopClosureEnable is false");
        return;
    }

    ros::Rate rate(loopClosureFrequency); //   回环频率
    while (ros::ok() && startFlag)
    {
        rate.sleep();

        if(!loopMode)continue;
        if(bFirstGPS || abs(curGpsTime - lidarEndTime) > 30)//有问题，从室外到室内如何处理:无gps超过20s才开启回环
        {
            performLoopClosure();   //  回环检测
            visualizeLoopClosure(); // rviz展示闭环边
        }
        
        // if(bUseGps)
        //     visualGPSConstraint();
    }
}

// 分层保存标志回调
void optimization::segmentSignalHandler(const std_msgs::UInt8ConstPtr& signal_msg)
{
    mtxFloorInfo.lock();
    unordered_map<int, FloorState>::iterator curFloorIter;
    FloorFrames curFloorFrame;
    curFloorFrame.startIdx = lastFloorFrame;
    curFloorFrame.endIdx   = (int)cloudKeyPoses3D->size();
    lastFloorFrame = (int)cloudKeyPoses3D->size();
    FloorState curFloorState;
    curFloorState.push_back(curFloorFrame);

    PrintInfo("floor segment:%d, current floor:%d", signal_msg->data, floorNum);

    if(signal_msg->data == 1)//加一层
    {
        PrintInfo("map floor up");
        curFloorIter = floorStates.find(floorNum);
        if(curFloorIter != floorStates.end())
        {
            floorStates.at(floorNum).push_back(curFloorFrame);
        }
        else
        {
            pair<int, FloorState>tmpState(floorNum, curFloorState);
            floorStates.insert(tmpState);
        }
        floorNum ++;

    }
    else if(signal_msg->data == 2)//减一层
    {
        PrintInfo("map floor down");
        curFloorIter = floorStates.find(floorNum);
        if(curFloorIter != floorStates.end())
        {
            floorStates.at(floorNum).push_back(curFloorFrame);
        }
        else
        {
            pair<int, FloorState>tmpState(floorNum, curFloorState);
            floorStates.insert(tmpState);
        }
        floorNum --;
    }
    else //单纯分割
    {
        curFloorIter = floorStates.find(floorNum);
        if(curFloorIter != floorStates.end())
        {
            floorStates.at(floorNum).push_back(curFloorFrame);
        }
        else
        {
            pair<int, FloorState>tmpState(floorNum, curFloorState);
            floorStates.insert(tmpState);
        }
    }
    mtxFloorInfo.unlock();

    //old segment
    is_seg_map_ = true;
    int map_size_ = (int)cloudKeyPoses3D->size();
    segmentList.push_back(map_size_);
}

// 分层保存
bool optimization::saveSegmentMap(int startIdx, int endIdx, double resolution, std::string saveMapPath, bool saveMap, bool visualMap)
{
    pcl::PointCloud<PointType>::Ptr globalMapCloud(new pcl::PointCloud<PointType>());
    pcl::PointCloud<PointType>::Ptr globalMapCloudDS(new pcl::PointCloud<PointType>());

    // 注意：拼接地图时，keyframe是lidar系，而fastlio更新后的存到的cloudKeyPoses6D 关键帧位姿是body系下的，需要把
    //cloudKeyPoses6D  转换为T_world_lidar 。 T_world_lidar = T_world_body * T_body_lidar , T_body_lidar 是外参
    for (int i = startIdx; i < endIdx; i++) {
        // if(i >= surfCloudKeyFrames.size() || i >= cloudKeyPoses6D->size())
        if(i >= cloudKeyFramesForVisual.size() || i >= cloudKeyPoses6D->size())
        {
            PrintWarn("saving map, idx is out of range");
            break;
        }
        *globalMapCloud   += *transformPointCloud(cloudKeyFramesForVisual[i], &cloudKeyPoses6D->points[i]);
        // *globalMapCloud   += *transformPointCloud(surfCloudKeyFrames[i], &cloudKeyPoses6D->points[i]);
        // cout << "\r" << std::flush << "Processing feature cloud " << i << " of " << cloudKeyPoses6D->size() << " ...";
        // PrintInfo("Processing feature cloud: %d of %d", i, cloudKeyPoses6D->size());
    }

    if(resolution != 0)
    {
        // cout << "\n\nSave resolution: " << resolution << endl;
        // 降采样
        downSizeFilterMap.setInputCloud(globalMapCloud);
        downSizeFilterMap.setLeafSize(resolution, resolution, resolution);
        downSizeFilterMap.filter(*globalMapCloudDS);
    }
    else
    {
        downSizeFilterMap.setInputCloud(globalMapCloud);
        downSizeFilterMap.setLeafSize(mappingSurfLeafSize, mappingSurfLeafSize, mappingSurfLeafSize);
        downSizeFilterMap.filter(*globalMapCloudDS);
    }

    pcl::StatisticalOutlierRemoval<PointType> sor;
    sor.setInputCloud(globalMapCloud);
    sor.setMeanK(30);
    sor.setStddevMulThresh(1.0);
    sor.filter(*globalMapCloud);

    // visial optimize global map on viz
    if(visualMap)
    {
        ros::Time timeLaserInfoStamp = ros::Time().fromSec(lidarEndTime);
        // string odometryFrame = mapFrame;//"camera_init";
        publishCloud(pubOptimizedGlobalMap, globalMapCloud, timeLaserInfoStamp, odometryFrame);
        // cout << "****************************************************" << endl;
        // cout << "Publish Map Successfully\n" << endl;
    }

    if(saveMap)
    {
        pcl::io::savePCDFileBinary(saveMapPath + "filterGlobalMap.pcd", *globalMapCloudDS);       //  滤波后地图
        int ret = pcl::io::savePCDFileBinary(saveMapPath + "GlobalMap.pcd", *globalMapCloud);       //  稠密地图
        // cout << "****************************************************" << endl;
        if(ret == 0)
        {
            // cout << "Saving map to pcd files completed\n" << endl;
            PrintInfo("Saving map to pcd files completed");
            return true;
        }
        else
        {
            // cout << "Saving map to pcd files failure\n" << endl;
            PrintWarn("Saving map to pcd files failure");
            return false;
        }

    }
    return true;
}

/**
 * 保存全局关键帧特征点集合
*/
bool optimization::saveMapService(fusion_slam::save_mapRequest& req, fusion_slam::save_mapResponse& res)
{
    // string saveMapDirectory;

    // // cout << "****************************************************" << endl;
    // // cout << "Saving map to pcd files ..." << endl;
    // PrintInfo("****************************************************");
    // PrintInfo("Saving map to pcd files ...");
    // if(req.destination.empty()) 
    //     saveMapDirectory = savePCDDirectory;
    //     // saveMapDirectory = std::getenv("HOME") + savePCDDirectory;
    // else
    //     saveMapDirectory = req.destination;
    //     // saveMapDirectory = std::getenv("HOME") + req.destination;
    // // cout << "Save destination: " << saveMapDirectory << endl;
    // PrintInfo("Save destination: %s", saveMapDirectory.c_str());
    // // 保存历史关键帧位姿
    // pcl::io::savePCDFileBinary(saveMapDirectory + "/trajectory.pcd", *cloudKeyPoses3D);                    // 关键帧位置
    // pcl::io::savePCDFileBinary(saveMapDirectory + "/transformations.pcd", *cloudKeyPoses6D);      // 关键帧位姿

    // res.success = 1;
    // int startIdx = 0, endIdx = cloudKeyPoses6D->size();
    // int mapCnt = 1;
    // for(int i = 0; i <= segmentList.size(); i ++)
    // {
    //     if(i < segmentList.size())endIdx = segmentList[i];
    //     else endIdx = cloudKeyPoses6D->size();
    //     if(endIdx - startIdx < 5)
    //         continue;
    //     // std::cout << "saving " << mapCnt << " map" << std::endl;
    //     PrintInfo("saving %d map", mapCnt);
    //     std::string saveMapPath = saveMapDirectory + "/" + std::to_string(mapCnt);
    //     if(!saveSegmentMap(startIdx, endIdx, req.resolution, saveMapPath, req.saveMap, req.visualMap))
    //     {
    //         res.success = 0;
    //     }
    //     startIdx = endIdx;
    //     mapCnt ++;
    // }

    // saveAndPublishMapInfo();
    savePCD = true;
    PrintInfo("savePCD is true");
    res.success = true;
    return true;
}

void optimization::mainShutDown()
{
    pose pose_gnss;
    pose pose_optimized;
    pose pose_without_optimized;

    // std::ofstream  file_pose_gnss;
    std::ofstream  file_pose_optimized;
    std::ofstream  file_pose_without_optimized;
    std::ofstream  file_odom_path;//保存轨迹用于返航

    string savePoseDirectory;
    PrintInfo("****************************************************");
    PrintInfo("Saving poses to pose files before shut down...");
    // savePoseDirectory = std::getenv("HOME") + savePCDDirectory;
    savePoseDirectory = savePCDDirectory;
    PrintInfo("Save destination: %s", savePoseDirectory.c_str());

    // 保存历史关键帧位姿 pcd格式
    // pcl::io::savePCDFileBinary(savePoseDirectory + "/trajectory.pcd", *cloudKeyPoses3D);                    // 关键帧位置
    // pcl::io::savePCDFileBinary(savePoseDirectory + "/transformations.pcd", *cloudKeyPoses6D);      // 关键帧位姿

    // create file 
    CreateFile(file_pose_optimized, savePoseDirectory + "/optimized_pose_all.txt");
    CreateFile(file_pose_without_optimized, savePoseDirectory + "/without_optimized_pose_all.txt");
    CreateFile(file_odom_path, savePoseDirectory + "/odom_path_all.csv");

    // save optimize data
    for(int i = 0; i  < cloudKeyPoses6D->size(); i++){  
        pose_optimized.t =  Eigen::Vector3d(cloudKeyPoses6D->points[i].x, cloudKeyPoses6D->points[i].y, cloudKeyPoses6D->points[i].z  );
        pose_optimized.R = Eigen::Quaterniond(cloudKeyPoses6D->points[i].qw,
                                              cloudKeyPoses6D->points[i].qx,
                                              cloudKeyPoses6D->points[i].qy,
                                              cloudKeyPoses6D->points[i].qz).normalized().toRotationMatrix();  

        // WriteTextKITTI(file_pose_optimized, pose_optimized);
        WriteTextTUM(file_pose_optimized, cloudKeyPoses6D->points[i].time, pose_optimized);
    }
    // cout << "Sucess global optimized  poses to pose files ..." << endl;
    PrintInfo("saved global optimized  poses to pose files");

    for(int i = 0; i  < lio_cloudKeyPoses6D->size(); i++){  
        pose_without_optimized.t =  Eigen::Vector3d(lio_cloudKeyPoses6D->points[i].x, lio_cloudKeyPoses6D->points[i].y, lio_cloudKeyPoses6D->points[i].z  );
        pose_without_optimized.R = Eigen::Quaterniond(lio_cloudKeyPoses6D->points[i].qw,
                                                      lio_cloudKeyPoses6D->points[i].qx,
                                                      lio_cloudKeyPoses6D->points[i].qy,
                                                      lio_cloudKeyPoses6D->points[i].qz).normalized().toRotationMatrix();

        // WriteTextKITTI(file_pose_without_optimized, pose_without_optimized);
        WriteTextTUM(file_pose_without_optimized, lio_cloudKeyPoses6D->points[i].time, pose_without_optimized);
    }
    // cout << "Sucess unoptimized  poses to pose files ..." << endl;
    PrintInfo("saved unoptimized  poses to pose files");

    /*写入文件名*/
    file_odom_path << "file: " << savePoseDirectory << std::endl;
    /* 写入时间*/
    time_t now;
    struct tm *timenow;
    time(&now);
    timenow = localtime(&now);
    char buf[32];
    strftime(buf, 32, "%F", timenow);    //转化为ISO日期格式的字符串（YY-MM-DD）
    file_odom_path << "date: " << buf << std::endl;
    //写入路径frame
    file_odom_path << "frame_id: " << "map" << std::endl;
    /* 写入路径点的个数 */
    file_odom_path << "size: " << static_cast<int>((int)cloudKeyPoses3D->size()) << std::endl;
    /* 写入路径点 */
    //ofstrm << "x" << "," << "y" << "," << "z" << std::endl;
    for (int i = 0; i < (int)cloudKeyPoses3D->size(); i++) {
        file_odom_path << setprecision(9) << "header:0,0,map,poses:" 
        << cloudKeyPoses3D->points[i].x << "," << cloudKeyPoses3D->points[i].y  << "," << cloudKeyPoses3D->points[i].z 
        << ",0,0,0,0," << endl;
    }
    // cout << "Sucess odom  poses to pose files ..." << endl;
    PrintInfo("saved odom  poses to pose files")

    file_pose_optimized.close();
    file_pose_without_optimized.close();
    file_odom_path.close();  
}

void optimization::saveAndPublishPose()
{
    pcl::PointCloud<PointType>::Ptr save_cloudKeyPoses3D{new pcl::PointCloud<PointType>()};
    pcl::PointCloud<PointTypePose>::Ptr save_cloudKeyPoses6D{new pcl::PointCloud<PointTypePose>()};
    pcl::PointCloud<PointType>::Ptr save_curFloorcloudKeyPoses3D{new pcl::PointCloud<PointType>()};
    pcl::PointCloud<PointTypePose>::Ptr save_curFloorcloudKeyPoses6D{new pcl::PointCloud<PointTypePose>()};
    mtx.lock();
    *save_cloudKeyPoses3D = *cloudKeyPoses3D;
    *save_cloudKeyPoses6D = *cloudKeyPoses6D;
    mtx.unlock();

    // 转到车身中心坐标
    Eigen::Isometry3d RtLidarToBody = Eigen::Isometry3d::Identity();
    // RtLidarToBody.rotate(Eigen::Quaterniond(1.0, 0.0, 0.0, 0.0));
    RtLidarToBody.rotate(lidarToBaselink);
    RtLidarToBody.pretranslate(Eigen::Vector3d(-carRadius, 0.0, 0.0));

    int curFloorNum;
    FloorState curFloorState;
    mtxFloorInfo.lock();
    curFloorNum = floorNum;
    if(floorStates.find(curFloorNum) != floorStates.end())
    {
        curFloorState = floorStates.at(curFloorNum);
    }
    mtxFloorInfo.unlock();

    int keyFramesSize = save_cloudKeyPoses3D->size();
    for (int i = lastFloorFrame; i < keyFramesSize; i++) {
        save_curFloorcloudKeyPoses3D->push_back(save_cloudKeyPoses3D->points[i]);
        save_curFloorcloudKeyPoses6D->push_back(save_cloudKeyPoses6D->points[i]);
    }
    // 分层保存的数据
    for(int i = 0; i < curFloorState.size(); i++)
    {
        int startIdx = curFloorState[i].startIdx;
        int endIdx = curFloorState[i].endIdx;
        if(endIdx - startIdx < 2)continue;
        for(int j = startIdx; j < endIdx; j++)
        {
            save_curFloorcloudKeyPoses3D->push_back(save_cloudKeyPoses3D->points[j]);
            save_curFloorcloudKeyPoses6D->push_back(save_cloudKeyPoses6D->points[j]);
        }
    }

    pose pose_gnss;
    pose pose_optimized;

    // std::ofstream  file_pose_gnss;
    std::ofstream  file_pose_optimized;
    std::ofstream  file_odom_path;//保存轨迹用于返航

    // send pose for return;
    // publishAllPath(pubPosePath);
    if (pubPosePath.getNumSubscribers() != 0)
    {
        // path.poses.push_back(globalPath);
        nav_msgs::Path curFloorPath;
        Eigen::Vector3d lastPoint(3, -1);

        for(int i = 0; i  < save_curFloorcloudKeyPoses6D->size(); i++)
        {
            geometry_msgs::PoseStamped pose_stamped;
            PointTypePose curPose = save_curFloorcloudKeyPoses6D->points[i];
            pose_stamped.header.stamp = ros::Time().fromSec(curPose.time);

            Eigen::Isometry3d transformPose = pclPointToIsometry3d(curPose);
            Eigen::Isometry3d bodyPose = transformPose*RtLidarToBody;
            Eigen::Vector3d bodyPoint = bodyPose.translation();

            double distPoint = sqrt(pow(bodyPoint[0]-lastPoint[0],2) + pow(bodyPoint[1]-lastPoint[1],2) + pow(bodyPoint[1]-lastPoint[1],2));
            if(distPoint < 0.3)
            {
                continue;
            }
            lastPoint = bodyPoint;

            pose_stamped.header.frame_id = odometryFrame;
            pose_stamped.pose.position.x = bodyPoint[0];//curPose.x;
            pose_stamped.pose.position.y = bodyPoint[1];//curPose.y;
            pose_stamped.pose.position.z = bodyPoint[2];//curPose.z;
            pose_stamped.pose.orientation.x = curPose.qx;
            pose_stamped.pose.orientation.y = curPose.qy;
            pose_stamped.pose.orientation.z = curPose.qz;
            pose_stamped.pose.orientation.w = curPose.qw;
            curFloorPath.poses.push_back(pose_stamped);
        }
    
        curFloorPath.header.stamp = ros::Time().fromSec(lidarEndTime); ;
        curFloorPath.header.frame_id = odometryFrame;//mapFrame;
        pubPosePath.publish(curFloorPath);
    }

    string savePoseDirectory;
    // cout << "****************************************************" << endl;
    // cout << "Saving poses to pose files ..." << endl;
    PrintInfo("****************************************************");
    PrintInfo("Saving poses to pose files ...");
    savePoseDirectory = savePCDDirectory;
    // savePoseDirectory = std::getenv("HOME") + savePCDDirectory;
    PrintInfo("Save destination: %s", savePoseDirectory.c_str());

    // create file 
    // CreateFile(file_pose_gnss, savePoseDirectory + "/gnss_pose.txt");
    CreateFile(file_pose_optimized, savePoseDirectory + "/optimized_pose.txt");
    CreateFile(file_odom_path, savePoseDirectory + "/odom_path.csv");

    // save optimize data
    for(int i = 0; i  < save_curFloorcloudKeyPoses6D->size(); i++){  
        pose_optimized.t =  Eigen::Vector3d(save_curFloorcloudKeyPoses6D->points[i].x, save_curFloorcloudKeyPoses6D->points[i].y, save_curFloorcloudKeyPoses6D->points[i].z  );
        pose_optimized.R = Eigen::Quaterniond(save_curFloorcloudKeyPoses6D->points[i].qw,
                                              save_curFloorcloudKeyPoses6D->points[i].qx,
                                              save_curFloorcloudKeyPoses6D->points[i].qy,
                                              save_curFloorcloudKeyPoses6D->points[i].qz).normalized().toRotationMatrix();  

        // WriteTextKITTI(file_pose_optimized, pose_optimized);
        WriteTextTUM(file_pose_optimized, save_curFloorcloudKeyPoses6D->points[i].time, pose_optimized);
    }
    // cout << "Sucess global optimized  poses to pose files ..." << endl;
    PrintInfo("saved global optimized  poses to pose files");

    // for(int i = 0; i  < gnss_cloudKeyPoses6D->size(); i++){  
    //     pose_gnss.t =  Eigen::Vector3d(gnss_cloudKeyPoses6D->points[i].x, gnss_cloudKeyPoses6D->points[i].y, gnss_cloudKeyPoses6D->points[i].z  );
    //     pose_gnss.R = Exp(double(gnss_cloudKeyPoses6D->points[i].roll), double(gnss_cloudKeyPoses6D->points[i].pitch), double(gnss_cloudKeyPoses6D->points[i].yaw) );
    //     // WriteTextKITTI(file_pose_gnss, pose_gnss);
    //     WriteTextTUM(file_pose_without_optimized, gnss_cloudKeyPoses6D->points[i].time, pose_gnss);
    // }
    // // cout << "Sucess gnss  poses to pose files ..." << endl;
    // PrintInfo("saved gnss poses to pose files");

    // 保存轨迹用于路径跟踪
    try{
        /*写入文件名*/
        file_odom_path << "file: " << savePoseDirectory << std::endl;
        /* 写入时间*/
        time_t now;
        struct tm *timenow;
        time(&now);
        timenow = localtime(&now);
        char buf[32];
        strftime(buf, 32, "%F", timenow);    //转化为ISO日期格式的字符串（YY-MM-DD）
        file_odom_path << "date: " << buf << std::endl;
        //写入路径frame
        file_odom_path << "frame_id: " << "map" << std::endl;
        /* 写入路径点的个数 */
        file_odom_path << "size: " << static_cast<int>((int)save_curFloorcloudKeyPoses3D->size()) << std::endl;
        /* 写入路径点 */
        //ofstrm << "x" << "," << "y" << "," << "z" << std::endl;
        for (int i = 0; i < (int)save_curFloorcloudKeyPoses3D->size(); i++) {
            file_odom_path << setprecision(9) << "header:0,0,map,poses:" 
            << save_curFloorcloudKeyPoses3D->points[i].x << "," << save_curFloorcloudKeyPoses3D->points[i].y  << "," << save_curFloorcloudKeyPoses3D->points[i].z 
            << ",0,0,0,0," << endl;
        }
        // cout << "Sucess odom  poses to pose files ..." << endl;
        PrintInfo("saved odom  poses to pose files");
    }
    catch(...){
        PrintError("savePath ERROR! Failed to Write file: %s", savePoseDirectory.c_str());
        return;
    }
    // file_pose_gnss.close();
    file_pose_optimized.close();
    file_odom_path.close();    
}

bool optimization::savePoseService(fusion_slam::save_poseRequest& req, fusion_slam::save_poseResponse& res)
{
    // pose pose_gnss;
    // pose pose_optimized;
    // pose pose_without_optimized;

    // // std::ofstream  file_pose_gnss;
    // std::ofstream  file_pose_optimized;
    // std::ofstream  file_pose_without_optimized;
    // std::ofstream  file_odom_path;//保存轨迹用于返航

    // publishAllPath(pubPosePath);//send pose for return;

    // string savePoseDirectory;
    // // cout << "****************************************************" << endl;
    // // cout << "Saving poses to pose files ..." << endl;
    // PrintInfo("****************************************************");
    // PrintInfo("Saving poses to pose files ...");
    // if(req.destination.empty()) 
    //     savePoseDirectory = savePCDDirectory;
    //     // savePoseDirectory = std::getenv("HOME") + savePCDDirectory;
    // else 
    //     savePoseDirectory = req.destination;
    //     // savePoseDirectory = std::getenv("HOME") + req.destination;
    // // cout << "Save destination: " << savePoseDirectory << endl;
    // PrintInfo("Save destination: %s", savePoseDirectory.c_str());

    // // create file 
    // // CreateFile(file_pose_gnss, savePoseDirectory + "/gnss_pose.txt");
    // CreateFile(file_pose_optimized, savePoseDirectory + "/optimized_pose.txt");
    // CreateFile(file_pose_without_optimized, savePoseDirectory + "/without_optimized_pose.txt");
    // CreateFile(file_odom_path, savePoseDirectory + "/odom_path.csv");

    // // save optimize data
    // for(int i = 0; i  < cloudKeyPoses6D->size(); i++){  
    //     pose_optimized.t =  Eigen::Vector3d(cloudKeyPoses6D->points[i].x, cloudKeyPoses6D->points[i].y, cloudKeyPoses6D->points[i].z  );
    //     pose_optimized.R = Eigen::Quaterniond(cloudKeyPoses6D->points[i].qw,
    //                                           cloudKeyPoses6D->points[i].qx,
    //                                           cloudKeyPoses6D->points[i].qy,
    //                                           cloudKeyPoses6D->points[i].qz).normalized().toRotationMatrix();  

    //     // WriteTextKITTI(file_pose_optimized, pose_optimized);
    //     WriteTextTUM(file_pose_optimized, cloudKeyPoses6D->points[i].time, pose_optimized);
    // }
    // // cout << "Sucess global optimized  poses to pose files ..." << endl;
    // PrintInfo("saved global optimized  poses to pose files");

    // for(int i = 0; i  < lio_cloudKeyPoses6D->size(); i++){  
    //     pose_without_optimized.t =  Eigen::Vector3d(lio_cloudKeyPoses6D->points[i].x, lio_cloudKeyPoses6D->points[i].y, lio_cloudKeyPoses6D->points[i].z  );
    //     pose_without_optimized.R = Eigen::Quaterniond(lio_cloudKeyPoses6D->points[i].qw,
    //                                                   lio_cloudKeyPoses6D->points[i].qx,
    //                                                   lio_cloudKeyPoses6D->points[i].qy,
    //                                                   lio_cloudKeyPoses6D->points[i].qz).normalized().toRotationMatrix();

    //     // WriteTextKITTI(file_pose_without_optimized, pose_without_optimized);
    //     WriteTextTUM(file_pose_without_optimized, lio_cloudKeyPoses6D->points[i].time, pose_without_optimized);
    // }
    // // cout << "Sucess unoptimized  poses to pose files ..." << endl;
    // PrintInfo("saved unoptimized  poses to pose files");

    // // for(int i = 0; i  < gnss_cloudKeyPoses6D->size(); i++){  
    // //     pose_gnss.t =  Eigen::Vector3d(gnss_cloudKeyPoses6D->points[i].x, gnss_cloudKeyPoses6D->points[i].y, gnss_cloudKeyPoses6D->points[i].z  );
    // //     pose_gnss.R = Exp(double(gnss_cloudKeyPoses6D->points[i].roll), double(gnss_cloudKeyPoses6D->points[i].pitch), double(gnss_cloudKeyPoses6D->points[i].yaw) );
    // //     // WriteTextKITTI(file_pose_gnss, pose_gnss);
    // //     WriteTextTUM(file_pose_without_optimized, gnss_cloudKeyPoses6D->points[i].time, pose_gnss);
    // // }
    // // // cout << "Sucess gnss  poses to pose files ..." << endl;
    // // PrintInfo("saved gnss poses to pose files");

    // // 保存轨迹用于路径跟踪
    // try{
    //     /*写入文件名*/
    //     file_odom_path << "file: " << savePoseDirectory << std::endl;
    //     /* 写入时间*/
    //     time_t now;
    //     struct tm *timenow;
    //     time(&now);
    //     timenow = localtime(&now);
    //     char buf[32];
    //     strftime(buf, 32, "%F", timenow);    //转化为ISO日期格式的字符串（YY-MM-DD）
    //     file_odom_path << "date: " << buf << std::endl;
    //     //写入路径frame
    //     file_odom_path << "frame_id: " << "map" << std::endl;
    //     /* 写入路径点的个数 */
    //     file_odom_path << "size: " << static_cast<int>((int)cloudKeyPoses3D->size()) << std::endl;
    //     /* 写入路径点 */
    //     //ofstrm << "x" << "," << "y" << "," << "z" << std::endl;
    //     for (int i = 0; i < (int)cloudKeyPoses3D->size(); i++) {
    //         file_odom_path << setprecision(9) << "header:0,0,map,poses:" 
    //         << cloudKeyPoses3D->points[i].x << "," << cloudKeyPoses3D->points[i].y  << "," << cloudKeyPoses3D->points[i].z 
    //         << ",0,0,0,0," << endl;
    //     }
    //     // cout << "Sucess odom  poses to pose files ..." << endl;
    //     PrintInfo("saved odom  poses to pose files");
    // }
    // catch(...){
    //     ROS_ERROR("savePath ERROR! Failed to Write file: %s", savePoseDirectory.c_str());
    //     return false;
    // }
    // // file_pose_gnss.close();
    // file_pose_optimized.close();
    // file_pose_without_optimized.close();
    // file_odom_path.close();
    savePose = true;
    res.success = 1;
    return true  ;
}

void optimization::modeHandler(const std_msgs::UInt8::ConstPtr& modeMsg)
{
    if(modeMsg->data == 1 || modeMsg->data == 2)
    {
        loopMode = 0;
    }
    else
    {
        loopMode = loopClosureEnableFlag;
    }
    // PrintInfo("loopMode :%d", loopMode);
}

void optimization::armModeHandler(const std_msgs::UInt8::ConstPtr& armModeMsg)
{
    if(armModeMsg->data == 1)
    {
        isNeedLocalMap = 1;
    }
    else
    {
        isNeedLocalMap = 0;
    }
}

// gps to map
bool optimization::gpsToMapService(fusion_slam::gps_to_mapRequest& req, fusion_slam::gps_to_mapResponse& res)
{
    // 未接受到gps
    if(bFirstGPS)
    {
        res.success = 0;
        res.x = -100000;
        res.y = -100000;
        res.z = -100000;
        return true;
    }
    // 使用当前gps值对应的map坐标
    if(req.latitude == 0.0 || req.longitude == 0.0)
    {
        Eigen::Vector3d odomLio = transformTobeMapped.translation();
        res.x = odomLio[0];
        res.y = odomLio[1];
        res.z = odomLio[2];
        res.success = 1;
        return true;        
    }

    double UTMNorthing;
    double UTMEasting;
    double latitude  = req.latitude;
    double longitude = req.longitude;
    double altitude  = req.altitude;

    gps_common::LLtoUTM(latitude, longitude, UTMNorthing, UTMEasting, UTMZone);

    Eigen::Vector3d curUTM(UTMEasting, UTMNorthing, altitude);
    curUTM = curUTM - zeroUTM;
    curUTM = T_lw*rotationVector*curUTM;

    res.x = curUTM(0);
    res.y = curUTM(1);
    res.z = curUTM(2);

    res.success = 1;
    return true;
}

// map to gps::TODO
bool optimization::mapToGpsService(fusion_slam::map_to_gpsRequest& req, fusion_slam::map_to_gpsResponse& res)
{
    if(bFirstGPS)
    {
        res.success = 0;
        res.latitude  = -1;
        res.longitude = -1;
        res.altitude  = -1;        
        return true;
    }

    if(gpsInitOrientation && !gpsInitialized)
    {
        res.success = 0;
        res.latitude  = -1;
        res.longitude = -1;
        res.altitude  = -1;        
        return true;        
    }

    Eigen::Vector3d curUTM(req.x, req.y, req.z);
    curUTM = rotationVector.inverse()*T_lw.inverse()*curUTM;
    curUTM = curUTM + zeroUTM;

    double latitude, longitude;
    gps_common::UTMtoLL(curUTM(1), curUTM(0), UTMZone, latitude, longitude);
    res.latitude  = latitude;
    res.longitude = longitude;
    res.altitude  = curUTM(2);
    res.success = 1;
    return true;
}

/**
 * @description: save cloud map to xyz
 * @param {Ptr} cloudIn
 * @param {string} xyzFilePath
 * @return {*}
 */
void optimization::savePclToXYZ(pcl::PointCloud<PointType>::Ptr cloudIn, std::string xyzFilePath)
{
    if(cloudIn->points.size() <= 0)return;

    std::ofstream xyzFile(xyzFilePath);

    // #pragma omp parallel for num_threads(numberOfCores)
    for (int i = 0; i < cloudIn->points.size(); ++i)
    {
        const auto &pointFrom = cloudIn->points[i];
        if(std::isnan(pointFrom.x) || std::isnan(pointFrom.y))
        {
            continue;
        }
        // xyzFile.write((char*)& pointFrom.x, sizeof(pointFrom.x));
        // xyzFile.write((char*)& pointFrom.y, sizeof(pointFrom.y));
        xyzFile << pointFrom.x << " " << pointFrom.y << " " << pointFrom.z << std::endl;
    }
    xyzFile.close();
}

// int keyFramesSize = 0;
void optimization::saveAndPublishMapInfo()
{
    pcl::PointCloud<PointType>::Ptr save_cloudKeyPoses3D{new pcl::PointCloud<PointType>()};
    pcl::PointCloud<PointTypePose>::Ptr save_cloudKeyPoses6D{new pcl::PointCloud<PointTypePose>()};
    mtx.lock();
    *save_cloudKeyPoses3D = *cloudKeyPoses3D;
    *save_cloudKeyPoses6D = *cloudKeyPoses6D;
    mtx.unlock();

    int curFloorNum;
    FloorState curFloorState;
    mtxFloorInfo.lock();
    curFloorNum = floorNum;
    if(floorStates.find(curFloorNum) != floorStates.end())
    {
        curFloorState = floorStates.at(curFloorNum);
    }
    mtxFloorInfo.unlock();

    // static int keyFramesSize = -9;

    std::ofstream poseJson;
    std::string posePath = savePCDDirectory + "pose.json";
    CreateFile(poseJson, posePath);

    std::string scanPath = savePCDDirectory + "scan/";
    std::ofstream scanPose;
    std::string scanPosePath = scanPath + "scan.txt";
    CreateFile(scanPose, scanPosePath);

    if(!isFullMap) // && save_cloudKeyPoses3D->size() - keyFramesSize >= saveFrameInterval)
    {
        PrintInfo("start save map");
        int keyFramesSize = save_cloudKeyPoses3D->size();
        globalMapCloudAll->clear();
        globalMapCloud2d->clear();
        // pcl::PointCloud<PointType>::Ptr globalMapCloud(new pcl::PointCloud<PointType>());

        // 注意：拼接地图时，keyframe是lidar系，而fastlio更新后的存到的cloudKeyPoses6D 关键帧位姿是body系下的，需要把
        //cloudKeyPoses6D  转换为T_world_lidar 。 T_world_lidar = T_world_body * T_body_lidar , T_body_lidar 是外参
        for (int i = lastFloorFrame; i < keyFramesSize; i++) 
        {
            // if(i >= surfCloudKeyFrames.size() || i >= save_cloudKeyPoses6D->size())
            if(i >= cloudKeyFramesForVisual.size() || i >= save_cloudKeyPoses6D->size())
            {
                PrintWarn("saving map, idx is out of range");
                break;
            }
            // *globalMapCloud   += *transformPointCloud(cloudKeyFramesForVisual[i], &save_cloudKeyPoses6D->points[i]);
            transformPointCloudNew(cloudKeyFramesForVisual[i], &save_cloudKeyPoses6D->points[i]);
        }
        // 分层保存的数据
        for(int i = 0; i < curFloorState.size(); i++)
        {
            int startIdx = curFloorState[i].startIdx;
            int endIdx = curFloorState[i].endIdx;
            if(endIdx - startIdx < 2)continue;
            for(int j = startIdx; j < endIdx; j++)
            {
                if(i >= cloudKeyFramesForVisual.size() || i >= save_cloudKeyPoses6D->size())
                {
                    PrintWarn("saving floor map, idx is out of range");
                    break;
                }
                // *globalMapCloud   += *transformPointCloud(cloudKeyFramesForVisual[j], &save_cloudKeyPoses6D->points[j]);
                transformPointCloudNew(cloudKeyFramesForVisual[j], &save_cloudKeyPoses6D->points[j]);
            }
        }

        if(globalMapCloud2d->size()==0 || globalMapCloudAll->size()==0)
        {
            PrintWarn("globalmap is empty");
            return;
        }

        pcl::PointCloud<PointType>::Ptr globalMapCloudAllDS(new pcl::PointCloud<PointType>());
        // pcl::PointCloud<PointType>::Ptr globalMapCloud2dDS(new pcl::PointCloud<PointType>());
        downSizeFilterMap.setInputCloud(globalMapCloudAll);
        downSizeFilterMap.setLeafSize(0.1, 0.1, 0.1);
        downSizeFilterMap.filter(*globalMapCloudAllDS);

        // downSizeFilterMap.setInputCloud(globalMapCloud2d);
        // downSizeFilterMap.setLeafSize(0.1, 0.1, 0.1);
        // downSizeFilterMap.filter(*globalMapCloud2dDS);     

        // pcl::RadiusOutlierRemoval<PointType> outrem;
        // outrem.setInputCloud(globalMapCloud2dDS);//输入点云
        // outrem.setRadiusSearch(0.5);//设置滤波搜索半径
        // outrem.setMinNeighborsInRadius(15);//设置半径内的至少的点云个数
        // outrem.setKeepOrganized(false);
        // outrem.filter(*globalMapCloud2dDS);//开始滤波，并获取滤波点云      

        // pcl::io::savePCDFileBinary(savePCDDirectory + std::to_string(curFloorNum) + "GlobalMap2d.pcd", *globalMapCloud2dDS);
        pcl::io::savePCDFileBinary(savePCDDirectory + std::to_string(curFloorNum) + "GlobalMapAll.pcd", *globalMapCloudAllDS);   
    }
    else
    {
        PrintInfo("start save full map");
        int keyFramesSize = save_cloudKeyPoses3D->size();
        globalMapCloudAll->clear();
        // globalMapCloud2d->clear();
        for (int i = 0; i < keyFramesSize; i++) 
        {
            if(i >= cloudKeyFramesForVisual.size())
            {
                PrintWarn("saving map, idx is out of range");
                break;
            }
            //save pose
            Eigen::Isometry3d frame_pose = pclPointToIsometry3d(save_cloudKeyPoses6D->points[i]);
            Eigen::Quaterniond pose_quat(frame_pose.rotation());
            poseJson << std::fixed << frame_pose.translation()[0] << " " << frame_pose.translation()[1] << " " << frame_pose.translation()[2] << " "
                << pose_quat.w() << " " << pose_quat.x() << " " << pose_quat.y() << " " << pose_quat.z()  << std::endl;
            
            std::string curFramePath = savePCDDirectory + "scan/" + std::to_string(i) + ".pcd";
            std::cout << "curFramePath:" << curFramePath << std::endl;
            pcl::io::savePCDFileBinary(curFramePath, *(surfCloudKeyFrames[i]));

            transformPointCloudNew(cloudKeyFramesForVisual[i], &save_cloudKeyPoses6D->points[i]);
        }
        PrintInfo("end save scan");
        
        if(globalMapCloudAll->size()==0)
        {
            PrintWarn("globalmap is empty");
            return;
        }
        pcl::PointCloud<PointType>::Ptr globalMapCloudAllDS(new pcl::PointCloud<PointType>());
        downSizeFilterMap.setInputCloud(globalMapCloudAll);
        downSizeFilterMap.setLeafSize(0.1, 0.1, 0.1);
        downSizeFilterMap.filter(*globalMapCloudAllDS);  

        pcl::io::savePCDFileBinary(savePCDDirectory + "GlobalMapAll.pcd", *globalMapCloudAllDS);  
        PrintInfo("end save full map");
    }

}

//地图保存发布线程
void optimization::saveMapThread()
{
    savePCD = false;
    savePose = false;
    ros::Rate rate(1); //   频率
    while (ros::ok() && startFlag)
    {
        rate.sleep();
        if(savePCD)
        {
            saveAndPublishMapInfo();   //保存地图检测
            savePCD = false;
        }

        if(savePose)
        {
            saveAndPublishPose();
            savePose = false;
        }
            
    }
}

void optimization::publishGrid3D(const Grid3D& grid, int size_x, int size_y, int size_z) {
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
    pubGrid.publish(msg);
}
 
void optimization::saveGrid3D(const Grid3D& grid, const std::string& path) {
    std::ofstream outfile(path, std::ios::binary);
    for (const auto& slice : grid) {
        for (const auto& row : slice) {
            outfile.write(reinterpret_cast<const  char*>(row.data()),  row.size()  * sizeof(int));
        }
    }
    outfile.close(); 
}

}
