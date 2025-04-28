/*
 * @Author: dyhan
 * @Date: 2022-10-12 09:22:35
 * @LastEditors: dyhan
 * @LastEditTime: 2024-10-28 09:32:27
 * @Description: 
 */
#include "lio/lioMapping.h"
namespace fusion_slam{
bool lioMapping::initROS(ros::NodeHandle* nh, ros::NodeHandle* priNh)
{
    LogMgmt::GetInstance();

    loadParams(nh);

    initVariable();

    //初始化，其中h_share_model定义了·平面搜索和残差计算
    // std::function<void()> func = [this](state_ikfom &s, esekfom::dyn_share_datastruct<double> &ekfom_data) ->void{h_share_model(s, ekfom_data);};
    kf.init_dyn_share(
        get_f, df_dx, df_dw,
        [this](state_ikfom &s, esekfom::dyn_share_datastruct<double> &ekfom_data){h_share_model(s, ekfom_data);}, NUM_MAX_ITERATIONS, epsi);

    subAndPubToROS(nh);

    /*** debug record ***/
    string pos_log_dir = root_dir + "/Log/pos_log.txt";
    fp = fopen(pos_log_dir.c_str(),"w");

    fout_pre.open(DEBUG_FILE_DIR("mat_pre.txt"),ios::out);
    fout_out.open(DEBUG_FILE_DIR("mat_out.txt"),ios::out);
    // fout_dbg.open(DEBUG_FILE_DIR("dbg.txt"),ios::out);
    if (fout_pre && fout_out)
        std::cout << "~~~~"<<ROOT_DIR<<" file opened" << std::endl;
    else
        std::cout << "~~~~"<<ROOT_DIR<<" doesn't exist" << std::endl;

    // mainLoopTimer = nh->createTimer(ros::Duration(1.0/10.0), &lioMapping::mainLoop(), this);
    // mainThread = new std::thread(std::bind(&lioMapping::mainLoop, this));
    // globalLocalizationThread = new std::thread(std::bind(&lioMapping::globalLocalization, this));

    return true;
}

void lioMapping::initVariable()
{
    path.header.stamp    = ros::Time::now();
    path.header.frame_id = odometryFrame;//mapFrame;
    
    FOV_DEG = (fov_deg + 10.0) > 179.9 ? 179.9 : (fov_deg + 10.0);
    HALF_FOV_COS = cos((FOV_DEG) * 0.5 * PI_M / 180.0);

    _featsArray.reset(new PointCloudXYZI());

    memset(point_selected_surf, true, sizeof(point_selected_surf));
    memset(res_last, -1000.0f, sizeof(res_last));
    downSizeFilterSurf.setLeafSize(filter_size_surf_min, filter_size_surf_min, filter_size_surf_min);
    memset(point_selected_surf, true, sizeof(point_selected_surf));
    memset(res_last, -1000.0f, sizeof(res_last));

    Lidar_T_wrt_IMU<<VEC_FROM_ARRAY(extrinT);
    Lidar_R_wrt_IMU<<MAT_FROM_ARRAY(extrinR);
    p_imu->set_extrinsic(Lidar_T_wrt_IMU, Lidar_R_wrt_IMU);
    p_imu->set_gyr_cov(V3D(gyr_cov, gyr_cov, gyr_cov));//陀螺仪协方差
    p_imu->set_acc_cov(V3D(acc_cov, acc_cov, acc_cov));//加速度协方差
    p_imu->set_gyr_bias_cov(V3D(b_gyr_cov, b_gyr_cov, b_gyr_cov));//陀螺仪噪声协方差
    p_imu->set_acc_bias_cov(V3D(b_acc_cov, b_acc_cov, b_acc_cov));//加速度噪声协方差

    Wheel_T_wrt_IMU<<VEC_FROM_ARRAY(extrinT_wheel);
    Wheel_R_wrt_IMU<<MAT_FROM_ARRAY(extrinR_wheel);
    useWheelVel = useWheelVelParam;

    fill(epsi, epsi+23, 0.001);

    mapDirectory = std::getenv("HOME") + mapDirectory;

    if(isLocalization)
    {
        if(loadPriorMap(mapDirectory))
        {
            // initIkdTree();
            globalLocalizationThread = new std::thread(std::bind(&lioMapping::globalLocalization, this));
        }
        else
        {
            // isPriorPoseInit = true;
            PrintInfo("loadPriorMap fail");
        }
    }
}

void lioMapping::loadParams(ros::NodeHandle *priNh)
{
    priNh->param<bool>("publish/path_en",path_en, true);
    priNh->param<bool>("publish/scan_publish_en",scan_pub_en, true);
    priNh->param<bool>("publish/dense_publish_en",dense_pub_en, true);
    priNh->param<bool>("publish/scan_bodyframe_pub_en",scan_body_pub_en, true);
    priNh->param<int>("max_iteration",NUM_MAX_ITERATIONS,4);
    priNh->param<string>("map_file_path",map_file_path,"");
    priNh->param<string>("common/lid_topic",lid_topic,"/livox/lidar");
    priNh->param<string>("common/imu_topic", imu_topic,"/livox/imu");
    priNh->param<string>("common/wheel_topic", wheel_topic,"/twist_feedback");
    priNh->param<string>("common/target_topic", target_topic,"/TargetRelativePos");
    priNh->param<bool>("common/time_sync_en", time_sync_en, false);
    priNh->param<double>("filter_size_corner",filter_size_corner_min,0.5);
    priNh->param<double>("filter_size_surf",filter_size_surf_min,0.5);
    priNh->param<double>("filter_size_map",filter_size_map_min,0.5);
    priNh->param<double>("cube_side_length",cube_len,200);
    priNh->param<float>("mapping/det_range",DET_RANGE,300.f);
    priNh->param<double>("mapping/fov_degree",fov_deg,180);
    priNh->param<double>("mapping/gyr_cov",gyr_cov,0.1);
    priNh->param<double>("mapping/acc_cov",acc_cov,0.1);
    priNh->param<double>("mapping/b_gyr_cov",b_gyr_cov,0.0001);
    priNh->param<double>("mapping/b_acc_cov",b_acc_cov,0.0001);
    priNh->param<double>("preprocess/blind", p_pre->blind, 0.01);
    priNh->param<double>("preprocess/blind_far", p_pre->blind_far, 0.01);
    priNh->param<int>("preprocess/lidar_type", p_pre->lidar_type, AVIA);
    priNh->param<int>("preprocess/scan_line", p_pre->N_SCANS, 16);
    priNh->param<int>("preprocess/timestamp_unit", p_pre->time_unit, US);
    priNh->param<int>("preprocess/scan_rate", p_pre->SCAN_RATE, 10);
    priNh->param<vector<float> >("preprocess/lidar_rot", p_pre->lidarRot, vector<float>());
    priNh->param<int>("preprocess/cut_frame_num", cut_frame_num, 0);
    priNh->param<int>("point_filter_num", p_pre->point_filter_num, 2);
    priNh->param<bool>("feature_extract_enable", p_pre->feature_enabled, false);
    priNh->param<bool>("runtime_pos_log_enable", runtime_pos_log, 0);
    priNh->param<bool>("mapping/extrinsic_est_en", extrinsic_est_en, true);
    priNh->param<bool>("pcd_save/pcd_save_en", pcd_save_en, false);
    priNh->param<int>("pcd_save/interval", pcd_save_interval, -1);
    priNh->param<vector<double>>("mapping/extrinsic_T", extrinT, vector<double>());
    priNh->param<vector<double>>("mapping/extrinsic_R", extrinR, vector<double>());

    priNh->param<vector<double>>("wheel/extrinsic_T", extrinT_wheel, vector<double>());
    priNh->param<vector<double>>("wheel/extrinsic_R", extrinR_wheel, vector<double>());
    priNh->param<bool>("wheel/use_wheel_vel", useWheelVelParam, false);
    
    // Frame
    priNh->param<std::string>("common/lidarFrame", lidarFrame, "base_link"); 
    priNh->param<std::string>("common/baselinkFrame", baselinkFrame, "base_link");
    priNh->param<std::string>("common/odometryFrame", odometryFrame, "odom");
    priNh->param<std::string>("common/mapFrame", mapFrame, "map");  

    // imu compensation 
    priNh->param<bool>("imu/imuRPYEnable", imuRPYEnable, false);
    priNh->param<vector<double>>("imu/imuRPYWeight", imuRPYWeight, vector<double>());
    priNh->param<double>("imu/angleDiff", angleDiff, 0.2);

    priNh->param<bool>("lio_sam/isLocalization", isLocalization, false);
    priNh->param<int>("lio_sam/reLocalizationMode", reLocalizationMode, 0);
    priNh->param<std::string>("lio_sam/savePCDDirectory", mapDirectory, "/Downloads/LOAM/");    

    std::vector<double> initPoseQuat{1.0,0,0,0};
    std::vector<double> initPoseTran{0,0,0};
    priNh->param<std::vector<double> >("lio_sam/initPoseQuat", initPoseQuat, std::vector<double>());
    priNh->param<std::vector<double> >("lio_sam/initPoseTran", initPoseTran, std::vector<double>());
    Eigen::Quaterniond qPose(initPoseQuat[0], initPoseQuat[1], initPoseQuat[2], initPoseQuat[3]);
    Eigen::Vector3d tPose(initPoseTran[0], initPoseTran[1], initPoseTran[2]);
    initPose3d.rotate(qPose);
    initPose3d.pretranslate(tPose);
    PrintInfo("initpose:%f,%f,%f,%f,%f,%f,%f", initPoseTran[0], initPoseTran[1], initPoseTran[2], initPoseQuat[0], initPoseQuat[1], initPoseQuat[2], initPoseQuat[3]);
}

void lioMapping::subAndPubToROS(ros::NodeHandle *nh)
{
    /*** ROS subscribe initialization ***/
#ifdef ENABLE_LIVOX
    if(p_pre->lidar_type == AVIA)
    {
        subPcl = nh->subscribe(lid_topic, 20, &lioMapping::livox_pcl_cbk, this);
    }
    else
    {
        subPcl = nh->subscribe(lid_topic, 20, &lioMapping::standard_pcl_cbk, this);
    }
#else
    subPcl = nh->subscribe(lid_topic, 20, &lioMapping::standard_pcl_cbk, this, ros::TransportHints().tcpNoDelay());
#endif
    // subPcl = p_pre->lidar_type == AVIA ? nh->subscribe(lid_topic, 20, &lioMapping::livox_pcl_cbk, this) 
    // : nh->subscribe(lid_topic, 20, &lioMapping::standard_pcl_cbk, this);
    subImu = nh->subscribe(imu_topic, 600, &lioMapping::imu_cbk, this, ros::TransportHints().tcpNoDelay());

    // subTargetPose = nh->subscribe<geometry_msgs::PoseStamped>(target_topic, 10, &lioMapping::target_cbk, this);
    subMode = nh->subscribe<std_msgs::UInt8>("/planMode", 5, &lioMapping::mode_cbk, this);

    if (useWheelVel)
    {
        subWheelVel = nh->subscribe(wheel_topic, 20, &lioMapping::wheel_cbk, this);
    }
    if(isLocalization)
    {
        subInitPose = nh->subscribe<geometry_msgs::PoseStamped>("/init_pose_topic", 50, &lioMapping::initPoseHandler, this);
    }

    pubLaserCloudFull      = nh->advertise<sensor_msgs::PointCloud2>("/cloud_registered", 10); //  world系下稠密点云
    pubLaserCloudFull_body = nh->advertise<sensor_msgs::PointCloud2>("/cloud_registered_body", 10); // body系下稠密点云
    pubOdomAftMapped       = nh->advertise<nav_msgs::Odometry> ("/Odometry", 10);
    pubPath                = nh->advertise<nav_msgs::Path> ("/path", 10);
    pubLaserCloudMap       = nh->advertise<sensor_msgs::PointCloud2>("/Laser_map", 10);
    pubPriorMap            = nh->advertise<sensor_msgs::PointCloud2>("/prior_map", 10);

    // pubTwist = nh->advertise<geometry_msgs::Twist>("/slam_twist", 10);
    //publish pose to operation
    // pubPosUpdate = nh->advertise<geometry_msgs::Point>("/robot_pose_topic", 10); 
}

void lioMapping::publish_frame_world(const ros::Publisher &pubLaserCloudFull)
{
    if(scan_pub_en)
    {
        PointCloudXYZI::Ptr laserCloudFullRes(dense_pub_en ? feats_undistort : feats_down_body);
        int size = laserCloudFullRes->points.size();
        PointCloudXYZI::Ptr laserCloudWorld(new PointCloudXYZI(size, 1));

        for (int i = 0; i < size; i++)
        {
            RGBpointBodyToWorld(&laserCloudFullRes->points[i], &laserCloudWorld->points[i]);
        }

        sensor_msgs::PointCloud2 laserCloudmsg;
        pcl::toROSMsg(*laserCloudWorld, laserCloudmsg);
        laserCloudmsg.header.stamp = ros::Time().fromSec(lidar_end_time);
        laserCloudmsg.header.frame_id = odometryFrame;//mapFrame;//"camera_init";
        pubLaserCloudFull.publish(laserCloudmsg);
        publish_count -= PUBFRAME_PERIOD;
    }

    /**************** save map ****************/
    /* 1. make sure you have enough memories
    /* 2. noted that pcd save will influence the real-time performences **/
    if (pcd_save_en)
    {
        int size = feats_undistort->points.size();
        PointCloudXYZI::Ptr laserCloudWorld(new PointCloudXYZI(size, 1));

        for (int i = 0; i < size; i++)
        {
            RGBpointBodyToWorld(&feats_undistort->points[i], &laserCloudWorld->points[i]);
        }
        *pcl_wait_save += *laserCloudWorld;

        static int scan_wait_num = 0;
        scan_wait_num ++;
        if (pcl_wait_save->size() > 0 && pcd_save_interval > 0  && scan_wait_num >= pcd_save_interval)
        {
            pcd_index ++;
            string all_points_dir(string(string(ROOT_DIR) + "PCD/scans_") + to_string(pcd_index) + string(".pcd"));
            pcl::PCDWriter pcd_writer;
            cout << "current scan saved to /PCD/" << all_points_dir << endl;
            pcd_writer.writeBinary(all_points_dir, *pcl_wait_save);
            pcl_wait_save->clear();
            scan_wait_num = 0;
        }
    }
}

void lioMapping::publish_frame_body(const ros::Publisher & pubLaserCloudFull_body)
{
    int size = feats_undistort->points.size();
    PointCloudXYZI::Ptr laserCloudIMUBody(new PointCloudXYZI(size, 1));

    for (int i = 0; i < size; i++)
    {
        RGBpointBodyLidarToIMU(&feats_undistort->points[i], &laserCloudIMUBody->points[i]);
    }

    if(lidarRpyInit)
    {
        pcl::transformPointCloud(*laserCloudIMUBody, *laserCloudIMUBody, rotateLidarToBody.matrix());
    }

    sensor_msgs::PointCloud2 laserCloudmsg;
    pcl::toROSMsg(*laserCloudIMUBody, laserCloudmsg);
    laserCloudmsg.header.stamp = ros::Time().fromSec(lidar_end_time);
    laserCloudmsg.header.frame_id = baselinkFrame;//lidarFrame;//body";
    pubLaserCloudFull_body.publish(laserCloudmsg);
    publish_count -= PUBFRAME_PERIOD;
}

void lioMapping::publish_effect_world(const ros::Publisher & pubLaserCloudEffect)
{
    PointCloudXYZI::Ptr laserCloudWorld(new PointCloudXYZI(effct_feat_num, 1));
    for (int i = 0; i < effct_feat_num; i++)
    {
        RGBpointBodyToWorld(&laserCloudOri->points[i], &laserCloudWorld->points[i]);
    }
    sensor_msgs::PointCloud2 laserCloudFullRes3;
    pcl::toROSMsg(*laserCloudWorld, laserCloudFullRes3);
    laserCloudFullRes3.header.stamp = ros::Time().fromSec(lidar_end_time);
    laserCloudFullRes3.header.frame_id = odometryFrame;//mapFrame;//"camera_init";
    pubLaserCloudEffect.publish(laserCloudFullRes3);
}

void lioMapping::publish_map(const ros::Publisher & pubLaserCloudMap)
{
    sensor_msgs::PointCloud2 laserCloudMap;
    pcl::toROSMsg(*featsFromMap, laserCloudMap);
    laserCloudMap.header.stamp = ros::Time().fromSec(lidar_end_time);
    laserCloudMap.header.frame_id = odometryFrame;//mapFrame;//"camera_init";
    pubLaserCloudMap.publish(laserCloudMap);
}

void lioMapping::publish_odometry(const ros::Publisher & pubOdomAftMapped)
{
    odomAftMapped.header.frame_id = odometryFrame;//mapFrame;//"camera_init";
    odomAftMapped.child_frame_id = lidarFrame;//"body";
    odomAftMapped.header.stamp = ros::Time().fromSec(lidar_end_time);// ros::Time().fromSec(lidar_end_time);
    set_posestamp(odomAftMapped.pose);
    set_twistamp(odomAftMapped.twist); 
    double quat_value = odomAftMapped.pose.pose.orientation.x*odomAftMapped.pose.pose.orientation.x+
                        odomAftMapped.pose.pose.orientation.y*odomAftMapped.pose.pose.orientation.y+
                        odomAftMapped.pose.pose.orientation.z*odomAftMapped.pose.pose.orientation.z+
                        odomAftMapped.pose.pose.orientation.w*odomAftMapped.pose.pose.orientation.w;
    if (fabs(quat_value-1.0) > 1e-2)
    {
        PrintError("odometry quat not norm");
        // return;
    }
    auto P = kf.get_P();
    for (int i = 0; i < 6; i ++)
    {
        int k = i < 3 ? i + 3 : i - 3;
        odomAftMapped.pose.covariance[i*6 + 0] = P(k, 3);
        odomAftMapped.pose.covariance[i*6 + 1] = P(k, 4);
        odomAftMapped.pose.covariance[i*6 + 2] = P(k, 5);
        odomAftMapped.pose.covariance[i*6 + 3] = P(k, 0);
        odomAftMapped.pose.covariance[i*6 + 4] = P(k, 1);
        odomAftMapped.pose.covariance[i*6 + 5] = P(k, 2);
    }

    pubOdomAftMapped.publish(odomAftMapped);

    static tf::TransformBroadcaster br;
    tf::Transform                   transform;
    tf::Quaternion                  q;
    transform.setOrigin(tf::Vector3(odomAftMapped.pose.pose.position.x,
                                    odomAftMapped.pose.pose.position.y,
                                    odomAftMapped.pose.pose.position.z));
    q.setW(odomAftMapped.pose.pose.orientation.w);
    q.setX(odomAftMapped.pose.pose.orientation.x);
    q.setY(odomAftMapped.pose.pose.orientation.y);
    q.setZ(odomAftMapped.pose.pose.orientation.z);
    transform.setRotation( q );
    // br.sendTransform( tf::StampedTransform( transform, odomAftMapped.header.stamp, odometryFrame, lidarFrame ) );

    /******* Publish pose to operation ********/
    geometry_msgs::Point robotPoint;
    robotPoint.x = odomAftMapped.pose.pose.position.x;
    robotPoint.y = odomAftMapped.pose.pose.position.y;
    double rollP, pitchP, yawP;
    tf::Matrix3x3(q).getRPY(rollP, pitchP, yawP);
    robotPoint.z = yawP;
    // pubPosUpdate.publish(robotPoint);

    // static tf::TransformBroadcaster br_base_body;
    // transform.setOrigin(tf::Vector3(-0.4,0.0,0.0));
    // q.setW(1.0);
    // q.setX(0.0);
    // q.setY(0.0);
    // q.setZ(0.0);
    // transform.setRotation( q );
    // br_base_body.sendTransform( tf::StampedTransform( transform, odomAftMapped.header.stamp, "base_link", "body" ) );
}

void lioMapping::publish_path(const ros::Publisher pubPath)
{
    set_posestamp(msg_body_pose);
    double quat_value = msg_body_pose.pose.orientation.x*msg_body_pose.pose.orientation.x+
                        msg_body_pose.pose.orientation.y*msg_body_pose.pose.orientation.y+
                        msg_body_pose.pose.orientation.z*msg_body_pose.pose.orientation.z+
                        msg_body_pose.pose.orientation.w*msg_body_pose.pose.orientation.w;
    if (fabs(quat_value-1.0) > 1e-2)
    {
        PrintError("path quat not norm");
        // return;
    }
    msg_body_pose.header.stamp = ros::Time().fromSec(lidar_end_time);
    msg_body_pose.header.frame_id = odometryFrame;//mapFrame;//"camera_init";

    /*** if path is too large, the rviz will crash ***/
    static int jjj = 0;
    jjj++;
    if (jjj % 20 == 0) 
    {
        path.poses.push_back(msg_body_pose);
        pubPath.publish(path); 
    }
}

void lioMapping::dump_lio_state_to_log(FILE *fp)  
{
    V3D rot_ang(Log(state_point.rot.toRotationMatrix()));
    fprintf(fp, "%lf ", Measures.lidar_beg_time - first_lidar_time);
    fprintf(fp, "%lf %lf %lf ", rot_ang(0), rot_ang(1), rot_ang(2));                   // Angle
    fprintf(fp, "%lf %lf %lf ", state_point.pos(0), state_point.pos(1), state_point.pos(2)); // Pos  
    fprintf(fp, "%lf %lf %lf ", 0.0, 0.0, 0.0);                                        // omega  
    fprintf(fp, "%lf %lf %lf ", state_point.vel(0), state_point.vel(1), state_point.vel(2)); // Vel  
    fprintf(fp, "%lf %lf %lf ", 0.0, 0.0, 0.0);                                        // Acc  
    fprintf(fp, "%lf %lf %lf ", state_point.bg(0), state_point.bg(1), state_point.bg(2));    // Bias_g  
    fprintf(fp, "%lf %lf %lf ", state_point.ba(0), state_point.ba(1), state_point.ba(2));    // Bias_a  
    fprintf(fp, "%lf %lf %lf ", state_point.grav[0], state_point.grav[1], state_point.grav[2]); // Bias_a  
    fprintf(fp, "\r\n");  
    fflush(fp);
}

bool lioMapping::initPriorPose(pcl::PointCloud<PointType>::Ptr laserCloudIn)
{
    // pcl::PointCloud<PointType>::Ptr laserCloudIn(new pcl::PointCloud<PointType>());

    //publishCloud(&fortest_publasercloudINWorld, laserCloudIn, timeLaserInfoStamp, "map");

    isPriorPoseInit = true;

    if(laserCloudIn->size() == 0)
    {
        PrintInfo("laserCloudIn->size() == 0");
        return false;
    }

    if(priorMap->points.size() == 0)
        return false;

    pcl::VoxelGrid<PointType> voxel;
    // voxel.setLeafSize(0.1, 0.1, 0.1);
    // voxel.setInputCloud(priorMap);
    // pcl::PointCloud<PointType>::Ptr priorMapFilter(new pcl::PointCloud<PointType>());
    // voxel.filter(*priorMapFilter);

    voxel.setLeafSize(0.1, 0.1, 0.1);
    voxel.setInputCloud(laserCloudIn);
    pcl::PointCloud<PointType>::Ptr laserCloudInFilter(new pcl::PointCloud<PointType>());
    voxel.filter(*laserCloudInFilter);

    if(pubPriorMap.getNumSubscribers() > 0)
    {
        ros::Time cur_t;
        sensor_msgs::PointCloud2 priorMapMsg;
        pcl::toROSMsg(*priorMapFilter, priorMapMsg);
        priorMapMsg.header.stamp = ros::Time().fromSec(lidar_end_time);
        priorMapMsg.header.frame_id = odometryFrame;//mapFrame;//"camera_init";
        pubPriorMap.publish(priorMapMsg);
    }

    Eigen::Affine3f poseInit = pcl::getTransformation(0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
    // Eigen::Affine3f poseInit = pcl::getTransformation(2.0, 2.0, 2.0, 0.0, 0.0, 0.2);

    #if 1
    // ndt
    pcl::NormalDistributionsTransform<PointType, PointType> ndt;
    ndt.setTransformationEpsilon(0.05);
    ndt.setResolution(1.0);
    ndt.setStepSize(0.7);
    ndt.setMaximumIterations(40);

    ndt.setInputSource(laserCloudInFilter);
    ndt.setInputTarget(priorMapFilter);
    pcl::PointCloud<PointType>::Ptr unused_result_ndt(new pcl::PointCloud<PointType>());

    ndt.align(*unused_result_ndt, poseInit.matrix());
    Eigen::Affine3f ndtPose;
    ndtPose = ndt.getFinalTransformation();

    // Eigen::Matrix4f mat4f = poseInit.matrix();
    // std::vector<double> res{10.0, 5.0, 3.0, 2.0};
    // for (auto& r : res) 
    // {
    //     pcl::PointCloud<PointType>::Ptr priorMapR(new pcl::PointCloud<PointType>());
    //     voxel.setLeafSize(r * 0.1, r * 0.1, r * 0.1);
    //     voxel.setInputCloud(priorMapFilter);
    //     voxel.filter(*priorMapR);
    //     ndt.setInputTarget(priorMapR);
    //     ndt.setResolution(r);
    //     ndt.align(*unused_result_ndt, mat4f);
    //     mat4f = ndt.getFinalTransformation();
    // }
    // Eigen::Affine3f ndtPose;
    // ndtPose = ndt.getFinalTransformation();

    float xPose, yPose, zPose, RollPose, PitchPose, YawPose;
    pcl::getTranslationAndEulerAngles (ndtPose, xPose, yPose, zPose, RollPose, PitchPose, YawPose);
    PrintInfo("ndt pose, trans: %f,%f,%f", xPose, yPose, zPose);
    PrintInfo("ndt pose, rpy: %f,%f,%f", RollPose, PitchPose, YawPose);
    #endif

    //icp
    pcl::IterativeClosestPoint<PointType, PointType> icp;
    icp.setMaxCorrespondenceDistance(20); //设置最大对应点的欧式距离，只有对应点之间的距离小于该设置值的对应点才作为ICP计算的点对
    icp.setMaximumIterations(100); //设置最大的迭代次数。迭代停止条件之一
    icp.setTransformationEpsilon(1e-6); //设置前后两次迭代的转换矩阵的最大容差（epsilion），一旦两次迭代小于这个最大容差，则认为已经收敛到最优解，迭代停止。迭代停止条件之二，默认值为：0 
    icp.setEuclideanFitnessEpsilon(1e-6); //设置前后两次迭代的点对的欧式距离均值的最大容差，迭代终止条件之三，默认值为：-std::numeric_limits::max ()
    // icp.setRANSACIterations(0);
    //use the outcome of ndt as the initial guess for ICP
    icp.setInputSource(laserCloudInFilter);
    icp.setInputTarget(priorMapFilter);
    pcl::PointCloud<PointType>::Ptr unused_result_icp(new pcl::PointCloud<PointType>());
    icp.align(*unused_result_icp, ndt.getFinalTransformation());
    // icp.align(*unused_result_icp, poseInit.matrix());

    if(icp.hasConverged())
    {
        PrintInfo("icp.hasConverged:%f", icp.getFitnessScore());
    }

    Eigen::Affine3f icpPose;
    icpPose = icp.getFinalTransformation();
    // float xPose, yPose, zPose, RollPose, PitchPose, YawPose;
    pcl::getTranslationAndEulerAngles (icpPose, xPose, yPose, zPose, RollPose, PitchPose, YawPose);

    PrintInfo("init pose, trans: %f,%f,%f", xPose, yPose, zPose);
    PrintInfo("init pose, rpy: %f,%f,%f", RollPose, PitchPose, YawPose);

    state_point = kf.get_x();
    state_point.pos = Eigen::Vector3d(xPose, yPose, zPose);
    Eigen::Quaterniond q = Eigen::AngleAxisd(YawPose, Eigen::Vector3d::UnitZ()) * 
                           Eigen::AngleAxisd(PitchPose, Eigen::Vector3d::UnitY()) * 
                           Eigen::AngleAxisd(RollPose, Eigen::Vector3d::UnitX());
    // Sophus::SO3 SO3_q(q);
    state_point.rot = q;
    kf.change_x(state_point);
    return true;
}

bool lioMapping::initIkdTree()
{
    if(priorMap->empty())
    {
        return false;
    }
    
    // pcl::VoxelGrid<PointType> voxel;
    // voxel.setLeafSize(filter_size_map_min, filter_size_map_min, filter_size_map_min);
    // voxel.setInputCloud(priorMapFilter);
    // // pcl::PointCloud<PointType>::Ptr priorMapKdtree(new pcl::PointCloud<PointType>());
    // voxel.filter(*priorMapKdtree);

    //加载读取点云数据到cloud中
    ikdtree.set_downsample_param(filter_size_map_min);
    ikdtree.Build(priorMapKdtree->points);
    // std::cout << "---- ikdtree size: " << ikdtree.size() << std::endl;
    PrintInfo("ikdtree.size():%d", ikdtree.size());
    return true;
}

bool lioMapping::loadPriorMap(const std::string& mapPath)
{
    // std::cout << "mapPath:" << mapPath << std::endl;
    PrintInfo("mapPath:%s", mapPath.c_str());
    std::string posePath = mapPath + "pose.json";
    std::vector<Eigen::Isometry3d> poseVec;
    std::fstream file;
    file.open(posePath);
    double tx, ty, tz, w, x, y, z;
    while(!file.eof())
    {
        file >> tx >> ty >> tz >> w >> x >> y >> z;
        if(file.fail())
            break;

        Eigen::Isometry3d curPose = Eigen::Isometry3d::Identity();
        Eigen::Quaterniond qPose(w, x, y, z);
        Eigen::Vector3d tPose(tx, ty, tz);

        position_map.push_back(tPose);
        pose_map.push_back(qPose);

        PointType curPt;
        curPt.x = tPose(0);
        curPt.y = tPose(1);
        curPt.z = tPose(2);
        priorMapKeyPose->push_back(curPt);

        curPose.rotate(qPose);
        curPose.pretranslate(tPose);
        poseVec.push_back(curPose);
    }

    PrintInfo("pose size:%d", poseVec.size());
    if(poseVec.size() < 0 )
    {
        PrintWarn("no prior pose");
        return false;
    }

    priorMapKeyPose->width = static_cast<int>(priorMapKeyPose->size());
    priorMapKeyPose->height = 1;
    priorMapKeyPoseKdtree->setInputCloud(priorMapKeyPose);

    ros::Time cur_t;

    // pcl::PointCloud<PointType>::Ptr priorMap(new pcl::PointCloud<PointType>);

    for(size_t i = 0; i < poseVec.size(); i++)
    {
        Eigen::Isometry3d curPose = poseVec[i];
        Eigen::Vector3d tPose = curPose.translation();
        Eigen::Quaterniond qPose(curPose.rotation());

        pcl::PointCloud<PointType>::Ptr curScan(new pcl::PointCloud<PointType>);
        std::string curScanPath = mapPath + "scan/" + std::to_string(i) + ".pcd";
        pcl::io::loadPCDFile(curScanPath, *curScan);

        if(curScan->size() <= 0)
        {
            continue;
        }

        scManager.makeAndSaveScancontextAndKeys(*curScan);

        pcl::PointCloud<PointType>::Ptr curScanMap(new pcl::PointCloud<PointType>);
        
        size_t curScanSize = curScan->points.size();
        curScanMap->points.resize(curScanSize);
        for(size_t j = 0; j < curScanSize; j++)
        {
            Eigen::Vector3d ptCur(curScan->points[j].x, curScan->points[j].y, curScan->points[j].z);
            Eigen::Vector3d ptMap = qPose * ptCur + tPose;
            curScanMap->points[j].x = ptMap(0);
            curScanMap->points[j].y = ptMap(1);
            curScanMap->points[j].z = ptMap(2);
        }

        *priorMap += *curScanMap;

        // map_key_frames_.push_back(curScanMap);

        // *map_filtered += *pc_filtered;

        // visualization_msgs::Marker marker;
        // marker.header.frame_id = odometryFrame;
        // marker.header.stamp = cur_t;
        // marker.ns = "basic_shapes";
        // marker.id = i;
        // marker.type = visualization_msgs::Marker::SPHERE;
        // marker.pose.position.x = tPose(0);
        // marker.pose.position.y = tPose(1);
        // marker.pose.position.z = tPose(2);
        // qPose.normalize();
        // marker.pose.orientation.x = qPose.x();
        // marker.pose.orientation.y = qPose.y();
        // marker.pose.orientation.z = qPose.x();
        // marker.pose.orientation.w = qPose.w();
        // marker.scale.x = 0.5; // Set the scale of the marker -- 1x1x1 here means 1m on a side
        // marker.scale.y = 0.5;
        // marker.scale.z = 0.5;
        // marker.color.r = float(1-float(i)/poseVec.size());
        // marker.color.g = float(float(i)/poseVec.size());
        // marker.color.b = float(float(i)/poseVec.size());
        // marker.color.a = 1.0;
        // marker.lifetime = ros::Duration();
        // pub_pose_marker_.publish(marker);
    }

    if(priorMap->size() <= 0)
    {
        PrintWarn("prior map is empty");
        return false;
    }

    PrintInfo("priorMap->size():%d", priorMap->size());

    pcl::VoxelGrid<PointType> voxel;
    voxel.setLeafSize(0.1, 0.1, 0.1);
    voxel.setInputCloud(priorMap);
    voxel.filter(*priorMapFilter);

    // pcl::VoxelGrid<PointType> voxel;
    voxel.setLeafSize(filter_size_map_min, filter_size_map_min, filter_size_map_min);
    voxel.setInputCloud(priorMapFilter);
    // pcl::PointCloud<PointType>::Ptr priorMapKdtree(new pcl::PointCloud<PointType>());
    voxel.filter(*priorMapKdtree);

    ikdtree_global.set_downsample_param(filter_size_map_min);
    ikdtree_global.Build(priorMapFilter->points);

    // if(pubPriorMap.getNumSubscribers() > 0)
    // {
    //     ros::Time cur_t;
    //     sensor_msgs::PointCloud2 priorMapMsg;
    //     pcl::toROSMsg(*priorMap, priorMapMsg);
    //     priorMapMsg.header.stamp = cur_t;
    //     priorMapMsg.header.frame_id = odometryFrame;//mapFrame;//"camera_init";
    //     pubPriorMap.publish(priorMapMsg);
    // }
    return true;
}

void lioMapping::initPoseHandler(const geometry_msgs::PoseStamped::ConstPtr &initPose)
{
    Eigen::Quaterniond qPose(1.0, 0.0, 0.0 ,0.0);
    if(initPose->pose.orientation.w != 0 || initPose->pose.orientation.x != 0 
        || initPose->pose.orientation.y != 0 || initPose->pose.orientation.z != 0)
    {
        qPose = Eigen::Quaterniond(initPose->pose.orientation.w, initPose->pose.orientation.x, 
            initPose->pose.orientation.y, initPose->pose.orientation.z);
        initPoseHeading = true;
    }
    else
    {
        initPoseHeading = false;
    }
    
    Eigen::Vector3d tPose(initPose->pose.position.x, initPose->pose.position.y, initPose->pose.position.z);
    PrintInfo("init pose trans:%f,%f,f, heading:%d", tPose(0), tPose(1), tPose(2), initPoseHeading);

    initPose3d.rotate(qPose);
    initPose3d.pretranslate(tPose);
}

void lioMapping::initPoseReLocalization()
{ 
    Eigen::Vector3d tInitPose =  initPose3d.translation();
    Eigen::Quaterniond qInitPose =  Eigen::Quaterniond(initPose3d.rotation());
    PointType searchPoint;
    searchPoint.x = tInitPose(0);
    searchPoint.y = tInitPose(1);
    searchPoint.z = tInitPose(2);
    std::vector<int> pointIdx;
    std::vector<float> pointDistance;
    std::unique_lock<std::mutex> lock_prior_map(prior_map_mutex);
    if (priorMapKeyPoseKdtree->radiusSearch(searchPoint, 10, pointIdx, pointDistance) > 0) 
    {
        lock_prior_map.unlock();
        pcl::PointCloud<PointType>::Ptr nearestFrameCloud(new pcl::PointCloud<PointType>);
        for (size_t i = 0; i < pointIdx.size(); i++) 
        {
            //使用时再加载
            int idx = pointIdx[i];
            Eigen::Vector3d tPose = position_map[idx];
            Eigen::Quaterniond qPose = pose_map[idx];
            Eigen::Isometry3d curPose3d = Eigen::Isometry3d::Identity();
            curPose3d.rotate(qPose);
            curPose3d.pretranslate(tPose);

            pcl::PointCloud<PointType>::Ptr idxFrameCloud(new pcl::PointCloud<PointType>);
            // pcl::PointCloud<PointType>::Ptr idxFrameCloudMap(new pcl::PointCloud<PointType>);
            pcl::io::loadPCDFile(mapDirectory + "scan/" + to_string(idx) + ".pcd", *idxFrameCloud);
            pcl::transformPointCloud(*idxFrameCloud, *idxFrameCloud, curPose3d.matrix());
            *nearestFrameCloud += *idxFrameCloud;
        }
        // downsample the local map
        pcl::VoxelGrid<PointType> voxelgrid;
        voxelgrid.setLeafSize(0.1, 0.1, 0.1);
        voxelgrid.setInputCloud(nearestFrameCloud);
        voxelgrid.filter(*nearestFrameCloud);

        // 初始化检查 两次成功初始化 位置增量小于阈值时通过检查
        int init_check = 0;
        // 重定位结果
        std::vector<int> init_ids;
        std::vector<Eigen::Matrix4d, Eigen::aligned_allocator<Eigen::Matrix4d>> init_poses;
        while (init_check < 2)
        {
            std::unique_lock<std::mutex> lock_init_feats(init_feats_down_body_mutex);
            int N = init_feats_down_bodys.size();
            lock_init_feats.unlock();
            if (N != 0)
            {
                // 获得初始化阶段去畸变后的当前帧点云
                lock_init_feats.lock();
                auto init_pair = init_feats_down_bodys.front();
                init_feats_down_bodys.pop();
                lock_init_feats.unlock();

                int current_init_id = init_pair.first;
                PointCloudXYZI::Ptr current_init_pc_origin = init_pair.second;
                PointCloudXYZI::Ptr current_init_pc(new PointCloudXYZI);
                pcl::copyPointCloud(*current_init_pc_origin, *current_init_pc);
                // pcl::transformPointCloud(*current_init_pc_origin, *current_init_pc, init_time_pose.matrix());

                Eigen::Matrix4d T_corr = Eigen::Matrix4d::Identity();

                Eigen::Matrix4d T_init = Eigen::Matrix4d::Identity();
                T_init.block<3, 3>(0, 0) = qInitPose.toRotationMatrix();
                T_init.block<3, 1>(0, 3) = tInitPose;
                Eigen::Affine3f poseInit(T_init.cast<float>());

                // ndt
                pcl::NormalDistributionsTransform<PointType, PointType> ndt;
                ndt.setTransformationEpsilon(0.05);
                ndt.setResolution(1.0);
                ndt.setStepSize(0.7);
                ndt.setMaximumIterations(40);

                ndt.setInputSource(current_init_pc);
                ndt.setInputTarget(nearestFrameCloud);
                pcl::PointCloud<PointType>::Ptr unused_result_ndt(new pcl::PointCloud<PointType>());
                ndt.align(*unused_result_ndt, poseInit.matrix());
                Eigen::Affine3f ndtPose;
                ndtPose = ndt.getFinalTransformation();

                pcl::IterativeClosestPoint<PointType, PointType> icp;
                icp.setMaxCorrespondenceDistance(10); //设置最大对应点的欧式距离，只有对应点之间的距离小于该设置值的对应点才作为ICP计算的点对
                icp.setMaximumIterations(100); //设置最大的迭代次数。迭代停止条件之一
                icp.setTransformationEpsilon(1e-6); //设置前后两次迭代的转换矩阵的最大容差（epsilion），一旦两次迭代小于这个最大容差，则认为已经收敛到最优解，迭代停止。迭代停止条件之二，默认值为：0 
                icp.setEuclideanFitnessEpsilon(1e-6); //设置前后两次迭代的点对的欧式距离均值的最大容差，迭代终止条件之三，默认值为：-std::numeric_limits::max ()
                icp.setInputSource(current_init_pc);
                icp.setInputTarget(nearestFrameCloud);
                pcl::PointCloud<PointType>::Ptr unused_result_icp(new pcl::PointCloud<PointType>());
                icp.align(*unused_result_icp, ndt.getFinalTransformation());

                if(icp.hasConverged())
                {
                    PrintInfo("icp.hasConverged:%f", icp.getFitnessScore());
                }

                Eigen::Matrix4d T_corr_current = icp.getFinalTransformation().cast<double>();
                Eigen::Affine3f icpPose;
                icpPose = icp.getFinalTransformation();
                float xPose, yPose, zPose, RollPose, PitchPose, YawPose;
                pcl::getTranslationAndEulerAngles (icpPose, xPose, yPose, zPose, RollPose, PitchPose, YawPose);

                PrintInfo("init pose, trans: %f,%f,%f", xPose, yPose, zPose);
                PrintInfo("init pose, rpy: %f,%f,%f", RollPose, PitchPose, YawPose);

                Eigen::Matrix4d T_i_l = Eigen::Matrix4d::Identity();
                T_i_l.block<3, 3>(0, 0) = Lidar_R_wrt_IMU;
                T_i_l.block<3, 1>(0, 3) = Lidar_T_wrt_IMU;

                Eigen::Matrix4d T = T_corr_current * T_i_l.inverse();

                init_poses.push_back(T);
                init_ids.push_back(current_init_id);
                init_check++;
                PrintInfo("init_check:%d", init_check);    
            }        
        } 

        Eigen::Vector3d pos_diff = init_poses[0].block<3, 1>(0, 3) - init_poses[1].block<3, 1>(0, 3);

        PrintInfo("pos_diff:%f", pos_diff.norm());
        if (pos_diff.norm() < 2)
        {
            std::unique_lock<std::mutex> lock_state(global_localization_finish_state_mutex);
            // lock_state.lock();
            global_localization_finish = true;
            lock_state.unlock();
            PrintInfo("Global localization successfully");
            init_result.first = init_ids[0];
            init_result.second = init_poses[0];
            std::queue<std::pair<int, PointCloudXYZI::Ptr>> swap_empty;
            std::unique_lock<std::mutex> lock_init_feats(init_feats_down_body_mutex);
            swap(init_feats_down_bodys, swap_empty);
            lock_init_feats.unlock();
        }
        else
        {
            PrintInfo("InitPose Global Localization Failed");
            init_ids.clear();
            init_poses.clear();
        }
        PrintInfo("End InitPose Global Localization");
    }
    else
    {
        PrintInfo("can not find cloest key pose");
        lock_prior_map.unlock();
    }
    // lock_prior_map.unlock();
}

void lioMapping::scReLocalization()
{
    // 初始化检查 两次成功初始化 位置增量小于阈值时通过检查
    int init_check = 0;
    // 重定位结果
    std::vector<int> init_ids;
    std::vector<Eigen::Matrix4d, Eigen::aligned_allocator<Eigen::Matrix4d>> init_poses;
    while (init_check < 2)
    {
        std::unique_lock<std::mutex> lock_init_feats(init_feats_down_body_mutex);
        int N = init_feats_down_bodys.size();
        lock_init_feats.unlock();
        if (N != 0)
        {
            // 获得初始化阶段去畸变后的当前帧点云
            lock_init_feats.lock();
            auto init_pair = init_feats_down_bodys.front();
            init_feats_down_bodys.pop();
            lock_init_feats.unlock();

            int current_init_id = init_pair.first;
            PointCloudXYZI::Ptr current_init_pc_origin = init_pair.second;
            PointCloudXYZI::Ptr current_init_pc(new PointCloudXYZI);
            pcl::copyPointCloud(*current_init_pc_origin, *current_init_pc);

            scManager.makeAndSaveScancontextAndKeys(*current_init_pc);
            // 获得全局定位ID
            int localization_id = scManager.detectLoopClosureID().first;
            float yaw_init = scManager.detectLoopClosureID().second;

            if (localization_id == -1)
            {
                init_check = 0;
                continue;
            }

            Eigen::AngleAxisd yaw(-yaw_init, Eigen::Vector3d(0, 0, 1));
            Eigen::Matrix4d T_init_sc = Eigen::Matrix4d::Identity();
            T_init_sc.block<3, 3>(0, 0) = Eigen::Matrix3d(yaw);
            pcl::transformPointCloud(*current_init_pc, *current_init_pc, T_init_sc);
            PrintInfo("Global match map id = %d", localization_id);
            // 加载匹配地图帧 及 状态
            PointCloudXYZI::Ptr current_loop_pc(new PointCloudXYZI);
            pcl::io::loadPCDFile(mapDirectory + "scan/" + to_string(localization_id) + ".pcd", *current_loop_pc);
            PrintInfo("loadPCDFile:%d", current_loop_pc->size());
            Eigen::Vector3d p = position_map[localization_id];
            Eigen::Quaterniond q = pose_map[localization_id];

            Eigen::Matrix4d T_corr = Eigen::Matrix4d::Identity();

            pcl::IterativeClosestPoint<PointType, PointType> icp;
            icp.setMaxCorrespondenceDistance(5);

            icp.setInputSource(current_init_pc);
            icp.setInputTarget(current_loop_pc);
            pcl::PointCloud<PointType>::Ptr unused(new pcl::PointCloud<PointType>);
            icp.align(*unused);
            Eigen::Matrix4d T_corr_current = icp.getFinalTransformation().cast<double>();
            pcl::transformPointCloud(*current_init_pc, *current_init_pc, T_corr_current);
            T_corr = T_corr_current * T_init_sc;

            icp.setMaxCorrespondenceDistance(1);
            icp.setInputSource(current_init_pc);
            icp.setInputTarget(current_loop_pc);
            icp.align(*unused);
            T_corr_current = icp.getFinalTransformation().cast<double>();
            pcl::transformPointCloud(*current_init_pc, *current_init_pc, T_corr_current);
            T_corr = (T_corr_current * T_corr).eval();

            Eigen::Affine3f icpPose;
            icpPose = icp.getFinalTransformation();
            float xPose, yPose, zPose, RollPose, PitchPose, YawPose;
            pcl::getTranslationAndEulerAngles (icpPose, xPose, yPose, zPose, RollPose, PitchPose, YawPose);
            PrintInfo("init pose, trans: %f,%f,%f", xPose, yPose, zPose);
            PrintInfo("init pose, rpy: %f,%f,%f", RollPose, PitchPose, YawPose);

            cout << T_corr << endl;

            Eigen::Matrix4d T_or = Eigen::Matrix4d::Identity();
            T_or.block<3, 3>(0, 0) = q.toRotationMatrix();
            T_or.block<3, 1>(0, 3) = p;

            Eigen::Matrix4d T_i_l = Eigen::Matrix4d::Identity();
            T_i_l.block<3, 3>(0, 0) = Lidar_R_wrt_IMU;
            T_i_l.block<3, 1>(0, 3) = Lidar_T_wrt_IMU;

            Eigen::Matrix4d T = T_or * T_corr * T_i_l.inverse();

            init_poses.push_back(T);
            init_ids.push_back(current_init_id);
            scManager.dropBackScancontextAndKeys();
            init_check++;
            PrintInfo("init_check:%d", init_check);
        }
    }

    Eigen::Vector3d pos_diff = init_poses[0].block<3, 1>(0, 3) - init_poses[1].block<3, 1>(0, 3);
    if (pos_diff.norm() < 2)
    {
        std::unique_lock<std::mutex> lock_state(global_localization_finish_state_mutex);
        // lock_state.lock();
        global_localization_finish = true;
        lock_state.unlock();
        PrintInfo("Global localization successfully");
        init_result.first = init_ids[0];
        init_result.second = init_poses[0];
        std::queue<std::pair<int, PointCloudXYZI::Ptr>> swap_empty;
        std::unique_lock<std::mutex> lock_init_feats(init_feats_down_body_mutex);
        swap(init_feats_down_bodys, swap_empty);
        lock_init_feats.unlock();
    }
    else
    {
        PrintInfo("SC Global Localization Failed");
        init_ids.clear();
        init_poses.clear();
    }
    PrintInfo("End Global Localization");
}

void lioMapping::globalLocalization()
{
    ros::Rate rate(10);
    while (ros::ok())
    {
        std::unique_lock<std::mutex> lock_state(global_localization_finish_state_mutex);
        bool global_localization_finish_state = global_localization_finish;
        lock_state.unlock();

        if (global_localization_finish_state)
            continue;

        if(reLocalizationMode == 0)
        {
            initPoseReLocalization();
        }
        else if(reLocalizationMode == 1)
        {
            scReLocalization();
        }

        rate.sleep();
    }
}

// 融合imu俯仰角和滚转角
void lioMapping::imuCompensation(state_ikfom &state)
{
    if (!Measures.imu.empty())
    {
        sensor_msgs::Imu lastestImu = *Measures.imu.back();

        static int init_imu_count = 0;
        static Eigen::Vector3d imu_rpy_avg;
        if (init_imu_count < 20)
        {
            if (init_imu_count == 0)
                imu_rpy_avg = quatToRPY(Eigen::Quaterniond(lastestImu.orientation.w, lastestImu.orientation.x, lastestImu.orientation.y, lastestImu.orientation.z));
            else
            {
                Eigen::Vector3d imu_rpy_cur = quatToRPY(Eigen::Quaterniond(lastestImu.orientation.w, lastestImu.orientation.x, lastestImu.orientation.y, lastestImu.orientation.z));
                imu_rpy_avg = init_imu_count * imu_rpy_avg.array() / (init_imu_count + 1) + imu_rpy_cur.array() / (init_imu_count + 1);
            }
            return;
        }
        Eigen::Vector3d imu_rpy = quatToRPY(Eigen::Quaterniond(lastestImu.orientation.w, lastestImu.orientation.x, lastestImu.orientation.y, lastestImu.orientation.z)) - imu_rpy_avg;
        // PrintInfo("imu rpy:%f, %f, %f", imu_rpy(0), imu_rpy(1), imu_rpy(2));
        // 获取状态欧拉角
        Eigen::Vector3d trans_rpy = quatToRPY(Eigen::Quaterniond(state.rot.matrix()));
        // ROS_INFO("roll:%f,   pitch:%f,   yaw:%f",trans_rpy[0],trans_rpy[1],trans_rpy[2]);
        bool compensation_flag = false;
        if ((fabs(imu_rpy[0] - trans_rpy[0]) > angleDiff || fabs(imu_rpy[1] - trans_rpy[1]) > angleDiff) && (fabs(trans_rpy[0]) > angleDiff || fabs(trans_rpy[1]) > angleDiff))
            compensation_flag = true;

        // 俯仰角小于15度
        if (compensation_flag && std::abs(imu_rpy[1]) < 0.2618)
        {
            Eigen::Quaterniond transformQuaternion, imuQuaternion;

            // slerp roll
            transformQuaternion = Eigen::AngleAxisd(0, Eigen::Vector3d::UnitZ()) * Eigen::AngleAxisd(0, Eigen::Vector3d::UnitY()) * Eigen::AngleAxisd(trans_rpy[0], Eigen::Vector3d::UnitX());
            imuQuaternion = Eigen::AngleAxisd(0, Eigen::Vector3d::UnitZ()) * Eigen::AngleAxisd(0, Eigen::Vector3d::UnitY()) * Eigen::AngleAxisd(imu_rpy[0], Eigen::Vector3d::UnitX());
            trans_rpy[0] = quatToRPY(transformQuaternion.slerp(imuRPYWeight[0], imuQuaternion))[0];

            // slerp pitch
            transformQuaternion = Eigen::AngleAxisd(0, Eigen::Vector3d::UnitZ()) * Eigen::AngleAxisd(trans_rpy[1], Eigen::Vector3d::UnitY()) * Eigen::AngleAxisd(0, Eigen::Vector3d::UnitX());
            imuQuaternion = Eigen::AngleAxisd(0, Eigen::Vector3d::UnitZ()) * Eigen::AngleAxisd(imu_rpy[1], Eigen::Vector3d::UnitY()) * Eigen::AngleAxisd(0, Eigen::Vector3d::UnitX());
            trans_rpy[1] = quatToRPY(transformQuaternion.slerp(imuRPYWeight[1], imuQuaternion))[1];

            // ROS_INFO("after--->rollMid:%f,   pitchMid:%f,   yawMid:%f",rollMid,pitchMid,yawMid);
            Eigen::Quaterniond q(Eigen::AngleAxisd(trans_rpy[2], Eigen::Vector3d::UnitZ()) * Eigen::AngleAxisd(trans_rpy[1], Eigen::Vector3d::UnitY()) * Eigen::AngleAxisd(trans_rpy[0], Eigen::Vector3d::UnitX()));

            //  更新状态量
            state.rot = q;
        }
    }
}

//构造优化函数并求解该函数的雅克比矩阵
//构造H矩阵
void lioMapping::h_share_model(state_ikfom &s, esekfom::dyn_share_datastruct<double> &ekfom_data)
{
    double match_start = omp_get_wtime();
    laserCloudOri->clear(); 
    corr_normvect->clear(); 
    total_residual = 0.0; 

    /** closest surface search and residual computation **/
    #ifdef MP_EN
        omp_set_num_threads(MP_PROC_NUM);
        #pragma omp parallel for
    #endif
    //对降采样后的每个点进行残差计算
    for (int i = 0; i < feats_down_size; i++) //判断每个点的对应邻域是否符合平面点的假设
    {
        PointType &point_body  = feats_down_body->points[i];  // lidar系下坐标
        PointType &point_world = feats_down_world->points[i]; // lidar数据点在world系下坐标

        /* transform to world frame */
        V3D p_body(point_body.x, point_body.y, point_body.z);                   // lidar系下坐标
        V3D p_global(s.rot * (s.offset_R_L_I*p_body + s.offset_T_L_I) + s.pos); // w系下坐标
        point_world.x = p_global(0);
        point_world.y = p_global(1);
        point_world.z = p_global(2);
        point_world.intensity = point_body.intensity;

        vector<float> pointSearchSqDis(NUM_MATCH_POINTS);

        auto &points_near = Nearest_Points[i];

        if (ekfom_data.converge)
        {
            /** Find the closest surfaces in the map **/// world系下从ikdtree找5个最近点用于平面拟合
            ikdtree.Nearest_Search(point_world, NUM_MATCH_POINTS, points_near, pointSearchSqDis);
            // 最近点数大于NUM_MATCH_POINTS，且最大距离小于等于5,point_selected_surf设置为true
            point_selected_surf[i] = points_near.size() < NUM_MATCH_POINTS ? false : pointSearchSqDis[NUM_MATCH_POINTS - 1] > 5 ? false : true;
        }

        // 不符合平面特征
        if (!point_selected_surf[i]) continue;

        VF(4) pabcd;
        point_selected_surf[i] = false;//二次筛选平面点
        // 拟合局部平面，返回：是否有内点大于距离阈值
        if (esti_plane(pabcd, points_near, 0.1f))
        {
            float pd2 = pabcd(0) * point_world.x + pabcd(1) * point_world.y + pabcd(2) * point_world.z + pabcd(3);
            float s = 1 - 0.9 * fabs(pd2) / sqrt(p_body.norm()); // 筛选条件 1 - 0.9 * （点到平面距离 / 点到lidar原点距离）

            if (s > 0.9)
            {
                point_selected_surf[i] = true;
                normvec->points[i].x = pabcd(0);
                normvec->points[i].y = pabcd(1);
                normvec->points[i].z = pabcd(2);
                normvec->points[i].intensity = pd2; // intensity记录点到面残差
                res_last[i] = abs(pd2);             // 残差，距离
            }
        }
    }
    
    effct_feat_num = 0; //有效匹配点数

    for (int i = 0; i < feats_down_size; i++)
    {
        if (point_selected_surf[i])
        {
            laserCloudOri->points[effct_feat_num] = feats_down_body->points[i]; // body系 平面特征点
            corr_normvect->points[effct_feat_num] = normvec->points[i];         // world系 平面参数
            total_residual += res_last[i];                                      // 残差和
            effct_feat_num ++;
        }
    }

    if (effct_feat_num < 1)
    {
        ekfom_data.valid = false;
        PrintWarn("No Effective Points! \n");
        return;
    }

    if (effct_feat_num < 20)
    {
        PrintWarn("Effective Points is too less:%d! \n", effct_feat_num);
    }

    res_mean_last = total_residual / effct_feat_num; // 残差均值 （距离）
    match_time  += omp_get_wtime() - match_start;
    double solve_start_  = omp_get_wtime();
    
    /*** Computation of Measuremnt Jacobian matrix H and measurents vector ***/
    ekfom_data.h_x = MatrixXd::Zero(effct_feat_num, 12); //23 //定义H维度
    ekfom_data.h.resize(effct_feat_num);                 // 有效方程个数
    //求观测值与误差的雅克比矩阵，如论文式14以及式12、13

    /*** Computation of Measuremnt Jacobian matrix H and measurements vector ***/
    // if (useWheelVel)
    // {
    //     ekfom_data.h_x = MatrixXd::Zero(effct_feat_num + 3, 15); //23
    //     ekfom_data.h.resize(effct_feat_num + 3);
    // }else
    // {
    //     ekfom_data.h_x = MatrixXd::Zero(effct_feat_num, 12); //23
    //     ekfom_data.h.resize(effct_feat_num);
    // }

    for (int i = 0; i < effct_feat_num; i++)
    {
        const PointType &laser_p  = laserCloudOri->points[i]; // lidar系 平面特征点
        V3D point_this_be(laser_p.x, laser_p.y, laser_p.z);
        M3D point_be_crossmat;
        point_be_crossmat << SKEW_SYM_MATRX(point_this_be);
        V3D point_this = s.offset_R_L_I * point_this_be + s.offset_T_L_I; // 当前状态imu系下 点坐标
        M3D point_crossmat;
        point_crossmat<<SKEW_SYM_MATRX(point_this); // 当前状态imu系下 点坐标反对称矩阵

        /*** get the normal vector of closest surface/corner ***/
        const PointType &norm_p = corr_normvect->points[i];
        V3D norm_vec(norm_p.x, norm_p.y, norm_p.z); // 对应局部法相量, world系下

        /*** calculate the Measuremnt Jacobian matrix H ***/
        V3D C(s.rot.conjugate() *norm_vec); // 将对应局部法相量旋转到imu系下 corr_normal_I
        V3D A(point_crossmat * C);          // 残差对角度求导系数 P(IMU)^ [R(imu <-- w) * normal_w]
        //添加数据到矩阵
        if (extrinsic_est_en)
        {
            V3D B(point_be_crossmat * s.offset_R_L_I.conjugate() * C); //s.rot.conjugate()*norm_vec);
            ekfom_data.h_x.block<1, 12>(i,0) << norm_p.x, norm_p.y, norm_p.z, VEC_FROM_ARRAY(A), VEC_FROM_ARRAY(B), VEC_FROM_ARRAY(C);
        }
        else
        {
            ekfom_data.h_x.block<1, 12>(i,0) << norm_p.x, norm_p.y, norm_p.z, VEC_FROM_ARRAY(A), 0.0, 0.0, 0.0, 0.0, 0.0, 0.0;
        }

        /*** Measuremnt: distance to the closest surface/corner ***/
        ekfom_data.h(i) = -norm_p.intensity;
    }

//     if (useWheelVel && twistAvail)
//     {
//         twistAvail = false;
//         ekfom_data.h_x.block<3, 3>(effct_feat_num,12) = - Wheel_R_wrt_IMU.transpose();
//         M3D angv_crossmat;
//         V3D gyr_vec(p_imu->IMUpose.back().gyr[0], p_imu->IMUpose.back().gyr[1] ,p_imu->IMUpose.back().gyr[2]);
//         angv_crossmat << SKEW_SYM_MATRX(gyr_vec);
//         ROS_WARN_STREAM("gyr " << p_imu->IMUpose.back().gyr[0] << " " << p_imu->IMUpose.back().gyr[1] << " " << p_imu->IMUpose.back().gyr[2]);
//         V3D wheel_v_vec = s.rot*curTwist;
//         ROS_WARN_STREAM("wheel_velocity " << wheel_v_vec.transpose());
//         V3D res = wheel_v_vec - Wheel_R_wrt_IMU.transpose() * (s.vel + angv_crossmat * Wheel_T_wrt_IMU);
//         ROS_WARN_STREAM("s.vel " << s.vel.transpose());
//         ROS_WARN_STREAM("res " << res.transpose());
//         ekfom_data.h(effct_feat_num) = res.x();
//         ekfom_data.h(effct_feat_num + 1) = res.y();
//         ekfom_data.h(effct_feat_num + 2) = res.z();
// //        ekfom_data.R(effct_feat_num,effct_feat_num) = wheel_cov * wheel_cov;
// //        ekfom_data.R(effct_feat_num + 1,effct_feat_num + 1) = nhc_y_cov * nhc_y_cov;
// //        ekfom_data.R(effct_feat_num + 2,effct_feat_num + 2) = nhc_z_cov * nhc_z_cov;
//     }

    solve_time += omp_get_wtime() - solve_start_;
}

// 根据激光雷达视场角分割场景图
void lioMapping::lasermap_fov_segment()
{
    cub_needrm.clear(); // 清空需要移除的区域
    kdtree_delete_counter = 0;
    kdtree_delete_time = 0.0;    
    pointBodyToWorld(XAxisPoint_body, XAxisPoint_world); // 机体坐标系转世界坐标系
    V3D pos_LiD = pos_lid;
    //初始化局部地图包围盒角点，以为w系下lidar位置为中心
    if (!Localmap_Initialized){
        for (int i = 0; i < 3; i++){
            LocalMap_Points.vertex_min[i] = pos_LiD(i) - cube_len / 2.0;
            LocalMap_Points.vertex_max[i] = pos_LiD(i) + cube_len / 2.0;
        }
        Localmap_Initialized = true;
        return;
    }
    float dist_to_map_edge[3][2];
    bool need_move = false;
    //各个方向与局部地图边界的距离
    for (int i = 0; i < 3; i++){
        dist_to_map_edge[i][0] = fabs(pos_LiD(i) - LocalMap_Points.vertex_min[i]);
        dist_to_map_edge[i][1] = fabs(pos_LiD(i) - LocalMap_Points.vertex_max[i]);
        //与某个方向上的边界距离太小，标记需要移除need_move
        if (dist_to_map_edge[i][0] <= MOV_THRESHOLD * DET_RANGE || dist_to_map_edge[i][1] <= MOV_THRESHOLD * DET_RANGE) need_move = true;
    }
    if (!need_move) return;
    BoxPointType New_LocalMap_Points, tmp_boxpoints;
    // 新的局部地图角点
    New_LocalMap_Points = LocalMap_Points;
    float mov_dist = max((cube_len - 2.0 * MOV_THRESHOLD * DET_RANGE) * 0.5 * 0.9, double(DET_RANGE * (MOV_THRESHOLD -1)));
    for (int i = 0; i < 3; i++){
        tmp_boxpoints = LocalMap_Points;
        //与包围盒最小值角点距离
        if (dist_to_map_edge[i][0] <= MOV_THRESHOLD * DET_RANGE){
            New_LocalMap_Points.vertex_max[i] -= mov_dist;
            New_LocalMap_Points.vertex_min[i] -= mov_dist;
            tmp_boxpoints.vertex_min[i] = LocalMap_Points.vertex_max[i] - mov_dist;
            cub_needrm.push_back(tmp_boxpoints);// 移除较远包围盒
        } else if (dist_to_map_edge[i][1] <= MOV_THRESHOLD * DET_RANGE){
            New_LocalMap_Points.vertex_max[i] += mov_dist;
            New_LocalMap_Points.vertex_min[i] += mov_dist;
            tmp_boxpoints.vertex_max[i] = LocalMap_Points.vertex_min[i] + mov_dist;
            cub_needrm.push_back(tmp_boxpoints);
        }
    }
    LocalMap_Points = New_LocalMap_Points;

    // points_cache_collect();
    PointVector points_history;
    ikdtree.acquire_removed_points(points_history);

    double delete_begin = omp_get_wtime();
    if(cub_needrm.size() > 0) kdtree_delete_counter = ikdtree.Delete_Point_Boxes(cub_needrm);
    kdtree_delete_time = omp_get_wtime() - delete_begin;
}

void lioMapping::map_incremental()
{
    // if(isLocalization)
    //     return;
    std::unique_lock<std::mutex> lock_prior_map(prior_map_mutex);
    if (isLocalization && global_localization_finish)
    {
        PointType searchPoint;
        searchPoint.x = state_point.pos(0);
        searchPoint.y = state_point.pos(1);
        searchPoint.z = state_point.pos(2);
        std::vector<int> pointIdx;
        std::vector<float> pointDistance;
        //先验地图5米内不用新增地图
        if (priorMapKeyPoseKdtree->radiusSearch(searchPoint, 5, pointIdx, pointDistance) > 0) 
        {
            lock_prior_map.unlock();
            return;
        }  
    }
    lock_prior_map.unlock();

    PointVector PointToAdd;
    PointVector PointNoNeedDownsample;
    PointToAdd.reserve(feats_down_size);
    PointNoNeedDownsample.reserve(feats_down_size);
    for (int i = 0; i < feats_down_size; i++)
    {
        /* transform to world frame */
        pointBodyToWorld(&(feats_down_body->points[i]), &(feats_down_world->points[i]));
        /* decide if need add to map */
        if (!Nearest_Points[i].empty() && flg_EKF_inited)
        {
            const PointVector &points_near = Nearest_Points[i];
            bool need_add = true;
            BoxPointType Box_of_Point;
            PointType downsample_result, mid_point; 
            mid_point.x = floor(feats_down_world->points[i].x/filter_size_map_min)*filter_size_map_min + 0.5 * filter_size_map_min;
            mid_point.y = floor(feats_down_world->points[i].y/filter_size_map_min)*filter_size_map_min + 0.5 * filter_size_map_min;
            mid_point.z = floor(feats_down_world->points[i].z/filter_size_map_min)*filter_size_map_min + 0.5 * filter_size_map_min;
            float dist  = calc_dist(feats_down_world->points[i],mid_point);
            if (fabs(points_near[0].x - mid_point.x) > 0.5 * filter_size_map_min && fabs(points_near[0].y - mid_point.y) > 0.5 * filter_size_map_min && fabs(points_near[0].z - mid_point.z) > 0.5 * filter_size_map_min){
                PointNoNeedDownsample.push_back(feats_down_world->points[i]);
                continue;
            }
            for (int readd_i = 0; readd_i < NUM_MATCH_POINTS; readd_i ++)
            {
                if (points_near.size() < NUM_MATCH_POINTS) break;
                if (calc_dist(points_near[readd_i], mid_point) < dist)
                {
                    need_add = false;
                    break;
                }
            }
            if (need_add) PointToAdd.push_back(feats_down_world->points[i]);
        }
        else
        {
            PointToAdd.push_back(feats_down_world->points[i]);
        }
    }

    double st_time = omp_get_wtime();
    add_point_size = ikdtree.Add_Points(PointToAdd, true);
    ikdtree.Add_Points(PointNoNeedDownsample, false); 
    add_point_size = PointToAdd.size() + PointNoNeedDownsample.size();
    kdtree_incremental_time = omp_get_wtime() - st_time;
}

// livox_ros_driver::CustomMsg点云数据处理回调
#ifdef ENABLE_LIVOX
void lioMapping::livox_pcl_cbk(const livox_ros_driver::CustomMsg::ConstPtr &msg) 
{
    mtx_buffer.lock();
    double preprocess_start_time = omp_get_wtime();
    scan_count ++;
    // 检查lidar时间戳，不能小于-1，也不能小于后一帧点云的时间
    if (msg->header.stamp.toSec() < last_timestamp_lidar)
    {
        PrintError("lidar loop back, clear buffer");
        lidar_buffer.clear();
    }
    last_timestamp_lidar = msg->header.stamp.toSec();
    
    if (!time_sync_en && abs(last_timestamp_imu - last_timestamp_lidar) > 10.0 && !imu_buffer.empty() && !lidar_buffer.empty() )
    {
        printf("IMU and LiDAR not Synced, IMU time: %lf, lidar header time: %lf \n",last_timestamp_imu, last_timestamp_lidar);
    }

    if (time_sync_en && !timediff_set_flg && abs(last_timestamp_lidar - last_timestamp_imu) > 1 && !imu_buffer.empty())
    {
        timediff_set_flg = true;
        timediff_lidar_wrt_imu = last_timestamp_lidar + 0.1 - last_timestamp_imu;
        printf("Self sync IMU and LiDAR, time diff is %.10lf \n", timediff_lidar_wrt_imu);
    }

    PointCloudXYZI::Ptr  ptr(new PointCloudXYZI());
    p_pre->process(msg, ptr);
    lidar_buffer.push_back(ptr);
    time_buffer.push_back(last_timestamp_lidar);
    
    s_plot11[scan_count] = omp_get_wtime() - preprocess_start_time;
    mtx_buffer.unlock();
    sig_buffer.notify_all();
}
#endif

void lioMapping::standard_pcl_cbk(const sensor_msgs::PointCloud2::ConstPtr &msg) 
{
    mtx_buffer.lock();
    scan_count ++;
    double preprocess_start_time = omp_get_wtime();
    if (msg->header.stamp.toSec() < last_timestamp_lidar)
    {
        PrintError("lidar loop back, clear buffer");
        lidar_buffer.clear();
    }

    if (msg->header.stamp.toSec() - last_timestamp_lidar > 0.2)
    {
        PrintWarn("lidar time diff is too large:%.2f", msg->header.stamp.toSec() - last_timestamp_lidar);
    }

    //滤出目标人物的点云
    // if(bTargetReceived)
    // {
    //     double timeTarget = targetPointCur.header.stamp.toSec();
    //     double timeCloud  = msg->header.stamp.toSec();
    //     if(fabs(timeTarget - timeCloud) <= 1.0)
    //     {
    //         p_pre->target_point = targetPointCur;
    //         p_pre->target_avail = true;
    //     }
    // }

    PointCloudXYZI::Ptr ptr(new PointCloudXYZI());
    p_pre->process(msg, ptr);

    if(ptr->size() <= 3000)
    {
        PrintWarn("lidar point is too less:%d!!!", ptr->size());
    }

    if(ptr->size() <= 0)
    {
        PrintError("lidar point is empty!!!");
    }
    else
    {
        lidar_buffer.push_back(ptr);

        if(p_pre->lidar_type == LS16 || p_pre->lidar_type == RS32 || p_pre->lidar_type == RS32_DENSE) // 获得起点时间
            time_buffer.push_back(msg->header.stamp.toSec()- ptr->points.back().curvature/1000.0);
        else
            time_buffer.push_back(msg->header.stamp.toSec());
    }

    
    #if 0
    if (cut_frame_num > 0) { // 点云分割
        deque<PointCloudXYZI::Ptr> ptr;
        deque<double> timestamp_lidar;
        p_pre->process_cut_frame_pcl2(msg, ptr, timestamp_lidar, cut_frame_num, scan_count);
        while (!ptr.empty() && !timestamp_lidar.empty()) { // 将分割点云填充到lidar_buffer和time_buffer
            lidar_buffer.push_back(ptr.front());
            ptr.pop_front();
            time_buffer.push_back(timestamp_lidar.front() / double(1000));//unit:s
            timestamp_lidar.pop_front();
        }
    }
    else{
        PointCloudXYZI::Ptr ptr(new PointCloudXYZI());
        p_pre->process(msg, ptr);

        // 高频lidar点云前几帧进行叠加处理，避免初始化抖动
        if(cut_frame_num < 0 && cnt < 9){
            *cloudPtr += *ptr;
            cnt ++;
            if(cnt%3 == 0){
                lidar_buffer.push_back(cloudPtr);
                cloudPtr.reset(new PointCloudXYZI());
            }
        }
        else{
            lidar_buffer.push_back(ptr);
        }

        if(p_pre->lidar_type == LS16 || p_pre->lidar_type == RS32 || p_pre->lidar_type == RS32_DENSE) // 获得起点时间
            time_buffer.push_back(msg->header.stamp.toSec()- ptr->points.back().curvature/1000.0);
        else
            time_buffer.push_back(msg->header.stamp.toSec());
    }
    #endif
    
    while(lidar_buffer.size() > 4000)
    {
        lidar_buffer.pop_front();
        time_buffer.pop_front();
    }

    last_timestamp_lidar = msg->header.stamp.toSec();
    s_plot11[scan_count] = omp_get_wtime() - preprocess_start_time;
    mtx_buffer.unlock();
    sig_buffer.notify_all();
}

void lioMapping::imu_cbk(const sensor_msgs::Imu::ConstPtr &msg_in) 
{
    publish_count ++;
    // cout<<"IMU got at: "<<msg_in->header.stamp.toSec()<<endl;
    sensor_msgs::Imu::Ptr msg(new sensor_msgs::Imu(*msg_in));

    // msg->header.stamp = ros::Time::now();

    if (abs(timediff_lidar_wrt_imu) > 0.1 && time_sync_en)
    {
        msg->header.stamp = ros::Time().fromSec(timediff_lidar_wrt_imu + msg_in->header.stamp.toSec());
    }

    double timestamp = msg->header.stamp.toSec();

    mtx_buffer.lock();
    // 检查IMU时间戳，当前帧时间大于上一帧时间
    if (timestamp < last_timestamp_imu)
    {
        PrintWarn("imu loop back, clear buffer");
        imu_buffer.clear();
    }

    // 检查IMU时间戳，当前帧时间和上一帧时间間隔
    // if (timestamp - last_timestamp_imu <= 0.001)
    // {
    //     PrintWarn("imu time diff is too small:%.5f", timestamp - last_timestamp_imu);
    // }
    // if (timestamp - last_timestamp_imu >= 0.010)
    // {
    //     PrintWarn("imu time diff is too big:%.5f", timestamp - last_timestamp_imu);
    // }

    last_timestamp_imu = timestamp;

    imu_buffer.push_back(msg);

    while(imu_buffer.size() > 4000)
    {
        imu_buffer.pop_front();
    }
    mtx_buffer.unlock();
    sig_buffer.notify_all();
}

void lioMapping::wheel_cbk(const geometry_msgs::Twist::ConstPtr &msg_vel)
{
    // if(fabs(msg_vel->header.stamp.toSec() - ros::Time().now().toSec()) < 0.1)
    {
        curTwist(0) = msg_vel->linear.x;
        curTwist(1) = msg_vel->linear.y;
        curTwist(2) = msg_vel->linear.z;
        twistAvail = true;
    }
}

void lioMapping::mode_cbk(const std_msgs::UInt8::ConstPtr& modeMsg)
{
    if(modeMsg->data == 1 || modeMsg->data == 2)
    {
        useWheelVel = true;
    }
    else
    {
        useWheelVel = useWheelVelParam;
    }
    // PrintInfo("loopMode :%d", loopMode);
}

void lioMapping::target_cbk(const geometry_msgs::PoseStampedConstPtr& targetMsg)
{
    static int cnt = 0;
    if(targetMsg->pose.orientation.w == 0)
    {
        cnt ++;
        if(cnt > 5)
        {
            bTargetReceived = false;
            return;
        }
    }
    else
    {
        cnt = 0;
    }

    targetPointCur.header.frame_id = "base_link";
    targetPointCur.header.stamp = targetMsg->header.stamp;

    targetPointCur.point.x = targetMsg->pose.position.x;
    targetPointCur.point.y = targetMsg->pose.position.y;
    targetPointCur.point.z = targetMsg->pose.position.z;

    bTargetReceived = true;
}

// 时间同步
bool lioMapping::sync_packages(MeasureGroup &meas)
{
    if (lidar_buffer.empty()) {
        // PrintWarn("lidar_buffer.empty()!\n");
        return false;
    }
    if(imu_buffer.empty())
    {
        PrintWarn("imu_buffer.empty()!\n");
        return false;        
    }

    /*** push a lidar scan ***/
    if(!lidar_pushed)
    {
        meas.lidar = lidar_buffer.front();
        meas.lidar_beg_time = time_buffer.front();
        if (meas.lidar->points.size() <= 1) // time too little
        {
            lidar_end_time = meas.lidar_beg_time + lidar_mean_scantime;
            PrintWarn("Too few input point cloud!\n");
        }
        else if (meas.lidar->points.back().curvature / double(1000) < 0.5 * lidar_mean_scantime)
        {
            lidar_end_time = meas.lidar_beg_time + lidar_mean_scantime;
        }
        else
        {
            scan_num ++;
            lidar_end_time = meas.lidar_beg_time + meas.lidar->points.back().curvature / double(1000);
            lidar_mean_scantime += (meas.lidar->points.back().curvature / double(1000) - lidar_mean_scantime) / scan_num;
        }

        meas.lidar_end_time = lidar_end_time;

        lidar_pushed = true;
    }

    // IMU的时间得包住Lidar，即最后一个点的时间得比IMU的时间小，所以IMU的频率必须比lidar的频率要高
    if (last_timestamp_imu < lidar_end_time)
    {
        // PrintWarn("last_timestamp_imu < lidar_end_time!\n");
        return false;
    }

    /*** push imu data, and pop from imu buffer ***/
    double imu_time = imu_buffer.front()->header.stamp.toSec();
    meas.imu.clear();
    while ((!imu_buffer.empty()) && (imu_time < lidar_end_time)) // imu的时间必须比lidar的时间大或相等
    {
        imu_time = imu_buffer.front()->header.stamp.toSec();
        if(imu_time > lidar_end_time) break; // IMU频率得大于50HZ
        meas.imu.push_back(imu_buffer.front());
        imu_buffer.pop_front();
    }

    lidar_buffer.pop_front();
    time_buffer.pop_front();
    lidar_pushed = false;
    return true;
}

// 获取里程计和当前帧点云
bool lioMapping::getOdomAndFrame(Eigen::Isometry3d& transformTobeMapped, PointCloudXYZI::Ptr curFrame, double& frameEndTime)
{
    if(isLocalization && !global_localization_finish)
        return false;

    if(isLocalization)
    {
        static int init_cnt = 0;
        if(global_localization_finish)
        {
            if(init_cnt < 20)
            {
                init_cnt ++;
                return false;
            }
        }
        else
        {
            init_cnt = 0;
        }
    }
    
    state_ikfom cur_state = kf.get_x();
    transformTobeMapped.setIdentity();
    transformTobeMapped.rotate(cur_state.rot.matrix());
    transformTobeMapped.pretranslate(cur_state.pos);

    pcl::copyPointCloud(*feats_undistort, *curFrame);

    frameEndTime = lidar_end_time;
    return true;
}

void lioMapping::updateState(const Eigen::Isometry3d& poseOp)
{
    state_ikfom state_updated = kf.get_x(); //  获取cur_pose (还没修正)
    state_updated.pos = poseOp.translation();
    state_updated.rot = poseOp.rotation().matrix();
    // PrintInfo("update state----------");

    // 更新状态量
    // state_point = state_updated; // 对state_point进行更新，state_point可视化用到
    // if(aLoopIsClosed == true )
    kf.change_x(state_updated);  //  对cur_pose 进行isam2优化后的修正    
    // TODO:  P的修正有待考察，按照yanliangwang的做法，修改了p，会跑飞
    // esekfom::esekf<state_ikfom, 12, input_ikfom>::cov P_updated = kf.get_P(); // 获取当前的状态估计的协方差矩阵
    // P_updated.setIdentity();
    // P_updated(6, 6) = P_updated(7, 7) = P_updated(8, 8) = 0.00001;
    // P_updated(9, 9) = P_updated(10, 10) = P_updated(11, 11) = 0.00001;
    // P_updated(15, 15) = P_updated(16, 16) = P_updated(17, 17) = 0.0001;
    // P_updated(18, 18) = P_updated(19, 19) = P_updated(20, 20) = 0.001;
    // P_updated(21, 21) = P_updated(22, 22) = 0.00001;
    // kf.change_P(P_updated);
}

void lioMapping::updateMap(const PointCloudXYZI::Ptr mapKeyFramesDS)
{
    PrintInfo("Reconstructing  ikdtree ");
    ikdtree.reconstruct(mapKeyFramesDS->points);
        // updateKdtreeCount = 0;
    PrintInfo("Reconstruct  ikdtree success");
        // int featsFromMapNum = ikdtree.validnum();
        // kdtree_size_st = ikdtree.size();  
    // esekfom::esekf<state_ikfom, 12, input_ikfom>::cov P_updated = kf.get_P(); // 获取当前的状态估计的协方差矩阵
    // P_updated.setIdentity();
    // P_updated(6, 6) = P_updated(7, 7) = P_updated(8, 8) = 0.00001;
    // P_updated(9, 9) = P_updated(10, 10) = P_updated(11, 11) = 0.00001;
    // P_updated(15, 15) = P_updated(16, 16) = P_updated(17, 17) = 0.0001;
    // P_updated(18, 18) = P_updated(19, 19) = P_updated(20, 20) = 0.001;
    // P_updated(21, 21) = P_updated(22, 22) = 0.00001;
    // kf.change_P(P_updated);
}

bool lioMapping::mainRun()
{ 
    if(sync_packages(Measures)) 
    {
        // 检查是否需要更新全局定位
        if(isLocalization)
        {
            std::unique_lock<std::mutex> lock_state(global_localization_finish_state_mutex);
            if (global_localization_finish && !global_update)
            {
                PrintInfo("global_localization_finish");
                int init_id = init_result.first;
                Eigen::Vector3d init_time_p = position_init[init_id];
                Eigen::Quaterniond init_time_q = pose_init[init_id];

                PrintInfo("init_time_p:%f,%f,%f", init_time_p(0), init_time_p(1), init_time_p(2));

                Eigen::Matrix4d T_odom_init_time = Eigen::Matrix4d::Identity();
                T_odom_init_time.block<3, 3>(0, 0) = init_time_q.toRotationMatrix();
                T_odom_init_time.block<3, 1>(0, 3) = init_time_p;

                Eigen::Matrix4d T_odom_current = Eigen::Matrix4d::Identity();
                T_odom_current.block<3, 3>(0, 0) = state_point.rot.toRotationMatrix();
                T_odom_current.block<3, 1>(0, 3) = state_point.pos;

                PrintInfo("state_point.pos:%f,%f,%f", state_point.pos(0), state_point.pos(1), state_point.pos(2));

                Eigen::Matrix4d T_map_init_time = init_result.second;

                Eigen::Matrix4d T_map_current = T_map_init_time * T_odom_init_time.inverse() * T_odom_current;

                state_ikfom global_state = state_point;
                global_state.pos = T_map_current.block<3, 1>(0, 3);
                global_state.rot = T_map_current.block<3, 3>(0, 0);
                kf.change_x(global_state);
                state_point = kf.get_x();
                double t_build_kdtree_start = omp_get_wtime();
                // ikdtree = std::move(ikdtree_global);
                ikdtree.reconstruct(priorMapKdtree->points);
                double t_build_kdtree_end = omp_get_wtime();
                PrintInfo("build kdtree time:%f", t_build_kdtree_end-t_build_kdtree_start);
                global_update = true;
            }
            lock_state.unlock();
        }

        if (flg_first_scan)
        {
            first_lidar_time = Measures.lidar_beg_time;
            p_imu->first_lidar_time = first_lidar_time;
            flg_first_scan = false;
            return false;
        }

        match_time = 0;
        kdtree_search_time = 0.0;
        solve_time = 0;
        solve_const_H_time = 0;
        svd_time   = 0;
        t0 = omp_get_wtime();

        // 根据imu数据序列和lidar数据，向前传播纠正点云的畸变, 此前已经完成间隔采样或特征提取
        // feats_undistort 为畸变纠正之后的点云,lidar系
        // if(twistAvail)
        // {
        //     twistAvail = false;
        //     p_imu->cur_vel = curTwist;
        //     p_imu->vel_avail = true;
        // }

        p_imu->Process(Measures, kf, feats_undistort);                          // IMU预处理，其中包含了点云畸变处理
        state_point = kf.get_x();                                               // 前向传播后body的状态预测值
        pos_lid = state_point.pos + state_point.rot * state_point.offset_T_L_I; // global系 lidar位置

        if (feats_undistort->empty() || (feats_undistort == NULL))
        {
            PrintWarn("No point, skip this scan!\n");
            return false;
        }

        // if(isLocalization && !isPriorPoseInit)
        // {
        //     PrintInfo("init pose");
        //     // Eigen::Isometry3d poseInit = Eigen::Isometry3d::Identity();
        //     initPriorPose(feats_undistort);
        //     return false;
        // }

        // if (feats_undistort->empty() || (feats_undistort == NULL))
        // {
        //     PrintWarn("No point, skip this scan!\n");
        //     return false;
        // }
        // if(!Measures.imu.empty()){
        //     sensor_msgs::Imu lastestImu = *Measures.imu.back();
        //     // Eigen::Vector3d imu_rpy = Eigen::Quaterniond(lastestImu.orientation.w,lastestImu.orientation.x,lastestImu.orientation.y,lastestImu.orientation.z).toRotationMatrix().eulerAngles(2,1,0);
        //     Eigen::Vector3d imu_rpy = quatToRPY(Eigen::Quaterniond(lastestImu.orientation.w,lastestImu.orientation.x,lastestImu.orientation.y,lastestImu.orientation.z));
        //     // PrintInfo("imu rpy:%f, %f, %f", imu_rpy(0), imu_rpy(1), imu_rpy(2));
        // }

        // 检查当前lidar数据时间，与最早lidar数据时间是否足够
        flg_EKF_inited = (Measures.lidar_beg_time - first_lidar_time) < INIT_TIME ? false : true;
        /*** Segment the map in lidar FOV ***/// 根据lidar在W系下的位置，重新确定局部地图的包围盒角点，移除远端的点
        lasermap_fov_segment();

        /*** downsample the feature points in a scan ***/
        downSizeFilterSurf.setInputCloud(feats_undistort);
        downSizeFilterSurf.filter(*feats_down_body);
        t1 = omp_get_wtime();
        feats_down_size = feats_down_body->points.size();
        if(feats_down_size < 1000)
        {
            filter_size_map_min = 0.2;
        }
        else
        {
            filter_size_map_min = 0.5;
        }
        /*** initialize the map kdtree ***/
        if(ikdtree.Root_Node == nullptr)
        {
            if(feats_down_size > 5)
            {
                ikdtree.set_downsample_param(filter_size_map_min);
                feats_down_world->resize(feats_down_size);
                for(int i = 0; i < feats_down_size; i++)
                {
                    pointBodyToWorld(&(feats_down_body->points[i]), &(feats_down_world->points[i]));
                }
                ikdtree.Build(feats_down_world->points); // world系下对当前帧降采样后的点云，初始化ikd-tree
            }
            return false;
        }
        int featsFromMapNum = ikdtree.validnum();
        kdtree_size_st = ikdtree.size();
        
        // cout<<"[ mapping ]: In num: "<<feats_undistort->points.size()<<" downsamp "<<feats_down_size<<" Map num: "<<featsFromMapNum<<"effect num:"<<effct_feat_num<<endl;

        /*** ICP and iterated Kalman filter update ***/
        if (feats_down_size < 5)
        {
            PrintWarn("No point, skip this scan!\n");
            return false;
        }
        
        normvec->resize(feats_down_size);
        feats_down_world->resize(feats_down_size);

        V3D ext_euler = SO3ToEuler(state_point.rot);
        // fout_pre << setw(20) << Measures.lidar_beg_time - first_lidar_time << " " << ext_euler.transpose() << " " << state_point.pos.transpose()<< " " << ext_euler.transpose() << " "<<state_point.offset_T_L_I.transpose()<<" "<< state_point.vel.transpose() \
        // <<" "<<state_point.bg.transpose()<<" "<<state_point.ba.transpose()<<" "<<state_point.grav<<" "<<feats_undistort->points.size()<<endl;

        // fout_pre << setw(20) << Measures.lidar_beg_time - first_lidar_time << " " << euler_cur.transpose() << " " << state_point.pos.transpose() <<" "<< state_point.vel.transpose() << std::endl;

        if(0) // If you need to see map point, change to "if(1)"
        {
            PointVector ().swap(ikdtree.PCL_Storage);
            ikdtree.flatten(ikdtree.Root_Node, ikdtree.PCL_Storage, NOT_RECORD);
            featsFromMap->clear();
            featsFromMap->points = ikdtree.PCL_Storage;
        }

        pointSearchInd_surf.resize(feats_down_size);
        Nearest_Points.resize(feats_down_size);
        int  rematch_num = 0;
        bool nearest_search_en = true; //

        t2 = omp_get_wtime();
        
        /*** iterated state estimation ***/
        state_ikfom state_point_imu = state_point;
        double t_update_start = omp_get_wtime();
        double solve_H_time = 0;
        kf.update_iterated_dyn_share_modified(LASER_POINT_COV, solve_H_time);
        // if (useWheelVel)
        // {
        //     kf.update_iterated_dyn_share_modified_wheel(LASER_POINT_COV, solve_H_time);
        // }
        // else
        // {
        //     kf.update_iterated_dyn_share_modified(LASER_POINT_COV, solve_H_time);
        // }

        if (useWheelVel && twistAvail)
        {
            twistAvail = false;
            try
            {
                auto cur_state = kf.get_x();
                auto kf_cov_ = kf.get_P();
                Eigen::Matrix<double, 3, 23> H_WHEEL = Eigen::Matrix<double, 3, 23>::Zero();
                H_WHEEL.template block<3, 3>(0, 12) = Eigen::Matrix<double, 3, 3>::Identity(); //p rot R_l_i T_l_i vel bg ba g
                Eigen::Matrix<double, 3, 3> odom_noise_ = Eigen::Matrix<double, 3, 3>::Zero();
                odom_noise_.diagonal() << 0.1, 0.1, 0.1;
                // if (flag_degen_)
                //     if (slip_cov_[1] < 1)
                //         odom_noise_.diagonal() << cov_wheel_, cov_wheel_, cov_wheel_;
                //     else
                //         odom_noise_.diagonal() << 3.0 * cov_wheel_, 3.0 * cov_wheel_, 3.0 * cov_wheel_;
                // else
                //     odom_noise_.diagonal() << 10.0 * cov_wheel_, 10.0 * cov_wheel_, 10.0 * cov_wheel_;
                Eigen::Matrix<double, 23, 3> K_WHEEL = kf_cov_ * H_WHEEL.transpose() * (H_WHEEL * kf_cov_ * H_WHEEL.transpose() + odom_noise_).inverse();
                Eigen::Matrix<double, 3, 1> vel_odom(curTwist(0), 0.0 , 0.0); // + pow(angvel_last[3] * 0.8164, 2)
                Eigen::Matrix<double, 3, 1> vel_world = cur_state.rot * vel_odom;
                Eigen::Matrix<double, 23, 1> dx_ = H_WHEEL.transpose() * (vel_world - cur_state.vel);
                //std::cerr << "dx_" << dx_ << std::endl;
                // kf_cov_ = (Eigen::Matrix<double, 23, 23>::Identity() - K_WHEEL * H_WHEEL) * kf_cov_;
                // update cov
                // kf.change_P(kf_cov_);
                cur_state.pos += dx_.template block<3, 1>(0, 0);
                cur_state.vel += dx_.template block<3, 1>(12, 0);
                kf.change_x(cur_state);
                //R_ = R_ * SO3::exp(dx_.template block<3, 1>(6, 0));
                //Eigen::Matrix<double, 23, 23> J = Eigen::Matrix<double, 23, 23>::Identity();
                //J.template block<3, 3>(6, 6) = Eigen::Matrix<double, 3, 3>::Identity() - 0.5 * SO3::hat(dx_.template block<3, 1>(6, 0));
                //kf.change_P(J * kf.get_P() * J.transpose());
            }
            catch (exception &e) 
            {
                PrintError("Standard exception");
                // std::cerr << "Standard exception: " << e.what() << std::endl;
            }
        }

        state_point = kf.get_x();

        // if(pubTwist.getNumSubscribers() > 0)
        // {
        //     // PrintWarn("map state vel: %f, %f, %f !!!", state_point.vel(0), state_point.vel(1), state_point.vel(2));
        //     Eigen::Vector3d curVel(state_point.vel(0), state_point.vel(1), state_point.vel(2));
        //     Eigen::Isometry3d curPose3d = Eigen::Isometry3d::Identity();
        //     Eigen::Isometry3d curPoseRotate = Eigen::Isometry3d::Identity();
        //     curPoseRotate.rotate(state_point.rot.matrix());
        //     curVel = curPoseRotate.inverse() * curVel;
        //     // PrintWarn("lidar state vel: %f, %f, %f !!!", curVel(0), curVel(1), curVel(2));
        //     curPose3d.rotate(state_point.rot.matrix());
        //     curPose3d.pretranslate(state_point.pos);
        //     Eigen::Isometry3d diffPose3d = lastPose3d.inverse() * curPose3d;
        //     Eigen::Vector3d transDiff = diffPose3d.translation();
        //     Eigen::Vector3d rpyDiff = quatToRPY(Eigen::Quaterniond(diffPose3d.rotation()));
        //     Eigen::Vector3d curVelTrans = transDiff/(lidar_end_time-last_lidar_end_time);
        //     Eigen::Vector3d curVelRPY = rpyDiff/(lidar_end_time-last_lidar_end_time);
        //     // PrintWarn("lidar pose vel: %f, %f, %f !!!", curVelTrans(0), curVelTrans(1), curVelTrans(2));
        //     // PrintWarn("lidar rpy vel: %f, %f, %f !!!", curVelRPY(0), curVelRPY(1), curVelRPY(2));
        //     last_lidar_end_time = lidar_end_time;
        //     lastPose3d = curPose3d;
        //     geometry_msgs::Twist curSlamTwist;
        //     curSlamTwist.linear.x = curVel(0);
        //     curSlamTwist.linear.y = curVel(1);
        //     curSlamTwist.linear.z = curVel(2);
        //     pubTwist.publish(curSlamTwist);
        // }

        V3D euler_cur = SO3ToEuler(state_point.rot);
        // fout_out << setw(20) << Measures.lidar_beg_time - first_lidar_time << " " << euler_cur.transpose() << " " << state_point.pos.transpose() <<" "<< state_point.vel.transpose() << std::endl;

        // PrintInfo("vel: %f %f %f ", state_point.vel(0), state_point.vel(1), state_point.vel(2)); 
        if(fabs(state_point.vel(0)) > 3.0 || fabs(state_point.vel(1)) > 3.0 || fabs(state_point.vel(2)) > 3.0)
        {
            PrintError("imu state vel is too large: %f, %f, %f !!!", state_point_imu.vel(0), state_point_imu.vel(1), state_point_imu.vel(2));
            PrintError("vel is too large: %f, %f, %f !!!", state_point.vel(0), state_point.vel(1), state_point.vel(2));
            // kf.change_x(state_point_imu);
            // return false;
        }

        // IMU补偿
        if(imuRPYEnable){
            imuCompensation(state_point);
            kf.change_x(state_point);
        }

        euler_cur = SO3ToEuler(state_point.rot);
        pos_lid = state_point.pos + state_point.rot * state_point.offset_T_L_I;
        geoQuat.x = state_point.rot.coeffs()[0];
        geoQuat.y = state_point.rot.coeffs()[1];
        geoQuat.z = state_point.rot.coeffs()[2];
        geoQuat.w = state_point.rot.coeffs()[3];

        if(!lidarRpyInit)
        {
            Eigen::Quaterniond qLidarInit(geoQuat.w, geoQuat.x,geoQuat.y,geoQuat.z);
            Eigen::Vector3d lidar_rpy = quatToRPY(qLidarInit);
            PrintInfo("lidar rpy:%f, %f, %f", lidar_rpy(0), lidar_rpy(1), lidar_rpy(2));
            // Eigen::Isometry3d rotateLidarToBody = Eigen::Isometry3d::Identity();
            rotateLidarToBody.rotate(qLidarInit);
            rotateLidarToBody.pretranslate(Eigen::Vector3d(0.0, 0.0, 0.0));
            lidarRpyInit = true;
        }

        // for init
        if (isLocalization && !global_localization_finish)
        {
            position_init.push_back(state_point.pos);
            pose_init.push_back(state_point.rot);
            std::unique_lock<std::mutex> lock_init_feats(init_feats_down_body_mutex);
            init_feats_down_bodys.push(std::make_pair(init_count, feats_undistort));
            lock_init_feats.unlock();
            init_count++;
        }

        double t_update_end = omp_get_wtime();

        // PrintInfo("odom rpy:%f, %f", euler_cur(0)/57.3, euler_cur(1)/57.3);

        // optimization

        /*** add the feature points to map kdtree ***/
        // map_incremental();

        return true;
        
    }
    return false;
}

lioMapping::lioMapping(){}
// lioMapping::~lioMapping(){}

void lioMapping::mainShutDown()
{
    /**************** save map ****************/
    /* 1. make sure you have enough memories
    /* 2. pcd save will largely influence the real-time performences **/
    if (pcl_wait_save->size() > 0 && pcd_save_en)
    {
        string file_name = string("scans.pcd");
        string all_points_dir(string(string(ROOT_DIR) + "PCD/") + file_name);
        pcl::PCDWriter pcd_writer;
        cout << "current scan saved to /PCD/" << file_name<<endl;
        pcd_writer.writeBinary(all_points_dir, *pcl_wait_save);
    }

    fout_out.close();
    fout_pre.close();

    if (runtime_pos_log)
    {
        vector<double> t, s_vec, s_vec2, s_vec3, s_vec4, s_vec5, s_vec6, s_vec7;    
        FILE *fp2;
        string log_dir = root_dir + "/Log/fast_lio_time_log.csv";
        fp2 = fopen(log_dir.c_str(),"w");
        fprintf(fp2,"time_stamp, total time, scan point size, incremental time, search time, delete size, delete time, tree size st, tree size end, add point size, preprocess time\n");
        for (int i = 0;i<time_log_counter; i++){
            fprintf(fp2,"%0.8f,%0.8f,%d,%0.8f,%0.8f,%d,%0.8f,%d,%d,%d,%0.8f\n",T1[i],s_plot[i],int(s_plot2[i]),s_plot3[i],s_plot4[i],int(s_plot5[i]),s_plot6[i],int(s_plot7[i]),int(s_plot8[i]), int(s_plot10[i]), s_plot11[i]);
            t.push_back(T1[i]);
            s_vec.push_back(s_plot9[i]);
            s_vec2.push_back(s_plot3[i] + s_plot6[i]);
            s_vec3.push_back(s_plot4[i]);
            s_vec5.push_back(s_plot[i]);
        }
        fclose(fp2);
    }    

}
void lioMapping::saveAndPub()
{
    /******* Publish odometry *******/
    publish_odometry(pubOdomAftMapped);

    /******* Publish map *******/
    // saveAndPublishMapInfo();
    
    /******* Publish points *******/
    // if (path_en)                         publish_path(pubPath);
    if (path_en){
        publish_path(pubPath);
        static int jjj = 0;
        jjj++;
    }

    if (scan_pub_en || pcd_save_en)      publish_frame_world(pubLaserCloudFull);
    if (scan_pub_en && scan_body_pub_en) publish_frame_body(pubLaserCloudFull_body);
    // publish_effect_world(pubLaserCloudEffect);
    // publish_map(pubLaserCloudMap);
}
}