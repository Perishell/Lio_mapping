/*
 * @Author: dyhan
 * @Date: 2022-10-31 09:09:03
 * @LastEditors: dyhan
 * @LastEditTime: 2024-10-22 16:41:40
 * @Description: 
 */
#include "lio/lioMapping.h"
#include "lio/optimization.h"

int main(int argc, char** argv)
{

    ros::init(argc, argv, "runMapping");
    ros::NodeHandle nh;
    ros::NodeHandle priNh("~");

    bool updateLio = 0;
    bool recontructKdTree = 0;
    bool backEndEnable = false;
    nh.param<bool>("lio_sam/updateLio", updateLio, true);
    nh.param<bool>("lio_sam/recontructKdTree", recontructKdTree, true);
    nh.param<bool>("lio_sam/backEndEnable", backEndEnable, true);
    
    auto lioNode = std::make_shared<fusion_slam::lioMapping>();

    auto optNode = std::make_shared<fusion_slam::optimization>();

    lioNode->initROS(&nh, &priNh);

    optNode->initROS(&nh, &priNh);

    ros::Rate rate(200);

    while(ros::ok())
    {
        rate.sleep();

        if(lioNode->mainRun())
        {
            Eigen::Isometry3d poseLio;
            PointCloudXYZI::Ptr frameLio(new PointCloudXYZI());
            double frameEndTime;
            if(lioNode->getOdomAndFrame(poseLio, frameLio, frameEndTime))
            {
                optNode->setOdomAndFrame(poseLio, frameLio, frameEndTime);
                if(backEndEnable && optNode->mainRun())
                {
                    // 更新lio位姿
                    // if(updateLio)
                    // {
                    //     Eigen::Isometry3d poseOp;
                    //     if(optNode->getOpPose(poseOp))
                    //     {
                    //         lioNode->updateState(poseOp);
                    //     }
                    // }
                    // 重构ikdtree
                    if(recontructKdTree)
                    {
                        PointCloudXYZI::Ptr mapCloudFrames(new PointCloudXYZI());
                        if(optNode->getIKdTreeCloud(mapCloudFrames))
                        {
                            lioNode->updateMap(mapCloudFrames);
                            // 更新lio位姿
                            Eigen::Isometry3d poseOp;
                            if(optNode->getOpPose(poseOp))
                            {
                                lioNode->updateState(poseOp);
                            }
                        }
                    }

                }
            }

            lioNode->map_incremental();
            lioNode->saveAndPub();
        }
        ros::spinOnce();
        
    }

    lioNode->mainShutDown();
    // optNode->mainShutDown();

    // ros::spinOnce();

    return 0;
}