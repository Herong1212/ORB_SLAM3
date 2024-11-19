//
// Created by yuwenlu on 2022/7/2.
//
#include "PointCloudMapper.h"

namespace ORB_SLAM3
{

    typedef pcl::PointXYZRGB PointT;            // PointT 是点云中的单个点，包含位置信息 (x, y, z) 和颜色信息 (r, g, b)。
    typedef pcl::PointCloud<PointT> PointCloud; // 点云的集合

    // 构造函数：初始化点云地图和体素滤波器
    PointCloudMapper::PointCloudMapper()
    {
        // mpGlobalMap: 全局点云地图，使用共享指针（pcl::make_shared）分配内存，存储所有关键帧点云。
        mpGlobalMap = pcl::make_shared<PointCloud>();

        std::cout << "---------------------- voxel set start ----------------------" << std::endl;
        // mpVoxel 体素滤波器，用于简化点云，减少计算量。
        mpVoxel = pcl::make_shared<pcl::VoxelGrid<PointT>>();

        // 叶子越大，点云简化越多，但细节损失越多。
        // 体素叶子太小，容易出现：[pcl::VoxelGrid::applyFilter] Leaf size is too small for the input dataset. Integer indices would overflow. 的错误！
        mpVoxel->setLeafSize(0.1f, 0.1f, 0.1f);

        std::cout << "---------------------- voxel set finish ----------------------" << std::endl;
    }

    // 功能：将新的关键帧和其对应的 RGB 图像与深度图压入队列，供后续处理
    void PointCloudMapper::InsertKeyFrame(KeyFrame *kf, cv::Mat &imRGB, cv::Mat &imDepth)
    {
        std::lock_guard<std::mutex> lck_loadKF(mmLoadKFMutex);

        mqKeyFrame.push(kf);
        mqRGB.push(imRGB.clone());
        mqDepth.push(imDepth.clone());
    }

    // 功能：从关键帧和对应的 RGB-D 图像生成一个点云
    PointCloud::Ptr PointCloudMapper::GeneratePointCloud(KeyFrame *kf, cv::Mat &imRGB, cv::Mat &imDepth)
    {
        PointCloud::Ptr pointCloud_temp(new PointCloud);

        for (int v = 0; v < imRGB.rows; v++)
        {
            for (int u = 0; u < imRGB.cols; u++)
            {
                cv::Point2i pt(u, v);

                // 检查是否是动态区域
                bool IsDynamic = false;
                for (auto area : kf->mvDynamicArea)
                    if (area.contains(pt))
                        IsDynamic = true;

                // 不是动态区域，则继续处理
                if (!IsDynamic)
                {
                    // 获取深度图像中对应像素的深度值 d
                    float d = imDepth.ptr<float>(v)[u];

                    if (d < 0.01 || d > 10)
                        continue;

                    PointT p;
                    // 根据相机内参计算三维点的 (x, y, z) 坐标
                    p.z = d;
                    p.x = (u - kf->cx) * p.z / kf->fx;
                    p.y = (v - kf->cy) * p.z / kf->fy;

                    // 添加点云颜色信息（r, g, b）
                    p.b = imRGB.ptr<cv::Vec3b>(v)[u][0];
                    p.g = imRGB.ptr<cv::Vec3b>(v)[u][1];
                    p.r = imRGB.ptr<cv::Vec3b>(v)[u][2];
                    pointCloud_temp->push_back(p);
                }
            }
        }

        // 将点云从相机坐标系变换到世界坐标系
        Eigen::Isometry3d T = ORB_SLAM3::Converter::toSE3Quat(kf->GetPose());
        PointCloud::Ptr pointCloud(new PointCloud);
        pcl::transformPointCloud(*pointCloud_temp, *pointCloud, T.inverse().matrix()); // 使用 PCL 的 pcl::transformPointCloud 完成点云变换
        pointCloud->is_dense = false;
        return pointCloud;
    }

    // 功能：点云生成和地图更新的主线程。
    void PointCloudMapper::run()
    {
        pcl::visualization::CloudViewer Viewer("Viewer");

        std::cout << endl
                  << "PointCloudMapping thread start!" << std::endl;

        int ID = 0;
        while (1)
        {
            {
                std::lock_guard<std::mutex> lck_loadKFSize(mmLoadKFMutex);

                // step1：检查队列中是否有未处理的关键帧
                mKeyFrameSize = mqKeyFrame.size();
            }
            if (mKeyFrameSize != 0)
            {
                // step2：取出一个关键帧及其 RGB-D 图像生成点云
                PointCloud::Ptr pointCloud_new(new PointCloud);
                pointCloud_new = GeneratePointCloud(mqKeyFrame.front(), mqRGB.front(), mqDepth.front());

                mqKeyFrame.pop();
                mqRGB.pop();
                mqDepth.pop();

                //            std::cout << "==============Insert No. " << ID << "KeyFrame ================" << std::endl;
                ID++;

                // step3：更新全局地图
                *mpGlobalMap += *pointCloud_new; // 将生成的关键帧点云累加到 mpGlobalMap
                PointCloud::Ptr temp(new PointCloud);
                pcl::copyPointCloud(*mpGlobalMap, *temp);

                // 使用体素滤波器对全局点云地图进行下采样
                mpVoxel->setInputCloud(temp);
                mpVoxel->filter(*mpGlobalMap);
            }

            // step4：显示点云
            Viewer.showCloud(mpGlobalMap);
        }
    }
}