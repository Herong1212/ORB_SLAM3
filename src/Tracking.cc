/**
 * This file is part of ORB-SLAM3
 *
 * Copyright (C) 2017-2021 Carlos Campos, Richard Elvira, Juan J. Gómez Rodríguez, José M.M. Montiel and Juan D. Tardós, University of Zaragoza.
 * Copyright (C) 2014-2016 Raúl Mur-Artal, José M.M. Montiel and Juan D. Tardós, University of Zaragoza.
 *
 * ORB-SLAM3 is free software: you can redistribute it and/or modify it under the terms of the GNU General Public
 * License as published by the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * ORB-SLAM3 is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even
 * the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along with ORB-SLAM3.
 * If not, see <http://www.gnu.org/licenses/>.
 */

#include "Tracking.h"

#include "ORBmatcher.h"
#include "FrameDrawer.h"
#include "Converter.h"
#include "G2oTypes.h"
#include "Optimizer.h"
#include "Pinhole.h"
#include "KannalaBrandt8.h"
#include "MLPnPsolver.h"
#include "GeometricTools.h"

#include <iostream>

#include <mutex>
#include <chrono>

using namespace std;

// 程序中变量名的第一个字母如果为"m"则表示为类中的成员变量，member
// 第一个、第二个字母:
// "p"表示指针数据类型
// "n"表示int类型
// "b"表示bool类型
// "s"表示set类型
// "v"表示vector数据类型
// 'l'表示list数据类型
// "KF"表示 KeyFrame 数据类型

namespace ORB_SLAM3
{
    // ps：跟踪线程构造函数，用于初始化跟踪线程的各种成员变量和依赖对象，并从配置文件中加载参数。
    Tracking::Tracking(System *pSys,                                    // 指向系统类的指针，用于全局管理 SLAM 流程
                       ORBVocabulary *pVoc,                             // 词典，用于回环检测和关键帧的词袋模型构建
                       FrameDrawer *pFrameDrawer,                       // 绘制当前帧特征点和轨迹的可视化模块
                       MapDrawer *pMapDrawer,                           // 绘制地图点和关键帧的可视化模块
                       Atlas *pAtlas,                                   // 地图管理器，用于管理全局地图、子地图等结构
                       KeyFrameDatabase *pKFDB,                         // 关键帧数据库，用于快速查找相似关键帧，配合词典进行回环检测
                       const string &strSettingPath,                    // 配置文件路径，包含相机、ORB、IMU 参数等配置
                       const int sensor,                                // 传感器类型，如单目、双目、RGB-D 或 VIO 模式
                       Settings *settings,                              // 参数类指针，若不为空，直接从该类加载参数，而不是读取配置文件
                       const string &_nameSeq) :                        // ! 序列名称（不常用，仅用于日志记录或调试）
                                                 mState(NO_IMAGES_YET), // 初始状态为未处理图像
                                                 mSensor(sensor),       // 传感器类型
                                                 mTrackedFr(0),         // 已跟踪的帧数量
                                                 mbStep(false),
                                                 mbOnlyTracking(false), // 是否仅追踪模式
                                                 mbMapUpdated(false),
                                                 mbVO(false),
                                                 mpORBVocabulary(pVoc), // 初始化 ORB 词典
                                                 mpKeyFrameDB(pKFDB),
                                                 mbReadyToInitializate(false),
                                                 mpSystem(pSys), // 初始化系统指针
                                                 mpViewer(NULL),
                                                 bStepByStep(false),
                                                 mpFrameDrawer(pFrameDrawer), mpMapDrawer(pMapDrawer), mpAtlas(pAtlas), mnLastRelocFrameId(0), time_recently_lost(5.0),
                                                 mnInitialFrameId(0), mbCreatedMap(false), mnFirstFrameId(0), mpCamera2(nullptr), mpLastKeyFrame(static_cast<KeyFrame *>(NULL))
    {
        // Step 1 从配置文件中加载相机参数
        if (settings)
        {
            newParameterLoader(settings);
        }

        // 否则，从配置文件中解析参数
        else
        {
            cv::FileStorage fSettings(strSettingPath, cv::FileStorage::READ);

            bool b_parse_cam = ParseCamParamFile(fSettings);
            if (!b_parse_cam)
            {
                std::cout << "*Error with the camera parameters in the config file*" << std::endl;
            }

            // Load ORB parameters
            bool b_parse_orb = ParseORBParamFile(fSettings);
            if (!b_parse_orb)
            {
                std::cout << "*Error with the ORB parameters in the config file*" << std::endl;
            }

            // Load IMU parameters
            bool b_parse_imu = true;
            if (sensor == System::IMU_MONOCULAR || sensor == System::IMU_STEREO || sensor == System::IMU_RGBD)
            {
                b_parse_imu = ParseIMUParamFile(fSettings);
                if (!b_parse_imu)
                {
                    std::cout << "*Error with the IMU parameters in the config file*" << std::endl;
                }

                // 将 mnFramesToResetIMU 设置为 mMaxFrames，即在经过 mMaxFrames 帧之后，IMU 数据会被重置
                mnFramesToResetIMU = mMaxFrames;
            }

            if (!b_parse_cam || !b_parse_orb || !b_parse_imu)
            {
                std::cerr << "**ERROR in the config file, the format is not correct**" << std::endl;
                try
                {
                    throw -1;
                }
                catch (exception &e)
                {
                }
            }
        }

        initID = 0;
        lastID = 0;
        mbInitWith3KFs = false;
        mnNumDataset = 0;

        // 遍历下地图中的相机，然后打印出来了
        vector<GeometricCamera *> vpCams = mpAtlas->GetAllCameras();
        std::cout << "There are " << vpCams.size() << " cameras in the atlas" << std::endl;
        for (GeometricCamera *pCam : vpCams)
        {
            std::cout << "Camera " << pCam->GetId();
            if (pCam->GetType() == GeometricCamera::CAM_PINHOLE)
            {
                std::cout << " is pinhole" << std::endl;
            }
            else if (pCam->GetType() == GeometricCamera::CAM_FISHEYE)
            {
                std::cout << " is fisheye" << std::endl;
            }
            else
            {
                std::cout << " is unknown" << std::endl;
            }
        }

#ifdef REGISTER_TIMES
        vdRectStereo_ms.clear();
        vdResizeImage_ms.clear();
        vdORBExtract_ms.clear();
        vdStereoMatch_ms.clear();
        vdIMUInteg_ms.clear();
        vdPosePred_ms.clear();
        vdLMTrack_ms.clear();
        vdNewKF_ms.clear();
        vdTrackTotal_ms.clear();
#endif
    }

#ifdef REGISTER_TIMES
    double calcAverage(vector<double> v_times)
    {
        double accum = 0;
        for (double value : v_times)
        {
            accum += value;
        }

        return accum / v_times.size();
    }

    double calcDeviation(vector<double> v_times, double average)
    {
        double accum = 0;
        for (double value : v_times)
        {
            accum += pow(value - average, 2);
        }
        return sqrt(accum / v_times.size());
    }

    double calcAverage(vector<int> v_values)
    {
        double accum = 0;
        int total = 0;
        for (double value : v_values)
        {
            if (value == 0)
                continue;
            accum += value;
            total++;
        }

        return accum / total;
    }

    double calcDeviation(vector<int> v_values, double average)
    {
        double accum = 0;
        int total = 0;
        for (double value : v_values)
        {
            if (value == 0)
                continue;
            accum += pow(value - average, 2);
            total++;
        }
        return sqrt(accum / total);
    }

    void Tracking::LocalMapStats2File()
    {
        ofstream f;
        f.open("LocalMapTimeStats.txt");
        f << fixed << setprecision(6);
        f << "#Stereo rect[ms], MP culling[ms], MP creation[ms], LBA[ms], KF culling[ms], Total[ms]" << endl;
        for (int i = 0; i < mpLocalMapper->vdLMTotal_ms.size(); ++i)
        {
            f << mpLocalMapper->vdKFInsert_ms[i] << "," << mpLocalMapper->vdMPCulling_ms[i] << ","
              << mpLocalMapper->vdMPCreation_ms[i] << "," << mpLocalMapper->vdLBASync_ms[i] << ","
              << mpLocalMapper->vdKFCullingSync_ms[i] << "," << mpLocalMapper->vdLMTotal_ms[i] << endl;
        }

        f.close();

        f.open("LBA_Stats.txt");
        f << fixed << setprecision(6);
        f << "#LBA time[ms], KF opt[#], KF fixed[#], MP[#], Edges[#]" << endl;
        for (int i = 0; i < mpLocalMapper->vdLBASync_ms.size(); ++i)
        {
            f << mpLocalMapper->vdLBASync_ms[i] << "," << mpLocalMapper->vnLBA_KFopt[i] << ","
              << mpLocalMapper->vnLBA_KFfixed[i] << "," << mpLocalMapper->vnLBA_MPs[i] << ","
              << mpLocalMapper->vnLBA_edges[i] << endl;
        }

        f.close();
    }

    void Tracking::TrackStats2File()
    {
        ofstream f;
        f.open("SessionInfo.txt");
        f << fixed;
        f << "Number of KFs: " << mpAtlas->GetAllKeyFrames().size() << endl;
        f << "Number of MPs: " << mpAtlas->GetAllMapPoints().size() << endl;

        f << "OpenCV version: " << CV_VERSION << endl;

        f.close();

        f.open("TrackingTimeStats.txt");
        f << fixed << setprecision(6);

        f << "#Image Rect[ms], Image Resize[ms], ORB ext[ms], Stereo match[ms], IMU preint[ms], Pose pred[ms], LM track[ms], KF dec[ms], Total[ms]" << endl;

        for (int i = 0; i < vdTrackTotal_ms.size(); ++i)
        {
            double stereo_rect = 0.0;
            if (!vdRectStereo_ms.empty())
            {
                stereo_rect = vdRectStereo_ms[i];
            }

            double resize_image = 0.0;
            if (!vdResizeImage_ms.empty())
            {
                resize_image = vdResizeImage_ms[i];
            }

            double stereo_match = 0.0;
            if (!vdStereoMatch_ms.empty())
            {
                stereo_match = vdStereoMatch_ms[i];
            }

            double imu_preint = 0.0;
            if (!vdIMUInteg_ms.empty())
            {
                imu_preint = vdIMUInteg_ms[i];
            }

            f << stereo_rect << "," << resize_image << "," << vdORBExtract_ms[i] << "," << stereo_match << "," << imu_preint << ","
              << vdPosePred_ms[i] << "," << vdLMTrack_ms[i] << "," << vdNewKF_ms[i] << "," << vdTrackTotal_ms[i] << endl;
        }

        f.close();
    }

    void Tracking::PrintTimeStats()
    {
        // Save data in files
        TrackStats2File();
        LocalMapStats2File();

        ofstream f;
        f.open("ExecMean.txt");
        f << fixed;
        // Report the mean and std of each one
        std::cout << std::endl
                  << " TIME STATS in ms (mean$\\pm$std)" << std::endl;
        f << " TIME STATS in ms (mean$\\pm$std)" << std::endl;
        cout << "OpenCV version: " << CV_VERSION << endl;
        f << "OpenCV version: " << CV_VERSION << endl;
        std::cout << "---------------------------" << std::endl;
        std::cout << "Tracking" << std::setprecision(5) << std::endl
                  << std::endl;
        f << "---------------------------" << std::endl;
        f << "Tracking" << std::setprecision(5) << std::endl
          << std::endl;
        double average, deviation;
        if (!vdRectStereo_ms.empty())
        {
            average = calcAverage(vdRectStereo_ms);
            deviation = calcDeviation(vdRectStereo_ms, average);
            std::cout << "Stereo Rectification: " << average << "$\\pm$" << deviation << std::endl;
            f << "Stereo Rectification: " << average << "$\\pm$" << deviation << std::endl;
        }

        if (!vdResizeImage_ms.empty())
        {
            average = calcAverage(vdResizeImage_ms);
            deviation = calcDeviation(vdResizeImage_ms, average);
            std::cout << "Image Resize: " << average << "$\\pm$" << deviation << std::endl;
            f << "Image Resize: " << average << "$\\pm$" << deviation << std::endl;
        }

        average = calcAverage(vdORBExtract_ms);
        deviation = calcDeviation(vdORBExtract_ms, average);
        std::cout << "ORB Extraction: " << average << "$\\pm$" << deviation << std::endl;
        f << "ORB Extraction: " << average << "$\\pm$" << deviation << std::endl;

        if (!vdStereoMatch_ms.empty())
        {
            average = calcAverage(vdStereoMatch_ms);
            deviation = calcDeviation(vdStereoMatch_ms, average);
            std::cout << "Stereo Matching: " << average << "$\\pm$" << deviation << std::endl;
            f << "Stereo Matching: " << average << "$\\pm$" << deviation << std::endl;
        }

        if (!vdIMUInteg_ms.empty())
        {
            average = calcAverage(vdIMUInteg_ms);
            deviation = calcDeviation(vdIMUInteg_ms, average);
            std::cout << "IMU Preintegration: " << average << "$\\pm$" << deviation << std::endl;
            f << "IMU Preintegration: " << average << "$\\pm$" << deviation << std::endl;
        }

        average = calcAverage(vdPosePred_ms);
        deviation = calcDeviation(vdPosePred_ms, average);
        std::cout << "Pose Prediction: " << average << "$\\pm$" << deviation << std::endl;
        f << "Pose Prediction: " << average << "$\\pm$" << deviation << std::endl;

        average = calcAverage(vdLMTrack_ms);
        deviation = calcDeviation(vdLMTrack_ms, average);
        std::cout << "LM Track: " << average << "$\\pm$" << deviation << std::endl;
        f << "LM Track: " << average << "$\\pm$" << deviation << std::endl;

        average = calcAverage(vdNewKF_ms);
        deviation = calcDeviation(vdNewKF_ms, average);
        std::cout << "New KF decision: " << average << "$\\pm$" << deviation << std::endl;
        f << "New KF decision: " << average << "$\\pm$" << deviation << std::endl;

        average = calcAverage(vdTrackTotal_ms);
        deviation = calcDeviation(vdTrackTotal_ms, average);
        std::cout << "Total Tracking: " << average << "$\\pm$" << deviation << std::endl;
        f << "Total Tracking: " << average << "$\\pm$" << deviation << std::endl;

        // Local Mapping time stats
        std::cout << std::endl
                  << std::endl
                  << std::endl;
        std::cout << "Local Mapping" << std::endl
                  << std::endl;
        f << std::endl
          << "Local Mapping" << std::endl
          << std::endl;

        average = calcAverage(mpLocalMapper->vdKFInsert_ms);
        deviation = calcDeviation(mpLocalMapper->vdKFInsert_ms, average);
        std::cout << "KF Insertion: " << average << "$\\pm$" << deviation << std::endl;
        f << "KF Insertion: " << average << "$\\pm$" << deviation << std::endl;

        average = calcAverage(mpLocalMapper->vdMPCulling_ms);
        deviation = calcDeviation(mpLocalMapper->vdMPCulling_ms, average);
        std::cout << "MP Culling: " << average << "$\\pm$" << deviation << std::endl;
        f << "MP Culling: " << average << "$\\pm$" << deviation << std::endl;

        average = calcAverage(mpLocalMapper->vdMPCreation_ms);
        deviation = calcDeviation(mpLocalMapper->vdMPCreation_ms, average);
        std::cout << "MP Creation: " << average << "$\\pm$" << deviation << std::endl;
        f << "MP Creation: " << average << "$\\pm$" << deviation << std::endl;

        average = calcAverage(mpLocalMapper->vdLBA_ms);
        deviation = calcDeviation(mpLocalMapper->vdLBA_ms, average);
        std::cout << "LBA: " << average << "$\\pm$" << deviation << std::endl;
        f << "LBA: " << average << "$\\pm$" << deviation << std::endl;

        average = calcAverage(mpLocalMapper->vdKFCulling_ms);
        deviation = calcDeviation(mpLocalMapper->vdKFCulling_ms, average);
        std::cout << "KF Culling: " << average << "$\\pm$" << deviation << std::endl;
        f << "KF Culling: " << average << "$\\pm$" << deviation << std::endl;

        average = calcAverage(mpLocalMapper->vdLMTotal_ms);
        deviation = calcDeviation(mpLocalMapper->vdLMTotal_ms, average);
        std::cout << "Total Local Mapping: " << average << "$\\pm$" << deviation << std::endl;
        f << "Total Local Mapping: " << average << "$\\pm$" << deviation << std::endl;

        // Local Mapping LBA complexity
        std::cout << "---------------------------" << std::endl;
        std::cout << std::endl
                  << "LBA complexity (mean$\\pm$std)" << std::endl;
        f << "---------------------------" << std::endl;
        f << std::endl
          << "LBA complexity (mean$\\pm$std)" << std::endl;

        average = calcAverage(mpLocalMapper->vnLBA_edges);
        deviation = calcDeviation(mpLocalMapper->vnLBA_edges, average);
        std::cout << "LBA Edges: " << average << "$\\pm$" << deviation << std::endl;
        f << "LBA Edges: " << average << "$\\pm$" << deviation << std::endl;

        average = calcAverage(mpLocalMapper->vnLBA_KFopt);
        deviation = calcDeviation(mpLocalMapper->vnLBA_KFopt, average);
        std::cout << "LBA KF optimized: " << average << "$\\pm$" << deviation << std::endl;
        f << "LBA KF optimized: " << average << "$\\pm$" << deviation << std::endl;

        average = calcAverage(mpLocalMapper->vnLBA_KFfixed);
        deviation = calcDeviation(mpLocalMapper->vnLBA_KFfixed, average);
        std::cout << "LBA KF fixed: " << average << "$\\pm$" << deviation << std::endl;
        f << "LBA KF fixed: " << average << "$\\pm$" << deviation << std::endl;

        average = calcAverage(mpLocalMapper->vnLBA_MPs);
        deviation = calcDeviation(mpLocalMapper->vnLBA_MPs, average);
        std::cout << "LBA MP: " << average << "$\\pm$" << deviation << std::endl
                  << std::endl;
        f << "LBA MP: " << average << "$\\pm$" << deviation << std::endl
          << std::endl;

        std::cout << "LBA executions: " << mpLocalMapper->nLBA_exec << std::endl;
        std::cout << "LBA aborts: " << mpLocalMapper->nLBA_abort << std::endl;
        f << "LBA executions: " << mpLocalMapper->nLBA_exec << std::endl;
        f << "LBA aborts: " << mpLocalMapper->nLBA_abort << std::endl;

        // Map complexity
        std::cout << "---------------------------" << std::endl;
        std::cout << std::endl
                  << "Map complexity" << std::endl;
        std::cout << "KFs in map: " << mpAtlas->GetAllKeyFrames().size() << std::endl;
        std::cout << "MPs in map: " << mpAtlas->GetAllMapPoints().size() << std::endl;
        f << "---------------------------" << std::endl;
        f << std::endl
          << "Map complexity" << std::endl;
        vector<Map *> vpMaps = mpAtlas->GetAllMaps();
        Map *pBestMap = vpMaps[0];
        for (int i = 1; i < vpMaps.size(); ++i)
        {
            if (pBestMap->GetAllKeyFrames().size() < vpMaps[i]->GetAllKeyFrames().size())
            {
                pBestMap = vpMaps[i];
            }
        }

        f << "KFs in map: " << pBestMap->GetAllKeyFrames().size() << std::endl;
        f << "MPs in map: " << pBestMap->GetAllMapPoints().size() << std::endl;

        f << "---------------------------" << std::endl;
        f << std::endl
          << "Place Recognition (mean$\\pm$std)" << std::endl;
        std::cout << "---------------------------" << std::endl;
        std::cout << std::endl
                  << "Place Recognition (mean$\\pm$std)" << std::endl;
        average = calcAverage(mpLoopClosing->vdDataQuery_ms);
        deviation = calcDeviation(mpLoopClosing->vdDataQuery_ms, average);
        f << "Database Query: " << average << "$\\pm$" << deviation << std::endl;
        std::cout << "Database Query: " << average << "$\\pm$" << deviation << std::endl;
        average = calcAverage(mpLoopClosing->vdEstSim3_ms);
        deviation = calcDeviation(mpLoopClosing->vdEstSim3_ms, average);
        f << "SE3 estimation: " << average << "$\\pm$" << deviation << std::endl;
        std::cout << "SE3 estimation: " << average << "$\\pm$" << deviation << std::endl;
        average = calcAverage(mpLoopClosing->vdPRTotal_ms);
        deviation = calcDeviation(mpLoopClosing->vdPRTotal_ms, average);
        f << "Total Place Recognition: " << average << "$\\pm$" << deviation << std::endl
          << std::endl;
        std::cout << "Total Place Recognition: " << average << "$\\pm$" << deviation << std::endl
                  << std::endl;

        f << std::endl
          << "Loop Closing (mean$\\pm$std)" << std::endl;
        std::cout << std::endl
                  << "Loop Closing (mean$\\pm$std)" << std::endl;
        average = calcAverage(mpLoopClosing->vdLoopFusion_ms);
        deviation = calcDeviation(mpLoopClosing->vdLoopFusion_ms, average);
        f << "Loop Fusion: " << average << "$\\pm$" << deviation << std::endl;
        std::cout << "Loop Fusion: " << average << "$\\pm$" << deviation << std::endl;
        average = calcAverage(mpLoopClosing->vdLoopOptEss_ms);
        deviation = calcDeviation(mpLoopClosing->vdLoopOptEss_ms, average);
        f << "Essential Graph: " << average << "$\\pm$" << deviation << std::endl;
        std::cout << "Essential Graph: " << average << "$\\pm$" << deviation << std::endl;
        average = calcAverage(mpLoopClosing->vdLoopTotal_ms);
        deviation = calcDeviation(mpLoopClosing->vdLoopTotal_ms, average);
        f << "Total Loop Closing: " << average << "$\\pm$" << deviation << std::endl
          << std::endl;
        std::cout << "Total Loop Closing: " << average << "$\\pm$" << deviation << std::endl
                  << std::endl;

        f << "Numb exec: " << mpLoopClosing->nLoop << std::endl;
        std::cout << "Num exec: " << mpLoopClosing->nLoop << std::endl;
        average = calcAverage(mpLoopClosing->vnLoopKFs);
        deviation = calcDeviation(mpLoopClosing->vnLoopKFs, average);
        f << "Number of KFs: " << average << "$\\pm$" << deviation << std::endl;
        std::cout << "Number of KFs: " << average << "$\\pm$" << deviation << std::endl;

        f << std::endl
          << "Map Merging (mean$\\pm$std)" << std::endl;
        std::cout << std::endl
                  << "Map Merging (mean$\\pm$std)" << std::endl;
        average = calcAverage(mpLoopClosing->vdMergeMaps_ms);
        deviation = calcDeviation(mpLoopClosing->vdMergeMaps_ms, average);
        f << "Merge Maps: " << average << "$\\pm$" << deviation << std::endl;
        std::cout << "Merge Maps: " << average << "$\\pm$" << deviation << std::endl;
        average = calcAverage(mpLoopClosing->vdWeldingBA_ms);
        deviation = calcDeviation(mpLoopClosing->vdWeldingBA_ms, average);
        f << "Welding BA: " << average << "$\\pm$" << deviation << std::endl;
        std::cout << "Welding BA: " << average << "$\\pm$" << deviation << std::endl;
        average = calcAverage(mpLoopClosing->vdMergeOptEss_ms);
        deviation = calcDeviation(mpLoopClosing->vdMergeOptEss_ms, average);
        f << "Optimization Ess.: " << average << "$\\pm$" << deviation << std::endl;
        std::cout << "Optimization Ess.: " << average << "$\\pm$" << deviation << std::endl;
        average = calcAverage(mpLoopClosing->vdMergeTotal_ms);
        deviation = calcDeviation(mpLoopClosing->vdMergeTotal_ms, average);
        f << "Total Map Merging: " << average << "$\\pm$" << deviation << std::endl
          << std::endl;
        std::cout << "Total Map Merging: " << average << "$\\pm$" << deviation << std::endl
                  << std::endl;

        f << "Numb exec: " << mpLoopClosing->nMerges << std::endl;
        std::cout << "Num exec: " << mpLoopClosing->nMerges << std::endl;
        average = calcAverage(mpLoopClosing->vnMergeKFs);
        deviation = calcDeviation(mpLoopClosing->vnMergeKFs, average);
        f << "Number of KFs: " << average << "$\\pm$" << deviation << std::endl;
        std::cout << "Number of KFs: " << average << "$\\pm$" << deviation << std::endl;
        average = calcAverage(mpLoopClosing->vnMergeMPs);
        deviation = calcDeviation(mpLoopClosing->vnMergeMPs, average);
        f << "Number of MPs: " << average << "$\\pm$" << deviation << std::endl;
        std::cout << "Number of MPs: " << average << "$\\pm$" << deviation << std::endl;

        f << std::endl
          << "Full GBA (mean$\\pm$std)" << std::endl;
        std::cout << std::endl
                  << "Full GBA (mean$\\pm$std)" << std::endl;
        average = calcAverage(mpLoopClosing->vdGBA_ms);
        deviation = calcDeviation(mpLoopClosing->vdGBA_ms, average);
        f << "GBA: " << average << "$\\pm$" << deviation << std::endl;
        std::cout << "GBA: " << average << "$\\pm$" << deviation << std::endl;
        average = calcAverage(mpLoopClosing->vdUpdateMap_ms);
        deviation = calcDeviation(mpLoopClosing->vdUpdateMap_ms, average);
        f << "Map Update: " << average << "$\\pm$" << deviation << std::endl;
        std::cout << "Map Update: " << average << "$\\pm$" << deviation << std::endl;
        average = calcAverage(mpLoopClosing->vdFGBATotal_ms);
        deviation = calcDeviation(mpLoopClosing->vdFGBATotal_ms, average);
        f << "Total Full GBA: " << average << "$\\pm$" << deviation << std::endl
          << std::endl;
        std::cout << "Total Full GBA: " << average << "$\\pm$" << deviation << std::endl
                  << std::endl;

        f << "Numb exec: " << mpLoopClosing->nFGBA_exec << std::endl;
        std::cout << "Num exec: " << mpLoopClosing->nFGBA_exec << std::endl;
        f << "Numb abort: " << mpLoopClosing->nFGBA_abort << std::endl;
        std::cout << "Num abort: " << mpLoopClosing->nFGBA_abort << std::endl;
        average = calcAverage(mpLoopClosing->vnGBAKFs);
        deviation = calcDeviation(mpLoopClosing->vnGBAKFs, average);
        f << "Number of KFs: " << average << "$\\pm$" << deviation << std::endl;
        std::cout << "Number of KFs: " << average << "$\\pm$" << deviation << std::endl;
        average = calcAverage(mpLoopClosing->vnGBAMPs);
        deviation = calcDeviation(mpLoopClosing->vnGBAMPs, average);
        f << "Number of MPs: " << average << "$\\pm$" << deviation << std::endl;
        std::cout << "Number of MPs: " << average << "$\\pm$" << deviation << std::endl;

        f.close();
    }

#endif

    Tracking::~Tracking()
    {
        // f_track_stats.close();
    }

    // ----------------------------------- 根据文件读取配置文件参数，可快速略过不看 --------------------------------------------------

    // 根据参数类读取参数，可快速略过不看
    void Tracking::newParameterLoader(Settings *settings)
    {
        // step1. 读取主相机
        mpCamera = settings->camera1();
        // 将相机添加到地图（mpAtlas）中，mpCamera 现在是一个已注册的相机对象
        mpCamera = mpAtlas->AddCamera(mpCamera);

        // 如果需要校正相机畸变，加载畸变系数
        if (settings->needToUndistort())
        {
            mDistCoef = settings->camera1DistortionCoef();
        }
        // 如果不需要去畸变，使用零矩阵表示无畸变
        else
        {
            mDistCoef = cv::Mat::zeros(4, 1, CV_32F);
        }

        // TODO: missing image scaling and rectification
        mImageScale = 1.0f; // 设置图像缩放系数为 1.0f，表示没有缩放

        // mK 是相机内参矩阵，使用相机1的参数来填充
        mK = cv::Mat::eye(3, 3, CV_32F);
        mK.at<float>(0, 0) = mpCamera->getParameter(0);
        mK.at<float>(1, 1) = mpCamera->getParameter(1);
        mK.at<float>(0, 2) = mpCamera->getParameter(2);
        mK.at<float>(1, 2) = mpCamera->getParameter(3);

        // mK_ 是另一种形式的相机内参矩阵，通过 setIdentity() 初始化为单位矩阵，然后设置焦距和主点位置
        mK_.setIdentity();
        mK_(0, 0) = mpCamera->getParameter(0);
        mK_(1, 1) = mpCamera->getParameter(1);
        mK_(0, 2) = mpCamera->getParameter(2);
        mK_(1, 2) = mpCamera->getParameter(3);

        // 加载 相机2 的参数（双目或 RGB-D）
        if ((mSensor == System::STEREO || mSensor == System::IMU_STEREO || mSensor == System::IMU_RGBD) && settings->cameraType() == Settings::KannalaBrandt)
        {
            mpCamera2 = settings->camera2();
            mpCamera2 = mpAtlas->AddCamera(mpCamera2);

            mTlr = settings->Tlr();

            // 告知绘图模块需要处理双目
            mpFrameDrawer->both = true;
        }

        // 读取双目
        if (mSensor == System::STEREO || mSensor == System::RGBD || mSensor == System::IMU_STEREO || mSensor == System::IMU_RGBD)
        {
            mbf = settings->bf(); // mbf：双目相机的基线乘以焦距，用于三角测量
            mThDepth = settings->b() * settings->thDepth();
        }

        // 加载深度信息（仅对 RGBD 或 IMU RGBD 有效）
        if (mSensor == System::RGBD || mSensor == System::IMU_RGBD)
        {

            // 从配置中获取深度图因子 depthMapFactor()，并进行适当的调整
            mDepthMapFactor = settings->depthMapFactor();

            if (fabs(mDepthMapFactor) < 1e-5)
                mDepthMapFactor = 1;
            else
                mDepthMapFactor = 1.0f / mDepthMapFactor;
        }

        mMinFrames = 0;
        mMaxFrames = settings->fps();
        mbRGB = settings->rgb();

        // step2.ORB parameters
        int nFeatures = settings->nFeatures();        // 每帧提取的最大特征数
        int nLevels = settings->nLevels();            // 金字塔层数
        int fIniThFAST = settings->initThFAST();      // FAST特征提取的阈值
        int fMinThFAST = settings->minThFAST();       // FAST特征提取的阈值
        float fScaleFactor = settings->scaleFactor(); // 图像金字塔的尺度因子

        // 初始化 ORB 特征提取器（左相机为主相机）
        mpORBextractorLeft = new ORBextractor(nFeatures, fScaleFactor, nLevels, fIniThFAST, fMinThFAST);

        // 如果是 双目 或 IMU+双目 相机，还会初始化右相机的 ORB 特征提取器
        if (mSensor == System::STEREO || mSensor == System::IMU_STEREO)
            mpORBextractorRight = new ORBextractor(nFeatures, fScaleFactor, nLevels, fIniThFAST, fMinThFAST);

        // 如果是 单目 或 IMU+单目 相机，初始化用于初始化阶段的 ORB 特征提取器。特征点数更多，这里设置为 5 * nFeatures
        if (mSensor == System::MONOCULAR || mSensor == System::IMU_MONOCULAR)
            mpIniORBextractor = new ORBextractor(5 * nFeatures, fScaleFactor, nLevels, fIniThFAST, fMinThFAST);

        // step3.IMU parameters
        Sophus::SE3f Tbc = settings->Tbc(); // bc 是相机与 IMU 之间的变换矩阵（相机和 IMU 的相对位置）
        mInsertKFsLost = settings->insertKFsWhenLost();
        mImuFreq = settings->imuFrequency();

        // 每次 IMU 数据的时间间隔
        mImuPer = 0.001; // 1.0 / (double) mImuFreq;     // TODO: ESTO ESTA BIEN?

        // 读取 IMU 的噪声模型和偏移量
        float Ng = settings->noiseGyro();
        float Na = settings->noiseAcc();
        float Ngw = settings->gyroWalk();
        float Naw = settings->accWalk();

        // 初始化 IMU 校准对象 mpImuCalib，使用上面读取的 IMU 参数
        const float sf = sqrt(mImuFreq);
        mpImuCalib = new IMU::Calib(Tbc, Ng * sf, Na * sf, Ngw / sf, Naw / sf);

        // 初始化 IMU 预积分对象 ，初始偏置为零
        mpImuPreintegratedFromLastKF = new IMU::Preintegrated(IMU::Bias(), *mpImuCalib);
    }

    // 根据文件读取相机参数，可快速略过不看
    bool Tracking::ParseCamParamFile(cv::FileStorage &fSettings)
    {
        mDistCoef = cv::Mat::zeros(4, 1, CV_32F);
        cout << endl
             << "Camera Parameters: " << endl;
        bool b_miss_params = false;

        string sCameraName = fSettings["Camera.type"];

        if (sCameraName == "PinHole")
        {
            float fx, fy, cx, cy;
            mImageScale = 1.f;

            // Camera calibration parameters
            cv::FileNode node = fSettings["Camera.fx"];
            if (!node.empty() && node.isReal())
            {
                fx = node.real();
            }
            else
            {
                std::cerr << "*Camera.fx parameter doesn't exist or is not a real number*" << std::endl;
                b_miss_params = true;
            }

            node = fSettings["Camera.fy"];
            if (!node.empty() && node.isReal())
            {
                fy = node.real();
            }
            else
            {
                std::cerr << "*Camera.fy parameter doesn't exist or is not a real number*" << std::endl;
                b_miss_params = true;
            }

            node = fSettings["Camera.cx"];
            if (!node.empty() && node.isReal())
            {
                cx = node.real();
            }
            else
            {
                std::cerr << "*Camera.cx parameter doesn't exist or is not a real number*" << std::endl;
                b_miss_params = true;
            }

            node = fSettings["Camera.cy"];
            if (!node.empty() && node.isReal())
            {
                cy = node.real();
            }
            else
            {
                std::cerr << "*Camera.cy parameter doesn't exist or is not a real number*" << std::endl;
                b_miss_params = true;
            }

            // Distortion parameters
            node = fSettings["Camera.k1"];
            if (!node.empty() && node.isReal())
            {
                mDistCoef.at<float>(0) = node.real();
            }
            else
            {
                std::cerr << "*Camera.k1 parameter doesn't exist or is not a real number*" << std::endl;
                b_miss_params = true;
            }

            node = fSettings["Camera.k2"];
            if (!node.empty() && node.isReal())
            {
                mDistCoef.at<float>(1) = node.real();
            }
            else
            {
                std::cerr << "*Camera.k2 parameter doesn't exist or is not a real number*" << std::endl;
                b_miss_params = true;
            }

            node = fSettings["Camera.p1"];
            if (!node.empty() && node.isReal())
            {
                mDistCoef.at<float>(2) = node.real();
            }
            else
            {
                std::cerr << "*Camera.p1 parameter doesn't exist or is not a real number*" << std::endl;
                b_miss_params = true;
            }

            node = fSettings["Camera.p2"];
            if (!node.empty() && node.isReal())
            {
                mDistCoef.at<float>(3) = node.real();
            }
            else
            {
                std::cerr << "*Camera.p2 parameter doesn't exist or is not a real number*" << std::endl;
                b_miss_params = true;
            }

            node = fSettings["Camera.k3"];
            if (!node.empty() && node.isReal())
            {
                mDistCoef.resize(5);
                mDistCoef.at<float>(4) = node.real();
            }

            node = fSettings["Camera.imageScale"];
            if (!node.empty() && node.isReal())
            {
                mImageScale = node.real();
            }

            if (b_miss_params)
            {
                return false;
            }

            if (mImageScale != 1.f)
            {
                // K matrix parameters must be scaled.
                fx = fx * mImageScale;
                fy = fy * mImageScale;
                cx = cx * mImageScale;
                cy = cy * mImageScale;
            }

            vector<float> vCamCalib{fx, fy, cx, cy};

            mpCamera = new Pinhole(vCamCalib);

            mpCamera = mpAtlas->AddCamera(mpCamera);

            std::cout << "- Camera: Pinhole" << std::endl;
            std::cout << "- Image scale: " << mImageScale << std::endl;
            std::cout << "- fx: " << fx << std::endl;
            std::cout << "- fy: " << fy << std::endl;
            std::cout << "- cx: " << cx << std::endl;
            std::cout << "- cy: " << cy << std::endl;
            std::cout << "- k1: " << mDistCoef.at<float>(0) << std::endl;
            std::cout << "- k2: " << mDistCoef.at<float>(1) << std::endl;

            std::cout << "- p1: " << mDistCoef.at<float>(2) << std::endl;
            std::cout << "- p2: " << mDistCoef.at<float>(3) << std::endl;

            if (mDistCoef.rows == 5)
                std::cout << "- k3: " << mDistCoef.at<float>(4) << std::endl;

            mK = cv::Mat::eye(3, 3, CV_32F);
            mK.at<float>(0, 0) = fx;
            mK.at<float>(1, 1) = fy;
            mK.at<float>(0, 2) = cx;
            mK.at<float>(1, 2) = cy;

            mK_.setIdentity();
            mK_(0, 0) = fx;
            mK_(1, 1) = fy;
            mK_(0, 2) = cx;
            mK_(1, 2) = cy;
        }
        else if (sCameraName == "KannalaBrandt8")
        {
            float fx, fy, cx, cy;
            float k1, k2, k3, k4;
            mImageScale = 1.f;

            // Camera calibration parameters
            cv::FileNode node = fSettings["Camera.fx"];
            if (!node.empty() && node.isReal())
            {
                fx = node.real();
            }
            else
            {
                std::cerr << "*Camera.fx parameter doesn't exist or is not a real number*" << std::endl;
                b_miss_params = true;
            }
            node = fSettings["Camera.fy"];
            if (!node.empty() && node.isReal())
            {
                fy = node.real();
            }
            else
            {
                std::cerr << "*Camera.fy parameter doesn't exist or is not a real number*" << std::endl;
                b_miss_params = true;
            }

            node = fSettings["Camera.cx"];
            if (!node.empty() && node.isReal())
            {
                cx = node.real();
            }
            else
            {
                std::cerr << "*Camera.cx parameter doesn't exist or is not a real number*" << std::endl;
                b_miss_params = true;
            }

            node = fSettings["Camera.cy"];
            if (!node.empty() && node.isReal())
            {
                cy = node.real();
            }
            else
            {
                std::cerr << "*Camera.cy parameter doesn't exist or is not a real number*" << std::endl;
                b_miss_params = true;
            }

            // Distortion parameters
            node = fSettings["Camera.k1"];
            if (!node.empty() && node.isReal())
            {
                k1 = node.real();
            }
            else
            {
                std::cerr << "*Camera.k1 parameter doesn't exist or is not a real number*" << std::endl;
                b_miss_params = true;
            }
            node = fSettings["Camera.k2"];
            if (!node.empty() && node.isReal())
            {
                k2 = node.real();
            }
            else
            {
                std::cerr << "*Camera.k2 parameter doesn't exist or is not a real number*" << std::endl;
                b_miss_params = true;
            }

            node = fSettings["Camera.k3"];
            if (!node.empty() && node.isReal())
            {
                k3 = node.real();
            }
            else
            {
                std::cerr << "*Camera.k3 parameter doesn't exist or is not a real number*" << std::endl;
                b_miss_params = true;
            }

            node = fSettings["Camera.k4"];
            if (!node.empty() && node.isReal())
            {
                k4 = node.real();
            }
            else
            {
                std::cerr << "*Camera.k4 parameter doesn't exist or is not a real number*" << std::endl;
                b_miss_params = true;
            }

            node = fSettings["Camera.imageScale"];
            if (!node.empty() && node.isReal())
            {
                mImageScale = node.real();
            }

            if (!b_miss_params)
            {
                if (mImageScale != 1.f)
                {
                    // K matrix parameters must be scaled.
                    fx = fx * mImageScale;
                    fy = fy * mImageScale;
                    cx = cx * mImageScale;
                    cy = cy * mImageScale;
                }

                vector<float> vCamCalib{fx, fy, cx, cy, k1, k2, k3, k4};
                mpCamera = new KannalaBrandt8(vCamCalib);
                mpCamera = mpAtlas->AddCamera(mpCamera);
                std::cout << "- Camera: Fisheye" << std::endl;
                std::cout << "- Image scale: " << mImageScale << std::endl;
                std::cout << "- fx: " << fx << std::endl;
                std::cout << "- fy: " << fy << std::endl;
                std::cout << "- cx: " << cx << std::endl;
                std::cout << "- cy: " << cy << std::endl;
                std::cout << "- k1: " << k1 << std::endl;
                std::cout << "- k2: " << k2 << std::endl;
                std::cout << "- k3: " << k3 << std::endl;
                std::cout << "- k4: " << k4 << std::endl;

                mK = cv::Mat::eye(3, 3, CV_32F);
                mK.at<float>(0, 0) = fx;
                mK.at<float>(1, 1) = fy;
                mK.at<float>(0, 2) = cx;
                mK.at<float>(1, 2) = cy;

                mK_.setIdentity();
                mK_(0, 0) = fx;
                mK_(1, 1) = fy;
                mK_(0, 2) = cx;
                mK_(1, 2) = cy;
            }

            if (mSensor == System::STEREO || mSensor == System::IMU_STEREO || mSensor == System::IMU_RGBD)
            {
                // Right camera
                // Camera calibration parameters
                cv::FileNode node = fSettings["Camera2.fx"];
                if (!node.empty() && node.isReal())
                {
                    fx = node.real();
                }
                else
                {
                    std::cerr << "*Camera2.fx parameter doesn't exist or is not a real number*" << std::endl;
                    b_miss_params = true;
                }
                node = fSettings["Camera2.fy"];
                if (!node.empty() && node.isReal())
                {
                    fy = node.real();
                }
                else
                {
                    std::cerr << "*Camera2.fy parameter doesn't exist or is not a real number*" << std::endl;
                    b_miss_params = true;
                }

                node = fSettings["Camera2.cx"];
                if (!node.empty() && node.isReal())
                {
                    cx = node.real();
                }
                else
                {
                    std::cerr << "*Camera2.cx parameter doesn't exist or is not a real number*" << std::endl;
                    b_miss_params = true;
                }

                node = fSettings["Camera2.cy"];
                if (!node.empty() && node.isReal())
                {
                    cy = node.real();
                }
                else
                {
                    std::cerr << "*Camera2.cy parameter doesn't exist or is not a real number*" << std::endl;
                    b_miss_params = true;
                }

                // Distortion parameters
                node = fSettings["Camera2.k1"];
                if (!node.empty() && node.isReal())
                {
                    k1 = node.real();
                }
                else
                {
                    std::cerr << "*Camera2.k1 parameter doesn't exist or is not a real number*" << std::endl;
                    b_miss_params = true;
                }
                node = fSettings["Camera2.k2"];
                if (!node.empty() && node.isReal())
                {
                    k2 = node.real();
                }
                else
                {
                    std::cerr << "*Camera2.k2 parameter doesn't exist or is not a real number*" << std::endl;
                    b_miss_params = true;
                }

                node = fSettings["Camera2.k3"];
                if (!node.empty() && node.isReal())
                {
                    k3 = node.real();
                }
                else
                {
                    std::cerr << "*Camera2.k3 parameter doesn't exist or is not a real number*" << std::endl;
                    b_miss_params = true;
                }

                node = fSettings["Camera2.k4"];
                if (!node.empty() && node.isReal())
                {
                    k4 = node.real();
                }
                else
                {
                    std::cerr << "*Camera2.k4 parameter doesn't exist or is not a real number*" << std::endl;
                    b_miss_params = true;
                }

                int leftLappingBegin = -1;
                int leftLappingEnd = -1;

                int rightLappingBegin = -1;
                int rightLappingEnd = -1;

                node = fSettings["Camera.lappingBegin"];
                if (!node.empty() && node.isInt())
                {
                    leftLappingBegin = node.operator int();
                }
                else
                {
                    std::cout << "WARNING: Camera.lappingBegin not correctly defined" << std::endl;
                }
                node = fSettings["Camera.lappingEnd"];
                if (!node.empty() && node.isInt())
                {
                    leftLappingEnd = node.operator int();
                }
                else
                {
                    std::cout << "WARNING: Camera.lappingEnd not correctly defined" << std::endl;
                }
                node = fSettings["Camera2.lappingBegin"];
                if (!node.empty() && node.isInt())
                {
                    rightLappingBegin = node.operator int();
                }
                else
                {
                    std::cout << "WARNING: Camera2.lappingBegin not correctly defined" << std::endl;
                }
                node = fSettings["Camera2.lappingEnd"];
                if (!node.empty() && node.isInt())
                {
                    rightLappingEnd = node.operator int();
                }
                else
                {
                    std::cout << "WARNING: Camera2.lappingEnd not correctly defined" << std::endl;
                }

                node = fSettings["Tlr"];
                cv::Mat cvTlr;
                if (!node.empty())
                {
                    cvTlr = node.mat();
                    if (cvTlr.rows != 3 || cvTlr.cols != 4)
                    {
                        std::cerr << "*Tlr matrix have to be a 3x4 transformation matrix*" << std::endl;
                        b_miss_params = true;
                    }
                }
                else
                {
                    std::cerr << "*Tlr matrix doesn't exist*" << std::endl;
                    b_miss_params = true;
                }

                if (!b_miss_params)
                {
                    if (mImageScale != 1.f)
                    {
                        // K matrix parameters must be scaled.
                        fx = fx * mImageScale;
                        fy = fy * mImageScale;
                        cx = cx * mImageScale;
                        cy = cy * mImageScale;

                        leftLappingBegin = leftLappingBegin * mImageScale;
                        leftLappingEnd = leftLappingEnd * mImageScale;
                        rightLappingBegin = rightLappingBegin * mImageScale;
                        rightLappingEnd = rightLappingEnd * mImageScale;
                    }

                    static_cast<KannalaBrandt8 *>(mpCamera)->mvLappingArea[0] = leftLappingBegin;
                    static_cast<KannalaBrandt8 *>(mpCamera)->mvLappingArea[1] = leftLappingEnd;

                    mpFrameDrawer->both = true;

                    vector<float> vCamCalib2{fx, fy, cx, cy, k1, k2, k3, k4};
                    mpCamera2 = new KannalaBrandt8(vCamCalib2);
                    mpCamera2 = mpAtlas->AddCamera(mpCamera2);

                    mTlr = Converter::toSophus(cvTlr);

                    static_cast<KannalaBrandt8 *>(mpCamera2)->mvLappingArea[0] = rightLappingBegin;
                    static_cast<KannalaBrandt8 *>(mpCamera2)->mvLappingArea[1] = rightLappingEnd;

                    std::cout << "- Camera1 Lapping: " << leftLappingBegin << ", " << leftLappingEnd << std::endl;

                    std::cout << std::endl
                              << "Camera2 Parameters:" << std::endl;
                    std::cout << "- Camera: Fisheye" << std::endl;
                    std::cout << "- Image scale: " << mImageScale << std::endl;
                    std::cout << "- fx: " << fx << std::endl;
                    std::cout << "- fy: " << fy << std::endl;
                    std::cout << "- cx: " << cx << std::endl;
                    std::cout << "- cy: " << cy << std::endl;
                    std::cout << "- k1: " << k1 << std::endl;
                    std::cout << "- k2: " << k2 << std::endl;
                    std::cout << "- k3: " << k3 << std::endl;
                    std::cout << "- k4: " << k4 << std::endl;

                    std::cout << "- mTlr: \n"
                              << cvTlr << std::endl;

                    std::cout << "- Camera2 Lapping: " << rightLappingBegin << ", " << rightLappingEnd << std::endl;
                }
            }

            if (b_miss_params)
            {
                return false;
            }
        }
        else
        {
            std::cerr << "*Not Supported Camera Sensor*" << std::endl;
            std::cerr << "Check an example configuration file with the desired sensor" << std::endl;
        }

        if (mSensor == System::STEREO || mSensor == System::RGBD || mSensor == System::IMU_STEREO || mSensor == System::IMU_RGBD)
        {
            cv::FileNode node = fSettings["Camera.bf"];
            if (!node.empty() && node.isReal())
            {
                mbf = node.real();
                if (mImageScale != 1.f)
                {
                    mbf *= mImageScale;
                }
            }
            else
            {
                std::cerr << "*Camera.bf parameter doesn't exist or is not a real number*" << std::endl;
                b_miss_params = true;
            }
        }

        float fps = fSettings["Camera.fps"];
        if (fps == 0)
            fps = 30;

        // Max/Min Frames to insert keyframes and to check relocalisation
        mMinFrames = 0;
        mMaxFrames = fps;

        cout << "- fps: " << fps << endl;

        int nRGB = fSettings["Camera.RGB"];
        mbRGB = nRGB;

        if (mbRGB)
            cout << "- color order: RGB (ignored if grayscale)" << endl;
        else
            cout << "- color order: BGR (ignored if grayscale)" << endl;

        // ps：ThDepth 其实就是表示基线长度的多少倍，用来判断一个 3D 点远/近的阈值 mbf * 35 / fx
        if (mSensor == System::STEREO || mSensor == System::RGBD || mSensor == System::IMU_STEREO || mSensor == System::IMU_RGBD)
        {
            float fx = mpCamera->getParameter(0);
            cv::FileNode node = fSettings["ThDepth"];
            if (!node.empty() && node.isReal())
            {
                mThDepth = node.real();
                mThDepth = mbf * mThDepth / fx;
                cout << endl
                     << "Depth Threshold (Close/Far Points): " << mThDepth << endl;
            }
            else
            {
                std::cerr << "*ThDepth parameter doesn't exist or is not a real number*" << std::endl;
                b_miss_params = true;
            }
        }

        // ps：深度相机 disparity 转化为 depth 时的因子，这个地方出过很大问题！
        if (mSensor == System::RGBD || mSensor == System::IMU_RGBD)
        {
            cv::FileNode node = fSettings["DepthMapFactor"];
            if (!node.empty() && node.isReal())
            {
                mDepthMapFactor = node.real();
                if (fabs(mDepthMapFactor) < 1e-5)
                    mDepthMapFactor = 1;
                else
                    mDepthMapFactor = 1.0f / mDepthMapFactor;
            }
            else
            {
                std::cerr << "*DepthMapFactor parameter doesn't exist or is not a real number*" << std::endl;
                b_miss_params = true;
            }
        }

        if (b_miss_params)
        {
            return false;
        }

        return true;
    }

    // 根据文件读取特征点参数，可快速略过不看
    bool Tracking::ParseORBParamFile(cv::FileStorage &fSettings)
    {
        bool b_miss_params = false;
        int nFeatures, nLevels, fIniThFAST, fMinThFAST;
        float fScaleFactor;

        // 每一帧提取的特征点数：1000
        cv::FileNode node = fSettings["ORBextractor.nFeatures"];
        if (!node.empty() && node.isInt())
        {
            nFeatures = node.operator int();
        }
        else
        {
            std::cerr << "*ORBextractor.nFeatures parameter doesn't exist or is not an integer*" << std::endl;
            b_miss_params = true;
        }

        // 图像建立金字塔时的变化尺度：1.2
        node = fSettings["ORBextractor.scaleFactor"];
        if (!node.empty() && node.isReal())
        {
            fScaleFactor = node.real();
        }
        else
        {
            std::cerr << "*ORBextractor.scaleFactor parameter doesn't exist or is not a real number*" << std::endl;
            b_miss_params = true;
        }

        // 尺度金字塔的层数：8
        node = fSettings["ORBextractor.nLevels"];
        if (!node.empty() && node.isInt())
        {
            nLevels = node.operator int();
        }
        else
        {
            std::cerr << "*ORBextractor.nLevels parameter doesn't exist or is not an integer*" << std::endl;
            b_miss_params = true;
        }

        // 提取fast特征点的默认阈值：20
        node = fSettings["ORBextractor.iniThFAST"];
        if (!node.empty() && node.isInt())
        {
            fIniThFAST = node.operator int();
        }
        else
        {
            std::cerr << "*ORBextractor.iniThFAST parameter doesn't exist or is not an integer*" << std::endl;
            b_miss_params = true;
        }

        // 如果默认阈值提取不出足够fast特征点，则使用最小阈值：8
        node = fSettings["ORBextractor.minThFAST"];
        if (!node.empty() && node.isInt())
        {
            fMinThFAST = node.operator int();
        }
        else
        {
            std::cerr << "*ORBextractor.minThFAST parameter doesn't exist or is not an integer*" << std::endl;
            b_miss_params = true;
        }

        if (b_miss_params)
        {
            return false;
        }

        // tracking 过程都会用到 mpORBextractorLeft 作为特征点提取器
        mpORBextractorLeft = new ORBextractor(nFeatures, fScaleFactor, nLevels, fIniThFAST, fMinThFAST);

        // 如果是双目，tracking 过程中还会用用到 mpORBextractorRight 作为右目特征点提取器
        if (mSensor == System::STEREO || mSensor == System::IMU_STEREO)
            mpORBextractorRight = new ORBextractor(nFeatures, fScaleFactor, nLevels, fIniThFAST, fMinThFAST);

        // 在单目初始化的时候，会用 mpIniORBextractor 来作为特征点提取器
        if (mSensor == System::MONOCULAR || mSensor == System::IMU_MONOCULAR)
            mpIniORBextractor = new ORBextractor(5 * nFeatures, fScaleFactor, nLevels, fIniThFAST, fMinThFAST);

        cout << endl
             << "ORB Extractor Parameters: " << endl;
        cout << "- Number of Features: " << nFeatures << endl;
        cout << "- Scale Levels: " << nLevels << endl;
        cout << "- Scale Factor: " << fScaleFactor << endl;
        cout << "- Initial Fast Threshold: " << fIniThFAST << endl;
        cout << "- Minimum Fast Threshold: " << fMinThFAST << endl;

        return true;
    }

    // 根据文件读取 IMU 参数，可快速略过不看
    bool Tracking::ParseIMUParamFile(cv::FileStorage &fSettings)
    {
        bool b_miss_params = false;

        cv::Mat cvTbc;
        cv::FileNode node = fSettings["Tbc"];
        if (!node.empty())
        {
            cvTbc = node.mat();
            if (cvTbc.rows != 4 || cvTbc.cols != 4)
            {
                std::cerr << "*Tbc matrix have to be a 4x4 transformation matrix*" << std::endl;
                b_miss_params = true;
            }
        }
        else
        {
            std::cerr << "*Tbc matrix doesn't exist*" << std::endl;
            b_miss_params = true;
        }
        cout << endl;
        cout << "Left camera to Imu Transform (Tbc): " << endl
             << cvTbc << endl;
        Eigen::Matrix<float, 4, 4, Eigen::RowMajor> eigTbc(cvTbc.ptr<float>(0));
        Sophus::SE3f Tbc(eigTbc);

        node = fSettings["InsertKFsWhenLost"];
        mInsertKFsLost = true;
        if (!node.empty() && node.isInt())
        {
            mInsertKFsLost = (bool)node.operator int();
        }

        if (!mInsertKFsLost)
            cout << "Do not insert keyframes when lost visual tracking " << endl;

        float Ng, Na, Ngw, Naw;

        node = fSettings["IMU.Frequency"];
        if (!node.empty() && node.isInt())
        {
            mImuFreq = node.operator int();
            mImuPer = 0.001; // 1.0 / (double) mImuFreq;
        }
        else
        {
            std::cerr << "*IMU.Frequency parameter doesn't exist or is not an integer*" << std::endl;
            b_miss_params = true;
        }

        node = fSettings["IMU.NoiseGyro"];
        if (!node.empty() && node.isReal())
        {
            Ng = node.real();
        }
        else
        {
            std::cerr << "*IMU.NoiseGyro parameter doesn't exist or is not a real number*" << std::endl;
            b_miss_params = true;
        }

        node = fSettings["IMU.NoiseAcc"];
        if (!node.empty() && node.isReal())
        {
            Na = node.real();
        }
        else
        {
            std::cerr << "*IMU.NoiseAcc parameter doesn't exist or is not a real number*" << std::endl;
            b_miss_params = true;
        }

        node = fSettings["IMU.GyroWalk"];
        if (!node.empty() && node.isReal())
        {
            Ngw = node.real();
        }
        else
        {
            std::cerr << "*IMU.GyroWalk parameter doesn't exist or is not a real number*" << std::endl;
            b_miss_params = true;
        }

        node = fSettings["IMU.AccWalk"];
        if (!node.empty() && node.isReal())
        {
            Naw = node.real();
        }
        else
        {
            std::cerr << "*IMU.AccWalk parameter doesn't exist or is not a real number*" << std::endl;
            b_miss_params = true;
        }

        node = fSettings["IMU.fastInit"];
        mFastInit = false;
        if (!node.empty())
        {
            mFastInit = static_cast<int>(fSettings["IMU.fastInit"]) != 0;
        }

        if (mFastInit)
            cout << "Fast IMU initialization. Acceleration is not checked \n";

        if (b_miss_params)
        {
            return false;
        }

        const float sf = sqrt(mImuFreq);
        cout << endl;
        cout << "IMU frequency: " << mImuFreq << " Hz" << endl;
        cout << "IMU gyro noise: " << Ng << " rad/s/sqrt(Hz)" << endl;
        cout << "IMU gyro walk: " << Ngw << " rad/s^2/sqrt(Hz)" << endl;
        cout << "IMU accelerometer noise: " << Na << " m/s^2/sqrt(Hz)" << endl;
        cout << "IMU accelerometer walk: " << Naw << " m/s^3/sqrt(Hz)" << endl;

        mpImuCalib = new IMU::Calib(Tbc, Ng * sf, Na * sf, Ngw / sf, Naw / sf);

        mpImuPreintegratedFromLastKF = new IMU::Preintegrated(IMU::Bias(), *mpImuCalib);

        return true;
    }

    // -----------------------------------------------------------------------------------------------------------------------------

    // notice：设置局部建图器
    void Tracking::SetLocalMapper(LocalMapping *pLocalMapper)
    {
        mpLocalMapper = pLocalMapper;
    }

    // notice：设置回环检测器
    void Tracking::SetLoopClosing(LoopClosing *pLoopClosing)
    {
        mpLoopClosing = pLoopClosing;
    }

    // notice：设置可视化显示器
    void Tracking::SetViewer(Viewer *pViewer)
    {
        mpViewer = pViewer;
    }

    // TODO 设置动态物体检测器
    void Tracking::SetDetector(YoloDetection *pDetector)
    {
        mpDetector = pDetector;
    }

    // ----------------------------------------------- 这两个一般是在调试的时候用的 ---------------------------------------------------

    // 设置是否进入逐步执行模式
    void Tracking::SetStepByStep(bool bSet)
    {
        bStepByStep = bSet;
    }

    // 获取当前是否在逐步执行模式
    bool Tracking::GetStepByStep()
    {
        return bStepByStep;
    }

    // ----------------------------------- 获取相机捕捉的每一帧图像，并都封装成一个 Frame 对象 -----------------------------------------

    // notice1：获取双目相机图像 -- 将获取的双目图像封装成一个 Frame 对象
    /**
     * @brief 输入：左右目图像，可以为 RGB、BGR、RGBA、GRAY
     *          1、将图像转为 mImGray 和 imGrayRight 并初始化 mCurrentFrame
     *          2、进行 tracking 过程
     *        输出：世界坐标系到该帧相机坐标系的变换矩阵
     *
     * @param imRectLeft 左目图像
     * @param imRectRight 右目图像
     * @param timestamp 时间戳
     * @param filename 文件名字，貌似调试用的
     */
    Sophus::SE3f Tracking::GrabImageStereo(const cv::Mat &imRectLeft, const cv::Mat &imRectRight, const double &timestamp, string filename)
    {
        // cout << "GrabImageStereo" << endl;

        // ? 这里是不是冗余了？还需要中间变量 imGrayRight 吗？
        mImGray = imRectLeft;
        // ! 这里使用中间变量 imGrayRight，gpt 说可能是为了后续处理时不直接修改原始图像 imRectRight，避免引入副作用。
        cv::Mat imGrayRight = imRectRight; // 使用中间变量 imGrayRight 可以增加代码的可读性，并防止在后续代码中误修改原始传入的 imRectRight 图像。
        mImRight = imRectRight;

        // step 1 ：将 RGB、BGR 或 RGBA 图像转为灰度图像
        if (mImGray.channels() == 3)
        {
            // cout << "Image with 3 channels" << endl;
            if (mbRGB)
            {
                cvtColor(mImGray, mImGray, cv::COLOR_RGB2GRAY);
                cvtColor(imGrayRight, imGrayRight, cv::COLOR_RGB2GRAY);
            }
            else
            {
                cvtColor(mImGray, mImGray, cv::COLOR_BGR2GRAY);
                cvtColor(imGrayRight, imGrayRight, cv::COLOR_BGR2GRAY);
            }
        }
        // 这里考虑得十分周全，甚至连四通道的图像都考虑到了
        else if (mImGray.channels() == 4)
        {
            // cout << "Image with 4 channels" << endl;
            if (mbRGB)
            {
                cvtColor(mImGray, mImGray, cv::COLOR_RGBA2GRAY);
                cvtColor(imGrayRight, imGrayRight, cv::COLOR_RGBA2GRAY);
            }
            else
            {
                cvtColor(mImGray, mImGray, cv::COLOR_BGRA2GRAY);
                cvtColor(imGrayRight, imGrayRight, cv::COLOR_BGRA2GRAY);
            }
        }

        // cout << "Incoming frame creation" << endl;

        // Step 2 ：构造 Frame，就是将输入的图像数据和其他信息（如相机参数、时间戳等）封装成一个 Frame 对象
        //  双目模式，注意跟两个相机模式区分开
        // 如：ZED 2i 属于无第二相机的双目设备，在 ORB-SLAM3 中，mpCamera2 是 NULL
        if (mSensor == System::STEREO && !mpCamera2)
            mCurrentFrame = Frame(mImGray, imGrayRight, timestamp, mpORBextractorLeft, mpORBextractorRight, mpORBVocabulary, mK, mDistCoef, mbf, mThDepth, mpCamera);
        else if (mSensor == System::STEREO && mpCamera2)
            mCurrentFrame = Frame(mImGray, imGrayRight, timestamp, mpORBextractorLeft, mpORBextractorRight, mpORBVocabulary, mK, mDistCoef, mbf, mThDepth, mpCamera, mpCamera2, mTlr);

        else if (mSensor == System::IMU_STEREO && !mpCamera2)
            mCurrentFrame = Frame(mImGray, imGrayRight, timestamp, mpORBextractorLeft, mpORBextractorRight, mpORBVocabulary, mK, mDistCoef, mbf, mThDepth, mpCamera, &mLastFrame, *mpImuCalib);
        else if (mSensor == System::IMU_STEREO && mpCamera2)
            mCurrentFrame = Frame(mImGray, imGrayRight, timestamp, mpORBextractorLeft, mpORBextractorRight, mpORBVocabulary, mK, mDistCoef, mbf, mThDepth, mpCamera, mpCamera2, mTlr, &mLastFrame, *mpImuCalib);

        // cout << "Incoming frame ended" << endl;

        // 调试信息：将当前帧的文件名和数据集编号记录到 mCurrentFrame，这些信息可能用于调试或者后期分析。
        mCurrentFrame.mNameFile = filename;
        mCurrentFrame.mnDataset = mnNumDataset;

#ifdef REGISTER_TIMES
        vdORBExtract_ms.push_back(mCurrentFrame.mTimeORB_Ext);
        vdStereoMatch_ms.push_back(mCurrentFrame.mTimeStereoMatch);
#endif

        cout << "Tracking start" << endl;

        // Step 3 ：跟踪
        Track();

        cout << "Tracking end" << endl;

        return mCurrentFrame.GetPose();
    }

    // notice2：获取 RGBD 图像
    /**
     * @brief 输入：左目 RGB 或 RGBA 图像和深度图
     *          1、将图像转为 mImGray 和 imDepth 并初始化 mCurrentFrame
     *          2、进行 tracking 过程
     *        输出：世界坐标系到该帧相机坐标系的变换矩阵
     *
     * @param imRGB 彩色图
     * @param imD 深度图
     * @param timestamp 时间戳
     * @param filename 文件名字，貌似调试用的
     */
    Sophus::SE3f Tracking::GrabImageRGBD(const cv::Mat &imRGB, const cv::Mat &imD, const double &timestamp, string filename)
    {
        mImGray = imRGB;
        mImColor = imRGB.clone(); // TODO Yolo 用的，存储彩色图像，后续进行 YOLO 目标检测时需要彩色图像
        cv::Mat imDepth = imD;    // ! 又是中间变量

        // TODO 目标检测（YOLO）
        cv::Mat InputImage;
        InputImage = imRGB.clone();
        mpDetector->GetImage(InputImage);
        mpDetector->Detect();
        mpORBextractorLeft->mvDynamicArea = mpDetector->mvDynamicArea;
        {
            std::unique_lock<std::mutex> lock(mpViewer->mMutexPAFinsh);
            mpViewer->mmDetectMap = mpDetector->mmDetectMap;
        }

        // step 1：将 RGB、BGR 或 RGBA、BGRA 图像转为灰度图像
        if (mImGray.channels() == 3)
        {
            if (mbRGB)
                cvtColor(mImGray, mImGray, cv::COLOR_RGB2GRAY);
            else
                cvtColor(mImGray, mImGray, cv::COLOR_BGR2GRAY);
        }
        else if (mImGray.channels() == 4)
        {
            if (mbRGB)
                cvtColor(mImGray, mImGray, cv::COLOR_RGBA2GRAY);
            else
                cvtColor(mImGray, mImGray, cv::COLOR_BGRA2GRAY);
        }

        // Step 2 ：将深度相机的 disparity 转为 Depth , 以 米 为单位的深度值，也就是转换成为真正尺度下的深度
        if ((fabs(mDepthMapFactor - 1.0f) > 1e-5) || imDepth.type() != CV_32F) // fabs -- floating-point absolute value，即浮点数的绝对值
            imDepth.convertTo(imDepth, CV_32F, mDepthMapFactor);               // mDepthMapFactor --- 缩放系数。如果相机输出的深度图单位是毫米，而后续处理需要以米为单位，那 mDepthMapFactor 就是 0.001（因为 1mm = 0.001m）

        mDepth = imDepth.clone(); // ! mDepth 是第二个中间变量了

        // Step 3：构造 Frame，就是把这张图像变为一个帧
        if (mSensor == System::RGBD)
            mCurrentFrame = Frame(mImGray, imDepth, timestamp, mpORBextractorLeft, mpORBVocabulary, mK, mDistCoef, mbf, mThDepth, mpCamera);
        else if (mSensor == System::IMU_RGBD)
            mCurrentFrame = Frame(mImGray, imDepth, timestamp, mpORBextractorLeft, mpORBVocabulary, mK, mDistCoef, mbf, mThDepth, mpCamera, &mLastFrame, *mpImuCalib);

        // TODO 将目标检测器（YOLO）中检测到的动态区域信息传递给当前帧 mCurrentFrame
        mCurrentFrame.mvDynamicArea = mpDetector->mvDynamicArea;

        mpDetector->mmDetectMap.clear();
        mpDetector->mvDynamicArea.clear();

        // 调试信息：将当前帧的文件名和数据集编号记录到 mCurrentFrame，这些信息可能用于调试或者后期分析。
        mCurrentFrame.mNameFile = filename;
        mCurrentFrame.mnDataset = mnNumDataset;

#ifdef REGISTER_TIMES
        vdORBExtract_ms.push_back(mCurrentFrame.mTimeORB_Ext);
#endif

        // Step 4：跟踪
        Track();

        // 返回当前帧的位姿
        return mCurrentFrame.GetPose();
    }

    // notice3：获取单目图像
    /**
     * @brief 输入：左目 RGB 或 RGBA 图像，输出：世界坐标系到该帧相机坐标系的变换矩阵
     *
     * @param im 图像
     * @param timestamp 时间戳
     * @param filename 文件名字，貌似调试用的
     *
     * Step 1 ：将彩色图像转为灰度图像
     * Step 2 ：构造Frame
     * Step 3 ：跟踪
     */
    Sophus::SE3f Tracking::GrabImageMonocular(const cv::Mat &im, const double &timestamp, string filename)
    {
        mImGray = im; // 为了让 mImGray 能调用其余的函数，如果单纯用 im 肯定是不行的。

        // Step 1 ：将彩色图像转为灰度图像
        if (mImGray.channels() == 3)
        {
            if (mbRGB)
                cvtColor(mImGray, mImGray, cv::COLOR_RGB2GRAY);
            else
                cvtColor(mImGray, mImGray, cv::COLOR_BGR2GRAY);
        }
        else if (mImGray.channels() == 4)
        {
            if (mbRGB)
                cvtColor(mImGray, mImGray, cv::COLOR_RGBA2GRAY);
            else
                cvtColor(mImGray, mImGray, cv::COLOR_BGRA2GRAY);
        }

        // Step 2 ：构造 Frame
        if (mSensor == System::MONOCULAR)
        {
            // case1：如果系统尚未初始化 or 还没有接收到图像数据 or 当前帧数还不到最大帧数，就用一个专门的 mpIniORBextractor 提取器来处理图像：
            if (mState == NOT_INITIALIZED || mState == NO_IMAGES_YET || (lastID - initID) < mMaxFrames)
                mCurrentFrame = Frame(mImGray, timestamp, mpIniORBextractor, mpORBVocabulary, mpCamera, mDistCoef, mbf, mThDepth);

            // case2：否则，就使用常规的 mpORBextractorLeft 特征提取器来处理图像：
            else
                mCurrentFrame = Frame(mImGray, timestamp, mpORBextractorLeft, mpORBVocabulary, mpCamera, mDistCoef, mbf, mThDepth);
        }
        else if (mSensor == System::IMU_MONOCULAR)
        {
            if (mState == NOT_INITIALIZED || mState == NO_IMAGES_YET)
            {
                // 使用带 IMU 初始化的特征提取器：
                mCurrentFrame = Frame(mImGray, timestamp, mpIniORBextractor, mpORBVocabulary, mpCamera, mDistCoef, mbf, mThDepth, &mLastFrame, *mpImuCalib);
            }
            else
                // 使用带 IMU 的常规特征提取器：
                mCurrentFrame = Frame(mImGray, timestamp, mpORBextractorLeft, mpORBVocabulary, mpCamera, mDistCoef, mbf, mThDepth, &mLastFrame, *mpImuCalib);
        }

        // 如果系统处于 NO_IMAGES_YET 状态，表示这是系统的第一帧图像，就把这帧的时间戳 timestamp 记录下来
        // ? 提问：为什么只有单目模式记录时间戳？答：因为其依赖帧间时间计算运动，单目SLAM必须通过时间戳来知道帧与帧之间的间隔，以便进行运动模型预测（如恒速模型）
        if (mState == NO_IMAGES_YET)
            t0 = timestamp;

        mCurrentFrame.mNameFile = filename;
        mCurrentFrame.mnDataset = mnNumDataset;

#ifdef REGISTER_TIMES
        vdORBExtract_ms.push_back(mCurrentFrame.mTimeORB_Ext);
#endif

        // 更新 lastID，表示当前帧是第 lastID 帧
        lastID = mCurrentFrame.mnId;

        // Step 3 ：跟踪
        Track();

        // 返回当前帧的位姿
        return mCurrentFrame.GetPose();
    }

    // notice4：获取 IMU 数据 -- 将 imu 数据存放在 mlQueueImuData 的 list链表 里
    void Tracking::GrabImuData(const IMU::Point &imuMeasurement)
    {
        unique_lock<mutex> lock(mMutexImuQueue); // * unique_lock 是比 lock_guard 更灵活的锁机制，适用于需要锁定和解锁操作的情况
        mlQueueImuData.push_back(imuMeasurement);
    }

    // -----------------------------------------------------------------------------------------------------------------------------

    // notice：预积分，对于一个帧有两种预积分，一种是相对于上一【普通帧】，一种是相对于上一个【关键帧】
    void Tracking::PreintegrateIMU()
    {
        // step1. 拿到两两帧之间待处理的预积分数据，组成一个集合
        // 首先检查 mCurrentFrame 是否有上一帧 mpPrevFrame。如果没有上一帧，表示当前帧是第一帧或者没有可用的前一帧，说明两帧之间没有 imu 数据，系统就会返回并跳过当前帧的IMU预积分过程。
        if (!mCurrentFrame.mpPrevFrame)
        {
            Verbose::PrintMess("non prev frame ", Verbose::VERBOSITY_NORMAL);
            mCurrentFrame.setIntegrated(); // ? 这里调用这个函数是为什么？是直接跳过积分？还是设置为已经做完积分？答：并不是跳过积分，而是说明积分操作已经完成。
            return;
        }

        // step2. 清空 mvImuFromLastFrame 并预分配内存空间，以便存储从IMU数据队列中提取的IMU数据
        // ? mvImuFromLastFrame 存的是 上一帧（不一定是 KF） ---> 当前帧 的 IMU 数据。
        mvImuFromLastFrame.clear();
        mvImuFromLastFrame.reserve(mlQueueImuData.size());

        // step3. 检查IMU数据队列
        // 没有 imu 数据，不进行预积分
        if (mlQueueImuData.size() == 0)
        {
            Verbose::PrintMess("Not IMU data in mlQueueImuData!!", Verbose::VERBOSITY_NORMAL);
            mCurrentFrame.setIntegrated();
            return;
        }

        // step4. 从IMU队列中提取数据
        while (true)
        {
            // 数据还没有时，会等待一段时间，直到 mlQueueImuData 中有 imu 数据。一开始不需要等待
            bool bSleep = false;
            {
                unique_lock<mutex> lock(mMutexImuQueue);

                // case1：当前有 IMU 数据
                if (!mlQueueImuData.empty())
                {
                    // 拿到第一个 imu 数据作为起始数据
                    IMU::Point *FirstImuData = &mlQueueImuData.front();
                    cout.precision(17);

                    // 情况一：imu 起始数据会比当前帧的前一帧时间戳早，如果相差 mImuPer = 0.001，则舍弃这个 imu 数据
                    // ? 为什么是比较 imu起始数据 和 当前帧的前一帧？而不是和当前帧时间戳比较？
                    // 正常来说，合理范围应该是：上一帧时间戳 < 当前IMU数据时间戳 < 当前帧时间戳，但IMU 数据的时间戳不一定精确地对应每一帧的时间戳。因此，
                    // ! 为了确保我们能正确利用 IMU 数据，系统会放宽一些时间（mImuPer）容忍度。所以变为：【上一帧时间戳 - mImuPer < 当前IMU数据时间戳 < 当前帧时间戳】
                    if (FirstImuData->t < mCurrentFrame.mpPrevFrame->mTimeStamp - mImuPer)
                    {
                        mlQueueImuData.pop_front();
                    }

                    // 情况二：同样最后一个的 imu 数据时间戳也不能比当前帧时间间隔多余 mImuPer = 0.001
                    // 也就是，最后一个 imu 数据，要在最后一个 帧 前面，相差时间为 mImuPer
                    else if (FirstImuData->t < mCurrentFrame.mTimeStamp - mImuPer)
                    {
                        mvImuFromLastFrame.push_back(*FirstImuData); // 把当前有效的 IMU 数据添加到 mvImuFromLastFrame 中
                        mlQueueImuData.pop_front();                  // 从 IMU 数据队列中移除当前数据
                    }
                    // ? 为什么不考虑当前时间戳之后 mImuPer 范围内的 IMU 数据？新增条件👇
                    else if (FirstImuData->t >= mCurrentFrame.mTimeStamp + mImuPer)
                    {
                        // 如果 IMU 数据时间戳比当前帧时间戳多出 mImuPer 时间，就跳过。
                        mlQueueImuData.pop_front();
                    }
                    else
                    {
                        // 得到两帧间的 imu 数据放入 mvImuFromLastFrame 中，得到后面预积分的处理数据
                        mvImuFromLastFrame.push_back(*FirstImuData);
                        break;
                    }
                }
                // case2：当前没有 IMU 数据
                else
                {
                    break;
                    bSleep = true;
                }
            }

            // 等待一段时间，再重新循环，检查有没有 IMU 数据
            if (bSleep)
                usleep(500);
        }

        // Step 5. IMU预积分
        const int n = mvImuFromLastFrame.size() - 1; // m 个 imu 组数据会有 m-1 个预积分量
        if (n == 0)
        {
            cout << "Empty IMU measurements vector!!!\n";
            return;
        }

        // ps：构造imu预处理器，并初始化标定数据，pImuPreintegratedFromLastFrame --- 存储从上一普通帧到当前帧之间的预积分结果
        IMU::Preintegrated *pImuPreintegratedFromLastFrame = new IMU::Preintegrated(mLastFrame.mImuBias, mCurrentFrame.mImuCalib); // 创建一个新的 Preintegrated 对象，用于存储当前帧相对于上一帧的预积分数据。

        // 针对预积分位置的不同做不同中值积分的处理
        /**
         *  根据上面imu帧的筛选，IMU与图像帧的时序如下：
         *  Frame---IMU0---IMU1---IMU2---IMU3---IMU4---------------IMUx---Frame---IMUx+1
         *  T_------T0-----T1-----T2-----T3-----T4-----------------Tx-----_T------Tx+1
         *  A_------A0-----A1-----A2-----A3-----A4-----------------Ax-----_T------Ax+1
         *  W_------W0-----W1-----W2-----W3-----W4-----------------Wx-----_T------Wx+1
         *  T_和_T分别表示上一图像帧和当前图像帧的时间戳，A(加速度数据)，W(陀螺仪数据)，同理
         */

        // step6. 预积分计算
        for (int i = 0; i < n; i++)
        {
            float tstep;
            Eigen::Vector3f acc, angVel;

            // 遍历从上一帧到当前帧之间的所有 IMU 数据👇
            // case1：处理第一帧数据，但不是最后两帧，imu 总帧数大于2
            if ((i == 0) && (i < (n - 1)))
            {
                // 获取相邻两次 imu 的时间间隔
                float tab = mvImuFromLastFrame[i + 1].t - mvImuFromLastFrame[i].t;

                // 获取当前 imu 到上一帧的时间间隔
                float tini = mvImuFromLastFrame[i].t - mCurrentFrame.mpPrevFrame->mTimeStamp;

                // 由于IMU数据的时间戳并不一定与上一帧的时间戳完全对齐，因此需要进行补偿。通过线性插值的方式（a0 - (a1-a0)*(tini/tab)），计算上一帧时刻的加速度和角速度。这个补偿的过程是为了得到上一帧时刻的IMU数据。
                //  设当前时刻 imu 的加速度a0，下一时刻加速度a1，时间间隔 tab 为t10，tini t0p
                //  正常情况下时为了求上一帧到当前时刻imu的一个平均加速度，但是imu时间不会正好落在上一帧的时刻，需要做补偿，要求得a0时刻到上一帧这段时间加速度的改变量
                //  有了这个改变量将其加到a0上之后就可以表示上一帧时的加速度了。其中a0 - (a1-a0)*(tini/tab) 为上一帧时刻的加速度再加上a1 之后除以2就为这段时间的加速度平均值
                //  其中tstep表示a1到上一帧的时间间隔，a0 - (a1-a0)*(tini/tab)这个式子中tini可以是正也可以是负表示时间上的先后，(a1-a0)也是一样，多种情况下这个式子依然成立

                // 计算上一帧时的加速度
                acc = (mvImuFromLastFrame[i].a + mvImuFromLastFrame[i + 1].a - (mvImuFromLastFrame[i + 1].a - mvImuFromLastFrame[i].a) * (tini / tab)) * 0.5f;
                // 计算上一帧时角速度
                angVel = (mvImuFromLastFrame[i].w + mvImuFromLastFrame[i + 1].w - (mvImuFromLastFrame[i + 1].w - mvImuFromLastFrame[i].w) * (tini / tab)) * 0.5f;
                // 计算从上一帧到当前 IMU 数据的时间间隔，用于后续的预积分计算
                tstep = mvImuFromLastFrame[i + 1].t - mCurrentFrame.mpPrevFrame->mTimeStamp;
            }
            // case2：处理中间数据
            else if (i < (n - 1))
            {
                // 中间的数据不存在帧的干扰，正常计算
                acc = (mvImuFromLastFrame[i].a + mvImuFromLastFrame[i + 1].a) * 0.5f;
                angVel = (mvImuFromLastFrame[i].w + mvImuFromLastFrame[i + 1].w) * 0.5f;
                // 获取当前IMU数据和前一个IMU数据之间的时间间隔
                tstep = mvImuFromLastFrame[i + 1].t - mvImuFromLastFrame[i].t;
            }
            // case3：处理倒数第二个 IMU 数据，，计算过程跟第一时刻类似，都需要考虑当前帧与 imu 时刻的关系
            else if ((i > 0) && (i == (n - 1)))
            {
                float tab = mvImuFromLastFrame[i + 1].t - mvImuFromLastFrame[i].t;
                float tend = mvImuFromLastFrame[i + 1].t - mCurrentFrame.mTimeStamp;
                acc = (mvImuFromLastFrame[i].a + mvImuFromLastFrame[i + 1].a - (mvImuFromLastFrame[i + 1].a - mvImuFromLastFrame[i].a) * (tend / tab)) * 0.5f;
                angVel = (mvImuFromLastFrame[i].w + mvImuFromLastFrame[i + 1].w - (mvImuFromLastFrame[i + 1].w - mvImuFromLastFrame[i].w) * (tend / tab)) * 0.5f;

                // 计算当前帧与最后一个IMU数据之间的时间间隔
                tstep = mCurrentFrame.mTimeStamp - mvImuFromLastFrame[i].t;
            }
            // case4：仅有一帧数据时的处理
            // ! 就两个数据时使用第一个时刻的，这种情况应该没有吧，，回头应该试试看
            else if ((i == 0) && (i == (n - 1)))
            {
                // 直接取该IMU数据的加速度和角速度
                acc = mvImuFromLastFrame[i].a;
                angVel = mvImuFromLastFrame[i].w;
                // tstep 是当前帧与上一帧的时间差
                tstep = mCurrentFrame.mTimeStamp - mCurrentFrame.mpPrevFrame->mTimeStamp;
            }

            // 依次进行预积分计算
            // ? 应该是必存在的吧，一个是相对上一关键帧，一个是相对上一帧
            if (!mpImuPreintegratedFromLastKF)
                cout << "mpImuPreintegratedFromLastKF does not exist" << endl;

            mpImuPreintegratedFromLastKF->IntegrateNewMeasurement(acc, angVel, tstep);   // ps：计算当前帧与上一【关键帧】之间的预积分
            pImuPreintegratedFromLastFrame->IntegrateNewMeasurement(acc, angVel, tstep); // ps：计算当前帧与上一【普通帧】之间的预积分
        }

        // step7. 完成IMU预积分，记录当前预积分的图像帧
        // ? 为什么 pImuPreintegratedFromLastFrame 是局部变量，而通过它积分的 IMU 数据 mpImuPreintegratedFrame 却是成员变量？
        // 答：因为这是一个 长期有效 的数据，它会被用于后续的图像跟踪、状态估计、优化等任务中。因此，它需要作为成员变量来存储。
        // 而 pImuPreintegratedFromLastFrame 只是临时计算需要的一个中间结果。只需要它在当前函数内使用，计算结束后它就不再需要，因此它不需要作为成员变量。
        mCurrentFrame.mpImuPreintegratedFrame = pImuPreintegratedFromLastFrame;
        mCurrentFrame.mpImuPreintegrated = mpImuPreintegratedFromLastKF;
        mCurrentFrame.mpLastKeyFrame = mpLastKeyFrame;

        mCurrentFrame.setIntegrated(); // 调用 setIntegrated() 方法标记当前帧的IMU预积分完成

        Verbose::PrintMess("Preintegration is finished!! ", Verbose::VERBOSITY_DEBUG);
    }

    // notice：利用上一帧的IMU数据（位姿、速度等信息）和预积分数据（如加速度、角速度等），推算出当前帧的位置、速度和姿态
    /**
     * @brief 跟踪不成功的时候，用初始化好的 imu 数据做跟踪处理，通过 IMU 预测状态
     * 两个地方用到：
     *      1. 匀速模型计算速度，但并没有给当前帧位姿赋值；
     *      2. 跟踪丢失时不直接判定丢失，通过这个函数预测当前帧位姿看看能不能拽回来，代替纯视觉中的重定位
     *
     * @return true
     * @return false
     */
    bool Tracking::PredictStateIMU()
    {
        // step1：判断是否存在上一帧
        if (!mCurrentFrame.mpPrevFrame)
        {
            Verbose::PrintMess("No last frame", Verbose::VERBOSITY_NORMAL);
            return false;
        }

        // 总结下都在什么时候地图更文件读取IMU参数，可快速略过不新，也就是mbMapUpdated为true
        // 1. 回环或融合
        // 2. 局部地图LocalBundleAdjustment
        // 3. IMU三阶段的初始化
        // 下面的代码流程一模一样，只不过计算时相对的帧不同，地图有更新后相对于上一关键帧做的，反之相对于上一帧
        // 地图更新后会更新关键帧与MP，所以相对于关键帧更准
        // 而没更新的话，距离上一帧更近，计算起来误差更小
        // 地图更新时，并且上一个图像关键帧存在

        // step2：当地图更新且有上一关键帧（mpLastKeyFrame）时：
        // ! 地图更新时，会使用上一【关键帧】的信息来进行 IMU 预测，关键帧的精度较高，因此能提供更准确的状态预测。
        if (mbMapUpdated && mpLastKeyFrame)
        {
            // 通过上一关键帧的信息（IMU的位置、旋转矩阵、速度）来推算当前帧的状态
            const Eigen::Vector3f twb1 = mpLastKeyFrame->GetImuPosition();
            const Eigen::Matrix3f Rwb1 = mpLastKeyFrame->GetImuRotation();
            const Eigen::Vector3f Vwb1 = mpLastKeyFrame->GetVelocity();

            const Eigen::Vector3f Gz(0, 0, -IMU::GRAVITY_VALUE); // 使用IMU的重力值（Gz）来修正加速度信息
            const float t12 = mpImuPreintegratedFromLastKF->dT;

            // 计算当前帧在世界坐标系的位姿，原理都是用预积分的位姿（预积分的值不会变化）与上一帧的位姿（会迭代变化）进行更新
            // 1、旋转 R_wb2 = R_wb1 * R_b1b2
            Eigen::Matrix3f Rwb2 = IMU::NormalizeRotation(Rwb1 * mpImuPreintegratedFromLastKF->GetDeltaRotation(mpLastKeyFrame->GetImuBias()));
            // 2、位移
            Eigen::Vector3f twb2 = twb1 + Vwb1 * t12 + 0.5f * t12 * t12 * Gz + Rwb1 * mpImuPreintegratedFromLastKF->GetDeltaPosition(mpLastKeyFrame->GetImuBias());
            // 3、速度
            Eigen::Vector3f Vwb2 = Vwb1 + t12 * Gz + Rwb1 * mpImuPreintegratedFromLastKF->GetDeltaVelocity(mpLastKeyFrame->GetImuBias());
            // 设置当前帧的世界坐标系的相机位姿
            mCurrentFrame.SetImuPoseVelocity(Rwb2, twb2, Vwb2);

            // 记录bias
            mCurrentFrame.mImuBias = mpLastKeyFrame->GetImuBias(); // 将上一关键帧的IMU偏置传递给当前帧，保证当前帧的IMU偏置与上一关键帧一致。
            mCurrentFrame.mPredBias = mCurrentFrame.mImuBias;      // 将当前帧的IMU偏置赋值给预测偏置，初始化预测偏置，使得当前帧的预测偏置与实际偏置相同，确保一致性。
            return true;
        }
        // step3：当地图没有更新时（!mbMapUpdated）：
        // ! 地图没有更新时，则会使用上一【普通帧 】的信息来进行 IMU 预测，普通帧的信息较为简单，预测的精度较低。
        else if (!mbMapUpdated)
        {
            // 通过上一帧（不一定是关键帧）的信息（IMU的位置、旋转矩阵、速度）来推算当前帧的状态
            const Eigen::Vector3f twb1 = mLastFrame.GetImuPosition();
            const Eigen::Matrix3f Rwb1 = mLastFrame.GetImuRotation();
            const Eigen::Vector3f Vwb1 = mLastFrame.GetVelocity();

            const Eigen::Vector3f Gz(0, 0, -IMU::GRAVITY_VALUE);
            // mpImuPreintegratedFrame 是当前帧的上一帧，不一定是关键帧
            const float t12 = mCurrentFrame.mpImuPreintegratedFrame->dT;

            Eigen::Matrix3f Rwb2 = IMU::NormalizeRotation(Rwb1 * mCurrentFrame.mpImuPreintegratedFrame->GetDeltaRotation(mLastFrame.mImuBias));
            Eigen::Vector3f twb2 = twb1 + Vwb1 * t12 + 0.5f * t12 * t12 * Gz + Rwb1 * mCurrentFrame.mpImuPreintegratedFrame->GetDeltaPosition(mLastFrame.mImuBias);
            Eigen::Vector3f Vwb2 = Vwb1 + t12 * Gz + Rwb1 * mCurrentFrame.mpImuPreintegratedFrame->GetDeltaVelocity(mLastFrame.mImuBias);

            mCurrentFrame.SetImuPoseVelocity(Rwb2, twb2, Vwb2);

            mCurrentFrame.mImuBias = mLastFrame.mImuBias; // 将上一帧的 IMU 偏置传递给当前帧，保证当前帧的 IMU 偏置与上一帧一致。
            // 无论是哪种情况，计算得到的IMU偏置（mImuBias）都会被更新为当前帧的IMU偏置（mPredBias）。偏置通常是IMU的系统误差，需要持续更新。
            mCurrentFrame.mPredBias = mCurrentFrame.mImuBias; // 将当前帧的 IMU 偏置赋值给预测偏置（mPredBias），初始化当前帧的预测偏置，确保它与实际偏置一致。
            return true;
        }
        // step4：没有预测
        else
            cout << "not IMU prediction!!" << endl;

        return false;
    }

    void Tracking::ResetFrameIMU()
    {
        // TODO To implement...
    }

    // note：线程主函数，大 Boss， 非常重要的！！！Track 包含两部分：估计运动、跟踪局部地图
    /**
     * @brief 跟踪过程，包括恒速模型跟踪、参考关键帧跟踪、局部地图跟踪
     * track 包含两部分：估计运动、跟踪局部地图
     *
     * Step 1：初始化
     * Step 2：跟踪
     * Step 3：记录位姿信息，用于轨迹复现
     */
    void Tracking::Track()
    {
        std::cout << std::endl;
        std::cout << "----------------------\n";
        std::cout << "[Tracking] Thread started." << std::endl;

        // 这个代码片段主要用于在步进调试模式下控制程序的执行流程。
        // 步进模式允许开发者在每一步中检查程序的状态，调试时可以手动或通过某个事件（例如用户按键）触发程序继续执行。
        if (bStepByStep)
        {
            // 用于提示用户或开发者，程序正在等待进入下一步。
            std::cout << "Tracking: Waiting to the next step" << std::endl;

            while (!mbStep && bStepByStep)
            {
                // 使程序暂停 500 微秒（0.5 毫秒），这样做是为了避免循环空转（busy-waiting），减少 CPU 的无效消耗。
                usleep(500);
            }
            mbStep = false;
        }

        // Step 1：如果局部建图里认为 IMU 有问题，重置当前活跃地图 --- 这里不用管，一般很少出现
        if (mpLocalMapper->mbBadImu)
        {
            cout << "TRACK: Reset map because local mapper set the bad imu flag " << endl;
            mpSystem->ResetActiveMap(); // 重置当前的地图，清空当前的地图数据，并可能启动新的地图构建过程
            return;                     // 在地图重置后，直接跳出 Track() 函数，不再进行后续的跟踪操作。这是一个保护机制，避免在地图状态异常时继续进行不正确的跟踪。
        }

        // 获取当前的活跃地图，保存在 pCurrentMap 中
        std::cout << "当前活跃地图: " << mpAtlas->GetCurrentMap()->GetId() << std::endl;
        Map *pCurrentMap = mpAtlas->GetCurrentMap();
        if (!pCurrentMap)
            cout << "ERROR: There is not an active map in the atlas" << endl;

        // Step 2：处理时间戳异常的情况，这种情况下需要考虑新建地图了。
        // ps2 创建新地图的时机之 2 —— 跟踪线程中时间戳异常时
        if (mState != NO_IMAGES_YET) // 如果图像复位过、或者第一次运行，则为 NO_IMAGE_YET 状态
        {
            // * 进入以下两个 if 语句都是不正常的情况，不进行跟踪直接返回

            // case1：如果当前图像时间戳比前一帧图像时间戳【小】，说明出错了，清除 imu 数据，创建新的子地图
            if (mLastFrame.mTimeStamp > mCurrentFrame.mTimeStamp)
            {
                cout << mCurrentFrame.mTimeStamp << ", " << mLastFrame.mTimeStamp << endl;
                cerr << "ERROR: Frame with a timestamp older than previous frame detected!" << endl;

                // ? 这里用锁的原因：mlQueueImuData.clear(); 对 mlQueueImuData 进行了修改操作。在多线程环境下，如果没有锁的保护，可能会有多个线程同时访问或修改这个队列，导致数据竞争和不一致。
                unique_lock<mutex> lock(mMutexImuQueue);
                mlQueueImuData.clear();

                // 创建新地图
                CreateMapInAtlas();
                return;
            }

            // case2：如果当前图像时间戳和前一帧图像时间戳【大于 1s -- 有 30 帧了】，说明时间戳明显跳变了，重置地图后直接返回
            else if (mCurrentFrame.mTimeStamp > mLastFrame.mTimeStamp + 1.0)
            {
                cout << mCurrentFrame.mTimeStamp << ", " << mLastFrame.mTimeStamp << endl;
                cout << "id last: " << mLastFrame.mnId << "    id curr: " << mCurrentFrame.mnId << endl;

                // 如果是 imu 模式，则做如下操作（执行 IMU 补偿）
                if (mpAtlas->isInertial())
                {
                    // ps：首先会判断 IMU 是否已经完成【第一阶段】初始化，如果完成：
                    if (mpAtlas->isImuInitialized())
                    {
                        cout << "Timestamp jump detected. State set to LOST. Reseting IMU integration..." << endl;

                        // 如果没有完成 IMU 的【第三阶段】优化（BA2）（在 localmapping 线程里），说明当前地图的数据不再可信，系统会调用 ResetActiveMap() 重置当前活跃地图，丢弃当前地图的数据
                        if (!pCurrentMap->GetIniertialBA2())
                        {
                            // 如果当前子图中 imu 没有经过 BA2，则重置 active 地图，也就是之前的数据不要了
                            mpSystem->ResetActiveMap();
                        }
                        else
                        {
                            // 如果当前子图中 imu 进行了 BA2，则重新创建新的子图，保存当前地图
                            CreateMapInAtlas();
                        }
                    }
                    // ps：如果当前子图中 imu 还没有初始化，重置 active 地图
                    else
                    {
                        cout << "Timestamp jump detected, before IMU initialization. Reseting..." << endl;
                        mpSystem->ResetActiveMap();
                    }
                }
                // 非 IMU 模式，放弃追踪，直接返回
                else
                {
                    return;
                }
            }
        }

        // Step 3：如果是 IMU 模式下设置 IMU 的 Bias 参数，还要保证上一帧存在 --- 不重要
        if ((mSensor == System::IMU_MONOCULAR || mSensor == System::IMU_STEREO || mSensor == System::IMU_RGBD) && mpLastKeyFrame)
        {
            // 使用上一帧的 bias 作为当前帧的初值，认为 bias 在两帧间不变
            // IMU 的偏置是通过上一帧的数据来传递的，所以这一行的目的是保证偏置的传递。
            mCurrentFrame.SetNewBias(mpLastKeyFrame->GetImuBias());
        }

        // 如果系统还没有开始跟踪图像（状态是 NO_IMAGES_YET），那么就更新状态为 NOT_INITIALIZED，表示系统已经开始接收图像并进行初始化了。
        if (mState == NO_IMAGES_YET)
        {
            mState = NOT_INITIALIZED;
        }

        // mLastProcessedState 存储了 Tracking 最新的状态，用于 FrameDrawer 中的绘制，通常在图像显示（例如通过图形界面显示状态）时使用。
        mLastProcessedState = mState;

        // Step 4：IMU 模式且还没有创建地图的情况下，对 IMU 数据进行预积分
        // 只有在地图还未创建的情况下，IMU 数据才需要进行预积分。因为一旦创建了地图，通常会启动基于视觉和 IMU 的优化算法（例如，视觉-惯性融合），此时预积分的数据会被用于后续的优化和位姿估计。
        // 如果地图已经创建，那么系统通常会进行更精确的状态估计（例如使用视觉-IMU 卡尔曼滤波器、优化算法等），而不是继续进行预积分。
        if ((mSensor == System::IMU_MONOCULAR || mSensor == System::IMU_STEREO || mSensor == System::IMU_RGBD) && !mbCreatedMap)
        {

#ifdef REGISTER_TIMES
            std::chrono::steady_clock::time_point time_StartPreIMU = std::chrono::steady_clock::now();
#endif
            // note：IMU 数据进行预积分
            PreintegrateIMU();

#ifdef REGISTER_TIMES
            std::chrono::steady_clock::time_point time_EndPreIMU = std::chrono::steady_clock::now();

            double timePreImu = std::chrono::duration_cast<std::chrono::duration<double, std::milli>>(time_EndPreIMU - time_StartPreIMU).count();
            vdIMUInteg_ms.push_back(timePreImu);
#endif
        }

        mbCreatedMap = false; // 表示地图尚未创建，用于后续判断系统是否已经构建了完整地图。

        // Get Map Mutex -> Map cannot
        // 地图更新时加锁。保证地图不会发生变化。即：确保在更新地图期间不会发生并发访问
        // 疑问：这样子会不会影响地图的实时更新?
        // 回答：主要耗时在构造帧中特征点的提取和匹配部分，在那个时候地图是没有被上锁的，有足够的时间更新地图 be changed
        unique_lock<mutex> lock(pCurrentMap->mMutexMapUpdate);

        mbMapUpdated = false; // 表示当前地图没有更新，为了在后续检查时判断地图是否发生了变化。

        // 判断地图 id 是否更新了
        int nCurMapChangeIndex = pCurrentMap->GetMapChangeIndex(); // 获取的当前地图的变化计数
        int nMapChangeIndex = pCurrentMap->GetLastMapChange();     // 获取的上一次被处理的地图变化计数
        if (nCurMapChangeIndex > nMapChangeIndex)
        {
            // ? 提问：地图更新一般发生在什么时候？答：

            // 如果检测到地图更新了
            cout << "Map update detected!" << endl;

            // 更新当前线程的地图变化计数为最新值
            pCurrentMap->SetLastMapChange(nCurMapChangeIndex);

            mbMapUpdated = true;
        }

        // Step 5：地图初始化，从这里开始对照流程图来看
        // case1：没有初始化的情况，第一帧图像进入，需要进行地图初始化。
        if (mState == NOT_INITIALIZED)
        {
            if (mSensor == System::STEREO || mSensor == System::RGBD || mSensor == System::IMU_STEREO || mSensor == System::IMU_RGBD)
            {
                std::cout << "----------------------\n";
                std::cout << "当前正在初始化的帧的 id: " << mCurrentFrame.mnId << std::endl;
                std::cout << "当前正在初始化的帧的特征点数: " << mCurrentFrame.mvKeysUn.size() << std::endl;

                StereoInitialization();

                // 这里出来之后，mState = OK 了
                std::cout << "After StereoInitialization(), mState = " << to_string(mState) << std::endl;
            }
            else
            {
                MonocularInitialization();
                // 这里出来之后，mState = OK 了
                std::cout << "After MonocularInitialization(), mState = " << to_string(mState) << std::endl;
            }

            // 更新帧绘制器中存储的最新状态
            mpFrameDrawer->Update(this);

            // 这个状态量在上面的初始化函数中被更新，如果不是 OK ，说明初始化失败了，这一帧变为上一帧，再用新的一帧进行初始化
            if (mState != OK) // If rightly initialized, mState = OK
            {
                // Verbose::PrintMess("mState != OK!!!再来一帧！", Verbose::VERBOSITY_NORMAL);
                std::cout << "mState != OK!!!再来一帧！" << std::endl;
                mLastFrame = Frame(mCurrentFrame);
                return;
            }

            // notice 这个代码片段的核心功能是确保在 SLAM 系统的初始化过程中，正确记录下第一帧的 ID。
            if (mpAtlas->GetAllMaps().size() == 1)
            {
                // 将当前帧标记为当前地图的第一个普通帧的 ID
                mnFirstFrameId = mCurrentFrame.mnId;
                std::cout << "当前活跃地图的 mnFirstFrameId: " << mnFirstFrameId << std::endl;
            }
        }

        // case2：有初始化的情况，第二帧进来后走这里
        else
        {
            std::cout << "当前帧 id: " << mCurrentFrame.mnId << "，当前帧的特征点数: " << mCurrentFrame.N << std::endl;
            std::cout << "当前关键帧 id: " << GetLastKeyFrame()->mnId << std::endl;
            std::cout << "当前地图中关键帧数量: " << mpAtlas->GetAllKeyFrames().size() << std::endl;
            std::cout << "关键帧数量: " << mpAtlas->KeyFramesInMap() << std::endl;
            std::cout << "局部地图中的关键帧数量" << mvpLocalKeyFrames.size() << std::endl;

            // Step 6：系统成功初始化，开始追踪帧。下面是具体跟踪过程👇
            bool bOK; // bOK 为临时变量，表示当前帧是否成功跟踪到了初始位姿。

#ifdef REGISTER_TIMES
            std::chrono::steady_clock::time_point time_StartPosePred = std::chrono::steady_clock::now();
#endif

            // mbOnlyTracking 等于 false 表示正常 SLAM 模式（定位+地图更新），mbOnlyTracking 等于 true 表示仅定位模式
            // tracking 类构造时默认为 false。在 viewer 中有个开关 ActivateLocalizationMode，可以控制是否开启 mbOnlyTracking
            // Initial camera pose estimation using motion model or relocalization (if tracking is lost)

            // case1：正常 SLAM 模式
            if (!mbOnlyTracking)
            {
                // State OK --- 跟踪进入正常 SLAM 模式，有地图更新
                // Local Mapping is activated. This is the normal behaviour, unless you explicitly activate the "only tracking" mode.

                std::cout << "mState: " << mState << std::endl;

                // case1：跟踪正常按照下面处理，mState == OK
                if (mState == OK)
                {
                    // Step 6.1 检查并更新上一帧被替换的 MapPoints
                    // Local Mapping might have changed some MapPoints tracked in last frame
                    // 局部建图线程则可能会对原有的地图点进行替换。在这里进行检查。
                    CheckReplacedInLastFrame();

                    // Step 6.2 运动模型是空的并且 imu 未初始化 或 刚完成重定位，就用【参考关键帧】跟踪；否则【恒速模型】跟踪
                    // 第一个条件：如果运动模型为空并且 imu 未初始化，说明是刚开始第一帧跟踪，或者已经跟丢了。
                    // 第二个条件：如果当前帧紧紧地跟着在重定位的帧的后面，就用重定位帧来恢复位姿。
                    // mnLastRelocFrameId 上一次重定位的那一帧。mCurrentFrame.mnId < mnLastRelocFrameId + 2 说明当前帧紧跟在上一次重定位帧的后面一帧，如 5，6 帧
                    if ((!mbVelocity && !pCurrentMap->isImuInitialized()) || mCurrentFrame.mnId < mnLastRelocFrameId + 2)
                    {
                        // Verbose::PrintMess("TRACK: Track with respect to the reference KF ", Verbose::VERBOSITY_DEBUG);
                        std::cout << "TRACK: Track with respect to the reference KF " << std::endl;

                        // 用【最近的关键帧】来跟踪当前的普通帧。
                        // 通过 BoW 的方式在参考帧中找当前帧特征点的匹配点
                        // 优化每个特征点都对应 3D 点重投影误差即可得到位姿
                        bOK = TrackReferenceKeyFrame(); // ps1：参考关键帧跟踪
                        if (bOK)
                        {
                            std::cout << "TrackReferenceKeyFrame succcess" << std::endl;
                        }
                        else
                        {
                            std::cout << "TrackReferenceKeyFrame fail" << std::endl;
                        }
                    }
                    else
                    {
                        std::cout << "TRACK: Track with motion model" << std::endl;
                        // Verbose::PrintMess("TRACK: Track with motion model", Verbose::VERBOSITY_DEBUG);

                        // 用【恒速模型】来跟踪当前帧。所谓的恒速：假设上上帧到上一帧的位姿 = 上一帧的位姿到当前帧位姿
                        // 根据恒速模型设定当前帧的初始位姿，用最近的普通帧来跟踪当前的普通帧
                        // 通过【投影】的方式在参考帧中找当前帧特征点的匹配点，优化每个特征点所对应 3D 点的投影误差即可得到位姿
                        bOK = TrackWithMotionModel(); // ps2：恒速模型跟踪

                        if (bOK)
                        {
                            std::cout << "TrackWithMotionModel succcess" << std::endl;
                        }
                        else
                        {
                            std::cout << "TrackWithMotionModel fail" << std::endl;
                        }

                        if (!bOK)
                        {
                            // 根据恒速模型失败了，只能根据参考关键帧来跟踪
                            bOK = TrackReferenceKeyFrame();
                            if (bOK)
                            {
                                std::cout << "TrackReferenceKeyFrame succcess2" << std::endl;
                            }
                            else
                            {
                                std::cout << "TrackReferenceKeyFrame fail2" << std::endl;
                            }
                        }
                    }

                    // Step 6.3 如果上面的 if 和 else if 都失败了，，并满足一定条件，就 bOK = false 了，那就判断是 RECENTLY_LOST 还是 LOST，然后再按照下面 case2 跟踪不正常来处理！
                    if (!bOK)
                    {
                        // 条件1：如果当前帧距离上次重定位成功不到 1s；
                        // 条件2：IMU 模式；
                        // 同时满足条件 1，2，标记为 LOST
                        if (mCurrentFrame.mnId <= (mnLastRelocFrameId + mnFramesToResetIMU) &&
                            (mSensor == System::IMU_MONOCULAR || mSensor == System::IMU_STEREO || mSensor == System::IMU_RGBD))
                        {
                            mState = LOST;

                            std::cout << "I'm LOST!" << std::endl;
                        }

                        // 条件1：当前地图中关键帧数目较多（大于10）
                        // 条件2（隐藏条件）：当前帧距离上次重定位帧超过1s，说明跟踪失败并非刚刚发生；或者当前系统并非 IMU 模式（纯视觉模式）
                        // 同时满足条件1，2，则将状态标记为 RECENTLY_LOST，后面会结合 IMU 预测的位姿看看能不能拽回来
                        else if (pCurrentMap->KeyFramesInMap() > 10)
                        {
                            cout << "KF in map: " << pCurrentMap->KeyFramesInMap() << endl;

                            mState = RECENTLY_LOST; // ps：新增了一个状态 RECENTLY_LOST，主要是结合 IMU 看看能不能拽回来

                            std::cout << "I'm RECENTLY_LOST!" << std::endl;

                            // 记录丢失时间，下面要用到
                            mTimeStampLost = mCurrentFrame.mTimeStamp;
                        }
                        else
                        {
                            mState = LOST;

                            std::cout << "I'm LOST!" << std::endl;
                        }
                    }
                }

                // case2：跟踪不正常按照下面处理，mState !== OK
                else
                {
                    // case1：如果是 RECENTLY_LOST 状态
                    if (mState == RECENTLY_LOST)
                    {
                        Verbose::PrintMess("Lost for a short time", Verbose::VERBOSITY_NORMAL);

                        // bOK 先置为 true
                        bOK = true;

                        // case1、如果是 IMU 模式
                        if ((mSensor == System::IMU_MONOCULAR || mSensor == System::IMU_STEREO || mSensor == System::IMU_RGBD))
                        {
                            // IMU 模式下可以用 IMU 来预测位姿，看能否拽回来
                            // Step 6.4 如果当前地图中 IMU 已经成功初始化，就用 IMU 数据预测位姿
                            if (pCurrentMap->isImuInitialized())
                                PredictStateIMU();
                            else
                                bOK = false;

                            // 如果 IMU 模式下当前帧距离跟丢帧超过 5s 还没有找回（time_recently_lost 默认为 5s），就放弃了，将 RECENTLY_LOST 状态改为LOST
                            if (mCurrentFrame.mTimeStamp - mTimeStampLost > time_recently_lost)
                            {
                                mState = LOST;

                                Verbose::PrintMess("Track Lost...", Verbose::VERBOSITY_NORMAL);
                                bOK = false;
                            }
                        }

                        // case2、如果是 纯视觉 模式
                        else
                        {
                            // Step 6.5 纯视觉模式则进行重定位。主要是 BOW 搜索，EPnP 求解位姿
                            bOK = Relocalization(); // ps3：重定位跟踪
                            if (bOK)
                            {
                                std::cout << "Relocalization succcess" << std::endl;
                            }
                            else
                            {
                                std::cout << "Relocalization fail" << std::endl;
                            }

                            // std::cout << "mCurrentFrame.mTimeStamp:" << to_string(mCurrentFrame.mTimeStamp) << std::endl;
                            // std::cout << "mTimeStampLost:" << to_string(mTimeStampLost) << std::endl;

                            // 如果重定位失败，且当前帧距离跟踪丢失时间超过 3 秒（3.0f）
                            if (mCurrentFrame.mTimeStamp - mTimeStampLost > 3.0f && !bOK)
                            {
                                // 纯视觉模式下重定位失败，状态即为 LOST
                                mState = LOST;

                                // Verbose::PrintMess("Track Lost...", Verbose::VERBOSITY_NORMAL);
                                std::cout << "Track Lost..." << std::endl;
                                bOK = false;
                            }
                        }
                    }

                    // case2：上一帧为最近丢失且重定位失败时，即为 LOST 状态
                    // todo 情况1：在跟踪的第 1 阶段确定跟踪丢失
                    // ps1 创建新地图的时机之 3 —— 跟踪线程中确定跟踪丢失后
                    else if (mState == LOST)
                    {
                        // Step 6.6 如果是 LOST 状态
                        Verbose::PrintMess("A new map is started...", Verbose::VERBOSITY_NORMAL); // 开启一个新地图

                        if (pCurrentMap->KeyFramesInMap() < 10)
                        {
                            // case1：当前活跃地图中的关键帧数目【小于10个】，则认为该地图中有效信息太少，直接重置，丢弃当前活跃地图
                            mpSystem->ResetActiveMap();

                            Verbose::PrintMess("Reseting current map...", Verbose::VERBOSITY_NORMAL);
                        }
                        else
                        {
                            // case2：当前活跃地图中的关键帧数目【超过10个】，则认为该地图有一定价值，存储起来作为非活跃地图，然后再创建一个新地图
                            CreateMapInAtlas();
                            Verbose::PrintMess("Creating newnewnew map...", Verbose::VERBOSITY_NORMAL);
                        }

                        // 干掉上一个关键帧
                        if (mpLastKeyFrame)
                            mpLastKeyFrame = static_cast<KeyFrame *>(NULL);

                        Verbose::PrintMess("done666", Verbose::VERBOSITY_NORMAL);

                        return;
                    }
                }
            }

            // case2：纯定位模式，一般用不到~
            else
            {
                // Localization Mode: Local Mapping is deactivated (TODO Not available in inertial mode)
                if (mState == LOST)
                {
                    if (mSensor == System::IMU_MONOCULAR || mSensor == System::IMU_STEREO || mSensor == System::IMU_RGBD)
                        Verbose::PrintMess("IMU. State LOST", Verbose::VERBOSITY_NORMAL);
                    bOK = Relocalization();
                }
                else
                {
                    if (!mbVO)
                    {
                        // In last frame we tracked enough MapPoints in the map
                        if (mbVelocity)
                        {
                            bOK = TrackWithMotionModel();
                        }
                        else
                        {
                            bOK = TrackReferenceKeyFrame();
                        }
                    }
                    else
                    {
                        // In last frame we tracked mainly "visual odometry" points.

                        // We compute two camera poses, one from motion model and one doing relocalization.
                        // If relocalization is sucessfull we choose that solution, otherwise we retain
                        // the "visual odometry" solution.

                        bool bOKMM = false;
                        bool bOKReloc = false;
                        vector<MapPoint *> vpMPsMM;
                        vector<bool> vbOutMM;
                        Sophus::SE3f TcwMM;
                        if (mbVelocity)
                        {
                            bOKMM = TrackWithMotionModel();
                            vpMPsMM = mCurrentFrame.mvpMapPoints;
                            vbOutMM = mCurrentFrame.mvbOutlier;
                            TcwMM = mCurrentFrame.GetPose();
                        }
                        bOKReloc = Relocalization();

                        if (bOKMM && !bOKReloc)
                        {
                            mCurrentFrame.SetPose(TcwMM);
                            mCurrentFrame.mvpMapPoints = vpMPsMM;
                            mCurrentFrame.mvbOutlier = vbOutMM;

                            if (mbVO)
                            {
                                for (int i = 0; i < mCurrentFrame.N; i++)
                                {
                                    if (mCurrentFrame.mvpMapPoints[i] && !mCurrentFrame.mvbOutlier[i])
                                    {
                                        mCurrentFrame.mvpMapPoints[i]->IncreaseFound();
                                    }
                                }
                            }
                        }
                        else if (bOKReloc)
                        {
                            mbVO = false;
                        }

                        bOK = bOKReloc || bOKMM;
                    }
                }
            }

            // 动态设置参考关键帧 --- 将最新的关键帧作为当前帧的参考关键帧
            // mpReferenceKF 先是上一时刻的参考关键帧，如果当前为新关键帧则变成当前关键帧，如果不是新的关键帧则先为上一帧的参考关键帧，而后经过更新局部关键帧重新确定。
            if (!mCurrentFrame.mpReferenceKF)
                mCurrentFrame.mpReferenceKF = mpReferenceKF;

#ifdef REGISTER_TIMES
            std::chrono::steady_clock::time_point time_EndPosePred = std::chrono::steady_clock::now();

            double timePosePred = std::chrono::duration_cast<std::chrono::duration<double, std::milli>>(time_EndPosePred - time_StartPosePred).count();
            vdPosePred_ms.push_back(timePosePred);
#endif

#ifdef REGISTER_TIMES
            std::chrono::steady_clock::time_point time_StartLMTrack = std::chrono::steady_clock::now();
#endif
            // * ---------------------------- 下面是从流程图的左下角的局部地图跟踪往后的过程了！👇 -------------------------------
            // 前面只是跟踪一帧得到初始位姿，这里搜索局部关键帧、局部地图点，和当前帧进行投影匹配，得到更多匹配的MapPoints后进行Pose优化
            // 在帧间匹配得到初始的姿态后，现在对local map进行跟踪得到更多的匹配，并优化当前位姿
            // local map：当前帧、当前帧的 MapPoints、当前关键帧与其它关键帧共视关系
            // 前面主要是两两跟踪（恒速模型跟踪上一帧、跟踪参考帧），这里搜索局部关键帧后搜集所有局部 MapPoints，
            // 然后将局部MapPoints和当前帧进行投影匹配，得到更多匹配的MapPoints后进行Pose优化

            // Step 7 在跟踪得到当前帧的初始姿态后，现在对 local map 进行跟踪得到更多的匹配，并优化当前位姿
            if (!mbOnlyTracking) // 1、SLAM 模式
            {
                if (bOK)
                {
                    std::cout << "TRACK: Track with localMap" << std::endl;

                    bOK = TrackLocalMap(); // ps4：局部地图跟踪
                    if (bOK)
                    {
                        std::cout << "TrackLocalMap succcess" << std::endl;
                    }
                    else
                    {
                        std::cout << "TrackLocalMap fail" << std::endl;
                    }
                }
                if (!bOK)
                    cout << "Fail to track local map!" << endl;
            }

            else // 2、仅定位模式
            {
                // mbVO true means that there are few matches to MapPoints in the map. We cannot retrieve a local map and therefore we do not perform TrackLocalMap().
                // Once the system relocalizes the camera we will use the local map again.
                if (bOK && !mbVO) // 意味着当前帧已经成功跟踪到位姿
                    bOK = TrackLocalMap();
            }

            // 到此为止跟踪确定位姿阶段结束，下面开始做收尾工作和为下一帧做准备

            // 查看到此为止时的两个状态变化：
            // bOK 的历史变化---上一帧跟踪成功---当前帧跟踪成功---局部地图跟踪成功---true                     -->OK   1 跟踪局部地图成功
            //          \               \              \---局部地图跟踪失败---false
            //           \               \---当前帧跟踪失败---false
            //            \---上一帧跟踪失败---重定位成功---局部地图跟踪成功---true                       -->OK  2 重定位
            //                          \           \---局部地图跟踪失败---false
            //                           \---重定位失败---false

            //
            // mState 的历史变化---上一帧跟踪成功---当前帧跟踪成功---局部地图跟踪成功---OK                  -->OK  1 跟踪局部地图成功
            //            \               \              \---局部地图跟踪失败---OK                  -->OK  3 正常跟踪
            //             \               \---当前帧跟踪失败---非OK
            //              \---上一帧跟踪失败---重定位成功---局部地图跟踪成功---非OK
            //                            \           \---局部地图跟踪失败---非OK
            //                             \---重定位失败---非OK（传不到这里，因为直接return了）
            // 由上图可知当前帧的状态OK的条件是跟踪局部地图成功，重定位或正常跟踪都可

            // Step 8 根据上面的操作来判断是否追踪成功
            // case1：此时还 OK，才说明跟踪两个阶段都成功了
            if (bOK)
            {
                mState = OK;
                if (bOK)
                {
                    std::cout << "All succcess" << std::endl;
                }
                else
                {
                    std::cout << "First success, but second fail!" << std::endl;
                }
            }

            // case2：由上图可知，下面是第一阶段跟踪成功，但第二阶段局部地图跟踪失败了
            else if (mState == OK)
            {
                // 带 IMU 时状态变为最近丢失，否则变为直接丢失
                if (mSensor == System::IMU_MONOCULAR || mSensor == System::IMU_STEREO || mSensor == System::IMU_RGBD)
                {
                    // Verbose::PrintMess("Track lost for less than one second...", Verbose::VERBOSITY_NORMAL);
                    std::cout << "Track lost for less than one second..." << std::endl;

                    // IMU 模式下 IMU 没有成功初始化或者没有完成 IMU BA，则重置当前地图
                    if (!pCurrentMap->isImuInitialized() || !pCurrentMap->GetIniertialBA2())
                    {
                        std::cout << "IMU is not or recently initialized. Reseting active map..." << std::endl;
                        mpSystem->ResetActiveMap();
                    }

                    mState = RECENTLY_LOST;
                }

                // 不是 IMU 模式，则直接标记为 RECENTLY_LOST
                else
                {
                    mState = LOST; // 上一个版本这里直接判定丢失 LOST，现在给宽松一点
                    std::cout << "I'm Lost!" << std::endl;
                }

                // * 被注释掉了，记录丢失时间
                // 如果当前帧距离上次重定位帧超过 1s，用当前帧时间戳更新 lost 帧的时间戳
                // if (mCurrentFrame.mnId > mnLastRelocFrameId + mMaxFrames)
                // {
                mTimeStampLost = mCurrentFrame.mTimeStamp;
                // }
            }

            // 如果刚刚发生重定位并且 IMU 已经初始化，则保存当前帧信息，重置 IMU
            if ((mCurrentFrame.mnId < (mnLastRelocFrameId + mnFramesToResetIMU)) && (mCurrentFrame.mnId > mnFramesToResetIMU) &&
                (mSensor == System::IMU_MONOCULAR || mSensor == System::IMU_STEREO || mSensor == System::IMU_RGBD) && pCurrentMap->isImuInitialized())
            {
                // TODO check this situation
                // 存储指针
                Verbose::PrintMess("Saving pointer to frame. imu needs reset...", Verbose::VERBOSITY_NORMAL);
                Frame *pF = new Frame(mCurrentFrame);
                pF->mpPrevFrame = new Frame(mLastFrame);

                // IMU 重置
                // Load preintegration
                pF->mpImuPreintegratedFrame = new IMU::Preintegrated(mCurrentFrame.mpImuPreintegratedFrame);
            }

            // ps：下面代码没有用，可忽略掉
            if (pCurrentMap->isImuInitialized())
            {
                // 跟踪成功
                if (bOK)
                {
                    // 当前帧距离上次重定位帧刚好等于1s，重置（还未实现 TODO）
                    if (mCurrentFrame.mnId == (mnLastRelocFrameId + mnFramesToResetIMU))
                    {
                        cout << "RESETING FRAME!!!" << endl;
                        ResetFrameIMU();
                    }
                    else if (mCurrentFrame.mnId > (mnLastRelocFrameId + 30))
                        mLastBias = mCurrentFrame.mImuBias; // 没啥用，后面会重新赋值后传给普通帧
                }
            }

#ifdef REGISTER_TIMES
            std::chrono::steady_clock::time_point time_EndLMTrack = std::chrono::steady_clock::now();

            double timeLMTrack = std::chrono::duration_cast<std::chrono::duration<double, std::milli>>(time_EndLMTrack - time_StartLMTrack).count();
            vdLMTrack_ms.push_back(timeLMTrack);
#endif

            // Update drawer
            // 更新显示线程中的图像、特征点、地图点等信息
            mpFrameDrawer->Update(this);
            if (mCurrentFrame.isSet())
            {
                std::cout << "start update drawer!" << std::endl;
                mpMapDrawer->SetCurrentCameraPose(mCurrentFrame.GetPose());
            }

            // 查看到此为止时的两个状态变化
            // bOK 的历史变化---上一帧跟踪成功---当前帧跟踪成功---局部地图跟踪成功---true
            //          \               \              \---局部地图跟踪失败---false
            //           \               \---当前帧跟踪失败---false
            //            \---上一帧跟踪失败---重定位成功---局部地图跟踪成功---true
            //                          \           \---局部地图跟踪失败---false
            //                           \---重定位失败---false

            // mState 的历史变化---上一帧跟踪成功---当前帧跟踪成功---局部地图跟踪成功---OK
            //            \               \              \---局部地图跟踪失败---非OK（IMU时为RECENTLY_LOST）
            //             \               \---当前帧跟踪失败---非OK(地图超过10个关键帧时 RECENTLY_LOST)
            //              \---上一帧跟踪失败(RECENTLY_LOST)---重定位成功---局部地图跟踪成功---OK
            //               \                           \           \---局部地图跟踪失败---LOST
            //                \                           \---重定位失败---LOST（传不到这里，因为直接return了）
            //                 \--上一帧跟踪失败(LOST)--LOST（传不到这里，因为直接return了）

            std::cout << "mState: " << mState << std::endl;

            // Step 9：如果跟踪成功 或 RECENTLY_LOST，则更新速度，清除无效地图点，按需创建关键帧
            if (bOK || mState == RECENTLY_LOST)
            {
                std::cout << "处理前，当前帧中的地图点数量: " << mCurrentFrame.mvpMapPoints.size() << std::endl;

                // Step 9.1 Update motion model --- 更新恒速运动模型 TrackWithMotionModel 中的 mVelocity
                if (mLastFrame.isSet() && mCurrentFrame.isSet())
                {
                    std::cout << "更新恒速运动模型了" << std::endl;

                    Sophus::SE3f LastTwc = mLastFrame.GetPose().inverse();

                    // mVelocity = Tcl = Tcw * Twl，表示上一帧到当前帧的变换， 其中 Twl = LastTwc
                    mVelocity = mCurrentFrame.GetPose() * LastTwc;
                    mbVelocity = true;
                }
                else
                {
                    std::cout << "没有恒速运动模型，更新个屁" << std::endl;

                    // 否则没有速度
                    mbVelocity = false;
                }

                // 使用 IMU 积分的位姿显示
                if (mSensor == System::IMU_MONOCULAR || mSensor == System::IMU_STEREO || mSensor == System::IMU_RGBD)
                    mpMapDrawer->SetCurrentCameraPose(mCurrentFrame.GetPose());

                // Step 9.2 Clean VO matches --- 清除观测不到的地图点
                std::cout << "清除观测不到的地图点" << std::endl;
                for (int i = 0; i < mCurrentFrame.N; i++)
                {
                    MapPoint *pMP = mCurrentFrame.mvpMapPoints[i];
                    if (pMP)
                        if (pMP->Observations() < 1)
                        {
                            mCurrentFrame.mvbOutlier[i] = false;
                            mCurrentFrame.mvpMapPoints[i] = static_cast<MapPoint *>(NULL);
                        }
                }

                // Step 9.3 Delete temporal MapPoints --- 清除恒速模型跟踪中 UpdateLastFrame 中为当前帧临时添加的 MapPoints（仅双目和 RGBD）
                // 上个步骤中只是在当前帧中将这些 MapPoints 剔除，这里从 MapPoints 数据库中删除
                // 临时地图点仅仅是为了提高双目或rgbd摄像头的帧间跟踪效果，用完以后就扔了，没有添加到地图中
                std::cout << "删除临时地图点" << std::endl;
                for (list<MapPoint *>::iterator lit = mlpTemporalPoints.begin(), lend = mlpTemporalPoints.end(); lit != lend; lit++)
                {
                    MapPoint *pMP = *lit;
                    delete pMP;
                }

                // 这里不仅仅是清除 mlpTemporalPoints，通过 delete pMP 还删除了指针指向的 MapPoint
                // 不能够直接执行这个是因为其中存储的都是指针，之前的操作都是为了避免内存泄露
                mlpTemporalPoints.clear();

#ifdef REGISTER_TIMES
                std::chrono::steady_clock::time_point time_StartNewKF = std::chrono::steady_clock::now();
#endif
                // ps：判断是否需要插入关键帧
                bool bNeedKF = NeedNewKeyFrame();

                // Check if we need to insert a new keyframe
                // if(bNeedKF && bOK)

                std::cout << "bNeedKF: " << bNeedKF << "; bOK: " << bOK << std::endl;

                // Step 9.4 根据条件来判断是否插入关键帧
                // 需要同时满足下面条件1和2
                // 条件1：bNeedKF = true，需要插入关键帧
                // 条件2：bOK = true 跟踪成功 或 IMU模式下的 RECENTLY_LOST 模式且 mInsertKFsLost 为 true
                if (bNeedKF && (bOK || (mInsertKFsLost && mState == RECENTLY_LOST &&
                                        (mSensor == System::IMU_MONOCULAR || mSensor == System::IMU_STEREO || mSensor == System::IMU_RGBD))))
                {
                    std::cout << "将当前普通帧帧包装为关键帧" << std::endl;

                    // 创建关键帧，对于双目或 RGB-D 会产生新的地图点
                    CreateNewKeyFrame();
                }

#ifdef REGISTER_TIMES
                std::chrono::steady_clock::time_point time_EndNewKF = std::chrono::steady_clock::now();

                double timeNewKF = std::chrono::duration_cast<std::chrono::duration<double, std::milli>>(time_EndNewKF - time_StartNewKF).count();
                vdNewKF_ms.push_back(timeNewKF);
#endif

                // We allow points with high innovation (considererd outliers by the Huber Function)
                // pass to the new keyframe, so that bundle adjustment will finally decide
                // if they are outliers or not. We don't want next frame to estimate its position
                // with those points so we discard them in the frame. Only has effect if lastframe is tracked
                // 作者这里说允许在BA中被Huber核函数判断为外点的传入新的关键帧中，让后续的BA来审判他们是不是真正的外点
                // 但是估计下一帧位姿的时候我们不想用这些外点，所以删掉

                //  Step 9.5 删除那些在 BA 中检测为外点的地图点
                std::cout << "删除那些在 BA 中检测为外点的地图点" << std::endl;
                for (int i = 0; i < mCurrentFrame.N; i++)
                {
                    if (mCurrentFrame.mvpMapPoints[i] && mCurrentFrame.mvbOutlier[i])
                        mCurrentFrame.mvpMapPoints[i] = static_cast<MapPoint *>(NULL);
                }
                std::cout << "经过上述一系列操作，当前帧中的地图点数量: " << mCurrentFrame.mvpMapPoints.size() << std::endl;
            }

            // Step 10：如果第二阶段跟踪失败，跟踪状态为 LOST
            // todo 情况2：在跟踪的第 2 阶段确定跟踪丢失👇
            // Reset if the camera get lost soon after initialization
            if (mState == LOST)
            {
                std::cout << "mState: " << mState << std::endl;
                std::cout << "当前地图中关键帧数量: " << pCurrentMap->KeyFramesInMap() << std::endl;

                // 情况1：如果地图中关键帧【小于 10 个】，则重置当前地图，退出当前跟踪
                if (pCurrentMap->KeyFramesInMap() <= 10)
                {
                    std::cout << "要重置地图了嗷！" << std::endl;
                    mpSystem->ResetActiveMap();
                    return;
                }

                // 情况2：如果是 IMU 模式，但是还未进行 IMU 初始化，则重置当前地图，退出当前跟踪
                if (mSensor == System::IMU_MONOCULAR || mSensor == System::IMU_STEREO || mSensor == System::IMU_RGBD)
                {
                    if (!pCurrentMap->isImuInitialized())
                    {
                        Verbose::PrintMess("Track lost before IMU initialisation, reseting...", Verbose::VERBOSITY_QUIET);
                        mpSystem->ResetActiveMap();
                        return;
                    }
                }

                // 如果地图中关键帧【超过 10 个】并且是【纯视觉模式】，
                // 或者虽然是 IMU 模式但是已经完成 IMU 第一阶段初始化了，则保存当前地图，创建新的地图
                std::cout << "新建地图嗷" << std::endl;
                CreateMapInAtlas();
                std::cout << "新建地图完成了嗷" << std::endl;

                // 新增加了个 return
                return;
            }

            // 确保已经设置了参考关键帧
            if (!mCurrentFrame.mpReferenceKF)
                mCurrentFrame.mpReferenceKF = mpReferenceKF;

            // 保存上一帧的数据，当前帧变上一帧
            mLastFrame = Frame(mCurrentFrame);
            std::cout << "66666666666666666666666666666666666" << std::endl;
        }

        // 查看到此为止
        // mState的历史变化---上一帧跟踪成功---当前帧跟踪成功---局部地图跟踪成功---OK
        //            \               \              \---局部地图跟踪失败---非OK（IMU时为RECENTLY_LOST）
        //             \               \---当前帧跟踪失败---非OK(地图超过10个关键帧时 RECENTLY_LOST)
        //              \---上一帧跟踪失败(RECENTLY_LOST)---重定位成功---局部地图跟踪成功---OK
        //               \                           \           \---局部地图跟踪失败---LOST
        //                \                           \---重定位失败---LOST（传不到这里，因为直接return了）
        //                 \--上一帧跟踪失败(LOST)--LOST（传不到这里，因为直接return了）
        // last.记录位姿信息，用于轨迹复现

        // Step 11 记录位姿信息，用于最后保存所有的轨迹
        if (mState == OK || mState == RECENTLY_LOST)
        {
            // Store frame pose information to retrieve the complete camera trajectory afterwards.
            if (mCurrentFrame.isSet())
            {
                // 计算相对姿态 Tcr = Tcw * Twr, Twr = Trw^-1
                Sophus::SE3f Tcr_ = mCurrentFrame.GetPose() * mCurrentFrame.mpReferenceKF->GetPoseInverse();

                // 保存各种状态
                mlRelativeFramePoses.push_back(Tcr_);
                mlpReferences.push_back(mCurrentFrame.mpReferenceKF);
                mlFrameTimes.push_back(mCurrentFrame.mTimeStamp);
                mlbLost.push_back(mState == LOST);
            }
            else
            {
                // 如果跟踪丢失，则相对位姿使用上一次值
                // This can happen if tracking is lost
                mlRelativeFramePoses.push_back(mlRelativeFramePoses.back());
                mlpReferences.push_back(mlpReferences.back());
                mlFrameTimes.push_back(mlFrameTimes.back());
                mlbLost.push_back(mState == LOST);
            }
        }

        std::cout << "[Tracking] Thread finished." << std::endl;

#ifdef REGISTER_LOOP
        if (Stop())
        {

            // Safe area to stop
            while (isStopped())
            {
                usleep(3000);
            }
        }
#endif
    }

    // ---------------------------------------- 两类初始化函数：单目相机 和 双目/RGBD 相机 ----------------------------------------------

    // notice1：双目和 rgbd 的地图初始化，比单目简单很多 -- 由于具有深度信息，直接生成 MapPoints
    // 核心流程：利用当前帧的特征点和深度信息生成 3D 点（MapPoints），并将这些 MapPoints 插入到地图中。
    void Tracking::StereoInitialization()
    {
        // 初始化要求当前帧（左右图像的特征点总和）的特征点超过 500
        if (mCurrentFrame.N > 500)
        {
            std::cout << "------------------------ Start StereoInitialization! -------------------------------" << std::endl;

            // ps：如果是 双目+IMU 或者 RGBD+IMU 模式，还会进行下面处理👇
            if (mSensor == System::IMU_STEREO || mSensor == System::IMU_RGBD)
            {
                // 当前帧 && 上一帧的 IMU 数据是否均存在
                if (!mCurrentFrame.mpImuPreintegrated || !mLastFrame.mpImuPreintegrated)
                {
                    cout << "not IMU meas" << endl;
                    return;
                }

                // 加速度是否足够 && 如果当前帧和上一帧的加速度变化小于 0.5（即加速度变化不大），也不会进行初始化。
                // 这里，!mFastInit 表示当前不使用快速初始化模式，因此系统会要求加速度变化满足一定的条件，也就是加速度变化足够大，才进行初始化。
                if (!mFastInit && (mCurrentFrame.mpImuPreintegratedFrame->avgA - mLastFrame.mpImuPreintegratedFrame->avgA).norm() < 0.5)
                {
                    cout << "not enough acceleration" << endl;
                    return;
                }

                // 如果 mpImuPreintegratedFromLastKF 指针不为空，意味着之前已经存在一个 IMU 预积分对象，需要将其 删除，防止内存泄漏
                // ? 为什么要删除之前的？答：由于每次帧之间的 IMU 预积分结果是不同的，因此，在创建一个新的预积分对象时，需要 删除旧的对象，以确保内存的正确管理。
                if (mpImuPreintegratedFromLastKF)
                    delete mpImuPreintegratedFromLastKF;

                // 重新创建一个新的 IMU 预积分对象，并为当前帧分配该对象。
                mpImuPreintegratedFromLastKF = new IMU::Preintegrated(IMU::Bias(), *mpImuCalib);

                // 将新创建的 IMU 预积分对象赋值给 mCurrentFrame.mpImuPreintegrated，即为当前帧 mCurrentFrame 设置该帧对应的 IMU 预积分对象。
                mCurrentFrame.mpImuPreintegrated = mpImuPreintegratedFromLastKF;
            }

            // case1：如果传感器是 IMU 类型的（例如 IMU-双目 或 IMU-RGBD），则根据 IMU 标定参数设置当前帧的位姿
            if (mSensor == System::IMU_STEREO || mSensor == System::IMU_RGBD)
            {
                // ? 这里能直接用 IMU 标定的参数？答：相当于知道相机和 IMU 之间的关系，知道 IMU 的数据，所以通过这两者就能间接得到 相机位姿了
                Eigen::Matrix3f Rwb0 = mCurrentFrame.mImuCalib.mTcb.rotationMatrix(); // 得到旋转矩阵，即 IMU 的方向，相对于相机坐标系的旋转矩阵，简单来说就是 IMU 在相机坐标系中的“转动”
                Eigen::Vector3f twb0 = mCurrentFrame.mImuCalib.mTcb.translation();    // 得到平移向量，即 IMU 的位置，相对于相机坐标系的平移向量，也就是 IMU 在相机坐标系中的“位置”
                Eigen::Vector3f Vwb0;                                                 // 当前帧的速度
                Vwb0.setZero();                                                       // 将速度初始化为零，因为初始化阶段通常不需要考虑速度的影响
                mCurrentFrame.SetImuPoseVelocity(Rwb0, twb0, Vwb0);                   // 设置当前帧的位姿和速度
            }

            // case2：如果不是 IMU 传感器，直接设置当前帧的位姿为零（Sophus::SE3f()）
            // Sophus::SE3f() 表示一个初始化的 3D 位姿，通常用于设置为默认的零位姿（即没有旋转和平移，位于原点）
            else
            {
                mCurrentFrame.SetPose(Sophus::SE3f());
            }

            // step4. 创建关键帧（KeyFrame）：将当前帧构造为初始关键帧
            // mCurrentFrame 的数据类型为 Frame
            // KeyFrame 包含 Frame、地图3D点、以及BoW
            // KeyFrame 里有一个 mpMap，Tracking 里有一个mpMap，而 KeyFrame 里的 mpMap 都指向 Tracking 里的这个 mpMap
            // KeyFrame 里有一个 mpKeyFrameDB，Tracking 里有一个 mpKeyFrameDB，而 KeyFrame 里的 mpMap 都指向 Tracking 里的这个 mpKeyFrameDB
            // 提问: 为什么要指向 Tracking 中的相应的变量呢? -- 因为 Tracking 是主线程，是它创建和加载的这些模块

            // ? 怎样判断要不要将当前帧转为关键帧，我记得有个函数实现这个功能了。但当系统刚开始工作时，第一帧（或者前几帧）通常会成为关键帧。
            KeyFrame *pKFini = new KeyFrame(mCurrentFrame, mpAtlas->GetCurrentMap(), mpKeyFrameDB);

            // step5. 将关键帧插入地图：把该初始关键帧添加到地图中
            // KeyFrame 中包含了地图、反过来地图中也包含了 KeyFrame，相互包含
            mpAtlas->AddKeyFrame(pKFini);

            // step6. 根据当前帧的深度信息生成 MapPoints：
            // case1：没有第二个相机时（单目 / RGBD相机）：
            // ? 这个函数不是双目和 RGBD 相机的初始化函数？怎么还分有没有第二个相机？
            // 答：这部分是针对单目相机或者没有第二个相机的情况，比如 RGBD 相机。RGBD 相机包含一个 RGB 相机和一个深度传感器，它提供了每个像素的深度值（即深度图），但它并不是一个典型的双目相机，因此其深度信息也需要被处理。
            if (!mpCamera2)
            {
                std::cout << "没有第二个相机" << std::endl;

                // 为每个特征点构造 MapPoint
                for (int i = 0; i < mCurrentFrame.N; i++)
                {
                    float z = mCurrentFrame.mvDepth[i]; // 读取当前帧中第 i 个特征点的深度值。对于RGBD相机，这个深度值通常是由传感器直接提供的。

                    // ps：只有具有正深度的点才会被构造地图点
                    if (z > 0)
                    {
                        Eigen::Vector3f x3D;

                        // 通过 UnprojectStereo() 反投影得到该特征点的世界坐标系下 3D 坐标
                        // 根据当前帧的深度信息，UnprojectStereo() 会将图像坐标（即特征点）转换为 世界坐标系下的 3D 坐标。
                        mCurrentFrame.UnprojectStereo(i, x3D);

                        // 将 3D 点构造为 MapPoint
                        MapPoint *pNewMP = new MapPoint(x3D, pKFini, mpAtlas->GetCurrentMap());

                        // 为该 MapPoint 添加属性：
                        // a. 观测到该 MapPoint 的关键帧
                        // b. 该 MapPoint 的描述子
                        // c. 该 MapPoint 的平均观测方向和深度范围

                        // a.告诉地图，该 MapPoint 是被哪个 KeyFrame 的哪个特征点观测到
                        pNewMP->AddObservation(pKFini, i);
                        // 将该地图点加入到初始关键帧的观测集合中
                        pKFini->AddMapPoint(pNewMP, i);

                        // b.从众多观测到该 MapPoint 的特征点中挑选区分度最高的描述子
                        pNewMP->ComputeDistinctiveDescriptors();

                        // c.更新该 MapPoint 平均观测方向以及观测距离的范围
                        pNewMP->UpdateNormalAndDepth();

                        // 将新地图点保存到当前活动地图中
                        mpAtlas->AddMapPoint(pNewMP);

                        // 将该 MapPoint 添加到当前帧的 mvpMapPoints 中
                        // 为当前 Frame 的特征点与 MapPoint 之间建立索引
                        mCurrentFrame.mvpMapPoints[i] = pNewMP;
                    }
                }
            }

            // case2：如果有第二个相机（双目）：
            else
            {
                std::cout << "有第二个相机" << std::endl;

                // 遍历左图像的每个特征点
                for (int i = 0; i < mCurrentFrame.Nleft; i++)
                {
                    // 获取右图像上与左图像特征点匹配的特征点索引
                    // 这个代码段的核心逻辑并没有强制要求左图的第 i 个特征点一定要与右图的第 i 个特征点匹配。只要左图的某个特征点在右图中找到了匹配的特征点，那么就可以建立对应的 3D 点（MapPoint）
                    // 比如，左图特征点 15 与右图特征点 20 匹配，那么 mCurrentFrame.mvLeftToRightMatch[15] = 20;
                    int rightIndex = mCurrentFrame.mvLeftToRightMatch[i];

                    // 如果找到了匹配的右图像特征点
                    if (rightIndex != -1)
                    {
                        // 获取匹配特征点的 3D 坐标
                        // 从 mvStereo3Dpoints[i] 中获取当前左图第 i 个特征点的 3D 坐标，这通常是通过三角测量得到的。
                        Eigen::Vector3f x3D = mCurrentFrame.mvStereo3Dpoints[i];

                        // 创建一个新的 MapPoint
                        MapPoint *pNewMP = new MapPoint(x3D, pKFini, mpAtlas->GetCurrentMap());

                        // 为 MapPoint 添加两个观测：一个是左图像的特征点，另一个是右图像的特征点
                        pNewMP->AddObservation(pKFini, i);                                // 左图第 i 个特征点对当前地图点 pNewMP 进行了观测。
                        pNewMP->AddObservation(pKFini, rightIndex + mCurrentFrame.Nleft); // 右图第 rightIndex + mCurrentFrame.Nleft 个特征点对同一地图点也进行了观测。

                        // 更新关键帧与 MapPoint 的关系
                        pKFini->AddMapPoint(pNewMP, i);                                // 左图第 i 个特征点对当前地图点 pNewMP 进行了观测。
                        pKFini->AddMapPoint(pNewMP, rightIndex + mCurrentFrame.Nleft); // 右图第 rightIndex + mCurrentFrame.Nleft 个特征点对同一地图点也进行了观测。

                        // 计算描述子并更新
                        pNewMP->ComputeDistinctiveDescriptors();
                        // 更新地图点的法线和深度信息
                        pNewMP->UpdateNormalAndDepth();

                        // 将该 MapPoint 添加到地图中
                        mpAtlas->AddMapPoint(pNewMP);

                        // 将该 MapPoint 添加到当前帧的 mvpMapPoints 中，左右目都要添加上
                        mCurrentFrame.mvpMapPoints[i] = pNewMP;
                        mCurrentFrame.mvpMapPoints[rightIndex + mCurrentFrame.Nleft] = pNewMP;
                    }
                }
            }

            // 至此！初始帧的所有地图点（MapPoints）已经生成并与初始关键帧（KeyFrame）关联起来了。接下来是对地图和系统状态的更新，确保接下来的帧处理可以基于已经初始化好的地图进行跟踪和更新。

            // step7. 更新地图：
            Verbose::PrintMess("New Map created with " + to_string(mpAtlas->MapPointsInMap()) + " points", Verbose::VERBOSITY_QUIET);

            if (mpAtlas && mpAtlas->GetCurrentMap())
            {
                cout << "Current active map id : " << mpAtlas->GetCurrentMap()->GetId() << endl;
            }
            else
            {
                cerr << "Error: Atlas or Current Map is null!" << endl;
            }

            // 该初始关键帧插入到局部建图线程中，局部建图线程会以此关键帧为基础，优化局部地图的结构。
            // ps：重要说明 -- 这一操作通常是异步的，局部地图线程会以关键帧为单位，独立运行其地图维护和优化流程。
            mpLocalMapper->InsertKeyFrame(pKFini);

            // step8. 更新当前帧、关键帧和参考关键帧
            mLastFrame = Frame(mCurrentFrame);     // 把当前帧的内容复制给 mLastFrame，也就是说，现在的这一帧成为“上一帧”。
            mnLastKeyFrameId = mCurrentFrame.mnId; // 保存当前帧的 ID，标记为“最后一个关键帧的 ID”。
            mpLastKeyFrame = pKFini;               // 把刚才生成的初始关键帧的地址保存到 mpLastKeyFrame，以后系统可以通过这个指针找到这个关键帧。
            // mnLastRelocFrameId = mCurrentFrame.mnId;

            // ? 这个局部地图点竟然..不在 mpLocalMapper 中管理?
            // 我认为，这个点只是暂时被保存在了 Tracking 线程之中， 所以称之为 local
            // 初始化之后，通过双目图像生成的地图点，都应该被认为是局部地图点
            mvpLocalKeyFrames.push_back(pKFini);            // 把刚才生成的初始关键帧添加到局部关键帧列表里。
            mvpLocalMapPoints = mpAtlas->GetAllMapPoints(); // 把刚才生成的所有地图点拿出来，保存到局部地图点的列表中。
            mpReferenceKF = pKFini;                         // ps：将这个初始关键帧设置为参考关键帧
            mCurrentFrame.mpReferenceKF = pKFini;           // ps：将这个初始关键帧设置当前帧的参考关键帧

            // 把刚刚生成的这些地图点设置为 ReferenceMapPoints，即：告诉系统，这些地图点可以当成“参照物”，以后画地图时或者定位时用它们。
            // ReferenceMapPoints 是 DrawMapPoints 函数画图的时候用的
            mpAtlas->SetReferenceMapPoints(mvpLocalMapPoints);

            // 把初始关键帧保存为当前地图的起源关键帧。即：设定这张“第一张照片”是地图的“起点”，以后扩展地图都以它为基准。
            mpAtlas->GetCurrentMap()->mvpKeyFrameOrigins.push_back(pKFini);

            // step9. 地图可视化：将当前帧的相机位姿传递给地图绘制模块，用于更新可视化
            mpMapDrawer->SetCurrentCameraPose(mCurrentFrame.GetPose());

            // step10. 设置状态为初始化完成：追踪成功
            mState = OK;

            std::cout << "------------------ StereoInitialization Successful! -------------------------" << std::endl;
        }
    }

    // notice2：单目的地图初始化 -- 很复杂！
    /*
     * @brief 单目的地图初始化
     *
     * 并行地计算基础矩阵和单应性矩阵，选取其中一个模型，恢复出最开始两帧之间的相对姿态以及点云
     * 得到初始两帧的匹配、相对运动、初始MapPoints
     *
     * Step 1：（未创建）得到用于初始化的第一帧，初始化需要两帧
     * Step 2：（已创建）如果当前帧特征点数大于100，则得到用于单目初始化的第二帧
     * Step 3：在mInitialFrame与mCurrentFrame中找匹配的特征点对
     * Step 4：如果初始化的两帧之间的匹配点太少，重新初始化
     * Step 5：通过H模型或F模型进行单目初始化，得到两帧间相对运动、初始MapPoints
     * Step 6：删除那些无法进行三角化的匹配点
     * Step 7：将三角化得到的3D点包装成MapPoints
     */
    void Tracking::MonocularInitialization()
    {
        // 单目初始化的三个条件：（640 * 480 分辨率下的经验参数）
        // c1：参与初始化的两帧各自的特征点数目都要 ＞ 100；
        // c2：两帧特征点成功匹配的数目要 ＞ 100；
        // c3：两帧特征点三角化成功的三维点数目要 ＞50；

        // case1：如果单目初始器还没有被创建，则创建。后面如果重新初始化时会清掉这个
        if (!mbReadyToInitializate)
        {
            // ps：条件一 --- 单目初始化的两帧的特征点数必须都大于 100
            if (mCurrentFrame.mvKeys.size() > 100)
            {
                // 初始化需要两帧，分别是 mInitialFrame，mCurrentFrame
                mInitialFrame = Frame(mCurrentFrame);
                mLastFrame = Frame(mCurrentFrame); // 用当前帧更新上一帧

                mvbPrevMatched.resize(mCurrentFrame.mvKeysUn.size()); // mvbPrevMatched  记录"上一帧"所有特征点
                for (size_t i = 0; i < mCurrentFrame.mvKeysUn.size(); i++)
                    mvbPrevMatched[i] = mCurrentFrame.mvKeysUn[i].pt;

                // 初始化为 -1 表示没有任何匹配。这里面存储的是匹配的点的 id
                fill(mvIniMatches.begin(), mvIniMatches.end(), -1);

                // 如果是 单目+IMU 模式，则初始化预积分
                if (mSensor == System::IMU_MONOCULAR)
                {
                    if (mpImuPreintegratedFromLastKF)
                    {
                        delete mpImuPreintegratedFromLastKF;
                    }
                    mpImuPreintegratedFromLastKF = new IMU::Preintegrated(IMU::Bias(), *mpImuCalib);
                    mCurrentFrame.mpImuPreintegrated = mpImuPreintegratedFromLastKF;
                }

                // 下一帧准备做单目初始化了
                mbReadyToInitializate = true;

                return;
            }
        }

        // case2：如果单目初始化器已经被创建
        else
        {
            // 如果当前帧特征点数太少（不超过 100），则重新构造初始器
            if (((int)mCurrentFrame.mvKeys.size() <= 100) || ((mSensor == System::IMU_MONOCULAR) && (mLastFrame.mTimeStamp - mInitialFrame.mTimeStamp > 1.0)))
            {
                mbReadyToInitializate = false;
                return;
            }

            // 在 mInitialFrame 与 mCurrentFrame 中找匹配的特征点对
            ORBmatcher matcher(0.9,   // 0.9 表示最佳的和次佳特征点评分的比值阈值，这里是比较宽松的，跟踪时一般是 0.7
                               true); // true 表示检查特征点的方向

            // * 对进行初始化的两帧 mInitialFrame，mCurrentFrame 进行特征点匹配
            // mvbPrevMatched 为参考帧的特征点坐标，初始化存储的是 mInitialFrame 中特征点坐标，匹配后存储的是匹配好的当前帧的特征点坐标
            // mvIniMatches 保存参考帧 F1 中特征点是否匹配上，index 保存是 F1 对应特征点索引，值保存的是匹配好的 F2 特征点索引
            int nmatches = matcher.SearchForInitialization(mInitialFrame, mCurrentFrame, // 初始化时的参考帧和当前帧
                                                           mvbPrevMatched,               // 在初始化参考帧中提取得到的特征点
                                                           mvIniMatches,                 // 保存匹配关系
                                                           100);                         // 搜索窗口大小

            // ps：条件二 --- 如果初始化的两帧之间的匹配点＜100，则重新初始化
            if (nmatches < 100)
            {
                mbReadyToInitializate = false;
                return;
            }

            // 通过 H 模型或 F 模型进行单目初始化，得到两帧间相对运动、初始 MapPoints
            Sophus::SE3f Tcw;            // Current Camera's Rotation + Translation
            vector<bool> vbTriangulated; // Triangulated Correspondences (mvIniMatches)

            if (mpCamera->ReconstructWithTwoViews(mInitialFrame.mvKeysUn, mCurrentFrame.mvKeysUn, // 用于生成 MapPoints 的两帧【矫正后的】特征点
                                                  mvIniMatches,                                   // 当前帧和参考帧的特征点的匹配关系
                                                  Tcw,                                            // 初始化得到的相机的位姿
                                                  mvIniP3D,                                       // 进行三角化得到的空间点集合
                                                  vbTriangulated))                                // 以及对应于mvIniMatches来讲,其中哪些点被三角化了
            {
                // 初始化成功后，删除那些无法进行三角化的匹配点
                for (size_t i = 0, iend = mvIniMatches.size(); i < iend; i++)
                {
                    if (mvIniMatches[i] >= 0 && !vbTriangulated[i])
                    {
                        mvIniMatches[i] = -1;
                        nmatches--;
                    }
                }

                // Set Frame Poses
                // ! 将初始化的第一帧作为世界坐标系，因此第一帧变换矩阵为单位矩阵
                mInitialFrame.SetPose(Sophus::SE3f());
                // 由 Rcw 和 tcw 构造 Tcw，并赋值给 mTcw，mTcw 为世界坐标系到相机坐标系的变换矩阵
                mCurrentFrame.SetPose(Tcw);

                // 创建初始化地图点 MapPoints
                // Initialize 函数会得到 mvIniP3D，
                // mvIniP3D 是 cv::Point3f 类型的一个容器，是个存放 3D 点的临时变量，
                // CreateInitialMapMonocular 将 3D 点包装成 MapPoint 类型存入 KeyFrame 和 Map 中
                CreateInitialMapMonocular();
            }
        }
    }

    // notice3：单目相机成功初始化后，用三角化得到的点生成 MapPoints
    void Tracking::CreateInitialMapMonocular()
    {
        // 认为单目初始化时候的参考帧和当前帧都是关键帧
        KeyFrame *pKFini = new KeyFrame(mInitialFrame, mpAtlas->GetCurrentMap(), mpKeyFrameDB); // 第一帧
        KeyFrame *pKFcur = new KeyFrame(mCurrentFrame, mpAtlas->GetCurrentMap(), mpKeyFrameDB); // 第二帧

        if (mSensor == System::IMU_MONOCULAR)
            pKFini->mpImuPreintegrated = (IMU::Preintegrated *)(NULL);

        // Step 1 将初始关键帧、当前关键帧的描述子转为 BoW
        pKFini->ComputeBoW();
        pKFcur->ComputeBoW();

        // Step 2 将关键帧插入到地图
        mpAtlas->AddKeyFrame(pKFini);
        mpAtlas->AddKeyFrame(pKFcur);

        // Step 3 用初始化得到的 3D 点来生成地图点 MapPoints
        // mvIniMatches[i] 表示初始化两帧特征点匹配关系。
        // 具体解释：i 表示 帧1 中关键点的索引值，vMatches12[i] 的值为 帧2 的关键点索引值，没有匹配关系的话，vMatches12[i] 值为 -1
        for (size_t i = 0; i < mvIniMatches.size(); i++)
        {
            // 没有匹配，跳过
            if (mvIniMatches[i] < 0)
                continue;

            // 用三角化点初始化为空间点的世界坐标
            Eigen::Vector3f worldPos;
            worldPos << mvIniP3D[i].x, mvIniP3D[i].y, mvIniP3D[i].z;
            // Step 3.1 用 3D 点构造 MapPoint
            MapPoint *pMP = new MapPoint(worldPos, pKFcur, mpAtlas->GetCurrentMap());

            // Step 3.2 为该 MapPoint 添加属性：
            // a.观测到该 MapPoint 的关键帧
            // b.该 MapPoint 的描述子
            // c.该 MapPoint 的平均观测方向和深度范围

            // 表示该 KeyFrame 的 2D 特征点和对应的 3D 地图点
            pKFini->AddMapPoint(pMP, i);
            pKFcur->AddMapPoint(pMP, mvIniMatches[i]);

            // a.表示该 MapPoint 可以被哪个 KeyFrame 的哪个特征点观测到
            pMP->AddObservation(pKFini, i);
            pMP->AddObservation(pKFcur, mvIniMatches[i]);

            // b.从众多观测到该 MapPoint 的特征点中挑选最有代表性的描述子
            pMP->ComputeDistinctiveDescriptors();
            // c.更新该 MapPoint 平均观测方向以及观测距离的范围
            pMP->UpdateNormalAndDepth();

            // mvIniMatches下标 i 表示在初始化参考帧中的特征点的序号
            // mvIniMatches[i] 是初始化当前帧中的特征点的序号
            mCurrentFrame.mvpMapPoints[mvIniMatches[i]] = pMP;
            mCurrentFrame.mvbOutlier[mvIniMatches[i]] = false;

            // Add to Map
            mpAtlas->AddMapPoint(pMP);
        }

        // Step 3.3 更新关键帧间的连接关系
        // 在 3D点 和关键帧之间建立边，每个边有一个权重，边的权重是该关键帧与当前帧 公共3D点 的个数
        pKFini->UpdateConnections();
        pKFcur->UpdateConnections();

        std::set<MapPoint *> sMPs;
        sMPs = pKFini->GetMapPoints();

        // Step 4 全局 BA 优化，同时优化所有位姿和三维点
        Verbose::PrintMess("New Map created with " + to_string(mpAtlas->MapPointsInMap()) + " points", Verbose::VERBOSITY_QUIET);
        Optimizer::GlobalBundleAdjustemnt(mpAtlas->GetCurrentMap(), 20);

        // Step 5 取场景的中值深度，用于尺度归一化
        // 为什么是 pKFini 而不是 pKCur ? 答：都可以的，内部做了位姿变换了
        float medianDepth = pKFini->ComputeSceneMedianDepth(2);
        float invMedianDepth;
        if (mSensor == System::IMU_MONOCULAR)
            invMedianDepth = 4.0f / medianDepth; // 4.0f
        else
            invMedianDepth = 1.0f / medianDepth;

        // 两个条件：一个是平均深度要大于0；另外一个是在当前帧中被观测到的地图点的数目应该大于 100
        if (medianDepth < 0 || pKFcur->TrackedMapPoints(1) < 50) // TODO Check, originally 100 tracks
        {
            Verbose::PrintMess("Wrong initialization, reseting...", Verbose::VERBOSITY_QUIET);
            mpSystem->ResetActiveMap();
            return;
        }

        // Step 6 将两帧之间的变换归一化到平均深度1的尺度下
        Sophus::SE3f Tc2w = pKFcur->GetPose();
        // x/z y/z 将 z 归一化到 1
        Tc2w.translation() *= invMedianDepth;
        pKFcur->SetPose(Tc2w);

        // Step 7 把3D点的尺度也归一化到1
        // 为什么是 pKFini? 是不是就算是使用 pKFcur 得到的结果也是相同的? 答：是的，因为是同样的三维点
        vector<MapPoint *> vpAllMapPoints = pKFini->GetMapPointMatches();
        for (size_t iMP = 0; iMP < vpAllMapPoints.size(); iMP++)
        {
            if (vpAllMapPoints[iMP])
            {
                MapPoint *pMP = vpAllMapPoints[iMP];
                pMP->SetWorldPos(pMP->GetWorldPos() * invMedianDepth);
                pMP->UpdateNormalAndDepth();
            }
        }

        if (mSensor == System::IMU_MONOCULAR)
        {
            pKFcur->mPrevKF = pKFini;
            pKFini->mNextKF = pKFcur;
            pKFcur->mpImuPreintegrated = mpImuPreintegratedFromLastKF;

            mpImuPreintegratedFromLastKF = new IMU::Preintegrated(pKFcur->mpImuPreintegrated->GetUpdatedBias(), pKFcur->mImuCalib);
        }

        //  Step 8 将关键帧插入局部地图，更新归一化后的位姿、局部地图点
        mpLocalMapper->InsertKeyFrame(pKFini);
        mpLocalMapper->InsertKeyFrame(pKFcur);
        mpLocalMapper->mFirstTs = pKFcur->mTimeStamp;

        mCurrentFrame.SetPose(pKFcur->GetPose());
        mnLastKeyFrameId = mCurrentFrame.mnId;
        mpLastKeyFrame = pKFcur;
        // mnLastRelocFrameId = mInitialFrame.mnId;

        mvpLocalKeyFrames.push_back(pKFcur);
        mvpLocalKeyFrames.push_back(pKFini);
        // 单目初始化之后，得到的初始地图中的所有点都是局部地图点
        mvpLocalMapPoints = mpAtlas->GetAllMapPoints();
        mpReferenceKF = pKFcur;
        // 也只能这样子设置了，毕竟是最近的关键帧
        mCurrentFrame.mpReferenceKF = pKFcur;

        // Compute here initial velocity
        vector<KeyFrame *> vKFs = mpAtlas->GetAllKeyFrames();

        Sophus::SE3f deltaT = vKFs.back()->GetPose() * vKFs.front()->GetPoseInverse();
        mbVelocity = false;
        Eigen::Vector3f phi = deltaT.so3().log();

        double aux = (mCurrentFrame.mTimeStamp - mLastFrame.mTimeStamp) / (mCurrentFrame.mTimeStamp - mInitialFrame.mTimeStamp);
        phi *= aux;

        mLastFrame = Frame(mCurrentFrame);

        mpAtlas->SetReferenceMapPoints(mvpLocalMapPoints);

        mpMapDrawer->SetCurrentCameraPose(pKFcur->GetPose());

        mpAtlas->GetCurrentMap()->mvpKeyFrameOrigins.push_back(pKFini);

        // 初始化成功，至此，初始化过程完成！
        mState = OK;

        std::cout << "------------------ Initialization success! -------------------------" << std::endl;

        initID = pKFcur->mnId;
    }

    // -----------------------------------------------------------------------------------------------------------------------------

    // 在 Atlas 中保存当前地图，创建新地图，所有跟状态相关的变量全部重置
    /**
     * @brief 在 Atlas 中保存当前地图，创建新地图，所有跟状态相关的变量全部重置
     * 1. 前后两帧对应的时间戳反了
     * 2. imu 模式下前后帧超过 1s
     * 3. 上一帧为最近丢失且重定位失败时
     * 4. 重定位成功，局部地图跟踪失败
     */
    void Tracking::CreateMapInAtlas()
    {
        // 假如系统在 68 帧变为 LOST 状态，且关键帧总数＜10个，则从 69帧 开始新建地图！

        // ? TODO
        mnLastInitFrameId = mCurrentFrame.mnId; // mCurrentFrame.mnId 是 69，将它存储在 mnLastInitFrameId 中，表示新地图的初始化从这帧开始
        std::cout << "mCurrentFrame: " << mCurrentFrame.mnId << std::endl;
        std::cout << "mnLastInitFrameId: " << mnLastInitFrameId << std::endl;
        mpAtlas->CreateNewMap(); // 创建一个新的地图对象，这个新地图将记录从当前帧（第 69 帧）开始的所有地图数据

        // 如果系统使用了带 IMU（惯性测量单元）的传感器，系统就告诉 mpAtlas 这个新地图需要使用 IMU 数据来提升定位精度
        if (mSensor == System::IMU_STEREO || mSensor == System::IMU_MONOCULAR || mSensor == System::IMU_RGBD)
            mpAtlas->SetInertialSensor(); // mpAtlas 中 map 的 mbIsInertial = true

        mbSetInit = false; // 好像没什么用，重置初始化标志 mbSetInit 为 false，可能是为了标记初始化尚未完成。

        mnInitialFrameId = mCurrentFrame.mnId + 1; // ? 为什么要设置 mnInitialFrameId 为当前帧的 ID 加 1 ？
        std::cout << "mnInitialFrameId: " << mnInitialFrameId << std::endl;
        mState = NO_IMAGES_YET; // 系统状态设置为 NO_IMAGES_YET，表示当前还没有有效的图像输入

        // Restart the variable with information about the last KF
        mbVelocity = false; // 将 mbVelocity 标志重置为 false。mbVelocity 用于表示是否使用了速度模型，它在此被重置为初始状态

        // mnLastRelocFrameId = mnLastInitFrameId; // The last relocation KF_id is the current id, because it is the new starting point for new map
        Verbose::PrintMess("First frame id in map: " + to_string(mnLastInitFrameId + 1), Verbose::VERBOSITY_NORMAL); // 方便调试时查看当前地图从哪一帧开始构建。

        // 将 mbVO 设置为 false，表示目前系统没有足够的地图点来进行视觉里程计跟踪。通常在初始化时，系统会积累一定数量的地图点，才能开始进行有效的视觉里程计。
        mbVO = false;

        if (mSensor == System::MONOCULAR || mSensor == System::IMU_MONOCULAR)
        {
            mbReadyToInitializate = false; // 表示当前尚未准备好进行初始化
        }

        if ((mSensor == System::IMU_MONOCULAR || mSensor == System::IMU_STEREO || mSensor == System::IMU_RGBD) && mpImuPreintegratedFromLastKF)
        {
            delete mpImuPreintegratedFromLastKF;
            mpImuPreintegratedFromLastKF = new IMU::Preintegrated(IMU::Bias(), *mpImuCalib);
        }

        // 如果系统记录了上一个关键帧 mpLastKeyFrame，我们就把它清空。意味着我们现在开始新的地图，并且不再依赖之前的关键帧。
        if (mpLastKeyFrame)
            mpLastKeyFrame = static_cast<KeyFrame *>(NULL);

        // 同样地，如果有参考关键帧 mpReferenceKF，我们也将其清空
        if (mpReferenceKF)
            mpReferenceKF = static_cast<KeyFrame *>(NULL);

        // 清空 mLastFrame 和 mCurrentFrame，把它们重置为空帧对象，表示系统开始从第 69 帧（即 mCurrentFrame）开始新的初始化
        mLastFrame = Frame();
        mCurrentFrame = Frame();

        // 清空 mvIniMatches，这是用于初始化时的特征匹配点列表。初始化阶段通常会进行帧间匹配，通过匹配点来建立地图。
        mvIniMatches.clear();
        mlQueueImuData.clear(); // ? 还需要把 IMU 队列数据也清空？

        // 表示已经成功创建了地图
        mbCreatedMap = true;
    }

    // ps：检查上一帧中的地图点是否需要被替换
    /*
     * @brief 检查上一帧中的地图点是否需要被替换
     *
     * Local Mapping 线程可能会将关键帧中某些地图点进行替换，由于 tracking 中需要用到上一帧地图点，所以这里检查并更新上一帧中被替换的地图点
     * @see LocalMapping::SearchInNeighbors()
     */
    void Tracking::CheckReplacedInLastFrame()
    {
        // 遍历上一帧的所有地图点
        for (int i = 0; i < mLastFrame.N; i++)
        {
            MapPoint *pMP = mLastFrame.mvpMapPoints[i];

            // 如果这个地图点是否存在
            if (pMP)
            {
                // 获取其是否被替换，以及替换后的点
                // ps：这也是程序不直接删除这个地图点删除的原因
                MapPoint *pRep = pMP->GetReplaced();

                if (pRep)
                {
                    // 然后替换一下
                    mLastFrame.mvpMapPoints[i] = pRep;
                }
            }
        }
    }

    // -----------------------------------------------------------------------------------------------------------------------------

    // TODO 作用：跟踪线程的第 1 阶段跟踪 ——> 参考关键帧跟踪👇
    /*
     * @brief 用参考关键帧的地图点来对当前普通帧进行跟踪
     *
     * Step 1：将当前普通帧的描述子转化为BoW向量
     * Step 2：通过词袋BoW加速当前帧与参考帧之间的特征点匹配
     * Step 3: 将上一帧的位姿态作为当前帧位姿的初始值
     * Step 4: 通过优化3D-2D的重投影误差来获得位姿
     * Step 5：剔除优化后的匹配点中的外点
     * @return 如果匹配数超10，返回true
     *
     */
    bool Tracking::TrackReferenceKeyFrame()
    {
        // Verbose::PrintMess("Starting TrackReferenceKeyFrame", Verbose::VERBOSITY_NORMAL);
        std::cout << "Starting TrackReferenceKeyFrame" << std::endl;

        // Step 1：将当前帧的描述子转化为 BoW 向量
        mCurrentFrame.ComputeBoW();

        // We perform first an ORB matching with the reference keyframe
        // If enough matches are found we setup a PnP solver
        ORBmatcher matcher(0.7, true);
        vector<MapPoint *> vpMapPointMatches; // 存储匹配关系

        // Step 2：通过词袋 BoW 加速当前帧与参考帧之间的特征点匹配
        // ? 提问：vpMapPointMatches 的数据结构是怎样的？答：vector<MapPoint *>，每个 MapPoint 对应一个3D地图点，通常是视觉SLAM中用来表示空间中某个位置的特征点。
        int nmatches = matcher.SearchByBoW(mpReferenceKF, mCurrentFrame, vpMapPointMatches);

        // 若匹配数目小于 15，则认为跟踪失败
        if (nmatches < 15)
        {
            std::cout << "TRACK_REF_KF: Less than 15 matches!!\n";
            return false;
        }

        // Step 3：将上一帧的位姿态作为当前帧位姿的初始值，可以加速 BA 收敛
        mCurrentFrame.mvpMapPoints = vpMapPointMatches;
        mCurrentFrame.SetPose(mLastFrame.GetPose());

        // mCurrentFrame.PrintPointDistribution();

        // cout << " TrackReferenceKeyFrame mLastFrame.mTcw:  " << mLastFrame.mTcw << endl;

        // Step 4：通过优化 3D-2D 的重投影误差来获得位姿
        // 使用优化算法（通常是非线性优化）来最小化当前帧和参考关键帧之间的重投影误差。这个优化过程会基于匹配点来计算出当前帧的更准确位姿。
        // 即：这里的“优化”就是通过计算当前帧的特征点和参考帧的特征点之间的误差，不断调整当前帧的位置和角度，直到误差最小。
        Optimizer::PoseOptimization(&mCurrentFrame);

        // Step 5：剔除优化后的匹配点中的外点（outliers）
        // ? 提问：什么是外点？答：在两帧图像之间匹配时，出现的错误匹配点，而不是地图点本身。
        // 之所以在优化之后才剔除外点，是因为在优化的过程中就有了对这些外点的标记
        int nmatchesMap = 0;
        for (int i = 0; i < mCurrentFrame.N; i++)
        {
            // if(i >= mCurrentFrame.Nleft) break;
            if (mCurrentFrame.mvpMapPoints[i])
            {
                // 如果对应到的某个特征点是外点
                if (mCurrentFrame.mvbOutlier[i])
                {
                    // 清除它在当前帧中存在过的痕迹，即清除它的所有关系
                    MapPoint *pMP = mCurrentFrame.mvpMapPoints[i];
                    mCurrentFrame.mvpMapPoints[i] = static_cast<MapPoint *>(NULL);
                    mCurrentFrame.mvbOutlier[i] = false;

                    // 👇两个判断语句是新增的
                    if (i < mCurrentFrame.Nleft)
                    {
                        pMP->mbTrackInView = false;
                    }
                    else
                    {
                        pMP->mbTrackInViewR = false;
                    }

                    pMP->mbTrackInView = false;
                    pMP->mnLastFrameSeen = mCurrentFrame.mnId;
                    nmatches--;
                }

                // 如果不是外点
                else if (mCurrentFrame.mvpMapPoints[i]->Observations() > 0)
                {
                    // 匹配的内点计数++（累加成功匹配到的地图点数目）
                    nmatchesMap++;
                }
            }
        }

        // step 6：判断跟踪是否成功
        // case1：如果是 IMU 模式，则认为跟踪成功；
        if (mSensor == System::IMU_MONOCULAR || mSensor == System::IMU_STEREO || mSensor == System::IMU_RGBD)
            return true;
        // case2：如果是 纯视觉 模式，则只有在匹配的有效点数超过【10个】时，才认为跟踪成功；
        else
        {
            // Verbose::PrintMess("Finishing TrackReferenceKeyFrame", Verbose::VERBOSITY_NORMAL);
            std::cout << "Finishing TrackReferenceKeyFrame" << std::endl;
            return nmatchesMap >= 10;
        }
    }

    // TODO 作用：跟踪线程的第 1 阶段跟踪 ——> 恒速模型跟踪👇
    /**
     * @brief 根据恒定速度模型用上一帧地图点来对当前帧进行跟踪
     * Step 1：更新上一帧的位姿；对于双目或RGB-D相机，还会根据深度值生成临时地图点
     * Step 2：根据上一帧特征点对应地图点进行投影匹配
     * Step 3：优化当前帧位姿
     * Step 4：剔除地图点中外点
     * @return 如果匹配数大于10，认为跟踪成功，返回true
     */
    bool Tracking::TrackWithMotionModel()
    {
        // Verbose::PrintMess("Starting TrackWithMotionModel", Verbose::VERBOSITY_NORMAL);
        std::cout << "Starting TrackWithMotionModel" << std::endl;

        // 最小距离 < 0.9 * 次小距离，匹配成功，检查旋转
        ORBmatcher matcher(0.9, true);

        // Step 1：更新上一帧的位姿；对于双目或 RGB-D 相机，还会根据深度值生成临时地图点
        // Update last frame pose according to its reference keyframe
        // Create "visual odometry" points if in Localization Mode
        UpdateLastFrame(); // 通过参考上一帧的关键帧来估计当前帧的初始位姿

        // Step 2：根据 IMU 或者恒速模型，得到当前帧的初始位姿。
        // case1：如果是 IMU 模，并且完成了第一阶段初始化，并且当前帧距离上一次重定位帧时间很久，也不需要重置 IMU，则用 IMU 来估计位姿
        if (mpAtlas->isImuInitialized() && (mCurrentFrame.mnId > mnLastRelocFrameId + mnFramesToResetIMU))
        {
            PredictStateIMU();
            return true;
        }
        // case2：如果是 纯视觉 模式，则假设移动的速度是恒定的，通过 上一帧的位姿和速度 来预测当前帧的位姿。
        else
        {
            // 根据之前估计的速度，用恒速模型得到当前帧的初始位姿。
            mCurrentFrame.SetPose(mVelocity * mLastFrame.GetPose());
        }

        // 清空当前帧的地图点，就是用 NULL 来填充所有地图点啊！
        // ? 为什么需要清空当前帧的地图点？
        // ? 答：当前帧刚开始时，mCurrentFrame.mvpMapPoints 可能包含一些之前的地图点（比如来自上一帧的匹配点）。这些点可能是 过时的 或者 无关的，而在本次跟踪过程中，需要重新为当前帧匹配新的地图点。
        fill(mCurrentFrame.mvpMapPoints.begin(), mCurrentFrame.mvpMapPoints.end(), static_cast<MapPoint *>(NULL));

        // Step 3：用上一帧地图点进行投影匹配，如果匹配点不够，则扩大搜索半径再来一次
        int th; // 设置特征匹配过程中的搜索半径

        if (mSensor == System::STEREO)
            th = 7;
        else
            th = 15;

        // 将上一帧的地图点“投影”到当前帧上，进行匹配，找出当前帧上哪些点和上一帧上的点是相对应的。
        int nmatches = matcher.SearchByProjection(mCurrentFrame, mLastFrame, th, mSensor == System::MONOCULAR || mSensor == System::IMU_MONOCULAR);

        // 如果匹配点太少，则扩大搜索半径再来一次
        if (nmatches < 20)
        {
            Verbose::PrintMess("Not enough matches, wider window search!!", Verbose::VERBOSITY_NORMAL);
            fill(mCurrentFrame.mvpMapPoints.begin(), mCurrentFrame.mvpMapPoints.end(), static_cast<MapPoint *>(NULL));

            nmatches = matcher.SearchByProjection(mCurrentFrame, mLastFrame, 2 * th, mSensor == System::MONOCULAR || mSensor == System::IMU_MONOCULAR);
            Verbose::PrintMess("Matches with wider search: " + to_string(nmatches), Verbose::VERBOSITY_NORMAL);
        }

        if (nmatches < 20)
        {
            Verbose::PrintMess("Not enough matches!!", Verbose::VERBOSITY_NORMAL);

            if (mSensor == System::IMU_MONOCULAR || mSensor == System::IMU_STEREO || mSensor == System::IMU_RGBD)
                return true; // ? 提问：都不到 20 个点了，还认为跟踪成功啊？答：因为 IMU 不需要用匹配点进行估计位姿，所以这里不需要关注匹配点数量！
            else
                return false;
        }

        // Step 4：利用 3D-2D 投影关系，优化当前帧位姿
        // Optimize frame pose with all matches
        Optimizer::PoseOptimization(&mCurrentFrame);

        // Step 5：剔除地图点中外点
        int nmatchesMap = 0;
        for (int i = 0; i < mCurrentFrame.N; i++)
        {
            // 如果对应到的某个特征点是外点
            if (mCurrentFrame.mvpMapPoints[i])
            {
                if (mCurrentFrame.mvbOutlier[i])
                {
                    // 清除它在当前帧中存在过的痕迹，即清除它的所有关系
                    MapPoint *pMP = mCurrentFrame.mvpMapPoints[i];
                    mCurrentFrame.mvpMapPoints[i] = static_cast<MapPoint *>(NULL);
                    mCurrentFrame.mvbOutlier[i] = false;

                    // 👇两个判断语句是新增的
                    if (i < mCurrentFrame.Nleft)
                    {
                        pMP->mbTrackInView = false;
                    }
                    else
                    {
                        pMP->mbTrackInViewR = false;
                    }

                    pMP->mnLastFrameSeen = mCurrentFrame.mnId;
                    nmatches--;
                }

                // 如果不是外点
                else if (mCurrentFrame.mvpMapPoints[i]->Observations() > 0)
                    nmatchesMap++; // 累加成功匹配到的地图点数目
            }
        }

        // 纯定位模式下：如果成功追踪的地图点非常少，那么这里的 mbVO 标志就会置位
        if (mbOnlyTracking)
        {
            mbVO = nmatchesMap < 10;
            return nmatches > 20;
        }

        // step 6：判断跟踪是否成功
        // case1：如果是 IMU 模式，则认为跟踪成功；
        if (mSensor == System::IMU_MONOCULAR || mSensor == System::IMU_STEREO || mSensor == System::IMU_RGBD)
            return true;
        // case2：如果是 纯视觉 模式，则只有在匹配的有效点数超过【10个】时，才认为跟踪成功；
        else
        {
            // Verbose::PrintMess("Finishing TrackWithMotionModel!!", Verbose::VERBOSITY_NORMAL);
            std::cout << "Finishing TrackWithMotionModel" << std::endl;
            return nmatchesMap >= 10;
        }
    }

    // TODO 作用：跟踪线程的第 1 阶段跟踪 ——> 重定位跟踪👇
    /**
     * @details 重定位过程
     * @return true
     * @return false
     *
     * Step 1：计算当前帧特征点的词袋向量
     * Step 2：找到与当前帧相似的候选关键帧
     * Step 3：通过 BoW 进行匹配
     * Step 4：通过 EPnP 算法估计姿态
     * Step 5：通过 PoseOptimization 对姿态进行优化求解
     * Step 6：如果内点较少，则通过投影的方式对之前未匹配的点进行匹配，再进行优化求解
     */
    bool Tracking::Relocalization()
    {
        // Verbose::PrintMess("Starting relocalization", Verbose::VERBOSITY_NORMAL);
        std::cout << "Starting relocalization" << std::endl;

        // Step 1: 计算当前帧特征点的 Bow
        mCurrentFrame.ComputeBoW();

        // Step 2：用词袋找到与当前帧相似的候选关键帧组
        // Relocalization is performed when tracking is lost
        // Track Lost: Query KeyFrame Database for keyframe candidates for relocalisation
        // ? 注意是在【当前地图】中，而论文中写的是在【所有地图】中
        vector<KeyFrame *> vpCandidateKFs = mpKeyFrameDB->DetectRelocalizationCandidates(&mCurrentFrame, mpAtlas->GetCurrentMap());

        // 如果没有候选关键帧，则退出
        if (vpCandidateKFs.empty())
        {
            Verbose::PrintMess("There are not candidates", Verbose::VERBOSITY_NORMAL);
            return false;
        }

        const int nKFs = vpCandidateKFs.size();

        // We perform first an ORB matching with each candidate
        // If enough matches are found we setup a PnP solver
        ORBmatcher matcher(0.75, true);

        // 为每个关键帧构造一个 MLPnPsolvers 解算器，用来估计相机位姿
        vector<MLPnPsolver *> vpMLPnPsolvers;
        vpMLPnPsolvers.resize(nKFs);

        // 每个关键帧和当前帧中特征点的匹配关系
        vector<vector<MapPoint *>> vvpMapPointMatches; // ? 提问：vvpMapPointMatches 元素结构是怎样的？
        vvpMapPointMatches.resize(nKFs);

        // 放弃某个关键帧的标记
        vector<bool> vbDiscarded;
        vbDiscarded.resize(nKFs);

        // 有效的候选关键帧数目
        int nCandidates = 0;

        // Step 3：遍历所有的候选关键帧，通过 BoW 进行快速匹配，用匹配结果初始化 PnP Solver
        for (int i = 0; i < nKFs; i++)
        {
            KeyFrame *pKF = vpCandidateKFs[i];

            if (pKF->isBad())
                vbDiscarded[i] = true;

            else
            {
                // ps：当前帧和候选关键帧用【词袋】进行快速匹配，匹配结果记录在 vvpMapPointMatches，nmatches 表示匹配的数目
                int nmatches = matcher.SearchByBoW(pKF, mCurrentFrame, vvpMapPointMatches[i]);

                // 如果和当前帧的匹配数小于 15，那么只能放弃这个关键帧
                if (nmatches < 15)
                {
                    vbDiscarded[i] = true;
                    continue;
                }
                // 当前帧与候选帧的匹配点数超过 15个，就用匹配结果初始化解算器（PnP + RANSAC）
                else
                {
                    // ? 为什么用 MLPnP? 因为考虑了鱼眼相机模型，解耦某些关系？
                    // 参考论文《MLPNP-A REAL-TIME MAXIMUM LIKELIHOOD SOLUTION TO THE PERSPECTIVE-N-POINT PROBLEM》
                    MLPnPsolver *pSolver = new MLPnPsolver(mCurrentFrame, vvpMapPointMatches[i]);

                    // 构造函数调用了一遍，这里重新设置参数
                    pSolver->SetRansacParameters(0.99, 10, 300, 6, 0.5, 5.991); // This solver needs at least 6 points
                    vpMLPnPsolvers[i] = pSolver;
                    nCandidates++; // 1.0 版本新加的
                }
            }
        }

        // 足够的内点才能匹配使用 PNP 算法，而 MLPnP 需要至少【6对点】
        // Alternatively perform some iterations of P4P RANSAC
        // Until we found a camera pose supported by enough inliers
        bool bMatch = false; // 是否已经找到相匹配的关键帧的标志。如果找到合适的候选关键帧，bMatch 将设为 true，循环将结束。
        ORBmatcher matcher2(0.9, true);

        // Step 4: 通过一系列操作，直到找到能够匹配上的关键帧。
        // ? 为什么搞这么复杂？答：是担心误闭环
        while (nCandidates > 0 && !bMatch)
        {
            // 逐一遍历当前所有的候选关键帧
            for (int i = 0; i < nKFs; i++)
            {
                if (vbDiscarded[i])
                    continue;

                vector<bool> vbInliers; // 内点标记，内点是指那些能够满足几何约束、与模型（如位姿或运动模型）一致的匹配点。

                // 通过 MLPnP 算法估计姿态，迭代 5 次
                int nInliers;
                bool bNoMore; // 表示 RANSAC 已经没有更多的迭代次数可用 -- 也就是说数据不够好，RANSAC也已经尽力了。。。
                MLPnPsolver *pSolver = vpMLPnPsolvers[i];
                Eigen::Matrix4f eigTcw;

                // ps：使用 MLPnP 进行位姿估计，bTcw -- 估计出来的当前帧的位姿
                bool bTcw = pSolver->iterate(5, bNoMore, vbInliers, nInliers, eigTcw);

                // case1：解算当前帧位姿未成功
                if (bNoMore) // bNoMore 为 true 表示已经超过了 RANSAC 最大迭代次数，就放弃当前关键帧
                {
                    vbDiscarded[i] = true;
                    nCandidates--;
                }

                // case2：如果 MLPnP 估计出了当前帧的位姿，然后对内点进行 BA 优化
                if (bTcw)
                {
                    Sophus::SE3f Tcw(eigTcw);
                    mCurrentFrame.SetPose(Tcw); // 把当前帧位姿初始化为 解算出来的位姿，后面还要优化，加快收敛
                    // Tcw.copyTo(mCurrentFrame.mTcw);

                    // MLPnP 里 RANSAC 后的内点的集合
                    set<MapPoint *> sFound;
                    const int np = vbInliers.size();

                    // 遍历所有内点
                    for (int j = 0; j < np; j++)
                    {
                        if (vbInliers[j])
                        {
                            mCurrentFrame.mvpMapPoints[j] = vvpMapPointMatches[i][j];
                            sFound.insert(vvpMapPointMatches[i][j]);
                        }
                        else
                            mCurrentFrame.mvpMapPoints[j] = NULL;
                    }

                    // 只优化位姿，不优化地图点的坐标，返回的是内点的数量
                    int nGood = Optimizer::PoseOptimization(&mCurrentFrame);

                    // 如果优化之后的内点数目不多，跳过了当前候选关键帧，但是却没有放弃当前帧的重定位
                    if (nGood < 10)
                        continue;

                    // 删除外点对应的地图点，这里直接设为空指针
                    for (int io = 0; io < mCurrentFrame.N; io++)
                        if (mCurrentFrame.mvbOutlier[io])
                            mCurrentFrame.mvpMapPoints[io] = static_cast<MapPoint *>(NULL);

                    // Step 4.3：如果内点较少，则通过【投影】的方式对之前未匹配的点进行匹配，再进行优化求解
                    // ps：前面的匹配关系是用【词袋匹配】过程得到的
                    if (nGood < 50)
                    {
                        // 通过【投影】的方式将关键帧中未匹配的地图点投影到当前帧中, 生成新的匹配
                        int nadditional = matcher2.SearchByProjection(mCurrentFrame, vpCandidateKFs[i], sFound, 10, 100);

                        // 如果通过投影过程新增了比较多的匹配特征点对
                        if (nadditional + nGood >= 50)
                        {
                            // 根据投影匹配的结果，再次采用 3D-2D pnp BA 优化位姿
                            nGood = Optimizer::PoseOptimization(&mCurrentFrame);

                            // Step 4.4：如果 BA 后内点数还是比较少(<50)但是还不至于太少(>30)，可以挽救一下, 最后垂死挣扎
                            // 重新执行上一步 4.3 的过程，只不过使用更小的搜索窗口
                            // 这里的位姿已经使用了更多的点进行了优化，应该更准，所以使用更小的窗口搜索
                            // If many inliers but still not enough, search by projection again in a narrower window
                            // the camera has been already optimized with many points
                            if (nGood > 30 && nGood < 50)
                            {
                                // 用更小窗口、更严格的描述子阈值，重新进行投影搜索匹配
                                sFound.clear();
                                for (int ip = 0; ip < mCurrentFrame.N; ip++)
                                    if (mCurrentFrame.mvpMapPoints[ip])
                                        sFound.insert(mCurrentFrame.mvpMapPoints[ip]);
                                nadditional = matcher2.SearchByProjection(mCurrentFrame, vpCandidateKFs[i], sFound, 3, 64);

                                // 如果成功挽救回来，匹配数目达到要求，最后BA优化一下
                                // Final optimization
                                if (nGood + nadditional >= 50)
                                {
                                    nGood = Optimizer::PoseOptimization(&mCurrentFrame);

                                    // 更新地图点
                                    for (int io = 0; io < mCurrentFrame.N; io++)
                                        if (mCurrentFrame.mvbOutlier[io])
                                            mCurrentFrame.mvpMapPoints[io] = NULL;
                                }
                                // 如果还是不能够满足就放弃了
                            }
                        }
                    }

                    // 如果对于当前的候选关键帧已经有足够的内点(50个)了，那么就认为重定位成功
                    if (nGood >= 50)
                    {
                        bMatch = true;

                        std::cout << "The keyframe id for which the relocation was successful: " << vpCandidateKFs[i]->mnId << std::endl;

                        // 只要有一个候选关键帧重定位成功，就退出循环，不考虑其他候选关键帧了
                        break;
                    }
                }
            }
            // 一直运行，直到已经没有足够的关键帧，或者是已经有成功匹配上的关键帧
        }

        // 折腾了这么久还是没有匹配上，重定位失败
        if (!bMatch)
        {
            return false;
        }
        else
        {
            // 如果匹配上了，说明当前帧重定位成功了(当前帧已经有了自己的位姿)
            // 记录成功重定位帧的 id，防止短时间多次重定位
            mnLastRelocFrameId = mCurrentFrame.mnId;

            // Verbose::PrintMess("Finishing relocalization!!", Verbose::VERBOSITY_NORMAL);
            std::cout << " Finishing relocalization !!" << std::endl;

            return true;
        }
    }

    // TODO 作用：跟踪线程的第 2 阶段跟踪 ——> 局部地图跟踪👇
    /**
     * @brief 用局部地图进行跟踪，进一步优化位姿，即通过当前帧的特征点与局部地图点匹配，优化当前帧的位姿并统计跟踪效果
     *
     * 1. 更新局部地图，包括局部关键帧和局部地图点
     * 2. 对局部 MapPoints 进行投影匹配
     * 3. 根据匹配对估计当前帧的姿态
     * 4. 根据姿态剔除误匹配
     * @return true if success
     *
     * Step 1：更新局部关键帧 mvpLocalKeyFrames 和局部地图点 mvpLocalMapPoints
     * Step 2：在局部地图中查找与当前帧匹配的 MapPoints, 其实也就是对局部地图点进行跟踪
     * Step 3：更新局部所有 MapPoints 后对位姿再次优化
     * Step 4：更新当前帧的 MapPoints 被观测程度，并统计跟踪局部地图的效果
     * Step 5：决定是否跟踪成功
     */
    bool Tracking::TrackLocalMap()
    {
        // Verbose::PrintMess("Starting TrackLocalMap", Verbose::VERBOSITY_NORMAL);
        std::cout << "Starting TrackLocalMap" << std::endl;

        // We have an estimation of the camera pose and some map points tracked in the frame.
        // We retrieve the local map and try to find matches to points in the local map.
        mTrackedFr++; // 跟踪的帧计数器，表示当前帧的编号

        // Step 1：更新局部关键帧 mvpLocalKeyFrames 和局部地图点 mvpLocalMapPoints
        UpdateLocalMap();

        // Step 2：筛选局部地图中新增的在视野范围内的地图点，投影到当前帧搜索匹配，得到更多的匹配关系
        SearchLocalPoints();

        // 查看内外点数目，调试用
        // TOO check outliers before PO
        int aux1 = 0, aux2 = 0; // aux1 和 aux2 用于统计当前帧中地图点的数量（aux1）和外点的数量（aux2）
        for (int i = 0; i < mCurrentFrame.N; i++)
            if (mCurrentFrame.mvpMapPoints[i]) // 检查当前帧是否存在地图点
            {
                aux1++;
                if (mCurrentFrame.mvbOutlier[i]) // 检查该地图点是否被标记为外点
                    aux2++;
            }
        std::cout << "内点数目: " << aux1 << ", 外点数目: " << aux2 << std::endl;

        // Step 3：前面新增了更多的匹配关系，BA 优化得到更准确的位姿
        // Optimize Pose 在这个函数之前，在 Relocalization、TrackReferenceKeyFrame、TrackWithMotionModel 中都有位姿优化
        int inliers;

        // case1：视觉优化（非 IMU 模式） --- 如果 IMU 未初始化，那么仅使用【视觉信息】来优化当前帧的位姿
        if (!mpAtlas->isImuInitialized())
        {
            Optimizer::PoseOptimization(&mCurrentFrame);
        }

        // case2：视觉-惯性优化（IMU 模式） --- 如果 IMU 完成初始化，考虑采用【视觉 + IMU】联合优化
        else
        {
            // case1. 如果当前帧 ID 小于上一次重定位帧 ID 加上重置 IMU 的帧数，一般取 1s，那么仅使用【视觉信息】来优化当前帧的位姿
            if (mCurrentFrame.mnId <= mnLastRelocFrameId + mnFramesToResetIMU)
            {
                Verbose::PrintMess("TLM: PoseOptimization ", Verbose::VERBOSITY_DEBUG);
                Optimizer::PoseOptimization(&mCurrentFrame);
            }

            // case2. 如果积累的 IMU 数据量比较多，则使用【视觉 + IMU 数据】进行优化，包括位姿、速度和偏置
            else
            {
                // if(!mbMapUpdated && mState == OK) //  && (mnMatchesInliers>30))
                // mbMapUpdated 变化见 Tracking::PredictStateIMU()
                // 如果IMU已经初始化且地图未更新
                if (!mbMapUpdated) //  && (mnMatchesInliers>30))
                {
                    Verbose::PrintMess("TLM: PoseInertialOptimizationLastFrame ", Verbose::VERBOSITY_DEBUG);
                    // * 使用上一【普通帧】以及当前帧的视觉信息和 IMU 信息联合优化当前帧位姿、速度和 IMU 零偏
                    inliers = Optimizer::PoseInertialOptimizationLastFrame(&mCurrentFrame); // , !mpLastKeyFrame->GetMap()->GetIniertialBA1());
                }
                // 如果地图已经更新
                else
                {
                    Verbose::PrintMess("TLM: PoseInertialOptimizationLastKeyFrame ", Verbose::VERBOSITY_DEBUG);
                    // * 使用上一【关键帧】以及当前帧的视觉信息和 IMU 信息联合优化当前帧位姿、速度和 IMU 零偏
                    inliers = Optimizer::PoseInertialOptimizationLastKeyFrame(&mCurrentFrame); // , !mpLastKeyFrame->GetMap()->GetIniertialBA1());
                }
            }
        }

        // 查看内外点数目，原作者调试用的，可能忘记删了
        aux1 = 0, aux2 = 0;
        for (int i = 0; i < mCurrentFrame.N; i++)
            if (mCurrentFrame.mvpMapPoints[i])
            {
                aux1++;
                if (mCurrentFrame.mvbOutlier[i])
                    aux2++;
            }
        std::cout << "优化后内点数目: " << aux1 << ", 外点数目: " << aux2 << std::endl;

        mnMatchesInliers = 0; // 表示当前帧通过局部地图跟踪匹配到的内点（经过重投影误差或其他约束（例如RANSAC优化）验证后的有效匹配点）数量，即当前帧和局部地图点成功匹配的特征点数量

        // Step 4：更新当前帧的地图点被观测程度，并统计跟踪局部地图后匹配数目
        for (int i = 0; i < mCurrentFrame.N; i++)
        {
            if (mCurrentFrame.mvpMapPoints[i])
            {
                // 由于当前帧的地图点可以被当前帧观测到，其被观测统计量 +1
                if (!mCurrentFrame.mvbOutlier[i])
                {
                    // 如果地图点不是外点，则调用 IncreaseFound() 增加该地图点的观察次数。即：更新地图点被当前帧观测到的次数
                    mCurrentFrame.mvpMapPoints[i]->IncreaseFound();

                    // 查看当前是否是在纯定位过程
                    if (!mbOnlyTracking)
                    {
                        // 如果该地图点被相机观测数目 nObs 大于0，则匹配内点计数 +1
                        // observations() 用于检查该地图点被观察到的次数，如果大于0，则说明这个地图点是有效的并参与了匹配
                        // nObs： 被观测到的相机数目，单目+1，双目或 RGB-D 则+2
                        if (mCurrentFrame.mvpMapPoints[i]->Observations() > 0)
                            mnMatchesInliers++;
                    }
                    else
                    {
                        // 记录当前帧跟踪到的地图点数目，用于统计跟踪效果
                        mnMatchesInliers++;
                    }
                }

                // 如果这个地图点是外点，并且当前相机输入还是双目的时候，就直接剔除这个点，因为双目相机本身可以通过左右图像匹配生成新地图点，删掉无所谓。
                else if (mSensor == System::STEREO)
                {
                    mCurrentFrame.mvpMapPoints[i] = static_cast<MapPoint *>(NULL);
                }
            }
        }

        // ps：决定跟踪是否成功
        // More restrictive if there was a relocalization recently
        mpLocalMapper->mnMatchesInliers = mnMatchesInliers;

        // Step 5：根据跟踪匹配数目及重定位情况决定是否跟踪成功
        std::cout << "mState: " << mState << "，局部地图跟踪匹配数目: " << mnMatchesInliers << std::endl;

        // 如果最近刚刚发生了重定位，并且内点数量少于 50，那么至少成功匹配 50 个点才认为是成功跟踪
        if (mCurrentFrame.mnId < mnLastRelocFrameId + mMaxFrames && mnMatchesInliers < 50)
        {
            // Verbose::PrintMess("Finishing TrackLocalMap", Verbose::VERBOSITY_NORMAL);
            std::cout << "Finishing TrackLocalMap1" << std::endl;
            return false;
        }

        // RECENTLY_LOST 状态下，至少成功跟踪 10 个才算成功
        if ((mnMatchesInliers > 10) && (mState == RECENTLY_LOST))
        {
            // Verbose::PrintMess("Finishing TrackLocalMap", Verbose::VERBOSITY_NORMAL);
            std::cout << "Finishing TrackLocalMap2" << std::endl;
            return true;
        }

        // 单目+IMU 模式下做完初始化至少成功跟踪 15 个才算成功，没做初始化需要 50 个
        if (mSensor == System::IMU_MONOCULAR)
        {
            if ((mnMatchesInliers < 15 && mpAtlas->isImuInitialized()) || (mnMatchesInliers < 50 && !mpAtlas->isImuInitialized()))
            {
                // Verbose::PrintMess("Finishing TrackLocalMap", Verbose::VERBOSITY_NORMAL);
                std::cout << "Finishing TrackLocalMap3" << std::endl;
                return false;
            }
            else
            {
                // Verbose::PrintMess("Finishing TrackLocalMap", Verbose::VERBOSITY_NORMAL);
                std::cout << "Finishing TrackLocalMap4" << std::endl;
                return true;
            }
        }
        else if (mSensor == System::IMU_STEREO || mSensor == System::IMU_RGBD)
        {
            if (mnMatchesInliers < 15)
            {
                // Verbose::PrintMess("Finishing TrackLocalMap", Verbose::VERBOSITY_NORMAL);
                std::cout << "Finishing TrackLocalMap5" << std::endl;
                return false;
            }
            else
            {
                // Verbose::PrintMess("Finishing TrackLocalMap", Verbose::VERBOSITY_NORMAL);
                std::cout << "Finishing TrackLocalMap6" << std::endl;
                return true;
            }
        }
        // 以上情况都不满足，只要跟踪的地图点大于 30 个就认为成功了
        else
        {
            if (mnMatchesInliers < 30)
            {
                // Verbose::PrintMess("Finishing TrackLocalMap", Verbose::VERBOSITY_NORMAL);
                std::cout << "Finishing TrackLocalMap7" << std::endl;
                return false;
            }
            else
            {
                // Verbose::PrintMess("Finishing TrackLocalMap", Verbose::VERBOSITY_NORMAL);
                std::cout << "Finishing TrackLocalMap8" << std::endl;
                return true;
            }
        }
    }

    // -----------------------------------------------------------------------------------------------------------------------------

    // todo1：更新局部地图
    void Tracking::UpdateLocalMap()
    {
        // 局部地图包括：
        // 1、K1个关键帧、K2个临近关键帧和参考关键帧
        // 2、由这些关键帧观测到的 MapPoints

        // 将当前的局部地图点 mvpLocalMapPoints 设置为参考地图点，用于地图的可视化或绘图显示（例如红色点表示局部地图点）
        mpAtlas->SetReferenceMapPoints(mvpLocalMapPoints);

        // 用共视图来更新局部关键帧和局部地图点
        UpdateLocalKeyFrames();
        UpdateLocalPoints();
    }

    // todo2：更新局部地图点。先把局部地图清空，然后将局部关键帧的有效地图点添加到局部地图中
    void Tracking::UpdateLocalPoints()
    {
        // Step 1：清空局部地图点
        mvpLocalMapPoints.clear();

        int count_pts = 0;

        // Step 2：遍历局部关键帧 mvpLocalKeyFrames
        for (vector<KeyFrame *>::const_reverse_iterator itKF = mvpLocalKeyFrames.rbegin(), itEndKF = mvpLocalKeyFrames.rend(); itKF != itEndKF; ++itKF)
        {
            KeyFrame *pKF = *itKF;
            const vector<MapPoint *> vpMPs = pKF->GetMapPointMatches();

            // step 3：将局部关键帧的地图点添加到 mvpLocalMapPoints
            for (vector<MapPoint *>::const_iterator itMP = vpMPs.begin(), itEndMP = vpMPs.end(); itMP != itEndMP; itMP++)
            {

                MapPoint *pMP = *itMP;

                if (!pMP)
                    continue;

                // 用该地图点的成员变量 mnTrackReferenceForFrame 记录当前帧的 id
                // 表示它已经是当前帧的局部地图点了，可以防止重复添加局部地图点
                // mnTrackReferenceForFrame 是地图点的一个成员变量，用于记录它最后一次被添加到局部地图时的帧 ID。
                // 如果当前帧的 ID 已经存在，说明这个地图点已经在局部地图中了，避免重复添加。
                if (pMP->mnTrackReferenceForFrame == mCurrentFrame.mnId)
                    continue;

                if (!pMP->isBad())
                {
                    count_pts++;
                    mvpLocalMapPoints.push_back(pMP);
                    pMP->mnTrackReferenceForFrame = mCurrentFrame.mnId;
                }
            }
        }
    }

    // todo3：更新局部关键帧
    /**
     * @brief 跟踪局部地图函数里，更新局部关键帧
     *          方法是遍历当前帧的地图点，将观测到这些地图点的关键帧和相邻的关键帧及其父子关键帧，作为 mvpLocalKeyFrames
     *              Step 1：遍历当前帧的地图点，记录所有能观测到当前帧地图点的关键帧
     *              Step 2：更新局部关键帧（mvpLocalKeyFrames），添加局部关键帧包括以下3种类型
     *                  类型1：能观测到当前帧地图点的关键帧，也称一级共视关键帧
     *                  类型2：一级共视关键帧的共视关键帧，称为二级共视关键帧
     *                  类型3：一级共视关键帧的子关键帧、父关键帧
     *              Step 3：更新当前帧的参考关键帧，与自己共视程度最高的关键帧作为参考关键帧
     */
    void Tracking::UpdateLocalKeyFrames()
    {
        // Step 1：遍历当前帧的地图点，记录所有能观测到当前帧地图点的关键帧
        map<KeyFrame *, int> keyframeCounter;

        // case1：如果 IMU 未初始化 或者 刚刚完成重定位
        if (!mpAtlas->isImuInitialized() || (mCurrentFrame.mnId < mnLastRelocFrameId + 2))
        {
            for (int i = 0; i < mCurrentFrame.N; i++)
            {
                MapPoint *pMP = mCurrentFrame.mvpMapPoints[i];

                if (pMP)
                {
                    if (!pMP->isBad())
                    {
                        // 得到观测到该地图点的关键帧和该地图点在关键帧中的索引
                        const map<KeyFrame *, tuple<int, int>> observations = pMP->GetObservations();

                        // 由于一个地图点可以被多个关键帧观测到，因此对于每一次观测，都对观测到这个地图点的关键帧进行累计投票
                        for (map<KeyFrame *, tuple<int, int>>::const_iterator it = observations.begin(), itend = observations.end(); it != itend; it++)
                        {
                            // 这里的操作非常精彩！
                            // map[key] = value，当要插入的键存在时，会覆盖键对应的原来的值。如果键不存在，则添加一组键值对
                            // it->first 是地图点看到的关键帧，同一个关键帧看到的地图点会累加到该关键帧计数
                            // 所以最后 keyframeCounter 第一个参数表示某个关键帧，第2个参数表示该关键帧看到了多少当前帧(mCurrentFrame)的地图点，也就是共视程度
                            keyframeCounter[it->first]++;
                            // 比如，MP1 被第10帧、第12帧、第13帧观测到，那么：
                            // 投票：keyframeCounter[10]++，keyframeCounter[12]++，keyframeCounter[13]++。
                        }
                    }
                    else
                    {
                        mCurrentFrame.mvpMapPoints[i] = NULL;
                    }
                }
            }
        }
        // case2：IMU 完成初始化
        else
        {
            // ? 为什么 IMU 初始化后用 mLastFrame？mLastFrame 存储的是上一帧跟踪成功后帧数据。
            for (int i = 0; i < mLastFrame.N; i++)
            {
                // Using lastframe since current frame has not matches yet
                if (mLastFrame.mvpMapPoints[i])
                {
                    MapPoint *pMP = mLastFrame.mvpMapPoints[i];
                    if (!pMP)
                        continue;
                    if (!pMP->isBad())
                    {
                        const map<KeyFrame *, tuple<int, int>> observations = pMP->GetObservations();
                        for (map<KeyFrame *, tuple<int, int>>::const_iterator it = observations.begin(), itend = observations.end(); it != itend; it++)
                            keyframeCounter[it->first]++;
                    }
                    else
                    {
                        // MODIFICATION
                        mLastFrame.mvpMapPoints[i] = NULL;
                    }
                }
            }
        }

        // 找出与当前帧共享地图点最多的关键帧（pKFmax）
        int max = 0;
        KeyFrame *pKFmax = static_cast<KeyFrame *>(NULL);

        // Step 2：更新局部关键帧（mvpLocalKeyFrames），添加局部关键帧有 3 种类型
        mvpLocalKeyFrames.clear();                             // 清空局部关键帧
        mvpLocalKeyFrames.reserve(3 * keyframeCounter.size()); // 先申请 3 倍内存

        // Step 2.1 将所有共享地图点的关键帧加入局部关键帧列表

        // 类型1：能观测到当前帧地图点的关键帧作为局部关键帧【一级共视关键帧】 （将邻居拉拢入伙）
        for (map<KeyFrame *, int>::const_iterator it = keyframeCounter.begin(), itEnd = keyframeCounter.end(); it != itEnd; it++)
        {
            KeyFrame *pKF = it->first;

            if (pKF->isBad())
                continue;

            // 寻找具有最大观测数目的关键帧
            if (it->second > max)
            {
                max = it->second;
                pKFmax = pKF;
            }

            // 添加到局部关键帧的列表里
            mvpLocalKeyFrames.push_back(pKF);
            // 标记防止重复添加
            pKF->mnTrackReferenceForFrame = mCurrentFrame.mnId; // 加入第10帧，设置 10->mnTrackReferenceForFrame = 15。加入第12帧，设置 12->mnTrackReferenceForFrame = 15。
        }

        // ps：--------------------------- 到此为止，mvpLocalKeyFrames 里面有：一级共视关键帧

        // Step 2.2 遍历一级共视关键帧，寻找更多的局部关键帧
        for (vector<KeyFrame *>::const_iterator itKF = mvpLocalKeyFrames.begin(), itEndKF = mvpLocalKeyFrames.end(); itKF != itEndKF; itKF++)
        {
            // 处理的局部关键帧不超过 80 帧
            if (mvpLocalKeyFrames.size() > 80)
                break;

            KeyFrame *pKF = *itKF;

            // 类型2：一级共视关键帧的共视（前10个）关键帧，称为【二级共视关键帧】（将邻居的邻居拉拢入伙）
            // 如果共视帧不足 10 帧，那么就返回所有具有共视关系的关键帧
            const vector<KeyFrame *> vNeighs = pKF->GetBestCovisibilityKeyFrames(10);

            // vNeighs 是按照共视程度从大到小排列
            for (vector<KeyFrame *>::const_iterator itNeighKF = vNeighs.begin(), itEndNeighKF = vNeighs.end(); itNeighKF != itEndNeighKF; itNeighKF++)
            {
                KeyFrame *pNeighKF = *itNeighKF;
                if (!pNeighKF->isBad())
                {
                    // mnTrackReferenceForFrame 防止重复添加局部关键帧
                    if (pNeighKF->mnTrackReferenceForFrame != mCurrentFrame.mnId)
                    {
                        mvpLocalKeyFrames.push_back(pNeighKF);
                        pNeighKF->mnTrackReferenceForFrame = mCurrentFrame.mnId;
                        break;
                    }
                }
            }

            // ps：--------------------------- 到此为止，mvpLocalKeyFrames 里面有：一级共视关键帧 + 二级共视关键帧

            // 类型3：将一级共视关键帧的子关键帧作为局部关键帧（将邻居的孩子们拉拢入伙）
            const set<KeyFrame *> spChilds = pKF->GetChilds();
            for (set<KeyFrame *>::const_iterator sit = spChilds.begin(), send = spChilds.end(); sit != send; sit++)
            {
                KeyFrame *pChildKF = *sit;

                if (!pChildKF->isBad())
                {
                    if (pChildKF->mnTrackReferenceForFrame != mCurrentFrame.mnId)
                    {
                        mvpLocalKeyFrames.push_back(pChildKF);
                        pChildKF->mnTrackReferenceForFrame = mCurrentFrame.mnId;
                        break;
                    }
                }
            }

            // ps：--------------------------- 到此为止，mvpLocalKeyFrames 里面有：一级共视关键帧 + 二级共视关键帧 + 一级共视关键帧的子关键帧

            // 类型3：将一级共视关键帧的父关键帧（将邻居的父母们拉拢入伙）
            KeyFrame *pParent = pKF->GetParent();
            if (pParent)
            {
                if (pParent->mnTrackReferenceForFrame != mCurrentFrame.mnId)
                {
                    mvpLocalKeyFrames.push_back(pParent);
                    pParent->mnTrackReferenceForFrame = mCurrentFrame.mnId;
                    break;
                }
            }
        }

        // ps：--------------------------- 到此为止，mvpLocalKeyFrames 里面有：一级共视关键帧 + 二级共视关键帧 + 一级共视关键帧的子关键帧 + 一级共视关键帧的父关键帧

        // IMU 模式下增加了 10 个时间上的最近关键帧
        // ? 为什么？答：
        if ((mSensor == System::IMU_MONOCULAR || mSensor == System::IMU_STEREO || mSensor == System::IMU_RGBD) && mvpLocalKeyFrames.size() < 80)
        {
            KeyFrame *tempKeyFrame = mCurrentFrame.mpLastKeyFrame;

            const int Nd = 20;
            for (int i = 0; i < Nd; i++)
            {
                if (!tempKeyFrame)
                    break;
                if (tempKeyFrame->mnTrackReferenceForFrame != mCurrentFrame.mnId)
                {
                    mvpLocalKeyFrames.push_back(tempKeyFrame);
                    tempKeyFrame->mnTrackReferenceForFrame = mCurrentFrame.mnId;
                    tempKeyFrame = tempKeyFrame->mPrevKF;
                }
            }
        }

        // Step 3：更新当前帧的参考关键帧，与自己共视程度最高的关键帧作为参考关键帧
        if (pKFmax)
        {
            mpReferenceKF = pKFmax;
            mCurrentFrame.mpReferenceKF = mpReferenceKF;
        }
    }

    // todo4：在当前帧的局部地图中，找到那些能够被当前帧观测到但尚未匹配的地图点，并进行投影搜索匹配，从而增加当前帧与地图点之间的匹配关系
    // 注意：局部地图点中已经是当前帧地图点的不需要再投影，只需要将此外的并且在视野范围内的点和当前帧进行投影匹配
    void Tracking::SearchLocalPoints()
    {
        // Step 1：遍历当前帧的地图点，标记这些地图点不参与之后的投影搜索匹配
        for (vector<MapPoint *>::iterator vit = mCurrentFrame.mvpMapPoints.begin(), vend = mCurrentFrame.mvpMapPoints.end(); vit != vend; vit++)
        {
            MapPoint *pMP = *vit;

            if (pMP)
            {
                if (pMP->isBad())
                {
                    *vit = static_cast<MapPoint *>(NULL);
                }
                else
                {
                    // 更新能观测到该点的帧数 +1（被当前帧观测了）
                    pMP->IncreaseVisible();

                    // 标记该点被当前帧观测到
                    // ? mnLastFrameSeen 这是个啥玩意儿？答：
                    pMP->mnLastFrameSeen = mCurrentFrame.mnId;

                    // 标记该点在后面搜索匹配时不被投影，因为已经有匹配了
                    pMP->mbTrackInView = false;
                    pMP->mbTrackInViewR = false;
                }
            }
        }

        // 准备进行投影匹配的点的数目
        int nToMatch = 0;

        // Step 2：判断所有局部地图点中除当前帧地图点外的点，是否在当前帧视野范围内
        for (vector<MapPoint *>::iterator vit = mvpLocalMapPoints.begin(), vend = mvpLocalMapPoints.end(); vit != vend; vit++)
        {
            MapPoint *pMP = *vit;

            // 已经被当前帧观测到的地图点肯定在视野范围内，跳过
            if (pMP->mnLastFrameSeen == mCurrentFrame.mnId)
                continue;

            if (pMP->isBad())
                continue;

            // 判断地图点是否在在当前帧视野内
            if (mCurrentFrame.isInFrustum(pMP, 0.5))
            {
                // 观测到该点的帧数加 1
                pMP->IncreaseVisible();

                // 只有在视野范围内的地图点才参与之后的投影匹配
                nToMatch++;
            }

            if (pMP->mbTrackInView)
            {
                mCurrentFrame.mmProjectPoints[pMP->mnId] = cv::Point2f(pMP->mTrackProjX, pMP->mTrackProjY);
            }
        }

        // Step 3：如果需要进行投影匹配的点的数目大于0，就进行投影匹配，增加更多的匹配关系
        if (nToMatch > 0)
        {
            ORBmatcher matcher(0.8);

            int th = 1;

            // RGBD 相机输入的时候，搜索的阈值会变得稍微大一些
            if (mSensor == System::RGBD || mSensor == System::IMU_RGBD)
                th = 3;

            if (mpAtlas->isImuInitialized())
            {
                if (mpAtlas->GetCurrentMap()->GetIniertialBA2())
                    th = 2;
                else
                    th = 6; // 0.4 版本这里是 3
            }

            else if (!mpAtlas->isImuInitialized() && (mSensor == System::IMU_MONOCULAR || mSensor == System::IMU_STEREO || mSensor == System::IMU_RGBD))
            {
                th = 10;
            }

            // 如果不久前进行过重定位，那么进行一个更加宽泛的搜索，阈值需要增大
            if (mCurrentFrame.mnId < mnLastRelocFrameId + 2)
                th = 5;

            if (mState == LOST || mState == RECENTLY_LOST) // Lost for less than 1 second
                th = 15;                                   // 15

            // ps：投影匹配得到更多的匹配关系
            int matches = matcher.SearchByProjection(mCurrentFrame, mvpLocalMapPoints, th, mpLocalMapper->mbFarPoints, mpLocalMapper->mThFarPoints);
        }
    }

    // 更新上一帧的位姿；对于双目或 RGB-D 相机，还会根据深度值生成临时地图点
    /**
     * @brief 更新上一帧的位姿；对于双目或 RGB-D 相机，还会根据深度值生成临时地图点
     *          单目情况：只计算了上一帧的世界坐标系位姿
     *          双目和 rgbd 情况：选取有有深度值的并且没有被选为地图点的点生成新的临时地图点，提高跟踪鲁棒性
     */
    void Tracking::UpdateLastFrame()
    {
        // Step 1：利用参考关键帧更新上一帧在世界坐标系下的位姿
        // 上一普通帧的参考关键帧，注意这里用的是参考关键帧（位姿准）而不是上上一帧的普通帧
        KeyFrame *pRef = mLastFrame.mpReferenceKF;

        // ref_keyframe 到 lastframe的位姿变换
        Sophus::SE3f Tlr = mlRelativeFramePoses.back();

        // 将上一帧的世界坐标系下的位姿计算出来
        // l:last, r:reference, w:world
        // Tlw = Tlr*Trw
        mLastFrame.SetPose(Tlr * pRef->GetPose());

        // 如果上一帧为关键帧，或者单目/单目惯性，SLAM 模式的情况，则退出
        if (mnLastKeyFrameId == mLastFrame.mnId || mSensor == System::MONOCULAR || mSensor == System::IMU_MONOCULAR || !mbOnlyTracking)
            return;

        // Step 2：对于双目或 RGB-D 相机，为上一帧生成新的临时地图点
        // 注意这些地图点只是用来跟踪，不加入到地图中，跟踪完后会删除
        // Create "visual odometry" MapPoints
        // We sort points according to their measured depth by the stereo/RGB-D sensor

        // Step 2.1：得到上一帧中具有有效深度值的特征点（不一定是地图点）
        vector<pair<float, int>> vDepthIdx;
        const int Nfeat = mLastFrame.Nleft == -1 ? mLastFrame.N : mLastFrame.Nleft;
        vDepthIdx.reserve(Nfeat);
        for (int i = 0; i < Nfeat; i++)
        {
            float z = mLastFrame.mvDepth[i];
            if (z > 0)
            {
                // vDepthIdx 第一个元素是某个点的深度，第二个元素是对应的特征点 id
                vDepthIdx.push_back(make_pair(z, i));
            }
        }

        // 如果上一帧中没有有效深度的点，那么就直接退出
        if (vDepthIdx.empty())
            return;

        // 按照深度从小到大排序
        sort(vDepthIdx.begin(), vDepthIdx.end());

        // Step 2.2：从中找出不是地图点的部分
        // We insert all close points (depth < mThDepth)
        // If less than 100 close points, we insert the 100 closest ones.
        int nPoints = 0;
        for (size_t j = 0; j < vDepthIdx.size(); j++)
        {
            int i = vDepthIdx[j].second;

            bool bCreateNew = false;

            // 如果这个点对应在上一帧中的地图点没有，或者创建后就没有被观测到，那么就生成一个临时的地图点
            MapPoint *pMP = mLastFrame.mvpMapPoints[i];

            if (!pMP)
                bCreateNew = true;
            else if (pMP->Observations() < 1)
                bCreateNew = true; // 地图点被创建后就没有被观测，认为不靠谱，也需要重新创建

            if (bCreateNew)
            {
                // Step 2.3：需要创建的点，包装为地图点。只是为了提高双目和RGBD的跟踪成功率，并没有添加复杂属性，因为后面会扔掉
                // 反投影到世界坐标系中
                Eigen::Vector3f x3D;

                if (mLastFrame.Nleft == -1)
                {
                    mLastFrame.UnprojectStereo(i, x3D);
                }
                else
                {
                    x3D = mLastFrame.UnprojectStereoFishEye(i);
                }

                // 加入上一帧的地图点中
                MapPoint *pNewMP = new MapPoint(x3D, mpAtlas->GetCurrentMap(), &mLastFrame, i);
                mLastFrame.mvpMapPoints[i] = pNewMP;

                // 标记为临时添加的 MapPoint，之后在 CreateNewKeyFrame() 之前会全部删除
                mlpTemporalPoints.push_back(pNewMP);
                nPoints++;
            }
            else
            {
                // 因为从近到远排序，记录其中不需要创建地图点的个数
                nPoints++;
            }

            // Step 2.4：如果地图点质量不好，停止创建地图点
            // 停止新增临时地图点必须同时满足以下条件：
            // 1、当前的点的深度已经超过了设定的深度阈值（35倍基线）
            // 2、nPoints已经超过100个点，说明距离比较远了，可能不准确，停掉退出
            if (vDepthIdx[j].first > mThDepth && nPoints > 100)
                break;
        }
    }

    // -----------------------------------------------------------------------------------------------------------------------------

    //  作用：判断当前帧是否需要插入关键帧👇
    /**
     * @brief 判断当前帧是否需要插入关键帧
     *
     * Step 1：纯VO模式下不插入关键帧，如果局部地图被闭环检测使用，则不插入关键帧
     * Step 2：如果距离上一次重定位比较近，或者关键帧数目超出最大限制，不插入关键帧
     * Step 3：得到参考关键帧跟踪到的地图点数量
     * Step 4：查询局部地图管理器是否繁忙,也就是当前能否接受新的关键帧
     * Step 5：对于双目或RGBD摄像头，统计可以添加的有效地图点总数 和 跟踪到的地图点数量
     * Step 6：决策是否需要插入关键帧
     * @return true         需要
     * @return false        不需要
     */
    bool Tracking::NeedNewKeyFrame()
    {
        // case1：如果是 IMU 模式，并且当前地图中未完成 IMU 初始化
        if ((mSensor == System::IMU_MONOCULAR || mSensor == System::IMU_STEREO || mSensor == System::IMU_RGBD) && !mpAtlas->GetCurrentMap()->isImuInitialized())
        {
            // 如果是 IMU 模式，当前帧距离上一关键帧时间戳超过0.25s，则说明需要插入关键帧，不再进行后续判断
            if (mSensor == System::IMU_MONOCULAR && (mCurrentFrame.mTimeStamp - mpLastKeyFrame->mTimeStamp) >= 0.25)
                return true;
            else if ((mSensor == System::IMU_STEREO || mSensor == System::IMU_RGBD) && (mCurrentFrame.mTimeStamp - mpLastKeyFrame->mTimeStamp) >= 0.25)
                return true;
            // 否则，则说明不需要插入关键帧，不再进行后续判断
            else
                return false;
        }

        // Step 1：纯VO模式下不插入关键帧
        if (mbOnlyTracking)
            return false;

        // Step 2：如果局部地图线程被闭环检测使用，则不插入关键帧
        // If Local Mapping is freezed by a Loop Closure do not insert keyframes
        if (mpLocalMapper->isStopped() || mpLocalMapper->stopRequested())
        {
            /*if(mSensor == System::MONOCULAR)
            {
                std::cout << "NeedNewKeyFrame: localmap stopped" << std::endl;
            }*/
            return false;
        }

        // 获取当前地图中的关键帧数目
        const int nKFs = mpAtlas->KeyFramesInMap();

        // Step 3：如果距离上一次重定位比较近，并且关键帧数目超出最大限制，不插入关键帧
        if (mCurrentFrame.mnId < mnLastRelocFrameId + mMaxFrames && nKFs > mMaxFrames)
        {
            return false;
        }

        // Step 4：得到参考关键帧跟踪到的地图点数量
        // Tracked MapPoints in the reference keyframe
        // 地图点的最小观测次数
        int nMinObs = 3;
        if (nKFs <= 2)
            nMinObs = 2;

        // 参考关键帧地图点中观测的数目 >= nMinObs 的地图点数目
        // UpdateLocalKeyFrames() 函数中会将与当前关键帧共视程度最高的关键帧设定为当前帧的参考关键帧
        int nRefMatches = mpReferenceKF->TrackedMapPoints(nMinObs);

        // Step 5：查询局部地图线程是否繁忙，当前能否接受新的关键帧
        bool bLocalMappingIdle = mpLocalMapper->AcceptKeyFrames();

        // Step 6：对于双目或 RGBD 摄像头，统计成功跟踪的近点的数量，如果跟踪到的近点太少，没有跟踪到的近点较多，可以插入关键帧
        // Check how many "close" points are being tracked and how many could be potentially created.
        int nNonTrackedClose = 0; // 双目或 RGB-D 中没有跟踪到的近点
        int nTrackedClose = 0;    // 双目或 RGB-D 中成功跟踪的近点（三维点）

        if (mSensor != System::MONOCULAR && mSensor != System::IMU_MONOCULAR)
        {
            int N = (mCurrentFrame.Nleft == -1) ? mCurrentFrame.N : mCurrentFrame.Nleft;

            int outliers = 0;
            for (int i = 0; i < N; i++)
            {
                if (mCurrentFrame.mvbOutlier[i])
                    outliers++;
            }
            std::cout << "外点: " << outliers << std::endl;

            for (int i = 0; i < N; i++)
            {
                // 深度值在有效范围内
                if (mCurrentFrame.mvDepth[i] > 0 && mCurrentFrame.mvDepth[i] < mThDepth)
                {
                    if (mCurrentFrame.mvpMapPoints[i] && !mCurrentFrame.mvbOutlier[i])
                        nTrackedClose++;
                    else
                        nNonTrackedClose++;
                }
            }

            std::cout << "nTrackedClose:" << nTrackedClose << ", " << "non tracked closed points: " << nNonTrackedClose << std::endl;
            // Verbose::PrintMess("[NEEDNEWKF]-> closed points: " + to_string(nTrackedClose) + "; non tracked closed points: " + to_string(nNonTrackedClose), Verbose::VERBOSITY_NORMAL);// Verbose::VERBOSITY_DEBUG);
        }

        // 双目或RGBD情况下：跟踪到的地图点中近点太少 同时 没有跟踪到的三维点太多，可以插入关键帧了
        // 单目时，为false
        bool bNeedToInsertClose;
        bNeedToInsertClose = (nTrackedClose < 100) && (nNonTrackedClose > 70);

        // Step 7：决策是否需要插入关键帧 --- 重点！！！

        // Step 7.1：设定比例阈值，当前帧和参考关键帧跟踪到点的比例，比例越大，越倾向于增加关键帧
        float thRefRatio = 0.75f;

        // 如果关键帧只有一帧，那么插入关键帧的阈值设置的低一点，插入频率较低
        if (nKFs < 2)
            thRefRatio = 0.4f;

        /*int nClosedPoints = nTrackedClose + nNonTrackedClose;
        const int thStereoClosedPoints = 15;
        if(nClosedPoints < thStereoClosedPoints && (mSensor==System::STEREO || mSensor==System::IMU_STEREO))
        {
            //Pseudo-monocular, there are not enough close points to be confident about the stereo observations.
            thRefRatio = 0.9f;
        }*/

        // 单目情况下插入关键帧的频率很高
        if (mSensor == System::MONOCULAR)
            thRefRatio = 0.9f;

        if (mpCamera2)
            thRefRatio = 0.75f;

        // 如果是单目+IMU情况下，匹配内点数目超过 350，插入关键帧的频率可以适当降低
        if (mSensor == System::IMU_MONOCULAR)
        {
            if (mnMatchesInliers > 350) // Points tracked from the local map
                thRefRatio = 0.75f;
            else
                thRefRatio = 0.90f;
        }

        // Step 7.2：很长时间（一般是 1s，也就是 30帧）没有插入关键帧，可以插入
        const bool c1a = mCurrentFrame.mnId >= mnLastKeyFrameId + mMaxFrames;

        // Step 7.3：满足插入关键帧的最小间隔，并且 localMapper 处于空闲状态，可以插入
        const bool c1b = ((mCurrentFrame.mnId >= mnLastKeyFrameId + mMinFrames) && bLocalMappingIdle); // mpLocalMapper->KeyframesInQueue() < 2);

        std::cout << "nRefMatches: " << nRefMatches << std::endl;
        std::cout << "mnMatchesInliers: " << mnMatchesInliers << std::endl;
        //  Step 7.4：在双目/RGB-D 的情况下当前帧跟踪到的点比参考关键帧的 0.25 倍还少，或者满足 bNeedToInsertClose
        const bool c1c = mSensor != System::MONOCULAR && mSensor != System::IMU_MONOCULAR && mSensor != System::IMU_STEREO && mSensor != System::IMU_RGBD && (mnMatchesInliers < nRefMatches * 0.25 || bNeedToInsertClose);

        // Step 7.5：和参考帧相比当前跟踪到的点太少 或者满足 bNeedToInsertClose；同时跟踪到的内点还不能太少
        const bool c2 = (((mnMatchesInliers < nRefMatches * thRefRatio || bNeedToInsertClose)) && mnMatchesInliers > 15);

        // Temporal condition for Inertial cases
        //  * 新增的条件c3：单目/双目+IMU模式下，并且IMU完成了初始化（隐藏条件），当前帧和上一关键帧之间时间超过 0.5 秒，则 c3 = true
        bool c3 = false;
        if (mpLastKeyFrame)
        {
            if (mSensor == System::IMU_MONOCULAR)
            {
                if ((mCurrentFrame.mTimeStamp - mpLastKeyFrame->mTimeStamp) >= 0.5)
                    c3 = true;
            }
            else if (mSensor == System::IMU_STEREO || mSensor == System::IMU_RGBD)
            {
                if ((mCurrentFrame.mTimeStamp - mpLastKeyFrame->mTimeStamp) >= 0.5)
                    c3 = true;
            }
        }

        // * 新增的条件c4：单目+IMU模式下，当前帧匹配内点数在15~75之间或者是RECENTLY_LOST状态，c4=true
        bool c4 = false;
        if ((((mnMatchesInliers < 75) && (mnMatchesInliers > 15)) || mState == RECENTLY_LOST) && (mSensor == System::IMU_MONOCULAR)) // MODIFICATION_2, originally ((((mnMatchesInliers<75) && (mnMatchesInliers>15)) || mState==RECENTLY_LOST) && ((mSensor == System::IMU_MONOCULAR)))
            c4 = true;
        else
            c4 = false;

        std::cout << "NeedNewKF: c1a = " << c1a << "; c1b = " << c1b << "; c1c = " << c1c << "; c2 = " << c2 << "; c3 = " << c3 << "; c4 = " << c4
                  << "; bLocalMappingIdle: " << bLocalMappingIdle << "; Queue size: " << mpLocalMapper->KeyframesInQueue() << std::endl;

        // 相比 ORB-SLAM2 多了 c3,c4。满足以下条件后，再根据局部建图线程是否空闲进一步判断
        if (((c1a || c1b || c1c) && c2) || c3 || c4)
        {
            // Step 7.6：如果 local mapping 空闲时，或者正在做 imu初始化 时可以直接插入，不空闲的时候要根据情况插入
            if (bLocalMappingIdle || mpLocalMapper->IsInitializing())
            {
                std::cout << "[NeedNewKeyFrame] Keyframe insertion: Success" << std::endl;
                return true;
            }
            // 如果不空闲，先中断局部 BA
            else
            {
                std::cout << "[NeedNewKeyFrame] Local Mapping is busy." << std::endl;
                mpLocalMapper->InterruptBA();
                if (mSensor != System::MONOCULAR && mSensor != System::IMU_MONOCULAR)
                {
                    // 双目或双目+IMU 或 RGB-D 模式下，如队列里没有阻塞太多关键帧，可以插入
                    // tracking插入关键帧不是直接插入，而且先插入到 mlNewKeyFrames 中，
                    // 然后 localmapper 再逐个 pop 出来插入到 mspKeyFrames
                    if (mpLocalMapper->KeyframesInQueue() < 3)
                        return true; // 队列中的关键帧数目不是很多，可以插入
                    else
                        return false; // 队列中缓冲的关键帧数目太多，暂时不能插入
                }
                else
                {
                    // 对于单目情况，就直接无法插入关键帧了
                    //? 为什么这里对单目情况的处理不一样?
                    // 回答：可能是单目关键帧相对比较密集
                    return false;
                }
            }
        }
        else
            // 不满足上面的条件，自然不能插入关键帧
            return false;
    }

    //  作用：第二阶段跟踪结束后新建关键帧👇
    /**
     * @brief 创建新的关键帧
     * 对于非单目的情况，同时创建新的MapPoints
     *
     * Step 1：将当前帧构造成关键帧
     * Step 2：将当前关键帧设置为当前帧的参考关键帧
     * Step 3：对于双目或rgbd摄像头，为当前帧生成新的MapPoints
     */
    void Tracking::CreateNewKeyFrame()
    {
        // 如果局部建图线程正在初始化（IMU初始化时）或者局部建图被停止，不允许插入关键帧
        if (mpLocalMapper->IsInitializing() && !mpAtlas->isImuInitialized())
            return;

        if (!mpLocalMapper->SetNotStop(true))
            return;

        // Step 1：将当前帧构造成关键帧
        KeyFrame *pKF = new KeyFrame(mCurrentFrame, mpAtlas->GetCurrentMap(), mpKeyFrameDB);

        // 如果IMU已经初始化，将关键帧标记为IMU关键帧，并设置当前帧的IMU偏置
        if (mpAtlas->isImuInitialized()) //  || mpLocalMapper->IsInitializing())
            pKF->bImu = true;

        pKF->SetNewBias(mCurrentFrame.mImuBias);

        // Step 2：将当前关键帧设置为当前帧的参考关键帧
        // ps：在 UpdateLocalKeyFrames() 函数中会将与当前关键帧共视程度最高的关键帧设定为当前帧的参考关键帧
        mpReferenceKF = pKF;
        mCurrentFrame.mpReferenceKF = pKF;

        if (mpLastKeyFrame)
        {
            pKF->mPrevKF = mpLastKeyFrame; // 将当前关键帧的前向指针设置为上一关键帧
            mpLastKeyFrame->mNextKF = pKF; // 将上一关键帧的后向指针设置为当前关键帧
        }
        else
        {
            // Verbose::PrintMess("No last KF in KF creation!!", Verbose::VERBOSITY_NORMAL);
            std::cout << "No last KF in KF creation!!" << std::endl;
        }

        // Reset preintegration from last KF (Create new object)
        if (mSensor == System::IMU_MONOCULAR || mSensor == System::IMU_STEREO || mSensor == System::IMU_RGBD)
        {
            mpImuPreintegratedFromLastKF = new IMU::Preintegrated(pKF->GetImuBias(), pKF->mImuCalib);
        }

        // Step 3：对于双目或 rgbd 摄像头，为当前帧生成新的地图点；单目无操作
        // 这段代码和 Tracking::UpdateLastFrame 中的那一部分代码功能相同
        if (mSensor != System::MONOCULAR && mSensor != System::IMU_MONOCULAR) // TODO check if incluide imu_stereo
        {
            // 根据 Tcw 计算 mRcw、mtcw 和 mRwc、mOw
            mCurrentFrame.UpdatePoseMatrices();

            // cout << "create new MPs" << endl;
            // We sort points by the measured depth by the stereo/RGBD sensor.
            // We create all those MapPoints whose depth < mThDepth.
            // If there are less than 100 close points we create the 100 closest.
            int maxPoint = 100;
            if (mSensor == System::IMU_STEREO || mSensor == System::IMU_RGBD)
                maxPoint = 100;

            // Step 3.1：得到当前帧中有深度值的特征点（不一定是地图点）
            vector<pair<float, int>> vDepthIdx;
            int N = (mCurrentFrame.Nleft != -1) ? mCurrentFrame.Nleft : mCurrentFrame.N;
            vDepthIdx.reserve(mCurrentFrame.N);
            for (int i = 0; i < N; i++)
            {
                float z = mCurrentFrame.mvDepth[i];

                if (z > 0)
                {
                    // 第一个元素是深度，第二个元素是对应的特征点的id
                    vDepthIdx.push_back(make_pair(z, i));
                }
            }

            if (!vDepthIdx.empty())
            {
                // Step 3.2：按照深度从小到大排序，优先处理【近距离】的点
                sort(vDepthIdx.begin(), vDepthIdx.end());

                // Step 3.3：从中找出不是地图点的生成临时地图点
                int nPoints = 0; // 处理的近点的个数
                for (size_t j = 0; j < vDepthIdx.size(); j++)
                {
                    int i = vDepthIdx[j].second;

                    bool bCreateNew = false;

                    // 如果这个点对应在上一帧中的地图点没有，或者创建后就没有被观测到，那么就生成一个临时的地图点
                    MapPoint *pMP = mCurrentFrame.mvpMapPoints[i];

                    // 如果当前点没有对应的地图点或者观测次数不足，标记为需要创建新地图点
                    if (!pMP)
                        bCreateNew = true;

                    else if (pMP->Observations() < 1)
                    {
                        bCreateNew = true;
                        mCurrentFrame.mvpMapPoints[i] = static_cast<MapPoint *>(NULL);
                    }

                    // 如果需要就新建地图点，这里的地图点不是临时的，是全局地图中新建地图点，用于跟踪
                    if (bCreateNew)
                    {
                        Eigen::Vector3f x3D;

                        // 双目和 RGB-D 的反投影方式不同
                        if (mCurrentFrame.Nleft == -1)
                        {
                            mCurrentFrame.UnprojectStereo(i, x3D);
                        }
                        else
                        {
                            x3D = mCurrentFrame.UnprojectStereoFishEye(i);
                        }

                        // 添加地图点和关键帧的相互观测和属性信息，如最佳描述子、平均观测方向、观测距离范围
                        // ps：这些添加属性的操作是每次创建 MapPoint 后都要做的
                        MapPoint *pNewMP = new MapPoint(x3D, pKF, mpAtlas->GetCurrentMap());
                        pNewMP->AddObservation(pKF, i);

                        // 如果左右相机都有匹配点，也添加到地图点的观测列表中
                        if (mCurrentFrame.Nleft != -1 && mCurrentFrame.mvLeftToRightMatch[i] >= 0)
                        {
                            mCurrentFrame.mvpMapPoints[mCurrentFrame.Nleft + mCurrentFrame.mvLeftToRightMatch[i]] = pNewMP;
                            pNewMP->AddObservation(pKF, mCurrentFrame.Nleft + mCurrentFrame.mvLeftToRightMatch[i]);
                            pKF->AddMapPoint(pNewMP, mCurrentFrame.Nleft + mCurrentFrame.mvLeftToRightMatch[i]);
                        }

                        // 关键帧与地图点的关联
                        pKF->AddMapPoint(pNewMP, i);
                        pNewMP->ComputeDistinctiveDescriptors();
                        pNewMP->UpdateNormalAndDepth();
                        mpAtlas->AddMapPoint(pNewMP);

                        mCurrentFrame.mvpMapPoints[i] = pNewMP;
                        nPoints++;
                    }
                    else
                    {
                        // 因为从近到远排序，记录其中不需要创建地图点的个数
                        nPoints++;
                    }

                    // Step 3.4：停止新建地图点必须同时满足以下条件：
                    // 1、当前的点的深度已经超过了设定的深度阈值（35倍基线）
                    // 2、nPoints 已经超过 100 个点，说明距离比较远了，可能不准确，停掉退出
                    if (vDepthIdx[j].first > mThDepth && nPoints > maxPoint)
                    {
                        break;
                    }
                }
                // Verbose::PrintMess("new mps for stereo KF: " + to_string(nPoints), Verbose::VERBOSITY_NORMAL);
            }
        }

        pKF->mvDynamicArea = mCurrentFrame.mvDynamicArea;

        // Step 4：插入关键帧
        // 关键帧插入到列表 mlNewKeyFrames中，等待local mapping线程临幸
        std::cout << "再将当前关键帧插入到局部建图线程队列 mlNewKeyFrames 中" << std::endl;
        mpLocalMapper->InsertKeyFrame(pKF);

        // TODO 点云
        // mpPointCloudMapper->InsertKeyFrame(pKF, this->mImColor, this->mDepth);

        // 插入好了，允许局部建图停止
        mpLocalMapper->SetNotStop(false);

        // 当前帧成为新的关键帧，更新最后一个关键帧的 id 和指针
        mnLastKeyFrameId = mCurrentFrame.mnId;
        mpLastKeyFrame = pKF;
    }

    // -----------------------------------------------------------------------------------------------------------------------------

    // 全局重置：整个追踪线程执行复位操作，清除所有跟踪状态和地图
    void Tracking::Reset(bool bLocMap)
    {
        Verbose::PrintMess("System Reseting", Verbose::VERBOSITY_NORMAL);

        if (mpViewer)
        {
            mpViewer->RequestStop();
            while (!mpViewer->isStopped())
                usleep(3000);
        }

        // Reset Local Mapping
        if (!bLocMap)
        {
            Verbose::PrintMess("Reseting Local Mapper...", Verbose::VERBOSITY_NORMAL);
            mpLocalMapper->RequestReset();
            Verbose::PrintMess("done", Verbose::VERBOSITY_NORMAL);
        }

        // Reset Loop Closing
        Verbose::PrintMess("Reseting Loop Closing...", Verbose::VERBOSITY_NORMAL);
        mpLoopClosing->RequestReset();
        Verbose::PrintMess("done", Verbose::VERBOSITY_NORMAL);

        // Clear BoW Database
        Verbose::PrintMess("Reseting Database...", Verbose::VERBOSITY_NORMAL);
        mpKeyFrameDB->clear();
        Verbose::PrintMess("done", Verbose::VERBOSITY_NORMAL);

        // Clear Map (this erase MapPoints and KeyFrames)
        mpAtlas->clearAtlas();
        mpAtlas->CreateNewMap();
        if (mSensor == System::IMU_STEREO || mSensor == System::IMU_MONOCULAR || mSensor == System::IMU_RGBD)
            mpAtlas->SetInertialSensor();
        mnInitialFrameId = 0;

        KeyFrame::nNextId = 0;
        Frame::nNextId = 0;
        mState = NO_IMAGES_YET;

        mbReadyToInitializate = false;
        mbSetInit = false;

        mlRelativeFramePoses.clear();
        mlpReferences.clear();
        mlFrameTimes.clear();
        mlbLost.clear();
        mCurrentFrame = Frame();
        mnLastRelocFrameId = 0;
        mLastFrame = Frame();
        mpReferenceKF = static_cast<KeyFrame *>(NULL);
        mpLastKeyFrame = static_cast<KeyFrame *>(NULL);
        mvIniMatches.clear();

        if (mpViewer)
            mpViewer->Release();

        Verbose::PrintMess("   End reseting! ", Verbose::VERBOSITY_NORMAL);
    }

    // 局部重置：清除当前活动地图，保留系统状态
    void Tracking::ResetActiveMap(bool bLocMap)
    {
        Verbose::PrintMess("Active map Reseting", Verbose::VERBOSITY_NORMAL);
        if (mpViewer)
        {
            mpViewer->RequestStop();
            while (!mpViewer->isStopped())
                usleep(3000);
        }

        Map *pMap = mpAtlas->GetCurrentMap();

        if (!bLocMap)
        {
            Verbose::PrintMess("Reseting Local Mapper...", Verbose::VERBOSITY_VERY_VERBOSE);
            mpLocalMapper->RequestResetActiveMap(pMap);
            Verbose::PrintMess("done", Verbose::VERBOSITY_VERY_VERBOSE);
        }

        // Reset Loop Closing
        Verbose::PrintMess("Reseting Loop Closing...", Verbose::VERBOSITY_NORMAL);
        mpLoopClosing->RequestResetActiveMap(pMap);
        Verbose::PrintMess("done", Verbose::VERBOSITY_NORMAL);

        // Clear BoW Database
        Verbose::PrintMess("Reseting Database", Verbose::VERBOSITY_NORMAL);
        mpKeyFrameDB->clearMap(pMap); // Only clear the active map references
        Verbose::PrintMess("done", Verbose::VERBOSITY_NORMAL);

        // Clear Map (this erase MapPoints and KeyFrames)
        mpAtlas->clearMap();

        // KeyFrame::nNextId = mpAtlas->GetLastInitKFid();
        // Frame::nNextId = mnLastInitFrameId;
        mnLastInitFrameId = Frame::nNextId;
        // mnLastRelocFrameId = mnLastInitFrameId;
        mState = NO_IMAGES_YET; // NOT_INITIALIZED;

        mbReadyToInitializate = false;

        list<bool> lbLost;
        // lbLost.reserve(mlbLost.size());
        unsigned int index = mnFirstFrameId;
        cout << "mnFirstFrameId = " << mnFirstFrameId << endl;
        for (Map *pMap : mpAtlas->GetAllMaps())
        {
            if (pMap->GetAllKeyFrames().size() > 0)
            {
                if (index > pMap->GetLowerKFID())
                    index = pMap->GetLowerKFID();
            }
        }

        // cout << "First Frame id: " << index << endl;
        int num_lost = 0;
        cout << "mnInitialFrameId = " << mnInitialFrameId << endl;

        for (list<bool>::iterator ilbL = mlbLost.begin(); ilbL != mlbLost.end(); ilbL++)
        {
            if (index < mnInitialFrameId)
                lbLost.push_back(*ilbL);
            else
            {
                lbLost.push_back(true);
                num_lost += 1;
            }

            index++;
        }
        cout << num_lost << " Frames set to lost" << endl;

        mlbLost = lbLost;

        mnInitialFrameId = mCurrentFrame.mnId;
        mnLastRelocFrameId = mCurrentFrame.mnId;

        mCurrentFrame = Frame();
        mLastFrame = Frame();
        mpReferenceKF = static_cast<KeyFrame *>(NULL);
        mpLastKeyFrame = static_cast<KeyFrame *>(NULL);
        mvIniMatches.clear();

        mbVelocity = false;

        if (mpViewer)
            mpViewer->Release();

        Verbose::PrintMess("   End reseting! ", Verbose::VERBOSITY_NORMAL);
    }

    // -----------------------------------------------------------------------------------------------------------------------------

    // 显示用的
    vector<MapPoint *> Tracking::GetLocalMapMPS()
    {
        return mvpLocalMapPoints;
    }

    // 没用
    void Tracking::ChangeCalibration(const string &strSettingPath)
    {
        cv::FileStorage fSettings(strSettingPath, cv::FileStorage::READ);
        float fx = fSettings["Camera.fx"];
        float fy = fSettings["Camera.fy"];
        float cx = fSettings["Camera.cx"];
        float cy = fSettings["Camera.cy"];

        mK_.setIdentity();
        mK_(0, 0) = fx;
        mK_(1, 1) = fy;
        mK_(0, 2) = cx;
        mK_(1, 2) = cy;

        cv::Mat K = cv::Mat::eye(3, 3, CV_32F);
        K.at<float>(0, 0) = fx;
        K.at<float>(1, 1) = fy;
        K.at<float>(0, 2) = cx;
        K.at<float>(1, 2) = cy;
        K.copyTo(mK);

        cv::Mat DistCoef(4, 1, CV_32F);
        DistCoef.at<float>(0) = fSettings["Camera.k1"];
        DistCoef.at<float>(1) = fSettings["Camera.k2"];
        DistCoef.at<float>(2) = fSettings["Camera.p1"];
        DistCoef.at<float>(3) = fSettings["Camera.p2"];
        const float k3 = fSettings["Camera.k3"];
        if (k3 != 0)
        {
            DistCoef.resize(5);
            DistCoef.at<float>(4) = k3;
        }
        DistCoef.copyTo(mDistCoef);

        mbf = fSettings["Camera.bf"];

        Frame::mbInitialComputations = true;
    }

    void Tracking::InformOnlyTracking(const bool &flag)
    {
        mbOnlyTracking = flag;
    }

    // 在 IMU 模式下，利用关键帧的优化结果，调整普通帧的状态（包括位姿、速度、偏置），从而使整个系统在IMU初始化后更稳定、准确地进行跟踪和定位
    /**
     * @brief 更新了关键帧的位姿，但需要修改普通帧的位姿，因为正常跟踪需要普通帧
     * localmapping 中初始化 imu 中使用，速度的走向（仅在imu模式使用），最开始速度定义于imu初始化时，每个关键帧都根据位移除以时间得到，经过非线性优化保存于KF中.
     * 之后使用本函数，让上一帧与当前帧分别与他们对应的上一关键帧做速度叠加得到，后面新的 frame 速度由上一个帧速度决定，如果使用匀速模型（大多数情况下），通过imu积分更新速度。
     * 新的关键帧继承于对应帧
     * @param  s 尺度
     * @param  b 初始化后第一帧的偏置
     * @param  pCurrentKeyFrame 当前关键帧
     */
    void Tracking::UpdateFrameIMU(const float s, const IMU::Bias &b, KeyFrame *pCurrentKeyFrame)
    {
        Map *pMap = pCurrentKeyFrame->GetMap(); // 取出当前地图
        unsigned int index = mnFirstFrameId;    // 没用到

        // ? 什么叫每一帧的参考关键帧？答：每一帧都有自己的参考关键帧吗？
        list<ORB_SLAM3::KeyFrame *>::iterator lRit = mlpReferences.begin(); // 每一普通帧的参考关键帧
        list<bool>::iterator lbL = mlbLost.begin();                         // 对应帧是否跟踪丢失

        // mlRelativeFramePoses 存放的是 Tcr，是普通帧与关键帧的相对位姿
        // 三个变量一一对应
        // mlRelativeFramePoses 用于输出位姿，因此初始化之前里面数据没有尺度，所以要更新下尺度
        for (auto lit = mlRelativeFramePoses.begin(), lend = mlRelativeFramePoses.end(); lit != lend; lit++, lRit++, lbL++)
        {
            if (*lbL)
                continue;

            KeyFrame *pKF = *lRit;

            // 找到当前普通帧对应的有效参考关键帧。如果关键帧被标记为无效（isBad），就向上找它的父关键帧
            while (pKF->isBad())
            {
                pKF = pKF->GetParent();
            }

            // 如果参考关键帧属于当前地图 pMap，对普通帧的相对位姿 Tcr 的平移部分按比例 s 进行缩放
            // 如：如果 Tcr.translation() 是 [1.0, 2.0, 3.0]，比例 s = 1.2，那么调整后变成 [1.2, 2.4, 3.6]
            if (pKF->GetMap() == pMap)
            {
                (*lit).translation() *= s;
            }
        }

        mLastBias = b;                     // 设置偏置
        mpLastKeyFrame = pCurrentKeyFrame; // 设置上一关键帧，如果说 mpLastKeyFrame 已经是经过添加的新的 kf，而 pCurrentKeyFrame 还是上一个 kf，mpLastKeyFrame 直接指向之前的 kf
        // ? 这里为什么要更新到上一帧和当前帧？答：
        mLastFrame.SetNewBias(mLastBias);    // 将 IMU 的偏置更新到上一帧中
        mCurrentFrame.SetNewBias(mLastBias); // 将 IMU 的偏置更新到当前帧中

        // 等待当前帧完成 IMU 预积分，这段函数是在 localmapping 里调用的
        while (!mCurrentFrame.imuIsPreintegrated())
        {
            usleep(500);
        }

        // ps：更新上一帧的位姿
        // case1：如果上一帧刚好是上一关键帧，直接使用关键帧的位姿和速度（mLastFrame.mpLastKeyFrame 与 mLastFrame 不可能是一个，可以验证一下）
        // 这几步的目的：怎样利用更新好的关键帧的位姿，来更新普通帧的位姿，以提高前端的定位精度
        if (mLastFrame.mnId == mLastFrame.mpLastKeyFrame->mnFrameId)
        {
            mLastFrame.SetImuPoseVelocity(mLastFrame.mpLastKeyFrame->GetImuRotation(),
                                          mLastFrame.mpLastKeyFrame->GetImuPosition(),
                                          mLastFrame.mpLastKeyFrame->GetVelocity());
        }

        // case2：如果上一帧不是关键帧，使用IMU预积分重新计算上一帧的位姿
        else
        {
            const Eigen::Vector3f Gz(0, 0, -IMU::GRAVITY_VALUE); // 加速度为重力向量
            const Eigen::Vector3f twb1 = mLastFrame.mpLastKeyFrame->GetImuPosition();
            const Eigen::Matrix3f Rwb1 = mLastFrame.mpLastKeyFrame->GetImuRotation();
            const Eigen::Vector3f Vwb1 = mLastFrame.mpLastKeyFrame->GetVelocity();
            float t12 = mLastFrame.mpImuPreintegrated->dT; // 帧间时间间隔

            // 根据 mLastFrame 的上一个关键帧的信息（此时已经经过 imu 初始化了，所以关键帧的信息都是校正后的）以及imu的预积分重新计算上一帧的位姿
            mLastFrame.SetImuPoseVelocity(IMU::NormalizeRotation(Rwb1 * mLastFrame.mpImuPreintegrated->GetUpdatedDeltaRotation()),
                                          twb1 + Vwb1 * t12 + 0.5f * t12 * t12 * Gz + Rwb1 * mLastFrame.mpImuPreintegrated->GetUpdatedDeltaPosition(),
                                          Vwb1 + Gz * t12 + Rwb1 * mLastFrame.mpImuPreintegrated->GetUpdatedDeltaVelocity());
        }

        // 对当前帧进行同样的IMU积分计算，得到其位姿、速度
        if (mCurrentFrame.mpImuPreintegrated)
        {
            const Eigen::Vector3f Gz(0, 0, -IMU::GRAVITY_VALUE);

            const Eigen::Vector3f twb1 = mCurrentFrame.mpLastKeyFrame->GetImuPosition();
            const Eigen::Matrix3f Rwb1 = mCurrentFrame.mpLastKeyFrame->GetImuRotation();
            const Eigen::Vector3f Vwb1 = mCurrentFrame.mpLastKeyFrame->GetVelocity();
            float t12 = mCurrentFrame.mpImuPreintegrated->dT;

            mCurrentFrame.SetImuPoseVelocity(IMU::NormalizeRotation(Rwb1 * mCurrentFrame.mpImuPreintegrated->GetUpdatedDeltaRotation()),
                                             twb1 + Vwb1 * t12 + 0.5f * t12 * t12 * Gz + Rwb1 * mCurrentFrame.mpImuPreintegrated->GetUpdatedDeltaPosition(),
                                             Vwb1 + Gz * t12 + Rwb1 * mCurrentFrame.mpImuPreintegrated->GetUpdatedDeltaVelocity());
        }

        mnFirstImuFrameId = mCurrentFrame.mnId;
    }

    // -------------------------- 下面都没什么用了 ---------------------------------

    void Tracking::NewDataset()
    {
        mnNumDataset++;
    }

    int Tracking::GetNumberDataset()
    {
        return mnNumDataset;
    }

    int Tracking::GetMatchesInliers()
    {
        return mnMatchesInliers;
    }

    void Tracking::SaveSubTrajectory(string strNameFile_frames, string strNameFile_kf, string strFolder)
    {
        mpSystem->SaveTrajectoryEuRoC(strFolder + strNameFile_frames);
        // mpSystem->SaveKeyFrameTrajectoryEuRoC(strFolder + strNameFile_kf);
    }

    void Tracking::SaveSubTrajectory(string strNameFile_frames, string strNameFile_kf, Map *pMap)
    {
        mpSystem->SaveTrajectoryEuRoC(strNameFile_frames, pMap);
        if (!strNameFile_kf.empty())
            mpSystem->SaveKeyFrameTrajectoryEuRoC(strNameFile_kf, pMap);
    }

    float Tracking::GetImageScale()
    {
        return mImageScale;
    }

#ifdef REGISTER_LOOP
    void Tracking::RequestStop()
    {
        unique_lock<mutex> lock(mMutexStop);
        mbStopRequested = true;
    }

    bool Tracking::Stop()
    {
        unique_lock<mutex> lock(mMutexStop);
        if (mbStopRequested && !mbNotStop)
        {
            mbStopped = true;
            cout << "Tracking STOP" << endl;
            return true;
        }

        return false;
    }

    bool Tracking::stopRequested()
    {
        unique_lock<mutex> lock(mMutexStop);
        return mbStopRequested;
    }

    bool Tracking::isStopped()
    {
        unique_lock<mutex> lock(mMutexStop);
        return mbStopped;
    }

    void Tracking::Release()
    {
        unique_lock<mutex> lock(mMutexStop);
        mbStopped = false;
        mbStopRequested = false;
    }
#endif

} // namespace ORB_SLAM
