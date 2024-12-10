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

#ifndef TRACKING_H
#define TRACKING_H

#include <opencv2/core/core.hpp>
#include <opencv2/features2d/features2d.hpp>

#include "Viewer.h"
#include "FrameDrawer.h"
#include "Atlas.h"
#include "LocalMapping.h"
#include "LoopClosing.h"
#include "Frame.h"
#include "ORBVocabulary.h"
#include "KeyFrameDatabase.h"
#include "ORBextractor.h"
#include "MapDrawer.h"
#include "System.h"
#include "ImuTypes.h"
#include "Settings.h"

#include "GeometricCamera.h"

#include <mutex>
#include <unordered_set>

// todo 引入 YOLO 检测器 和 点云 头文件
#include "YoloDetect.h"
// #include "PointCloudMapper.h"

namespace ORB_SLAM3
{

    class Viewer;
    class FrameDrawer;
    class Atlas;
    class LocalMapping;
    class LoopClosing;
    class System;
    class Settings;

    class Tracking
    {

    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW

        Tracking(System *pSys, ORBVocabulary *pVoc, FrameDrawer *pFrameDrawer, MapDrawer *pMapDrawer, Atlas *pAtlas,
                 KeyFrameDatabase *pKFDB, const string &strSettingPath, const int sensor, Settings *settings, const string &_nameSeq = std::string());

        ~Tracking();

        // 提取配置文件数据
        bool ParseCamParamFile(cv::FileStorage &fSettings);
        bool ParseORBParamFile(cv::FileStorage &fSettings);
        bool ParseIMUParamFile(cv::FileStorage &fSettings);

        // 输入图像，输出位姿 Tcw
        //  Preprocess the input and call Track(). Extract features and performs stereo matching.
        Sophus::SE3f GrabImageStereo(const cv::Mat &imRectLeft, const cv::Mat &imRectRight, const double &timestamp, string filename);
        Sophus::SE3f GrabImageRGBD(const cv::Mat &imRGB, const cv::Mat &imD, const double &timestamp, string filename);
        Sophus::SE3f GrabImageMonocular(const cv::Mat &im, const double &timestamp, string filename);

        // 放置 IMU 数据
        void GrabImuData(const IMU::Point &imuMeasurement);

        // 设置线程指针
        void SetLocalMapper(LocalMapping *pLocalMapper);
        void SetLoopClosing(LoopClosing *pLoopClosing);
        void SetViewer(Viewer *pViewer);

        // TODO 设置动态物体检测器
        void SetDetector(YoloDetection *pDetector);

        void SetStepByStep(bool bSet);
        bool GetStepByStep();

        // 更换新的标定参数，代码中未使用
        // Load new settings
        // The focal lenght should be similar or scale prediction will fail when projecting points
        void ChangeCalibration(const string &strSettingPath);

        // 设置是否仅定位模式还是 SLAM 模式
        // Use this function if you have deactivated local mapping and you only want to localize the camera.
        void InformOnlyTracking(const bool &flag);

        // LocalMapping 中更新了关键帧的位姿后，更新普通帧的位姿，通过 IMU 积分更新速度，LocalMapping 中初始化使用
        void UpdateFrameIMU(const float s, const IMU::Bias &b, KeyFrame *pCurrentKeyFrame);

        KeyFrame *GetLastKeyFrame()
        {
            return mpLastKeyFrame;
        }

        // 地图集中新建地图
        void CreateMapInAtlas();
        std::mutex mMutexTracks;

        // 更新数据集
        void NewDataset();
        // 获得数据集总数
        int GetNumberDataset();
        // 获取匹配内点总数
        int GetMatchesInliers();

        // DEBUG
        void SaveSubTrajectory(string strNameFile_frames, string strNameFile_kf, string strFolder = "");
        void SaveSubTrajectory(string strNameFile_frames, string strNameFile_kf, Map *pMap);

        float GetImageScale();

#ifdef REGISTER_LOOP
        void RequestStop();
        bool isStopped();
        void Release();
        bool stopRequested();
#endif

    public:
        // Tracking states
        enum eTrackingState
        {
            SYSTEM_NOT_READY = -1, // 系统没有准备好的状态，一般就是在启动后，加载配置文件和词典文件时候的状态
            NO_IMAGES_YET = 0,     // 表示系统刚刚启动，尚未获得任何图像数据
            NOT_INITIALIZED = 1,   // 表示系统已经开始接收到图像，但还没有完成初步的初始化过程。之后，系统会继续根据图像数据进行进一步的初始化，准备进入正常的跟踪状态。
            OK = 2,                // 正常跟踪状态
            RECENTLY_LOST = 3,     // 新增的状态。IMU 模式：当前地图中的 KF ＞ 10，且丢失时间 ＜ 5 秒；纯视觉模式：没有该状态
            LOST = 4,              // IMU 模式：当前帧跟丢超过 5秒；纯视觉模式：即重定位失败
            OK_KLT = 5             // 未使用
        };

        eTrackingState mState;
        eTrackingState mLastProcessedState;

        // Input sensor
        int mSensor;

        // Current Frame
        Frame mCurrentFrame; // 追踪线程中有一个当前帧
        Frame mLastFrame;    // 跟踪成功后，保存当前帧数据

        ///> 还有当前帧的灰度图像 // ? 提问？那么在双目输入和在 RGBD 输入的时候呢?
        ///>                       答：在双目输入和在 RGBD 输入时，为左侧图像的灰度图。
        cv::Mat mImGray;

        // todo
        cv::Mat mImColor;
        cv::Mat mDepth;

        // todo 点云
        // PointCloudMapper *mpPointCloudMapper;

        // 单目初始化用到的一些变量
        // 初始化时前两帧相关变量
        /// 上一次的匹配
        std::vector<int> mvIniLastMatches;
        /// 初始化阶段中，当前帧中的特征点和参考帧中的特征点的匹配关系
        std::vector<int> mvIniMatches; // 跟踪初始化时前两帧之间的匹配
        /// 在初始化的过程中，保存参考帧中的特征点
        std::vector<cv::Point2f> mvbPrevMatched;
        /// 初始化过程中匹配后进行三角化得到的空间点
        std::vector<cv::Point3f> mvIniP3D;
        /// 初始化过程中的参考帧
        Frame mInitialFrame;

        // 代码结束后保存位姿用的列表
        // Lists used to recover the full camera trajectory at the end of the execution.
        // Basically we store the reference keyframe for each frame and its relative transformation
        list<Sophus::SE3f> mlRelativeFramePoses; // 存储的相对位姿列表
        list<KeyFrame *> mlpReferences;          // 参考关键帧列表
        list<double> mlFrameTimes;
        list<bool> mlbLost;

        // frames with estimated pose
        int mTrackedFr;
        bool mbStep;

        // true 表示仅定位模式，此时局部建图线程和闭环线程关闭
        // True if local mapping is deactivated and we are performing only localization
        bool mbOnlyTracking;

        void Reset(bool bLocMap = false);
        void ResetActiveMap(bool bLocMap = false);

        float mMeanTrack;
        bool mbInitWith3KFs;    // ps：是否使用 3 个关键帧进行初始化。
        double t0;              // time-stamp of first read frame --- 即：存储未初始化时的第一帧图像时间戳
        double t0vis;           // time-stamp of first inserted keyframe
        double t0IMU;           // time-stamp of IMU initialization
        bool mFastInit = false; // 表示是否启用快速初始化模式。

        // 获取局部地图点
        vector<MapPoint *> GetLocalMapMPS();

        bool mbWriteStats;

#ifdef REGISTER_TIMES
        void LocalMapStats2File();
        void TrackStats2File();
        void PrintTimeStats();

        vector<double> vdRectStereo_ms;
        vector<double> vdResizeImage_ms;
        vector<double> vdORBExtract_ms;
        vector<double> vdStereoMatch_ms;
        vector<double> vdIMUInteg_ms;
        vector<double> vdPosePred_ms;
        vector<double> vdLMTrack_ms;
        vector<double> vdNewKF_ms;
        vector<double> vdTrackTotal_ms;
#endif

    protected:
        // NOTE 主要的跟踪函数，大BOSS！
        // Main tracking function. It is independent of the input sensor.
        void Track();

        // Map initialization for stereo and RGB-D
        void StereoInitialization();
        // Map initialization for monocular
        void MonocularInitialization();
        // 单目模式下创建初始化地图
        void CreateInitialMapMonocular();

        // 检查上一帧中的地图点是否需要被替换
        void CheckReplacedInLastFrame();
        bool TrackReferenceKeyFrame();
        void UpdateLastFrame();
        bool TrackWithMotionModel();
        bool PredictStateIMU();

        bool Relocalization();

        // 更新局部地图
        void UpdateLocalMap();
        // 更新局部地图点
        void UpdateLocalPoints();
        // 更新局部地图里的关键帧
        void UpdateLocalKeyFrames();

        bool TrackLocalMap();
        // 搜索局部地图点
        void SearchLocalPoints();

        bool NeedNewKeyFrame();
        void CreateNewKeyFrame();

        // Perform preintegration from last frame
        void PreintegrateIMU();

        // Reset IMU biases and compute frame velocity
        void ResetFrameIMU();

        bool mbMapUpdated;

        // * 存储从上一【关键帧】到当前帧之间的预积分结果，用于推算当前帧的位姿。
        // Imu preintegration from last KeyFrame
        IMU::Preintegrated *mpImuPreintegratedFromLastKF;

        // 一个存储两帧之间 IMU 数据的 list 容器。它是一个双向链表，通常用于在需要频繁插入和删除操作的场景下。
        // Queue of IMU measurements between frames
        std::list<IMU::Point> mlQueueImuData;

        // Vector of IMU measurements from previous to current frame (to be filled by PreintegrateIMU)
        std::vector<IMU::Point> mvImuFromLastFrame; // 保存当前帧和上一帧之间（或上一关键帧和当前帧之间）获取到的IMU数据。换句话说，它保存了用于运动估计的IMU数据。
        std::mutex mMutexImuQueue;                  // 互斥锁，用来保护 mlQueueImuData

        // Imu calibration parameters
        IMU::Calib *mpImuCalib;

        // Last Bias Estimation (at keyframe creation)
        IMU::Bias mLastBias;

        // In case of performing only localization, this flag is true when there are no matches to
        // points in the map. Still tracking will continue if there are enough matches with temporal points.
        // In that case we are doing visual odometry. The system will try to do relocalization to recover
        // "zero-drift" localization to the map.
        bool mbVO; // 一个布尔变量，表示系统是否处于视觉里程计（VO，Visual Odometry）模式。若为 true，表示系统无法找到足够的特征点进行匹配，通常是在特征点非常少的情况下（比如相机运动很小，或场景过于简单）

        // Other Thread Pointers
        LocalMapping *mpLocalMapper;
        LoopClosing *mpLoopClosing;

        // ORB
        ORBextractor *mpORBextractorLeft, *mpORBextractorRight; // 左右目的特征提取器，单目默认用左目；双目跟踪时用 left，初始化的时候左右都用
        ORBextractor *mpIniORBextractor;                        // 初始化 ORB 特征点提取器会提取 2 倍的指定特征点数目

        // BoW
        ORBVocabulary *mpORBVocabulary; // 字典
        KeyFrameDatabase *mpKeyFrameDB; // 关键帧数据库，用于存储系统所有的关键帧。

        // Initalization (only for monocular)
        bool mbReadyToInitializate; // ps：单目初始化器
        bool mbSetInit;

        // Local Map
        KeyFrame *mpReferenceKF;                   // 参考关键帧
        std::vector<KeyFrame *> mvpLocalKeyFrames; // 局部关键帧
        std::vector<MapPoint *> mvpLocalMapPoints; // 包含局部关键帧观测到的所有地图点

        // System
        System *mpSystem;

        // Drawers
        Viewer *mpViewer;
        FrameDrawer *mpFrameDrawer;
        MapDrawer *mpMapDrawer;
        bool bStepByStep; // 控制程序的执行方式，一般用于调试模式下，控制每一步的执行，比如在每个步骤结束后暂停，等待用户输入或者手动触发下一步操作

        // todo YOLO 创建 YoloDetection 类的实例（对象）--- 即动态物体检测器指针
        YoloDetection *mpDetector;

        // Atlas -- 地图集
        Atlas *mpAtlas;

        // Calibration matrix
        cv::Mat mK;          // 内参矩阵
        Eigen::Matrix3f mK_; // ? 这是个啥？
        cv::Mat mDistCoef;   // 去畸变参数
        float mbf;           // 基线
        float mImageScale;   // ? 这是啥

        float mImuFreq;
        double mImuPer; // ? 这是个啥？哪个时刻的 IMU 时间戳？答：一个时间间隔（常量阈值），用于判断IMU数据是否在合理的时间窗口内。
        bool mInsertKFsLost;

        // New KeyFrame rules (according to fps)
        int mMinFrames; // ps：初始化为 0
        int mMaxFrames; // ps：表示系统在初始化过程中最多允许处理的图像帧数量，即当 IMU 运行到这个帧数时，触发重置。初始化为 fps

        int mnFirstImuFrameId;
        int mnFramesToResetIMU; // ps：表示经过多少帧后可以重置 IMU，一般设置为和帧率 30fps 相同，对应的时间是1s。该重置机制通常是为了在系统运行一段时间后，避免 IMU 数据的漂移或累计误差过大，保持系统的稳定性。

        // Threshold close/far points -- 远点，近点的区分阈值
        // Points seen as close by the stereo/RGBD sensor are considered reliable
        // and inserted from just one frame. Far points requiere a match in two keyframes.
        float mThDepth; // 近远点阈值，基线的倍数。超过就认为是远点

        // 仅对 RGB-D 输入而言。mDepthMapFactor 可能是用来调整深度图的比例因子（例如，如果深度图单位是厘米，而需要转为米）。
        float mDepthMapFactor;

        // Current matches in frame
        int mnMatchesInliers; // 当前帧匹配内点数，通过判断该值来判断是否跟踪成功

        // Last Frame, KeyFrame and Relocalisation Info
        KeyFrame *mpLastKeyFrame;
        unsigned int mnLastKeyFrameId;
        unsigned int mnLastRelocFrameId;
        double mTimeStampLost;
        double time_recently_lost; // 默认为 5s

        // ? 这三个有什么区别？
        unsigned int mnFirstFrameId;    // 记录新建地图时的第一帧的帧号。这个帧通常是初始化时通过某些条件选择的（比如说，IMU 或视觉初始化后）。
        unsigned int mnInitialFrameId;  // 表示地图初始化的起始帧。通常情况下，这一帧是你开始进行地图构建的第一帧，可能是关键帧或者是经过某种方式筛选出来的帧。mnInitialFrameId 在某些情况下会比 mnFirstFrameId 大
        unsigned int mnLastInitFrameId; // 记录了新地图初始化过程中最后使用的关键帧的 ID。这个帧通常是地图初始化后加入到地图中的最后一帧。

        bool mbCreatedMap; // 用于检查地图是否已经创建

        // Motion Model
        bool mbVelocity{false}; // 速度模型
        Sophus::SE3f mVelocity; // 恒速模型的速度，通过位姿增量获得，或者 IMU 积分得到

        // Color order (true RGB, false BGR, ignored if grayscale)
        bool mbRGB;

        list<MapPoint *> mlpTemporalPoints; // 存放临时地图点

        // int nMapChangeIndex;

        int mnNumDataset; // ps：用于表示当前数据集的数量或编号。它的初始值为 0，表示还没有加载任何数据集，可能用于后续的数据管理或调试，帮助标识当前处理的数据集。

        // 这些都是调试用的
        ofstream f_track_stats;

        ofstream f_track_times;
        double mTime_PreIntIMU;
        double mTime_PosePred;
        double mTime_LocalMapTrack;
        double mTime_NewKF_Dec;

        GeometricCamera *mpCamera;  // mpCamera：表示当前的主相机，通常是立体相机系统中的左相机或者单目相机。在没有第二个相机的情况下，mpCamera 就是用来获取图像的相机。
        GeometricCamera *mpCamera2; // mpCamera2：表示第二个相机，通常是立体相机中的右相机。如果系统配置了两个相机（如立体相机或双目相机），那么 mpCamera2 就是指右相机。

        int initID; // initID：表示系统初始化开始时的帧ID。这个ID通常是 初始化第一帧的ID。初始化的第一帧不一定是帧编号0，但它是初始化阶段的起点。
        int lastID; // lastID：表示当前帧的ID。每一帧图像被处理后，其对应的帧ID会递增，lastID 就是系统当前正在处理的帧的ID。

        Sophus::SE3f mTlr; // 左右相机之间的变换矩阵

        void newParameterLoader(Settings *settings);

#ifdef REGISTER_LOOP
        bool Stop();

        bool mbStopped;
        bool mbStopRequested;
        bool mbNotStop;
        std::mutex mMutexStop;
#endif

    public:
        cv::Mat mImRight;
    };

} // namespace ORB_SLAM

#endif // TRACKING_H
