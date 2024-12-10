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

#include "Frame.h"

#include "G2oTypes.h"
#include "MapPoint.h"
#include "KeyFrame.h"
#include "ORBextractor.h"
#include "Converter.h"
#include "ORBmatcher.h"
#include "GeometricCamera.h"

#include <thread>
#include <include/CameraModels/Pinhole.h>
#include <include/CameraModels/KannalaBrandt8.h>

namespace ORB_SLAM3
{
    // 下一个生成的帧的ID，这里是初始化类的静态成员变量
    long unsigned int Frame::nNextId = 0;

    // 是否要进行初始化操作的标志
    // 这里给这个标志置位的操作是在最初系统开始加载到内存的时候进行的，下一帧就是整个系统的第一帧，所以这个标志要置位
    bool Frame::mbInitialComputations = true;
    float Frame::cx, Frame::cy, Frame::fx, Frame::fy, Frame::invfx, Frame::invfy;
    float Frame::mnMinX, Frame::mnMinY, Frame::mnMaxX, Frame::mnMaxY;
    float Frame::mfGridElementWidthInv, Frame::mfGridElementHeightInv;

    // For stereo fisheye matching
    cv::BFMatcher Frame::BFmatcher = cv::BFMatcher(cv::NORM_HAMMING);

    // 默认构造函数
    Frame::Frame() : mpcpi(NULL), mpImuPreintegrated(NULL), mpPrevFrame(NULL), mpImuPreintegratedFrame(NULL), mpReferenceKF(static_cast<KeyFrame *>(NULL)), mbIsSet(false), mbImuPreintegrated(false), mbHasPose(false), mbHasVelocity(false)
    {
#ifdef REGISTER_TIMES
        mTimeStereoMatch = 0;
        mTimeORB_Ext = 0;
#endif
    }

    // 拷贝构造函数
    Frame::Frame(const Frame &frame)
        : mpcpi(frame.mpcpi), mpORBvocabulary(frame.mpORBvocabulary), mpORBextractorLeft(frame.mpORBextractorLeft), mpORBextractorRight(frame.mpORBextractorRight),
          mTimeStamp(frame.mTimeStamp), mK(frame.mK.clone()), mK_(Converter::toMatrix3f(frame.mK)), mDistCoef(frame.mDistCoef.clone()),
          mbf(frame.mbf), mb(frame.mb), mThDepth(frame.mThDepth), N(frame.N), mvKeys(frame.mvKeys),
          mvKeysRight(frame.mvKeysRight), mvKeysUn(frame.mvKeysUn), mvuRight(frame.mvuRight),
          mvDepth(frame.mvDepth), mBowVec(frame.mBowVec), mFeatVec(frame.mFeatVec),
          mDescriptors(frame.mDescriptors.clone()), mDescriptorsRight(frame.mDescriptorsRight.clone()),
          mvpMapPoints(frame.mvpMapPoints), mvbOutlier(frame.mvbOutlier), mImuCalib(frame.mImuCalib), mnCloseMPs(frame.mnCloseMPs),
          mpImuPreintegrated(frame.mpImuPreintegrated), mpImuPreintegratedFrame(frame.mpImuPreintegratedFrame), mImuBias(frame.mImuBias),
          mnId(frame.mnId), mpReferenceKF(frame.mpReferenceKF), mnScaleLevels(frame.mnScaleLevels),
          mfScaleFactor(frame.mfScaleFactor), mfLogScaleFactor(frame.mfLogScaleFactor),
          mvScaleFactors(frame.mvScaleFactors), mvInvScaleFactors(frame.mvInvScaleFactors), mNameFile(frame.mNameFile), mnDataset(frame.mnDataset),
          mvLevelSigma2(frame.mvLevelSigma2), mvInvLevelSigma2(frame.mvInvLevelSigma2), mpPrevFrame(frame.mpPrevFrame), mpLastKeyFrame(frame.mpLastKeyFrame),
          mbIsSet(frame.mbIsSet), mbImuPreintegrated(frame.mbImuPreintegrated), mpMutexImu(frame.mpMutexImu),
          mpCamera(frame.mpCamera), mpCamera2(frame.mpCamera2), Nleft(frame.Nleft), Nright(frame.Nright),
          monoLeft(frame.monoLeft), monoRight(frame.monoRight), mvLeftToRightMatch(frame.mvLeftToRightMatch),
          mvRightToLeftMatch(frame.mvRightToLeftMatch), mvStereo3Dpoints(frame.mvStereo3Dpoints),
          mTlr(frame.mTlr), mRlr(frame.mRlr), mtlr(frame.mtlr), mTrl(frame.mTrl),
          mTcw(frame.mTcw), mbHasPose(false), mbHasVelocity(false)
    {
        for (int i = 0; i < FRAME_GRID_COLS; i++)
            for (int j = 0; j < FRAME_GRID_ROWS; j++)
            {
                mGrid[i][j] = frame.mGrid[i][j];
                if (frame.Nleft > 0)
                {
                    mGridRight[i][j] = frame.mGridRight[i][j];
                }
            }

        if (frame.mbHasPose)
            SetPose(frame.GetPose());

        if (frame.HasVelocity())
        {
            SetVelocity(frame.GetVelocity());
        }

        mmProjectPoints = frame.mmProjectPoints;
        mmMatchedInImage = frame.mmMatchedInImage;

#ifdef REGISTER_TIMES
        mTimeStereoMatch = frame.mTimeStereoMatch;
        mTimeORB_Ext = frame.mTimeORB_Ext;
#endif
    }

    // ps1：立体匹配模式下的双目
    Frame::Frame(const cv::Mat &imLeft,        // 左目图像
                 const cv::Mat &imRight,       // 右目图像
                 const double &timeStamp,      // 时间戳
                 ORBextractor *extractorLeft,  // 左目图像特征点提取器句柄
                 ORBextractor *extractorRight, // 右目图像特征点提取器句柄
                 ORBVocabulary *voc,           // ORB 字典句柄
                 cv::Mat &K,                   // 相机内参矩阵
                 cv::Mat &distCoef,            // 相机去畸变参数
                 const float &bf,              // 相机基线长度和焦距的乘积
                 const float &thDepth,         // 远点和近点的深度区分阈值
                 GeometricCamera *pCamera,     // 相机模型
                 Frame *pPrevF,                //
                 const IMU::Calib &ImuCalib)   //
        : mpcpi(NULL), mpORBvocabulary(voc), mpORBextractorLeft(extractorLeft), mpORBextractorRight(extractorRight), mTimeStamp(timeStamp), mK(K.clone()), mK_(Converter::toMatrix3f(K)), mDistCoef(distCoef.clone()), mbf(bf), mThDepth(thDepth),
          mImuCalib(ImuCalib), mpImuPreintegrated(NULL), mpPrevFrame(pPrevF), mpImuPreintegratedFrame(NULL), mpReferenceKF(static_cast<KeyFrame *>(NULL)), mbIsSet(false), mbImuPreintegrated(false),
          mpCamera(pCamera), mpCamera2(nullptr), mbHasPose(false), mbHasVelocity(false)
    {
        // Step 1：帧的 ID 自增
        // Frame ID
        mnId = nNextId++;

        // Step 2：计算图像金字塔的参数
        // Scale Level Info
        mnScaleLevels = mpORBextractorLeft->GetLevels();                      // 获取图像金字塔的层数
        mfScaleFactor = mpORBextractorLeft->GetScaleFactor();                 // 获得层与层之间的缩放比
        mfLogScaleFactor = log(mfScaleFactor);                                // 计算上面缩放比的对数
        mvScaleFactors = mpORBextractorLeft->GetScaleFactors();               // 获取每层图像的缩放因子
        mvInvScaleFactors = mpORBextractorLeft->GetInverseScaleFactors();     // 同样获取每层图像缩放因子的倒数
        mvLevelSigma2 = mpORBextractorLeft->GetScaleSigmaSquares();           // 高斯模糊的时候，使用的方差
        mvInvLevelSigma2 = mpORBextractorLeft->GetInverseScaleSigmaSquares(); // 获取 sigma^2 的倒数

        // ORB extraction
#ifdef REGISTER_TIMES
        std::chrono::steady_clock::time_point time_StartExtORB = std::chrono::steady_clock::now();
#endif
        // Step 3：对左目右目图像提取 ORB 特征点, 第一个参数 0-左图， 1-右图。
        // * 这里操作很好，为加速计算，同时开了两个线程计算
        thread threadLeft(&Frame::ExtractORB, this, 0, imLeft, 0, 0);
        thread threadRight(&Frame::ExtractORB, this, 1, imRight, 0, 0);
        // 等待两张图像特征点提取过程完成
        threadLeft.join();
        threadRight.join();

#ifdef REGISTER_TIMES
        std::chrono::steady_clock::time_point time_EndExtORB = std::chrono::steady_clock::now();

        mTimeORB_Ext = std::chrono::duration_cast<std::chrono::duration<double, std::milli>>(time_EndExtORB - time_StartExtORB).count();
#endif

        // mvKeys 中保存的是左图像中的特征点，这里是获取左侧图像中特征点的个数
        N = mvKeys.size();
        // 如果左图像中没有成功提取到特征点那么就返回，也意味这这一帧的图像无法使用
        if (mvKeys.empty())
            return;

        // Step 4：用 OpenCV 的矫正函数、内参对提取到的特征点进行矫正
        UndistortKeyPoints();

#ifdef REGISTER_TIMES
        std::chrono::steady_clock::time_point time_StartStereoMatches = std::chrono::steady_clock::now();
#endif

        // Step 5：计算双目间的特征点的匹配，只有匹配成功的特征点会计算其深度，深度存放在 mvDepth 中
        ComputeStereoMatches();

#ifdef REGISTER_TIMES
        std::chrono::steady_clock::time_point time_EndStereoMatches = std::chrono::steady_clock::now();

        mTimeStereoMatch = std::chrono::duration_cast<std::chrono::duration<double, std::milli>>(time_EndStereoMatches - time_StartStereoMatches).count();
#endif

        // 初始化本帧的地图点
        mvpMapPoints = vector<MapPoint *>(N, static_cast<MapPoint *>(NULL));
        // 记录地图点是否为外点，初始化均为外点 false
        mvbOutlier = vector<bool>(N, false);
        mmProjectPoints.clear();
        mmMatchedInImage.clear();

        //  Step 5 计算去畸变后图像边界，将特征点分配到网格中。这个过程一般是在第一帧或者是相机标定参数发生变化之后进行
        // This is done only for the first Frame (or after a change in the calibration)
        if (mbInitialComputations)
        {
            // 计算去畸变后图像的边界
            ComputeImageBounds(imLeft);

            // 表示一个图像像素相当于多少个图像网格列（宽）
            mfGridElementWidthInv = static_cast<float>(FRAME_GRID_COLS) / (mnMaxX - mnMinX);
            // 表示一个图像像素相当于多少个图像网格行（高）
            mfGridElementHeightInv = static_cast<float>(FRAME_GRID_ROWS) / (mnMaxY - mnMinY);

            fx = K.at<float>(0, 0);
            fy = K.at<float>(1, 1);
            cx = K.at<float>(0, 2);
            cy = K.at<float>(1, 2);
            // 猜测是因为这种除法计算需要的时间略长，所以这里直接存储了这个中间计算结果
            invfx = 1.0f / fx;
            invfy = 1.0f / fy;

            // 特殊的初始化过程完成，标志复位
            mbInitialComputations = false;
        }

        // 双目相机基线长度
        mb = mbf / fx;

        if (pPrevF)
        {
            if (pPrevF->HasVelocity())
                SetVelocity(pPrevF->GetVelocity());
        }
        else
        {
            mVw.setZero();
        }

        mpMutexImu = new std::mutex();

        // Set no stereo fisheye information
        Nleft = -1;
        Nright = -1;
        mvLeftToRightMatch = vector<int>(0);
        mvRightToLeftMatch = vector<int>(0);
        mvStereo3Dpoints = vector<Eigen::Vector3f>(0);
        monoLeft = -1;
        monoRight = -1;

        // Step 6 将特征点分配到图像网格中
        // 上个版本这句话放在了 new 锁那个上面，放在目前这个位置更合理，因为要把一些当前模式不用的参数赋值，函数里面要用
        AssignFeaturesToGrid();
    }

    // ps2：RGBD
    /**
     * @brief 为RGBD相机准备的帧构造函数
     *
     * @param[in] imGray        对RGB图像灰度化之后得到的灰度图像
     * @param[in] imDepth       深度图像
     * @param[in] timeStamp     时间戳
     * @param[in] extractor     特征点提取器句柄
     * @param[in] voc           ORB特征点词典的句柄
     * @param[in] K             相机的内参数矩阵
     * @param[in] distCoef      相机的去畸变参数
     * @param[in] bf            baseline*bf
     * @param[in] thDepth       远点和近点的深度区分阈值
     */
    Frame::Frame(const cv::Mat &imGray, const cv::Mat &imDepth, const double &timeStamp, ORBextractor *extractor, ORBVocabulary *voc, cv::Mat &K, cv::Mat &distCoef, const float &bf, const float &thDepth, GeometricCamera *pCamera, Frame *pPrevF, const IMU::Calib &ImuCalib)
        : mpcpi(NULL), mpORBvocabulary(voc), mpORBextractorLeft(extractor), mpORBextractorRight(static_cast<ORBextractor *>(NULL)),
          mTimeStamp(timeStamp), mK(K.clone()), mK_(Converter::toMatrix3f(K)), mDistCoef(distCoef.clone()), mbf(bf), mThDepth(thDepth),
          mImuCalib(ImuCalib), mpImuPreintegrated(NULL), mpPrevFrame(pPrevF), mpImuPreintegratedFrame(NULL), mpReferenceKF(static_cast<KeyFrame *>(NULL)), mbIsSet(false), mbImuPreintegrated(false),
          mpCamera(pCamera), mpCamera2(nullptr), mbHasPose(false), mbHasVelocity(false)
    {
        // Step 1：帧的 ID 自增
        mnId = nNextId++;

        // Step 2：计算图像金字塔的参数
        mnScaleLevels = mpORBextractorLeft->GetLevels();
        mfScaleFactor = mpORBextractorLeft->GetScaleFactor();
        mfLogScaleFactor = log(mfScaleFactor);
        mvScaleFactors = mpORBextractorLeft->GetScaleFactors();
        mvInvScaleFactors = mpORBextractorLeft->GetInverseScaleFactors();
        mvLevelSigma2 = mpORBextractorLeft->GetScaleSigmaSquares();
        mvInvLevelSigma2 = mpORBextractorLeft->GetInverseScaleSigmaSquares();

        // ORB extraction

#ifdef REGISTER_TIMES
        std::chrono::steady_clock::time_point time_StartExtORB = std::chrono::steady_clock::now();
#endif
        // step 3：提取特征点
        ExtractORB(0, imGray, 0, 0);

#ifdef REGISTER_TIMES
        std::chrono::steady_clock::time_point time_EndExtORB = std::chrono::steady_clock::now();

        mTimeORB_Ext = std::chrono::duration_cast<std::chrono::duration<double, std::milli>>(time_EndExtORB - time_StartExtORB).count();
#endif

        N = mvKeys.size();

        if (mvKeys.empty())
            return;

        // Step 4：用 OpenCV 的矫正函数、内参对提取到的特征点进行矫正
        UndistortKeyPoints();

        // Step 5：获取图像的深度，并且根据这个深度推算其右图中匹配的特征点的视差
        ComputeStereoFromRGBD(imDepth);

        // 初始化本帧的地图点
        mvpMapPoints = vector<MapPoint *>(N, static_cast<MapPoint *>(NULL));

        mmProjectPoints.clear();
        mmMatchedInImage.clear();

        // 记录地图点是否为外点，初始化均为外点 false
        mvbOutlier = vector<bool>(N, false);

        //  Step 6：计算去畸变后图像边界，将特征点分配到网格中。这个过程一般是在第一帧或者是相机标定参数发生变化之后进行
        // This is done only for the first Frame (or after a change in the calibration)
        if (mbInitialComputations)
        {
            // 计算去畸变后图像的边界
            ComputeImageBounds(imGray);

            mfGridElementWidthInv = static_cast<float>(FRAME_GRID_COLS) / static_cast<float>(mnMaxX - mnMinX);
            mfGridElementHeightInv = static_cast<float>(FRAME_GRID_ROWS) / static_cast<float>(mnMaxY - mnMinY);

            fx = K.at<float>(0, 0);
            fy = K.at<float>(1, 1);
            cx = K.at<float>(0, 2);
            cy = K.at<float>(1, 2);
            invfx = 1.0f / fx;
            invfy = 1.0f / fy;

            mbInitialComputations = false;
        }

        // 计算假想的基线长度 baseline = mbf / fx
        // 后面要对从RGBD相机输入的特征点，结合相机基线长度，焦距，以及点的深度等信息来计算其在假想的"右侧图像"上的匹配点
        mb = mbf / fx;

        if (pPrevF)
        {
            if (pPrevF->HasVelocity())
                SetVelocity(pPrevF->GetVelocity());
        }
        else
        {
            mVw.setZero();
        }

        mpMutexImu = new std::mutex();

        // Set no stereo fisheye information
        Nleft = -1;
        Nright = -1;
        mvLeftToRightMatch = vector<int>(0);
        mvRightToLeftMatch = vector<int>(0);
        mvStereo3Dpoints = vector<Eigen::Vector3f>(0);
        monoLeft = -1;
        monoRight = -1;

        // 将特征点分配到图像网格中
        AssignFeaturesToGrid();
    }

    // ps3：单目模式
    /**
     * @brief 为单目相机准备的帧构造函数
     *
     * @param[in] imGray                            //灰度图
     * @param[in] timeStamp                         //时间戳
     * @param[in & out] extractor                   //ORB特征点提取器的句柄
     * @param[in] voc                               //ORB字典的句柄
     * @param[in] K                                 //相机的内参数矩阵
     * @param[in] distCoef                          //相机的去畸变参数
     * @param[in] bf                                //baseline*f
     * @param[in] thDepth                           //区分远近点的深度阈值
     */
    Frame::Frame(const cv::Mat &imGray, const double &timeStamp, ORBextractor *extractor, ORBVocabulary *voc, GeometricCamera *pCamera, cv::Mat &distCoef, const float &bf, const float &thDepth, Frame *pPrevF, const IMU::Calib &ImuCalib)
        : mpcpi(NULL), mpORBvocabulary(voc), mpORBextractorLeft(extractor), mpORBextractorRight(static_cast<ORBextractor *>(NULL)),
          mTimeStamp(timeStamp), mK(static_cast<Pinhole *>(pCamera)->toK()), mK_(static_cast<Pinhole *>(pCamera)->toK_()), mDistCoef(distCoef.clone()), mbf(bf), mThDepth(thDepth),
          mImuCalib(ImuCalib), mpImuPreintegrated(NULL), mpPrevFrame(pPrevF), mpImuPreintegratedFrame(NULL), mpReferenceKF(static_cast<KeyFrame *>(NULL)), mbIsSet(false), mbImuPreintegrated(false), mpCamera(pCamera),
          mpCamera2(nullptr), mbHasPose(false), mbHasVelocity(false)
    {
        // Step 1 帧的 ID 自增
        mnId = nNextId++;

        // Step 2 计算图像金字塔的参数
        mnScaleLevels = mpORBextractorLeft->GetLevels();
        mfScaleFactor = mpORBextractorLeft->GetScaleFactor();
        mfLogScaleFactor = log(mfScaleFactor);
        mvScaleFactors = mpORBextractorLeft->GetScaleFactors();
        mvInvScaleFactors = mpORBextractorLeft->GetInverseScaleFactors();
        mvLevelSigma2 = mpORBextractorLeft->GetScaleSigmaSquares();
        mvInvLevelSigma2 = mpORBextractorLeft->GetInverseScaleSigmaSquares();

        // ORB extraction

#ifdef REGISTER_TIMES
        std::chrono::steady_clock::time_point time_StartExtORB = std::chrono::steady_clock::now();
#endif
        // Step 3 对这个单目图像进行提取特征点, 第一个参数 0-左图， 1-右图
        ExtractORB(0, imGray, 0, 1000);

#ifdef REGISTER_TIMES
        std::chrono::steady_clock::time_point time_EndExtORB = std::chrono::steady_clock::now();

        mTimeORB_Ext = std::chrono::duration_cast<std::chrono::duration<double, std::milli>>(time_EndExtORB - time_StartExtORB).count();
#endif

        N = mvKeys.size();
        if (mvKeys.empty())
            return;

        // Step 4 用 OpenCV 的矫正函数、内参对提取到的特征点进行矫正
        UndistortKeyPoints();

        // 由于单目相机无法直接获得立体信息，所以这里要给右图像对应点和深度赋值-1表示没有相关信息
        // Set no stereo information
        mvuRight = vector<float>(N, -1);
        mvDepth = vector<float>(N, -1);
        mnCloseMPs = 0;

        // 初始化本帧的地图点
        mvpMapPoints = vector<MapPoint *>(N, static_cast<MapPoint *>(NULL));

        mmProjectPoints.clear(); // = map<long unsigned int, cv::Point2f>(N, static_cast<cv::Point2f>(NULL));
        mmMatchedInImage.clear();

        // 记录地图点是否为外点，初始化均为外点 false
        mvbOutlier = vector<bool>(N, false);

        //  Step 5 计算去畸变后图像边界，将特征点分配到网格中。这个过程一般是在第一帧或者是相机标定参数发生变化之后进行
        if (mbInitialComputations)
        {
            ComputeImageBounds(imGray);

            mfGridElementWidthInv = static_cast<float>(FRAME_GRID_COLS) / static_cast<float>(mnMaxX - mnMinX);
            mfGridElementHeightInv = static_cast<float>(FRAME_GRID_ROWS) / static_cast<float>(mnMaxY - mnMinY);

            fx = static_cast<Pinhole *>(mpCamera)->toK().at<float>(0, 0);
            fy = static_cast<Pinhole *>(mpCamera)->toK().at<float>(1, 1);
            cx = static_cast<Pinhole *>(mpCamera)->toK().at<float>(0, 2);
            cy = static_cast<Pinhole *>(mpCamera)->toK().at<float>(1, 2);
            invfx = 1.0f / fx;
            invfy = 1.0f / fy;

            mbInitialComputations = false;
        }

        // 计算 basline
        mb = mbf / fx;

        // Set no stereo fisheye information
        Nleft = -1;
        Nright = -1;
        mvLeftToRightMatch = vector<int>(0);
        mvRightToLeftMatch = vector<int>(0);
        mvStereo3Dpoints = vector<Eigen::Vector3f>(0);
        monoLeft = -1;
        monoRight = -1;

        // 将特征点分配到图像网格中
        AssignFeaturesToGrid();

        if (pPrevF)
        {
            if (pPrevF->HasVelocity())
            {
                SetVelocity(pPrevF->GetVelocity());
            }
        }
        else
        {
            mVw.setZero();
        }

        mpMutexImu = new std::mutex();
    }

    // 将提取到的特征点分配到图像网格中，该函数由构造函数调用
    void Frame::AssignFeaturesToGrid()
    {
        // Step 1  给存储特征点的网格数组 Frame::mGrid 预分配空间
        // Fill matrix with points
        const int nCells = FRAME_GRID_COLS * FRAME_GRID_ROWS;

        int nReserve = 0.5f * N / (nCells);

        // 开始对mGrid这个二维数组中的每一个vector元素遍历并预分配空间
        for (unsigned int i = 0; i < FRAME_GRID_COLS; i++)
            for (unsigned int j = 0; j < FRAME_GRID_ROWS; j++)
            {
                mGrid[i][j].reserve(nReserve);
                if (Nleft != -1)
                {
                    mGridRight[i][j].reserve(nReserve);
                }
            }

        // Step 2 遍历每个特征点，将每个特征点在 mvKeysUn 中的索引值放到对应的网格 mGrid 中
        for (int i = 0; i < N; i++)
        {
            const cv::KeyPoint &kp = (Nleft == -1) ? mvKeysUn[i]
                                     : (i < Nleft) ? mvKeys[i]
                                                   : mvKeysRight[i - Nleft];

            // 存储某个特征点所在网格的网格坐标，nGridPosX范围：[0,FRAME_GRID_COLS], nGridPosY范围：[0,FRAME_GRID_ROWS]
            int nGridPosX, nGridPosY;
            // 计算某个特征点所在网格的网格坐标，如果找到特征点所在的网格坐标，记录在nGridPosX,nGridPosY里，返回true，没找到返回false
            if (PosInGrid(kp, nGridPosX, nGridPosY))
            {
                if (Nleft == -1 || i < Nleft)
                    // 如果找到特征点所在网格坐标，将这个特征点的索引添加到对应网格的数组mGrid中
                    mGrid[nGridPosX][nGridPosY].push_back(i);
                else
                    mGridRight[nGridPosX][nGridPosY].push_back(i - Nleft);
            }
        }
    }

    /**
     * @brief 赋值新的偏置
     * @param flag 左右标志位
     * @param im 图片
     * @param x0 界限
     * @param x1 界限
     */
    /**
     * @brief 提取图像的ORB特征，提取的关键点存放在mvKeys，描述子存放在mDescriptors
     *
     * @param[in] flag          标记是左图还是右图。0：左图  1：右图
     * @param[in] im            等待提取特征点的图像
     */
    void Frame::ExtractORB(int flag, const cv::Mat &im, const int x0, const int x1)
    {
        vector<int> vLapping = {x0, x1};

        // 判断是左图还是右图
        if (flag == 0)
            // 左图的话就套使用左图指定的特征点提取器，并将提取结果保存到对应的变量中
            monoLeft = (*mpORBextractorLeft)(im, cv::Mat(), mvKeys, mDescriptors, vLapping);
        else
            // 右图的话就需要使用右图指定的特征点提取器，并将提取结果保存到对应的变量中
            monoRight = (*mpORBextractorRight)(im, cv::Mat(), mvKeysRight, mDescriptorsRight, vLapping);
    }

    bool Frame::isSet() const
    {
        return mbIsSet;
    }

    void Frame::SetPose(const Sophus::SE3<float> &Tcw)
    {
        mTcw = Tcw;

        UpdatePoseMatrices();
        mbIsSet = true;
        mbHasPose = true;
    }

    void Frame::SetNewBias(const IMU::Bias &b)
    {
        mImuBias = b;
        if (mpImuPreintegrated)
            mpImuPreintegrated->SetNewBias(b);
    }

    void Frame::SetVelocity(Eigen::Vector3f Vwb)
    {
        mVw = Vwb;
        mbHasVelocity = true;
    }

    Eigen::Vector3f Frame::GetVelocity() const
    {
        return mVw;
    }

    // 赋值位姿与速度
    void Frame::SetImuPoseVelocity(const Eigen::Matrix3f &Rwb, const Eigen::Vector3f &twb, const Eigen::Vector3f &Vwb)
    {
        mVw = Vwb;
        mbHasVelocity = true;

        Sophus::SE3f Twb(Rwb, twb);
        Sophus::SE3f Tbw = Twb.inverse();

        mTcw = mImuCalib.mTcb * Tbw;

        UpdatePoseMatrices();
        mbIsSet = true;
        mbHasPose = true;
    }

    void Frame::UpdatePoseMatrices()
    {
        Sophus::SE3<float> Twc = mTcw.inverse();
        
        mRwc = Twc.rotationMatrix();
        mOw = Twc.translation();
        mRcw = mTcw.rotationMatrix();
        mtcw = mTcw.translation();
    }

    Eigen::Matrix<float, 3, 1> Frame::GetImuPosition() const
    {
        return mRwc * mImuCalib.mTcb.translation() + mOw;
    }

    Eigen::Matrix<float, 3, 3> Frame::GetImuRotation()
    {
        return mRwc * mImuCalib.mTcb.rotationMatrix();
    }

    Sophus::SE3<float> Frame::GetImuPose()
    {
        return mTcw.inverse() * mImuCalib.mTcb;
    }

    Sophus::SE3f Frame::GetRelativePoseTrl()
    {
        return mTrl;
    }

    Sophus::SE3f Frame::GetRelativePoseTlr()
    {
        return mTlr;
    }

    Eigen::Matrix3f Frame::GetRelativePoseTlr_rotation()
    {
        return mTlr.rotationMatrix();
    }

    Eigen::Vector3f Frame::GetRelativePoseTlr_translation()
    {
        return mTlr.translation();
    }

    // 判断地图点是否投影到当前帧的图像中，并计算投影坐标
    /**
     * @brief 判断路标点是否在视野中
     * 步骤
     * Step 1 获得这个地图点的世界坐标
     * Step 2 关卡一：检查这个地图点在当前帧的相机坐标系下，是否有正的深度.如果是负的，表示出错，返回false
     * Step 3 关卡二：将MapPoint投影到当前帧的像素坐标(u,v), 并判断是否在图像有效范围内
     * Step 4 关卡三：计算MapPoint到相机中心的距离, 并判断是否在尺度变化的距离内
     * Step 5 关卡四：计算当前相机指向地图点向量和地图点的平均观测方向夹角的余弦值, 若小于设定阈值，返回false
     * Step 6 根据地图点到光心的距离来预测一个尺度（仿照特征点金字塔层级）
     * Step 7 记录计算得到的一些参数
     * @param[in] pMP                       当前地图点
     * @param[in] viewingCosLimit           夹角余弦，用于限制地图点和光心连线和法线的夹角
     * @return true                         地图点合格，且在视野内
     * @return false                        地图点不合格，抛弃
     */
    bool Frame::isInFrustum(MapPoint *pMP, float viewingCosLimit)
    {
        // case1：单目，立体匹配双目，rgbd
        if (Nleft == -1)
        {
            pMP->mbTrackInView = false;
            pMP->mTrackProjX = -1;
            pMP->mTrackProjY = -1;

            // 3D in absolute coordinates
            Eigen::Matrix<float, 3, 1> P = pMP->GetWorldPos();

            // 3D in camera coordinates
            const Eigen::Matrix<float, 3, 1> Pc = mRcw * P + mtcw;
            const float Pc_dist = Pc.norm();

            // Check positive depth
            const float &PcZ = Pc(2);
            const float invz = 1.0f / PcZ;
            if (PcZ < 0.0f)
                return false;

            const Eigen::Vector2f uv = mpCamera->project(Pc);

            if (uv(0) < mnMinX || uv(0) > mnMaxX)
                return false;
            if (uv(1) < mnMinY || uv(1) > mnMaxY)
                return false;

            pMP->mTrackProjX = uv(0);
            pMP->mTrackProjY = uv(1);

            // Check distance is in the scale invariance region of the MapPoint
            const float maxDistance = pMP->GetMaxDistanceInvariance();
            const float minDistance = pMP->GetMinDistanceInvariance();
            const Eigen::Vector3f PO = P - mOw;
            const float dist = PO.norm();

            if (dist < minDistance || dist > maxDistance)
                return false;

            // Check viewing angle
            Eigen::Vector3f Pn = pMP->GetNormal();

            const float viewCos = PO.dot(Pn) / dist;

            if (viewCos < viewingCosLimit)
                return false;

            // Predict scale in the image
            const int nPredictedLevel = pMP->PredictScale(dist, this);

            // Data used by the tracking
            pMP->mbTrackInView = true;
            pMP->mTrackProjX = uv(0);
            pMP->mTrackProjXR = uv(0) - mbf * invz;

            pMP->mTrackDepth = Pc_dist;

            pMP->mTrackProjY = uv(1);
            pMP->mnTrackScaleLevel = nPredictedLevel;
            pMP->mTrackViewCos = viewCos;

            return true;
        }

        // case2：左右目时分别验证
        else
        {
            pMP->mbTrackInView = false;
            pMP->mbTrackInViewR = false;
            pMP->mnTrackScaleLevel = -1;
            pMP->mnTrackScaleLevelR = -1;

            pMP->mbTrackInView = isInFrustumChecks(pMP, viewingCosLimit);
            pMP->mbTrackInViewR = isInFrustumChecks(pMP, viewingCosLimit, true);

            return pMP->mbTrackInView || pMP->mbTrackInViewR;
        }
    }

    bool Frame::ProjectPointDistort(MapPoint *pMP, cv::Point2f &kp, float &u, float &v)
    {

        // 3D in absolute coordinates
        Eigen::Vector3f P = pMP->GetWorldPos();

        // 3D in camera coordinates
        const Eigen::Vector3f Pc = mRcw * P + mtcw;
        const float &PcX = Pc(0);
        const float &PcY = Pc(1);
        const float &PcZ = Pc(2);

        // Check positive depth
        if (PcZ < 0.0f)
        {
            cout << "Negative depth: " << PcZ << endl;
            return false;
        }

        // Project in image and check it is not outside
        const float invz = 1.0f / PcZ;
        u = fx * PcX * invz + cx;
        v = fy * PcY * invz + cy;

        if (u < mnMinX || u > mnMaxX)
            return false;
        if (v < mnMinY || v > mnMaxY)
            return false;

        float u_distort, v_distort;

        float x = (u - cx) * invfx;
        float y = (v - cy) * invfy;
        float r2 = x * x + y * y;
        float k1 = mDistCoef.at<float>(0);
        float k2 = mDistCoef.at<float>(1);
        float p1 = mDistCoef.at<float>(2);
        float p2 = mDistCoef.at<float>(3);
        float k3 = 0;
        if (mDistCoef.total() == 5)
        {
            k3 = mDistCoef.at<float>(4);
        }

        // Radial distorsion
        float x_distort = x * (1 + k1 * r2 + k2 * r2 * r2 + k3 * r2 * r2 * r2);
        float y_distort = y * (1 + k1 * r2 + k2 * r2 * r2 + k3 * r2 * r2 * r2);

        // Tangential distorsion
        x_distort = x_distort + (2 * p1 * x * y + p2 * (r2 + 2 * x * x));
        y_distort = y_distort + (p1 * (r2 + 2 * y * y) + 2 * p2 * x * y);

        u_distort = x_distort * fx + cx;
        v_distort = y_distort * fy + cy;

        u = u_distort;
        v = v_distort;

        kp = cv::Point2f(u, v);

        return true;
    }

    Eigen::Vector3f Frame::inRefCoordinates(Eigen::Vector3f pCw)
    {
        return mRcw * pCw + mtcw;
    }

    vector<size_t> Frame::GetFeaturesInArea(const float &x, const float &y, const float &r, const int minLevel, const int maxLevel, const bool bRight) const
    {
        vector<size_t> vIndices;
        vIndices.reserve(N);

        float factorX = r;
        float factorY = r;

        const int nMinCellX = max(0, (int)floor((x - mnMinX - factorX) * mfGridElementWidthInv));
        if (nMinCellX >= FRAME_GRID_COLS)
        {
            return vIndices;
        }

        const int nMaxCellX = min((int)FRAME_GRID_COLS - 1, (int)ceil((x - mnMinX + factorX) * mfGridElementWidthInv));
        if (nMaxCellX < 0)
        {
            return vIndices;
        }

        const int nMinCellY = max(0, (int)floor((y - mnMinY - factorY) * mfGridElementHeightInv));
        if (nMinCellY >= FRAME_GRID_ROWS)
        {
            return vIndices;
        }

        const int nMaxCellY = min((int)FRAME_GRID_ROWS - 1, (int)ceil((y - mnMinY + factorY) * mfGridElementHeightInv));
        if (nMaxCellY < 0)
        {
            return vIndices;
        }

        const bool bCheckLevels = (minLevel > 0) || (maxLevel >= 0);

        for (int ix = nMinCellX; ix <= nMaxCellX; ix++)
        {
            for (int iy = nMinCellY; iy <= nMaxCellY; iy++)
            {
                const vector<size_t> vCell = (!bRight) ? mGrid[ix][iy] : mGridRight[ix][iy];
                if (vCell.empty())
                    continue;

                for (size_t j = 0, jend = vCell.size(); j < jend; j++)
                {
                    const cv::KeyPoint &kpUn = (Nleft == -1) ? mvKeysUn[vCell[j]]
                                               : (!bRight)   ? mvKeys[vCell[j]]
                                                             : mvKeysRight[vCell[j]];
                    if (bCheckLevels)
                    {
                        if (kpUn.octave < minLevel)
                            continue;
                        if (maxLevel >= 0)
                            if (kpUn.octave > maxLevel)
                                continue;
                    }

                    const float distx = kpUn.pt.x - x;
                    const float disty = kpUn.pt.y - y;

                    if (fabs(distx) < factorX && fabs(disty) < factorY)
                        vIndices.push_back(vCell[j]);
                }
            }
        }

        return vIndices;
    }

    bool Frame::PosInGrid(const cv::KeyPoint &kp, int &posX, int &posY)
    {
        posX = round((kp.pt.x - mnMinX) * mfGridElementWidthInv);
        posY = round((kp.pt.y - mnMinY) * mfGridElementHeightInv);

        // Keypoint's coordinates are undistorted, which could cause to go out of the image
        if (posX < 0 || posX >= FRAME_GRID_COLS || posY < 0 || posY >= FRAME_GRID_ROWS)
            return false;

        return true;
    }

    void Frame::ComputeBoW()
    {
        if (mBowVec.empty())
        {
            vector<cv::Mat> vCurrentDesc = Converter::toDescriptorVector(mDescriptors);
            mpORBvocabulary->transform(vCurrentDesc, mBowVec, mFeatVec, 4);
        }
    }

    // 用内参对特征点去畸变，结果保存在 mvKeysUn 中
    void Frame::UndistortKeyPoints()
    {
        if (mDistCoef.at<float>(0) == 0.0)
        {
            mvKeysUn = mvKeys;
            return;
        }

        // Fill matrix with points
        cv::Mat mat(N, 2, CV_32F);

        for (int i = 0; i < N; i++)
        {
            mat.at<float>(i, 0) = mvKeys[i].pt.x;
            mat.at<float>(i, 1) = mvKeys[i].pt.y;
        }

        // Undistort points
        mat = mat.reshape(2);
        cv::undistortPoints(mat, mat, static_cast<Pinhole *>(mpCamera)->toK(), mDistCoef, cv::Mat(), mK);
        mat = mat.reshape(1);

        // Fill undistorted keypoint vector
        mvKeysUn.resize(N);
        for (int i = 0; i < N; i++)
        {
            cv::KeyPoint kp = mvKeys[i];
            kp.pt.x = mat.at<float>(i, 0);
            kp.pt.y = mat.at<float>(i, 1);
            mvKeysUn[i] = kp;
        }
    }

    // 计算去畸变图像的边界
    void Frame::ComputeImageBounds(const cv::Mat &imLeft) // 需要计算边界的图像
    {
        if (mDistCoef.at<float>(0) != 0.0)
        {
            cv::Mat mat(4, 2, CV_32F);
            mat.at<float>(0, 0) = 0.0;
            mat.at<float>(0, 1) = 0.0;
            mat.at<float>(1, 0) = imLeft.cols;
            mat.at<float>(1, 1) = 0.0;
            mat.at<float>(2, 0) = 0.0;
            mat.at<float>(2, 1) = imLeft.rows;
            mat.at<float>(3, 0) = imLeft.cols;
            mat.at<float>(3, 1) = imLeft.rows;

            mat = mat.reshape(2);
            cv::undistortPoints(mat, mat, static_cast<Pinhole *>(mpCamera)->toK(), mDistCoef, cv::Mat(), mK);
            mat = mat.reshape(1);

            // Undistort corners
            mnMinX = min(mat.at<float>(0, 0), mat.at<float>(2, 0));
            mnMaxX = max(mat.at<float>(1, 0), mat.at<float>(3, 0));
            mnMinY = min(mat.at<float>(0, 1), mat.at<float>(1, 1));
            mnMaxY = max(mat.at<float>(2, 1), mat.at<float>(3, 1));
        }
        else
        {
            mnMinX = 0.0f;
            mnMaxX = imLeft.cols;
            mnMinY = 0.0f;
            mnMaxY = imLeft.rows;
        }
    }

    // 双目立体匹配
    /**
     * @brief 两帧图像稀疏立体匹配（即：ORB特征点匹配，非逐像素的密集匹配，但依然满足行对齐）
     *    输入：两帧立体矫正后的图像 img_left 和 img_right 对应的 ORB 特征点集
     *    过程：
            1. 行特征点统计. 统计img_right每一行上的ORB特征点集，便于使用立体匹配思路(行搜索/极线搜索）进行同名点搜索, 避免逐像素的判断.
            2. 粗匹配. 根据步骤1的结果，对img_left第i行的orb特征点pi，在img_right的第i行上的orb特征点集中搜索相似orb特征点, 得到qi
            3. 精确匹配. 以点qi为中心，半径为r的范围内，进行块匹配（归一化SAD），进一步优化匹配结果
            4. 亚像素精度优化. 步骤3得到的视差为uchar/int类型精度，并不一定是真实视差，通过亚像素差值（抛物线插值)获取float精度的真实视差
            5. 最优视差值/深度选择. 通过胜者为王算法（WTA）获取最佳匹配点。
            6. 删除离缺点(outliers). 块匹配相似度阈值判断，归一化sad最小，并不代表就一定是正确匹配，比如光照变化、弱纹理等会造成误匹配
     *    输出：稀疏特征点视差图/深度图（亚像素精度）mvDepth 匹配结果 mvuRight
     */
    void Frame::ComputeStereoMatches()
    {
        // mvuRight 中存储的应该是左图像中的点所匹配的在右图像中的点的横坐标（纵坐标相同）
        mvuRight = vector<float>(N, -1.0f);
        mvDepth = vector<float>(N, -1.0f);

        const int thOrbDist = (ORBmatcher::TH_HIGH + ORBmatcher::TH_LOW) / 2;

        const int nRows = mpORBextractorLeft->mvImagePyramid[0].rows;

        // Assign keypoints to row table
        vector<vector<size_t>> vRowIndices(nRows, vector<size_t>());

        for (int i = 0; i < nRows; i++)
            vRowIndices[i].reserve(200);

        const int Nr = mvKeysRight.size();

        for (int iR = 0; iR < Nr; iR++)
        {
            const cv::KeyPoint &kp = mvKeysRight[iR];
            const float &kpY = kp.pt.y;
            const float r = 2.0f * mvScaleFactors[mvKeysRight[iR].octave];
            const int maxr = ceil(kpY + r);
            const int minr = floor(kpY - r);

            for (int yi = minr; yi <= maxr; yi++)
                vRowIndices[yi].push_back(iR);
        }

        // Set limits for search
        const float minZ = mb;
        const float minD = 0;
        const float maxD = mbf / minZ;

        // For each left keypoint search a match in the right image
        vector<pair<int, int>> vDistIdx;
        vDistIdx.reserve(N);

        for (int iL = 0; iL < N; iL++)
        {
            const cv::KeyPoint &kpL = mvKeys[iL];
            const int &levelL = kpL.octave;
            const float &vL = kpL.pt.y;
            const float &uL = kpL.pt.x;

            const vector<size_t> &vCandidates = vRowIndices[vL];

            if (vCandidates.empty())
                continue;

            const float minU = uL - maxD;
            const float maxU = uL - minD;

            if (maxU < 0)
                continue;

            int bestDist = ORBmatcher::TH_HIGH;
            size_t bestIdxR = 0;

            const cv::Mat &dL = mDescriptors.row(iL);

            // Compare descriptor to right keypoints
            for (size_t iC = 0; iC < vCandidates.size(); iC++)
            {
                const size_t iR = vCandidates[iC];
                const cv::KeyPoint &kpR = mvKeysRight[iR];

                if (kpR.octave < levelL - 1 || kpR.octave > levelL + 1)
                    continue;

                const float &uR = kpR.pt.x;

                if (uR >= minU && uR <= maxU)
                {
                    const cv::Mat &dR = mDescriptorsRight.row(iR);
                    const int dist = ORBmatcher::DescriptorDistance(dL, dR);

                    if (dist < bestDist)
                    {
                        bestDist = dist;
                        bestIdxR = iR;
                    }
                }
            }

            // Subpixel match by correlation
            if (bestDist < thOrbDist)
            {
                // coordinates in image pyramid at keypoint scale
                const float uR0 = mvKeysRight[bestIdxR].pt.x;
                const float scaleFactor = mvInvScaleFactors[kpL.octave];
                const float scaleduL = round(kpL.pt.x * scaleFactor);
                const float scaledvL = round(kpL.pt.y * scaleFactor);
                const float scaleduR0 = round(uR0 * scaleFactor);

                // sliding window search
                const int w = 5;
                cv::Mat IL = mpORBextractorLeft->mvImagePyramid[kpL.octave].rowRange(scaledvL - w, scaledvL + w + 1).colRange(scaleduL - w, scaleduL + w + 1);

                int bestDist = INT_MAX;
                int bestincR = 0;
                const int L = 5;
                vector<float> vDists;
                vDists.resize(2 * L + 1);

                const float iniu = scaleduR0 + L - w;
                const float endu = scaleduR0 + L + w + 1;
                if (iniu < 0 || endu >= mpORBextractorRight->mvImagePyramid[kpL.octave].cols)
                    continue;

                for (int incR = -L; incR <= +L; incR++)
                {
                    cv::Mat IR = mpORBextractorRight->mvImagePyramid[kpL.octave].rowRange(scaledvL - w, scaledvL + w + 1).colRange(scaleduR0 + incR - w, scaleduR0 + incR + w + 1);

                    float dist = cv::norm(IL, IR, cv::NORM_L1);
                    if (dist < bestDist)
                    {
                        bestDist = dist;
                        bestincR = incR;
                    }

                    vDists[L + incR] = dist;
                }

                if (bestincR == -L || bestincR == L)
                    continue;

                // Sub-pixel match (Parabola fitting)
                const float dist1 = vDists[L + bestincR - 1];
                const float dist2 = vDists[L + bestincR];
                const float dist3 = vDists[L + bestincR + 1];

                const float deltaR = (dist1 - dist3) / (2.0f * (dist1 + dist3 - 2.0f * dist2));

                if (deltaR < -1 || deltaR > 1)
                    continue;

                // Re-scaled coordinate
                float bestuR = mvScaleFactors[kpL.octave] * ((float)scaleduR0 + (float)bestincR + deltaR);

                float disparity = (uL - bestuR);

                if (disparity >= minD && disparity < maxD)
                {
                    if (disparity <= 0)
                    {
                        disparity = 0.01;
                        bestuR = uL - 0.01;
                    }
                    mvDepth[iL] = mbf / disparity;
                    mvuRight[iL] = bestuR;
                    vDistIdx.push_back(pair<int, int>(bestDist, iL));
                }
            }
        }

        sort(vDistIdx.begin(), vDistIdx.end());
        const float median = vDistIdx[vDistIdx.size() / 2].first;
        const float thDist = 1.5f * 1.4f * median;

        for (int i = vDistIdx.size() - 1; i >= 0; i--)
        {
            if (vDistIdx[i].first < thDist)
                break;
            else
            {
                mvuRight[vDistIdx[i].second] = -1;
                mvDepth[vDistIdx[i].second] = -1;
            }
        }
    }

    void Frame::ComputeStereoFromRGBD(const cv::Mat &imDepth)
    {
        mvuRight = vector<float>(N, -1);
        mvDepth = vector<float>(N, -1);

        for (int i = 0; i < N; i++)
        {
            const cv::KeyPoint &kp = mvKeys[i];
            const cv::KeyPoint &kpU = mvKeysUn[i];

            const float &v = kp.pt.y;
            const float &u = kp.pt.x;

            const float d = imDepth.at<float>(v, u);

            if (d > 0)
            {
                mvDepth[i] = d;
                mvuRight[i] = kpU.pt.x - mbf / d;
            }
        }
    }

    // 当某个特征点的深度信息或者双目信息有效时，将它反投影到三维世界坐标系中
    bool Frame::UnprojectStereo(const int &i,         // 输入：图像中的第 i 个特征点位置和深度值
                                Eigen::Vector3f &x3D) // 输出：得到的世界坐标系下的三维点
    {
        const float z = mvDepth[i];

        if (z > 0)
        {
            const float u = mvKeysUn[i].pt.x;
            const float v = mvKeysUn[i].pt.y;
            const float x = (u - cx) * z * invfx;
            const float y = (v - cy) * z * invfy;
            Eigen::Vector3f x3Dc(x, y, z);
            x3D = mRwc * x3Dc + mOw;

            return true;
        }
        else
            return false;
    }

    // 是否做完预积分
    bool Frame::imuIsPreintegrated()
    {
        unique_lock<std::mutex> lock(*mpMutexImu);
        return mbImuPreintegrated;
    }

    // 设置为做完预积分
    void Frame::setIntegrated()
    {
        unique_lock<std::mutex> lock(*mpMutexImu);
        mbImuPreintegrated = true;
    }

    // ps5：好像是带 IMU 的？
    Frame::Frame(const cv::Mat &imLeft, const cv::Mat &imRight, const double &timeStamp, ORBextractor *extractorLeft, ORBextractor *extractorRight, ORBVocabulary *voc, cv::Mat &K, cv::Mat &distCoef, const float &bf, const float &thDepth, GeometricCamera *pCamera, GeometricCamera *pCamera2, Sophus::SE3f &Tlr, Frame *pPrevF, const IMU::Calib &ImuCalib)
        : mpcpi(NULL), mpORBvocabulary(voc), mpORBextractorLeft(extractorLeft), mpORBextractorRight(extractorRight), mTimeStamp(timeStamp), mK(K.clone()), mK_(Converter::toMatrix3f(K)), mDistCoef(distCoef.clone()), mbf(bf), mThDepth(thDepth),
          mImuCalib(ImuCalib), mpImuPreintegrated(NULL), mpPrevFrame(pPrevF), mpImuPreintegratedFrame(NULL), mpReferenceKF(static_cast<KeyFrame *>(NULL)), mbImuPreintegrated(false), mpCamera(pCamera), mpCamera2(pCamera2),
          mbHasPose(false), mbHasVelocity(false)

    {
        imgLeft = imLeft.clone();
        imgRight = imRight.clone();

        // Frame ID
        mnId = nNextId++;

        // Scale Level Info
        mnScaleLevels = mpORBextractorLeft->GetLevels();
        mfScaleFactor = mpORBextractorLeft->GetScaleFactor();
        mfLogScaleFactor = log(mfScaleFactor);
        mvScaleFactors = mpORBextractorLeft->GetScaleFactors();
        mvInvScaleFactors = mpORBextractorLeft->GetInverseScaleFactors();
        mvLevelSigma2 = mpORBextractorLeft->GetScaleSigmaSquares();
        mvInvLevelSigma2 = mpORBextractorLeft->GetInverseScaleSigmaSquares();

        // ORB extraction
#ifdef REGISTER_TIMES
        std::chrono::steady_clock::time_point time_StartExtORB = std::chrono::steady_clock::now();
#endif
        thread threadLeft(&Frame::ExtractORB, this, 0, imLeft, static_cast<KannalaBrandt8 *>(mpCamera)->mvLappingArea[0], static_cast<KannalaBrandt8 *>(mpCamera)->mvLappingArea[1]);
        thread threadRight(&Frame::ExtractORB, this, 1, imRight, static_cast<KannalaBrandt8 *>(mpCamera2)->mvLappingArea[0], static_cast<KannalaBrandt8 *>(mpCamera2)->mvLappingArea[1]);
        threadLeft.join();
        threadRight.join();
#ifdef REGISTER_TIMES
        std::chrono::steady_clock::time_point time_EndExtORB = std::chrono::steady_clock::now();

        mTimeORB_Ext = std::chrono::duration_cast<std::chrono::duration<double, std::milli>>(time_EndExtORB - time_StartExtORB).count();
#endif

        Nleft = mvKeys.size();
        Nright = mvKeysRight.size();
        N = Nleft + Nright;

        if (N == 0)
            return;

        // This is done only for the first Frame (or after a change in the calibration)
        if (mbInitialComputations)
        {
            ComputeImageBounds(imLeft);

            mfGridElementWidthInv = static_cast<float>(FRAME_GRID_COLS) / (mnMaxX - mnMinX);
            mfGridElementHeightInv = static_cast<float>(FRAME_GRID_ROWS) / (mnMaxY - mnMinY);

            fx = K.at<float>(0, 0);
            fy = K.at<float>(1, 1);
            cx = K.at<float>(0, 2);
            cy = K.at<float>(1, 2);
            invfx = 1.0f / fx;
            invfy = 1.0f / fy;

            mbInitialComputations = false;
        }

        mb = mbf / fx;

        // Sophus/Eigen
        mTlr = Tlr;
        mTrl = mTlr.inverse();
        mRlr = mTlr.rotationMatrix();
        mtlr = mTlr.translation();

#ifdef REGISTER_TIMES
        std::chrono::steady_clock::time_point time_StartStereoMatches = std::chrono::steady_clock::now();
#endif
        ComputeStereoFishEyeMatches();
#ifdef REGISTER_TIMES
        std::chrono::steady_clock::time_point time_EndStereoMatches = std::chrono::steady_clock::now();

        mTimeStereoMatch = std::chrono::duration_cast<std::chrono::duration<double, std::milli>>(time_EndStereoMatches - time_StartStereoMatches).count();
#endif

        // Put all descriptors in the same matrix
        cv::vconcat(mDescriptors, mDescriptorsRight, mDescriptors);

        mvpMapPoints = vector<MapPoint *>(N, static_cast<MapPoint *>(nullptr));
        mvbOutlier = vector<bool>(N, false);

        AssignFeaturesToGrid();

        mpMutexImu = new std::mutex();

        UndistortKeyPoints();
    }

    // notice：左右目匹配
    void Frame::ComputeStereoFishEyeMatches()
    {
        // 1. 分别取出特征点
        // Speed it up by matching keypoints in the lapping area
        vector<cv::KeyPoint> stereoLeft(mvKeys.begin() + monoLeft, mvKeys.end());
        vector<cv::KeyPoint> stereoRight(mvKeysRight.begin() + monoRight, mvKeysRight.end());

        // 2. 分别取出描述子
        cv::Mat stereoDescLeft = mDescriptors.rowRange(monoLeft, mDescriptors.rows);
        cv::Mat stereoDescRight = mDescriptorsRight.rowRange(monoRight, mDescriptorsRight.rows);

        // 一些在当前模式用不到的变量给他填一下
        mvLeftToRightMatch = vector<int>(Nleft, -1);
        mvRightToLeftMatch = vector<int>(Nright, -1);
        mvDepth = vector<float>(Nleft, -1.0f);
        mvuRight = vector<float>(Nleft, -1);
        mvStereo3Dpoints = vector<Eigen::Vector3f>(Nleft);
        mnCloseMPs = 0;

        // Perform a brute force between Keypoint in the left and right image
        vector<vector<cv::DMatch>> matches;

        // 3. 暴力匹配
        BFmatcher.knnMatch(stereoDescLeft, stereoDescRight, matches, 2);

        int nMatches = 0;
        int descMatches = 0;

        // Check matches using Lowe's ratio
        for (vector<vector<cv::DMatch>>::iterator it = matches.begin(); it != matches.end(); ++it)
        {
            // 对于每一对匹配的候选中，最小距离比次小距离的 0.7 倍还小的认为是好的匹配
            if ((*it).size() >= 2 && (*it)[0].distance < (*it)[1].distance * 0.7)
            {
                //  对于好的匹配，做三角化，且深度值有效的放入结果
                // For every good match, check parallax and reprojection error to discard spurious matches
                Eigen::Vector3f p3D;
                descMatches++;
                float sigma1 = mvLevelSigma2[mvKeys[(*it)[0].queryIdx + monoLeft].octave], sigma2 = mvLevelSigma2[mvKeysRight[(*it)[0].trainIdx + monoRight].octave];
                // 三角化
                float depth = static_cast<KannalaBrandt8 *>(mpCamera)->TriangulateMatches(mpCamera2, mvKeys[(*it)[0].queryIdx + monoLeft], mvKeysRight[(*it)[0].trainIdx + monoRight], mRlr, mtlr, sigma1, sigma2, p3D);
                // 填充数据
                if (depth > 0.0001f)
                {
                    mvLeftToRightMatch[(*it)[0].queryIdx + monoLeft] = (*it)[0].trainIdx + monoRight;
                    mvRightToLeftMatch[(*it)[0].trainIdx + monoRight] = (*it)[0].queryIdx + monoLeft;
                    mvStereo3Dpoints[(*it)[0].queryIdx + monoLeft] = p3D;
                    mvDepth[(*it)[0].queryIdx + monoLeft] = depth;
                    nMatches++;
                }
            }
        }
    }

    bool Frame::isInFrustumChecks(MapPoint *pMP, float viewingCosLimit, bool bRight)
    {
        // 3D in absolute coordinates
        Eigen::Vector3f P = pMP->GetWorldPos();

        Eigen::Matrix3f mR;
        Eigen::Vector3f mt, twc;
        if (bRight)
        {
            Eigen::Matrix3f Rrl = mTrl.rotationMatrix();
            Eigen::Vector3f trl = mTrl.translation();
            mR = Rrl * mRcw;
            mt = Rrl * mtcw + trl;
            twc = mRwc * mTlr.translation() + mOw;
        }
        else
        {
            mR = mRcw;
            mt = mtcw;
            twc = mOw;
        }

        // 3D in camera coordinates
        Eigen::Vector3f Pc = mR * P + mt;
        const float Pc_dist = Pc.norm();
        const float &PcZ = Pc(2);

        // Check positive depth
        if (PcZ < 0.0f)
            return false;

        // Project in image and check it is not outside
        Eigen::Vector2f uv;
        if (bRight)
            uv = mpCamera2->project(Pc);
        else
            uv = mpCamera->project(Pc);

        if (uv(0) < mnMinX || uv(0) > mnMaxX)
            return false;
        if (uv(1) < mnMinY || uv(1) > mnMaxY)
            return false;

        // Check distance is in the scale invariance region of the MapPoint
        const float maxDistance = pMP->GetMaxDistanceInvariance();
        const float minDistance = pMP->GetMinDistanceInvariance();
        const Eigen::Vector3f PO = P - twc;
        const float dist = PO.norm();

        if (dist < minDistance || dist > maxDistance)
            return false;

        // Check viewing angle
        Eigen::Vector3f Pn = pMP->GetNormal();

        const float viewCos = PO.dot(Pn) / dist;

        if (viewCos < viewingCosLimit)
            return false;

        // Predict scale in the image
        const int nPredictedLevel = pMP->PredictScale(dist, this);

        if (bRight)
        {
            pMP->mTrackProjXR = uv(0);
            pMP->mTrackProjYR = uv(1);
            pMP->mnTrackScaleLevelR = nPredictedLevel;
            pMP->mTrackViewCosR = viewCos;
            pMP->mTrackDepthR = Pc_dist;
        }
        else
        {
            pMP->mTrackProjX = uv(0);
            pMP->mTrackProjY = uv(1);
            pMP->mnTrackScaleLevel = nPredictedLevel;
            pMP->mTrackViewCos = viewCos;
            pMP->mTrackDepth = Pc_dist;
        }

        return true;
    }

    Eigen::Vector3f Frame::UnprojectStereoFishEye(const int &i)
    {
        return mRwc * mvStereo3Dpoints[i] + mOw;
    }

} // namespace ORB_SLAM
