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

#ifndef FRAME_H
#define FRAME_H

#include <vector>

#include "Thirdparty/DBoW2/DBoW2/BowVector.h"
#include "Thirdparty/DBoW2/DBoW2/FeatureVector.h"

#include "Thirdparty/Sophus/sophus/geometry.hpp"

#include "ImuTypes.h"
#include "ORBVocabulary.h"

#include "Converter.h"
#include "Settings.h"

#include <mutex>
#include <opencv2/opencv.hpp>

#include "Eigen/Core"
#include <sophus/se3.hpp>

namespace ORB_SLAM3
{
#define FRAME_GRID_ROWS 48
#define FRAME_GRID_COLS 64

    class MapPoint;
    class KeyFrame;
    class ConstraintPoseImu;
    class GeometricCamera;
    class ORBextractor;

    /**
     * @brief 帧👇
     */
    class Frame
    {
    public:
        /**
         * @brief Construct a new Frame object without parameter.
         *
         */
        Frame();

        // Copy constructor.
        /**
         * @brief 拷贝构造函数
         * @details 复制构造函数, mLastFrame = Frame(mCurrentFrame) \n
         * 如果不是自定以拷贝函数的话，系统自动生成的拷贝函数对于所有涉及分配内存的操作都将是浅拷贝 \n
         * @param[in] frame 引用
         * @note 另外注意，调用这个函数的时候，这个函数中隐藏的this指针其实是指向目标帧的
         */
        Frame(const Frame &frame);

        // Constructor for stereo cameras.
        Frame(const cv::Mat &imLeft, const cv::Mat &imRight, const double &timeStamp, ORBextractor *extractorLeft, ORBextractor *extractorRight, ORBVocabulary *voc, cv::Mat &K, cv::Mat &distCoef, const float &bf, const float &thDepth, GeometricCamera *pCamera, Frame *pPrevF = static_cast<Frame *>(NULL), const IMU::Calib &ImuCalib = IMU::Calib());

        // Constructor for RGB-D cameras.
        Frame(const cv::Mat &imGray, const cv::Mat &imDepth, const double &timeStamp, ORBextractor *extractor, ORBVocabulary *voc, cv::Mat &K, cv::Mat &distCoef, const float &bf, const float &thDepth, GeometricCamera *pCamera, Frame *pPrevF = static_cast<Frame *>(NULL), const IMU::Calib &ImuCalib = IMU::Calib());

        // Constructor for Monocular cameras.
        Frame(const cv::Mat &imGray, const double &timeStamp, ORBextractor *extractor, ORBVocabulary *voc, GeometricCamera *pCamera, cv::Mat &distCoef, const float &bf, const float &thDepth, Frame *pPrevF = static_cast<Frame *>(NULL), const IMU::Calib &ImuCalib = IMU::Calib());

        // Destructor
        // ~Frame();

        // Extract ORB on the image. 0 for left image and 1 for right image.
        void ExtractORB(int flag, const cv::Mat &im, const int x0, const int x1);

        // Compute Bag of Words representation.
        void ComputeBoW();

        // Set the camera pose. (Imu pose is not modified!)
        void SetPose(const Sophus::SE3<float> &Tcw);

        // Set IMU velocity
        void SetVelocity(Eigen::Vector3f Vw);

        Eigen::Vector3f GetVelocity() const;

        // Set IMU pose and velocity (implicitly changes camera pose)
        void SetImuPoseVelocity(const Eigen::Matrix3f &Rwb, const Eigen::Vector3f &twb, const Eigen::Vector3f &Vwb);

        Eigen::Matrix<float, 3, 1> GetImuPosition() const;
        Eigen::Matrix<float, 3, 3> GetImuRotation();
        Sophus::SE3<float> GetImuPose();

        Sophus::SE3f GetRelativePoseTrl();
        Sophus::SE3f GetRelativePoseTlr();
        Eigen::Matrix3f GetRelativePoseTlr_rotation();
        Eigen::Vector3f GetRelativePoseTlr_translation();

        void SetNewBias(const IMU::Bias &b);

        // Check if a MapPoint is in the frustum of the camera
        // and fill variables of the MapPoint to be used by the tracking
        bool isInFrustum(MapPoint *pMP, float viewingCosLimit);

        bool ProjectPointDistort(MapPoint *pMP, cv::Point2f &kp, float &u, float &v);

        Eigen::Vector3f inRefCoordinates(Eigen::Vector3f pCw);

        // Compute the cell of a keypoint (return false if outside the grid)
        bool PosInGrid(const cv::KeyPoint &kp, int &posX, int &posY);

        vector<size_t> GetFeaturesInArea(const float &x, const float &y, const float &r, const int minLevel = -1, const int maxLevel = -1, const bool bRight = false) const;

        // Search a match for each keypoint in the left image to a keypoint in the right image.
        // If there is a match, depth is computed and the right coordinate associated to the left keypoint is stored.
        void ComputeStereoMatches();

        // Associate a "right" coordinate to a keypoint if there is valid depth in the depthmap.
        void ComputeStereoFromRGBD(const cv::Mat &imDepth);

        // Backprojects a keypoint (if stereo/depth info available) into 3D world coordinates.
        bool UnprojectStereo(const int &i, Eigen::Vector3f &x3D);

        ConstraintPoseImu *mpcpi;

        bool imuIsPreintegrated();
        void setIntegrated();

        bool isSet() const;

        // Computes rotation, translation and camera center matrices from the camera pose.
        void UpdatePoseMatrices();

        // Returns the camera center.
        inline Eigen::Vector3f GetCameraCenter()
        {
            return mOw;
        }

        // Returns inverse of rotation
        inline Eigen::Matrix3f GetRotationInverse()
        {
            return mRwc;
        }

        inline Sophus::SE3<float> GetPose() const
        {
            // TODO: can the Frame pose be accsessed from several threads? should this be protected somehow?
            return mTcw;
        }

        inline Eigen::Matrix3f GetRwc() const
        {
            return mRwc;
        }

        inline Eigen::Vector3f GetOw() const
        {
            return mOw;
        }

        inline bool HasPose() const
        {
            return mbHasPose;
        }

        inline bool HasVelocity() const
        {
            return mbHasVelocity;
        }

        // todo--Yolo
        vector<cv::Rect2i> mvDynamicArea;

    private:
        // Sophus/Eigen migration

        // 和相机位姿有关的变量
        Sophus::SE3<float> mTcw;         ///< 相机姿态 世界坐标系到相机坐标坐标系的变换矩阵，是我们常规理解中的相机位姿，这个位姿描述了相机在空间中的位置和朝向，通常用于后续的地图构建、定位和其他计算。
        Eigen::Matrix<float, 3, 3> mRwc; ///< Rotation from camera to world
        Eigen::Matrix<float, 3, 1> mOw;  ///< mtwc,Translation from camera to world
        Eigen::Matrix<float, 3, 3> mRcw; ///< Rotation from world to camera
        Eigen::Matrix<float, 3, 1> mtcw; ///< Translation from world to camera
        bool mbHasPose;

        // Rcw_ not necessary as Sophus has a method for extracting the rotation matrix: Tcw_.rotationMatrix()
        // tcw_ not necessary as Sophus has a method for extracting the translation vector: Tcw_.translation()
        // Twc_ not necessary as Sophus has a method for easily computing the inverse pose: Tcw_.inverse()

        Sophus::SE3<float> mTlr, mTrl;
        Eigen::Matrix<float, 3, 3> mRlr;
        Eigen::Vector3f mtlr;

        // IMU linear velocity
        Eigen::Vector3f mVw;
        bool mbHasVelocity;

    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW

        /// 用于重定位的 ORB 特征字典
        // Vocabulary used for relocalization.
        ORBVocabulary *mpORBvocabulary;

        /// ORB特征提取器句柄，其中右侧的提取器句柄只会在【双目】输入的情况中才会被用到
        // Feature extractor. The right is used only in the stereo case.
        ORBextractor *mpORBextractorLeft, *mpORBextractorRight;

        // Frame timestamp.
        double mTimeStamp;

        /**
         * @name 相机的内参数
         * @{
         */

        // Calibration matrix and OpenCV distortion parameters.
        // NOTICE 注意，这里的相机内参数其实都是类的静态成员变量；此外相机的内参数矩阵和矫正参数矩阵却是普通的成员变量，这样是否有些浪费内存空间？
        cv::Mat mK;
        Eigen::Matrix3f mK_;
        static float fx;    ///< x轴方向焦距
        static float fy;    ///< y轴方向焦距
        static float cx;    ///< x轴方向光心偏移
        static float cy;    ///< y轴方向光心偏移
        static float invfx; ///< x轴方向焦距的逆
        static float invfy; ///< x轴方向焦距的逆

        // TODO 目测是 opencv 提供的图像去畸变参数矩阵的，但是其具体组成未知
        cv::Mat mDistCoef;

        // Stereo baseline multiplied by fx.
        float mbf;

        /// 相机的基线长度，单位为米
        // Stereo baseline in meters.
        float mb;

        /** @} */

        // Threshold close/far points. Close points are inserted from 1 view.
        // Far points are inserted as in the monocular case from 2 views.
        float mThDepth;

        // Number of KeyPoints.
        int N;

        /**
         * @name 关于特征点
         * @{
         */

        // Vector of keypoints (original for visualization) and undistorted (actually used by the system).
        // In the stereo case, mvKeysUn is redundant as images must be rectified.
        // In the RGB-D case, RGB images can be distorted.
        std::vector<cv::KeyPoint> mvKeys;      // ps：图像中提取的【未校正】的关键点，可能包括畸变。如果图像已经经过校正，则 mvKeys 和 mvKeysUn 是相同的。
        std::vector<cv::KeyPoint> mvKeysRight; // ps：（在双目模式中）存储右图像中对应的【未校正】的关键点，便于进行立体匹配。
        std::vector<cv::KeyPoint> mvKeysUn;    // ps：当前帧中，校正 mvKeys 后的关键点，注意是【校正后】！！！对于双目摄像头，一般得到的图像都是校正好的，再校正一次有点多余。

        std::vector<MapPoint *> mvpMapPoints; // 存储了当前帧中每个特征点对应的 MapPoint，即每个地图点在当前帧的观察。如果特征点没有对应的地图点，那么将存储一个【空指针】
        // "Monocular" keypoints have a negative value.
        std::vector<float> mvuRight; // ps：u-指代横坐标，因为最后这个坐标是通过各种拟合方法逼近出来的，所以使用 float 存储。
        std::vector<float> mvDepth;  // ps：（在 RGB-D 模式中）存储与 mvKeysUn 中每个关键点对应的深度信息。

        // Bag of Words Vector structures.
        DBoW2::BowVector mBowVec;
        DBoW2::FeatureVector mFeatVec;

        // ORB descriptor, each row associated to a keypoint.
        cv::Mat mDescriptors, mDescriptorsRight;

        // MapPoints associated to keypoints, NULL pointer if no association.
        // Flag to identify outlier associations.
        std::vector<bool> mvbOutlier; // 存储是外点的地图点
        int mnCloseMPs;

        // Keypoints are assigned to cells in a grid to reduce matching complexity when projecting MapPoints.
        static float mfGridElementWidthInv;
        static float mfGridElementHeightInv;
        std::vector<std::size_t> mGrid[FRAME_GRID_COLS][FRAME_GRID_ROWS];

        /** @} */

        IMU::Bias mPredBias;

        // IMU bias
        IMU::Bias mImuBias;

        // Imu calibration
        IMU::Calib mImuCalib;

        // Imu preintegration from last keyframe
        KeyFrame *mpLastKeyFrame; // 一个指向上一关键帧的指针
        // * 指向上一【关键帧】到当前帧之间的 IMU 数据的预积分结果
        IMU::Preintegrated *mpImuPreintegrated;

        // Pointer to previous frame
        Frame *mpPrevFrame; // 上一帧
        // * 指向上一【普通帧】到当前帧之间的 IMU 数据的预积分结果
        IMU::Preintegrated *mpImuPreintegratedFrame;

        // 类的静态成员变量，这些变量则是在整个系统开始执行的时候被初始化的——它在全局区被初始化
        // Current and Next Frame id.
        static long unsigned int nNextId; ///< Next Frame id.
        long unsigned int mnId;           ///< Current Frame id.

        // 普通帧与自己共视程度最高的关键帧作为参考关键帧
        // Reference Keyframe.
        KeyFrame *mpReferenceKF;

        /**
         * @name 图像金字塔信息
         * @{
         */
        // Scale pyramid info.
        int mnScaleLevels;      ///< 图像金字塔的层数
        float mfScaleFactor;    ///< 图像金字塔的尺度因子
        float mfLogScaleFactor; ///< 图像金字塔的尺度因子的对数值，用于仿照特征点尺度预测地图点的尺度

        vector<float> mvScaleFactors;    ///< 图像金字塔每一层的缩放因子
        vector<float> mvInvScaleFactors; ///< 以及上面的这个变量的倒数
        vector<float> mvLevelSigma2;     ///@todo 目前在frame.c中没有用到，无法下定论
        vector<float> mvInvLevelSigma2;  ///< 上面变量的倒数

        /** @} */

        // Undistorted Image Bounds (computed once).
        /**
         * @name 用于确定画格子时的边界
         * @note（未校正图像的边界，只需要计算一次，因为是类的静态成员变量）
         * @{
         */
        static float mnMinX;
        static float mnMaxX;
        static float mnMinY;
        static float mnMaxY;

        /** @} */

        /**
         * @brief 一个标志，标记是否已经进行了这些初始化计算
         * @note 由于第一帧以及SLAM系统进行重新校正后的第一帧会有一些特殊的初始化处理操作，所以这里设置了这个变量. \n
         * 如果这个标志被置位，说明再下一帧的帧构造函数中要进行这个“特殊的初始化操作”，如果没有被置位则不用。
         */
        static bool mbInitialComputations;

        map<long unsigned int, cv::Point2f> mmProjectPoints;
        map<long unsigned int, cv::Point2f> mmMatchedInImage;

        string mNameFile;

        int mnDataset;

#ifdef REGISTER_TIMES
        double mTimeORB_Ext;
        double mTimeStereoMatch;
#endif

    private:
        // Undistort keypoints given OpenCV distortion parameters.
        // Only for the RGB-D case. Stereo must be already rectified!
        // (called in the constructor).
        void UndistortKeyPoints();

        // Computes image bounds for the undistorted image (called in the constructor).
        void ComputeImageBounds(const cv::Mat &imLeft);

        // Assign keypoints to the grid for speed up feature matching (called in the constructor).
        void AssignFeaturesToGrid();

        bool mbIsSet;

        bool mbImuPreintegrated; // 是否做完预积分的标志

        std::mutex *mpMutexImu;

    public:
        GeometricCamera *mpCamera, *mpCamera2;

        // 左图像和右图像上提取到的特征点数量
        int Nleft, Nright;
        // Number of Non Lapping Keypoints
        int monoLeft, monoRight;

        // For stereo matching
        std::vector<int> mvLeftToRightMatch, mvRightToLeftMatch; // 存储了每个左图像特征点在右图像中的匹配点索引

        // For stereo fisheye matching
        static cv::BFMatcher BFmatcher;

        // Triangulated stereo observations using as reference the left camera. These are computed during ComputeStereoFishEyeMatches
        // 一个存储了所有 左图特征点的 3D 坐标 的容器（通常是一个向量数组），其中每个元素是一个 三维向量 Eigen::Vector3f，代表了某个左图特征点的 3D 坐标。
        std::vector<Eigen::Vector3f> mvStereo3Dpoints;

        // Grid for the right image
        std::vector<std::size_t> mGridRight[FRAME_GRID_COLS][FRAME_GRID_ROWS];

        Frame(const cv::Mat &imLeft, const cv::Mat &imRight, const double &timeStamp, ORBextractor *extractorLeft, ORBextractor *extractorRight, ORBVocabulary *voc, cv::Mat &K, cv::Mat &distCoef, const float &bf, const float &thDepth, GeometricCamera *pCamera, GeometricCamera *pCamera2, Sophus::SE3f &Tlr, Frame *pPrevF = static_cast<Frame *>(NULL), const IMU::Calib &ImuCalib = IMU::Calib());

        // Stereo fisheye
        void ComputeStereoFishEyeMatches();

        bool isInFrustumChecks(MapPoint *pMP, float viewingCosLimit, bool bRight = false);

        Eigen::Vector3f UnprojectStereoFishEye(const int &i);

        cv::Mat imgLeft, imgRight;

        void PrintPointDistribution()
        {
            int left = 0, right = 0;
            int Nlim = (Nleft != -1) ? Nleft : N;
            for (int i = 0; i < N; i++)
            {
                if (mvpMapPoints[i] && !mvbOutlier[i])
                {
                    if (i < Nlim)
                        left++;
                    else
                        right++;
                }
            }
            cout << "Point distribution in Frame: left-> " << left << " --- right-> " << right << endl;
        }

        Sophus::SE3<double> T_test;
    };

} // namespace ORB_SLAM

#endif // FRAME_H
