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

#ifndef SIM3SOLVER_H
#define SIM3SOLVER_H

#include <opencv2/opencv.hpp>
#include <vector>

#include "KeyFrame.h"

namespace ORB_SLAM3
{
    /** @brief Sim3 求解器 */
    class Sim3Solver
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW

        /**
         * @brief Sim 3 Solver 构造函数
         * @param[in] pKF1              当前关键帧
         * @param[in] pKF2              候选的闭环关键帧
         * @param[in] vpMatched12       通过词袋模型加速匹配所得到的,两帧特征点的匹配关系所得到的地图点,本质上是来自于候选闭环关键帧的地图点
         * @param[in] bFixScale         当前传感器类型的输入需不需要计算尺度。单目的时候需要，双目和RGBD的时候就不需要了
         * @param[in] vpKeyFrameMatchedMP   我也不知道这是啥
         */
        Sim3Solver(KeyFrame *pKF1, KeyFrame *pKF2, const std::vector<MapPoint *> &vpMatched12, const bool bFixScale = true,
                   const vector<KeyFrame *> vpKeyFrameMatchedMP = vector<KeyFrame *>());

        void SetRansacParameters(double probability = 0.99, int minInliers = 6, int maxIterations = 300);

        Eigen::Matrix4f find(std::vector<bool> &vbInliers12, int &nInliers);

        Eigen::Matrix4f iterate(int nIterations, bool &bNoMore, std::vector<bool> &vbInliers, int &nInliers);
        Eigen::Matrix4f iterate(int nIterations, bool &bNoMore, vector<bool> &vbInliers, int &nInliers, bool &bConverge);

        Eigen::Matrix4f GetEstimatedTransformation();
        Eigen::Matrix3f GetEstimatedRotation();
        Eigen::Vector3f GetEstimatedTranslation();
        float GetEstimatedScale();

    protected:
        void ComputeCentroid(Eigen::Matrix3f &P, Eigen::Matrix3f &Pr, Eigen::Vector3f &C);

        /**
         * @brief 根据两组匹配的 3D 点，计算 P2 到 P1 的 Sim3 变换
         * @param[in] P1    匹配的 3D 点(三个，每个的坐标都是列向量形式，三个点组成了 3x3 的矩阵)(也即当前关键帧)
         * @param[in] P2    匹配的 3D 点(即闭环关键帧)
         */
        void ComputeSim3(Eigen::Matrix3f &P1, Eigen::Matrix3f &P2);

        // 通过计算的 Sim3 投影，和自身投影的误差比较，进行内点检测
        void CheckInliers();

        /**
         * @brief 按照给定的 Sim3 变换进行投影操作，得到三维点的2D投影点
         *
         * @param[in] vP3Dw         3D点
         * @param[in & out] vP2D    投影到图像的2D点
         * @param[in] Tcw           Sim3变换
         * @param[in] pCamera       这是内参🐎？
         */
        void Project(const std::vector<Eigen::Vector3f> &vP3Dw, std::vector<Eigen::Vector2f> &vP2D, Eigen::Matrix4f Tcw, GeometricCamera *pCamera);

        /**
         * @brief 计算当前关键帧中的地图点在当前关键帧图像上的投影坐标
         *
         * @param[in] vP3Dc         相机坐标系下三维点坐标
         * @param[in] vP2D          投影的二维图像坐标
         * @param[in] pCamera       这是内参矩阵🐎？
         */
        void FromCameraToImage(const std::vector<Eigen::Vector3f> &vP3Dc, std::vector<Eigen::Vector2f> &vP2D, GeometricCamera *pCamera);

    protected:
        // KeyFrames and matches
        KeyFrame *mpKF1; // 当前关键帧
        KeyFrame *mpKF2; // 闭环关键帧

        std::vector<Eigen::Vector3f> mvX3Dc1;  // 存储匹配的,当前关键帧中的地图点在当前关键帧相机坐标系下的坐标
        std::vector<Eigen::Vector3f> mvX3Dc2;  // 存储匹配的,闭环关键帧中的地图点在闭环关键帧相机坐标系下的坐标
        std::vector<MapPoint *> mvpMapPoints1; // 匹配的地图点的中,存储当前关键帧的地图点
        std::vector<MapPoint *> mvpMapPoints2; // 匹配的地图点的中,存储闭环关键帧的地图点
        std::vector<MapPoint *> mvpMatches12;  // 下标是当前关键帧中特征点的id,内容是对应匹配的,闭环关键帧中的地图点
        std::vector<size_t> mvnIndices1;       // 有效的匹配关系,在 vpMatched12 (构造函数) 中的索引
        std::vector<size_t> mvSigmaSquare1;    // 这个变量好像是没有被用到
        std::vector<size_t> mvSigmaSquare2;    // 这个变量好像是没有被用到
        std::vector<size_t> mvnMaxError1;      // 当前关键帧中的某个特征点所允许的最大不确定度(和所在的金字塔图层有关)
        std::vector<size_t> mvnMaxError2;      // 闭环关键帧中的某个特征点所允许的最大不确定度(同上)

        int N;   // 下面的这个匹配关系去掉坏点和非法值之后,得到的可靠的匹配关系的点的数目
        int mN1; // 当前关键帧和闭环关键帧之间形成匹配关系的点的数目(Bow加速得到的匹配点)

        // Current Estimation
        Eigen::Matrix3f mR12i;         // 存储某次RANSAC过程中得到的旋转
        Eigen::Vector3f mt12i;         // 存储某次RANSAC过程中得到的平移
        float ms12i;                   // 存储某次RANSAC过程中得到的缩放系数
        Eigen::Matrix4f mT12i;         // 存储某次RANSAC过程中得到的变换矩阵
        Eigen::Matrix4f mT21i;         // 上面的逆
        std::vector<bool> mvbInliersi; // 内点标记,下标和N,mvpMapPoints1等一致,用于记录某次迭代过程中的内点情况
        int mnInliersi;                // 在某次迭代的过程中经过投影误差进行的inlier检测得到的内点数目

        // Current Ransac State
        int mnIterations;                 // RANSAC迭代次数(当前正在进行的)
        std::vector<bool> mvbBestInliers; // 累计的,多次RANSAC中最好的最多的内点个数时的内点标记
        int mnBestInliers;                // 最好的一次迭代中,得到的内点个数
        Eigen::Matrix4f mBestT12;         // 存储最好的一次迭代中得到的变换矩阵
        Eigen::Matrix3f mBestRotation;    // 存储最好的一次迭代中得到的旋转
        Eigen::Vector3f mBestTranslation; // 存储最好的一次迭代中得到的平移
        float mBestScale;                 // 存储最好的一次迭代中得到的缩放系数

        // 当前传感器输入的情况下，是否需要计算尺度。
        // 双目/RGB-D 固定是 s = 1
        bool mbFixScale;

        // Indices for random selection
        std::vector<size_t> mvAllIndices; // RANSAC中随机选择的时候，存储可以选择的点的id(去除那些存在问题的匹配点后重新排序)

        // Projections
        std::vector<Eigen::Vector2f> mvP1im1; // 当前关键帧中的地图点在当前关键帧图像上的投影坐标
        std::vector<Eigen::Vector2f> mvP2im2; // 闭环关键帧中的地图点在闭环关键帧图像上的投影坐标

        // RANSAC probability
        double mRansacProb; // 在计算RANSAC的理论迭代次数时使用到的概率,详细解释还是看函数 SetRansacParameters() 中的注释吧

        // RANSAC min inliers
        int mRansacMinInliers; // RANSAC 结束的理想条件: 结束RANSAC过程所需要的最少内点数

        // RANSAC max iterations
        int mRansacMaxIts; // RANSAC 结束的不理想条件: 最大迭代次数

        // Threshold inlier/outlier. e = dist(Pi,T_ij*Pj)^2 < 5.991*mSigma2
        float mTh;     // 没有使用到的变量
        float mSigma2; // 没有使用到的变量

        // Calibration
        // cv::Mat mK1; // 当前关键帧的内参矩阵
        // cv::Mat mK2; // 闭环关键帧的内参矩阵

        GeometricCamera *pCamera1, *pCamera2;
    };

} // namespace ORB_SLAM

#endif // SIM3SOLVER_H
