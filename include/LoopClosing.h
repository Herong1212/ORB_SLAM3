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

#ifndef LOOPCLOSING_H
#define LOOPCLOSING_H

#include "KeyFrame.h"
#include "LocalMapping.h"
#include "Atlas.h"
#include "ORBVocabulary.h"
#include "Tracking.h"

#include "KeyFrameDatabase.h"

#include <boost/algorithm/string.hpp>
#include <thread>
#include <mutex>
#include "Thirdparty/g2o/g2o/types/types_seven_dof_expmap.h"

namespace ORB_SLAM3
{

    class Tracking;
    class LocalMapping;
    class KeyFrameDatabase;
    class Map;

    /// 回环检测线程
    class LoopClosing
    {
    public:
        /// 自定义数据类型, ConsistentGroup.first --- 每个“连续组”中的关键帧，ConsistentGroup.second --- 每个“连续组”的序号
        typedef pair<set<KeyFrame *>, int> ConsistentGroup;

        /// 存储关键帧对象和位姿的键值对，这里是 map 的完整构造函数
        typedef map<KeyFrame *,                                                     // 键
                    g2o::Sim3,                                                      // 值
                    std::less<KeyFrame *>,                                          // 排序算法
                    Eigen::aligned_allocator<std::pair<KeyFrame *const, g2o::Sim3>> // 指定分配器，和内存空间开辟有关。为了能够使用 Eigen 库中的 SSE 和 AVX 指令集加速，需要将传统 STL 容器中的数据进行对齐处理
                    >
            KeyFrameAndPose;

    public:
        /**
         * @brief 构造函数
         * @param[in] pAtlas        地图指针
         * @param[in] pDB           词袋数据库
         * @param[in] pVoc          词典
         * @param[in] bFixScale     表示 sim3 中的尺度是否要计算，对于双目和 RGB-D 情况尺度是固定的，s=1，bFixScale=true；而单目下尺度是不确定的，此时 bFixScale=false，sim3中的s需要被计算；
         * @param[in] bActiveLC     表示啥玩意儿？？
         */
        LoopClosing(Atlas *pAtlas, KeyFrameDatabase *pDB, ORBVocabulary *pVoc, const bool bFixScale, const bool bActiveLC);

        void SetTracker(Tracking *pTracker);

        void SetLocalMapper(LocalMapping *pLocalMapper);

        // note：Main function！
        void Run();

        /**
         * @brief 将某个关键帧加入到回环检测的过程中，由局部建图线程调用
         *
         * @param[in] pKF
         */
        void InsertKeyFrame(KeyFrame *pKF);

        /**
         * @brief 由外部线程调用，请求复位当前线程。在回环检测复位完成之前，该函数将一直保持堵塞状态
         *
         */
        void RequestReset();
        void RequestResetActiveMap(Map *pMap);

        // notice：This function will run in a separate thread
        /**
         * @brief 全局 BA 线程，这个函数是这个线程的主函数
         * @param[in] nLoopKF 看名字是闭环关键帧，但是实际上给的是【当前关键帧】的 ID
         */
        void RunGlobalBundleAdjustment(Map *pActiveMap, unsigned long nLoopKF);

        // 在回环纠正的时候调用，查看当前是否已经有一个全局优化的线程在进行
        bool isRunningGBA()
        {
            unique_lock<std::mutex> lock(mMutexGBA);
            return mbRunningGBA;
        }
        bool isFinishedGBA()
        {
            unique_lock<std::mutex> lock(mMutexGBA);
            return mbFinishedGBA;
        }

        // 由外部线程调用，请求终止当前线程
        void RequestFinish();

        // 由外部线程调用，判断当前回环检测线程是否已经正确终止了
        bool isFinished();

        Viewer *mpViewer;

#ifdef REGISTER_TIMES

        vector<double> vdDataQuery_ms;
        vector<double> vdEstSim3_ms;
        vector<double> vdPRTotal_ms; // 存储多个区域处理操作的执行时间

        vector<double> vdMergeMaps_ms;
        vector<double> vdWeldingBA_ms;
        vector<double> vdMergeOptEss_ms;
        vector<double> vdMergeTotal_ms;
        vector<int> vnMergeKFs;
        vector<int> vnMergeMPs;
        int nMerges;

        vector<double> vdLoopFusion_ms;
        vector<double> vdLoopOptEss_ms;
        vector<double> vdLoopTotal_ms;
        vector<int> vnLoopKFs;
        int nLoop;

        vector<double> vdGBA_ms;
        vector<double> vdUpdateMap_ms;
        vector<double> vdFGBATotal_ms;
        vector<int> vnGBAKFs;
        vector<int> vnGBAMPs;
        int nFGBA_exec;
        int nFGBA_abort;

#endif

        EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    protected:
        // 查看列表中是否有等待被插入的关键帧
        bool CheckNewKeyFrames();

        // Methods to implement the new place recognition algorithm
        bool NewDetectCommonRegions();
        bool DetectAndReffineSim3FromLastKF(KeyFrame *pCurrentKF, KeyFrame *pMatchedKF, g2o::Sim3 &gScw, int &nNumProjMatches,
                                            std::vector<MapPoint *> &vpMPs, std::vector<MapPoint *> &vpMatchedMPs);
        bool DetectCommonRegionsFromBoW(std::vector<KeyFrame *> &vpBowCand, KeyFrame *&pMatchedKF, KeyFrame *&pLastCurrentKF, g2o::Sim3 &g2oScw,
                                        int &nNumCoincidences, std::vector<MapPoint *> &vpMPs, std::vector<MapPoint *> &vpMatchedMPs);
        bool DetectCommonRegionsFromLastKF(KeyFrame *pCurrentKF, KeyFrame *pMatchedKF, g2o::Sim3 &gScw, int &nNumProjMatches,
                                           std::vector<MapPoint *> &vpMPs, std::vector<MapPoint *> &vpMatchedMPs);
        int FindMatchesByProjection(KeyFrame *pCurrentKF, KeyFrame *pMatchedKFw, g2o::Sim3 &g2oScw,
                                    set<MapPoint *> &spMatchedMPinOrigin, vector<MapPoint *> &vpMapPoints,
                                    vector<MapPoint *> &vpMatchedMapPoints);

        void SearchAndFuse(const KeyFrameAndPose &CorrectedPosesMap, vector<MapPoint *> &vpMapPoints);
        void SearchAndFuse(const vector<KeyFrame *> &vConectedKFs, vector<MapPoint *> &vpMapPoints);

        /**
         * @brief 闭环纠正
         * @detials \n
         * 1. 通过求解的Sim3以及相对姿态关系，调整与当前帧相连的关键帧位姿以及这些关键帧观测到的MapPoints的位置（相连关键帧---当前帧） \n
         * 2. 将闭环帧以及闭环帧相连的关键帧的MapPoints和与当前帧相连的关键帧的点进行匹配（相连关键帧+当前帧---闭环帧+相连关键帧）     \n
         * 3. 通过MapPoints的匹配关系更新这些帧之间的连接关系，即更新covisibility graph                                      \n
         * 4. 对Essential Graph（Pose Graph）进行优化，MapPoints的位置则根据优化后的位姿做相对应的调整                         \n
         * 5. 创建线程进行全局Bundle Adjustment
         */
        void CorrectLoop();

        void MergeLocal();
        void MergeLocal2();

        void CheckObservations(set<KeyFrame *> &spKFsMap1, set<KeyFrame *> &spKFsMap2);

        // 当前线程调用，检查是否有外部线程请求复位当前线程，如果有的话就复位回环检测线程
        void ResetIfRequested();

        /// 是否有复位当前线程的请求
        bool mbResetRequested;

        bool mbResetActiveMapRequested;
        Map *mpMapToReset;

        /// 和复位当前线程相关的互斥量
        std::mutex mMutexReset;

        /// 当前线程调用，查看是否有外部线程请求当前线程
        bool CheckFinish();

        /// 有当前线程调用，执行完成该函数之后线程主函数退出，线程销毁
        void SetFinish();

        /// 是否有终止当前线程的请求
        bool mbFinishRequested;
        /// 当前线程是否已经停止工作
        bool mbFinished;
        /// 和当前线程终止状态操作有关的互斥量
        std::mutex mMutexFinish;

        /// (全局)地图的指针
        Atlas *mpAtlas;
        /// 追踪线程句柄
        Tracking *mpTracker;

        /// 关键帧数据库
        KeyFrameDatabase *mpKeyFrameDB;

        /// 词袋模型中的大字典
        ORBVocabulary *mpORBVocabulary;

        /// 局部建图线程句柄
        LocalMapping *mpLocalMapper;

        /// 一个队列，其中存储了参与到回环检测的关键帧 (当然这些关键帧也有可能因为各种原因被设置成为 bad，这样虽然这个关键帧还是存储在这里，但是实际上已经不再实质性地参与到回环检测的过程中去了)
        std::list<KeyFrame *> mlpLoopKeyFrameQueue;

        /// 操作参与到回环检测队列中的关键帧时，使用的互斥量
        std::mutex mMutexLoopQueue;

        /// 连续性阈值，构造函数中将其设置成为了3
        float mnCovisibilityConsistencyTh;

        // --------------------- Loop detector variables -------------------------
        // todo 当前关键帧，其实称之为【当前正在处理的关键帧】更加合适
        KeyFrame *mpCurrentKF;

        KeyFrame *mpLastCurrentKF;

        // ps：最终检测出来的，和当前关键帧形成闭环的闭环关键帧
        KeyFrame *mpMatchedKF;

        /// 上一次执行的时候产生的连续组 s
        std::vector<ConsistentGroup> mvConsistentGroups;
        /// 从上面的关键帧中进行筛选之后得到的具有足够的"连续性"的关键帧 -- 这个其实也是相当于更高层级的、更加优质的闭环候选帧
        std::vector<KeyFrame *> mvpEnoughConsistentCandidates;
        // todo [当前关键帧组]，和当前关键帧相连的关键帧形成的
        std::vector<KeyFrame *> mvpCurrentConnectedKFs;
        /// 下面的变量中存储的地图点在"当前关键帧"中成功地找到了匹配点的地图点的集合
        std::vector<MapPoint *> mvpCurrentMatchedPoints;
        /// 闭环关键帧上的所有相连关键帧的地图点
        std::vector<MapPoint *> mvpLoopMapPoints;
        // 下面的变量的 cv::Mat 格式版本
        cv::Mat mScw;
        // 当得到了当前关键帧的闭环关键帧以后，计算出来的从世界坐标系到当前帧的sim3变换
        g2o::Sim3 mg2oScw;

        //-------
        Map *mpLastMap;

        bool mbLoopDetected;
        int mnLoopNumCoincidences;
        int mnLoopNumNotFound;
        KeyFrame *mpLoopLastCurrentKF;
        g2o::Sim3 mg2oLoopSlw;
        g2o::Sim3 mg2oLoopScw;
        KeyFrame *mpLoopMatchedKF;
        std::vector<MapPoint *> mvpLoopMPs;
        std::vector<MapPoint *> mvpLoopMatchedMPs;
        bool mbMergeDetected;
        int mnMergeNumCoincidences;
        int mnMergeNumNotFound;
        KeyFrame *mpMergeLastCurrentKF;
        g2o::Sim3 mg2oMergeSlw;
        g2o::Sim3 mg2oMergeSmw;
        g2o::Sim3 mg2oMergeScw;
        KeyFrame *mpMergeMatchedKF;
        std::vector<MapPoint *> mvpMergeMPs;
        std::vector<MapPoint *> mvpMergeMatchedMPs;
        std::vector<KeyFrame *> mvpMergeConnectedKFs;

        g2o::Sim3 mSold_new;
        //-------

        /// 上一次闭环帧的 id
        long unsigned int mLastLoopKFid;

        // Variables related to Global Bundle Adjustment
        /// 全局BA线程是否在进行
        bool mbRunningGBA;
        /// 全局BA 线程在收到停止请求之后是否停止的标志 // ? 可是直接使用上面变量的逆不就可以表示了吗? // ? 表示全局BA工作是否正常结束?
        bool mbFinishedGBA;
        /// 由当前线程调用,请求停止当前正在进行的全局BA
        bool mbStopGBA;
        /// 在对和全局线程标志量有关的操作的时候使用的互斥量
        std::mutex mMutexGBA;
        /// 全局BA线程句柄
        std::thread *mpThreadGBA;

        // Fix scale in the stereo/RGB-D case
        // 如果是在双目或者是RGBD输入的情况下，就要固定尺度，这个变量就是是否要固定尺度的标志
        // ps：用于标记是否保持闭环检测的尺度不变，通常在单目系统中设置为false，而在双目或RGB-D系统中设置为true。
        bool mbFixScale;

        /// 已经进行了的 全局BA 次数(包含中途被打断的)
        // bool mnFullBAIdx;
        int mnFullBAIdx;

        vector<double> vdPR_CurrentTime;
        vector<double> vdPR_MatchedTime;
        vector<int> vnPR_TypeRecogn;

        // DEBUG
        string mstrFolderSubTraj;
        int mnNumCorrection;
        int mnCorrectionGBA;

        // To (de)activate LC
        bool mbActiveLC = true;

#ifdef REGISTER_LOOP
        string mstrFolderLoop;
#endif
    };

} // namespace ORB_SLAM

#endif // LOOPCLOSING_H
