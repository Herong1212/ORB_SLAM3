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

#include "MapPoint.h"
#include "ORBmatcher.h"

#include <mutex>

namespace ORB_SLAM3
{

    long unsigned int MapPoint::nNextId = 0;
    mutex MapPoint::mGlobalMutex;

    // ps：构造函数1
    MapPoint::MapPoint() : mnFirstKFid(0), mnFirstFrame(0), nObs(0), mnTrackReferenceForFrame(0),
                           mnLastFrameSeen(0), mnBALocalForKF(0), mnFuseCandidateForKF(0), mnLoopPointForKF(0), mnCorrectedByKF(0),
                           mnCorrectedReference(0), mnBAGlobalForKF(0), mnVisible(1), mnFound(1), mbBad(false),
                           mpReplaced(static_cast<MapPoint *>(NULL))
    {
        mpReplaced = static_cast<MapPoint *>(NULL);
    }

    // ps：构造函数2
    MapPoint::MapPoint(const Eigen::Vector3f &Pos,                             // 地图点的世界坐标系下的坐标
                       KeyFrame *pRefKF,                                       // 生成该地图点的关键帧
                       Map *pMap) :                                            // 地图点所存在的地图
                                    mnFirstKFid(pRefKF->mnId),                 // 第一次观测/生成它的关键帧 id
                                    mnFirstFrame(pRefKF->mnFrameId),           // 创建该地图点的帧ID(因为关键帧也是帧啊)
                                    nObs(0),                                   // 被观测次数
                                    mnTrackReferenceForFrame(0),               // 放置被重复添加到局部地图点的标记
                                    mnLastFrameSeen(0),                        // 是否决定判断在某个帧视野中的变量
                                    mnBALocalForKF(0),                         //
                                    mnFuseCandidateForKF(0),                   //
                                    mnLoopPointForKF(0),                       //
                                    mnCorrectedByKF(0),                        //
                                    mnCorrectedReference(0),                   //
                                    mnBAGlobalForKF(0),                        //
                                    mpRefKF(pRefKF),                           //
                                    mnVisible(1),                              // 在帧中的可视次数
                                    mnFound(1),                                // 被找到的次数 和上面的相比要求能够匹配上
                                    mbBad(false),                              // 坏点标记
                                    mpReplaced(static_cast<MapPoint *>(NULL)), // 替换掉当前地图点的点
                                    mfMinDistance(0),                          // 当前地图点在某帧下,可信赖的被找到时其到关键帧光心距离的下界
                                    mfMaxDistance(0),                          // 上界
                                    mpMap(pMap),                               // 从属地图
                                    mnOriginMapId(pMap->GetId())               //
    {
        SetWorldPos(Pos);

        // 平均观测方向初始化为 0
        mNormalVector.setZero();

        mbTrackInViewR = false;
        mbTrackInView = false;

        // MapPoints can be created from Tracking and Local Mapping. This mutex avoid conflicts with id.
        unique_lock<mutex> lock(mpMap->mMutexPointCreation);
        mnId = nNextId++;
    }

    // ps：构造函数3
    MapPoint::MapPoint(const double invDepth, cv::Point2f uv_init, KeyFrame *pRefKF, KeyFrame *pHostKF, Map *pMap) : mnFirstKFid(pRefKF->mnId), mnFirstFrame(pRefKF->mnFrameId), nObs(0), mnTrackReferenceForFrame(0),
                                                                                                                     mnLastFrameSeen(0), mnBALocalForKF(0), mnFuseCandidateForKF(0), mnLoopPointForKF(0), mnCorrectedByKF(0),
                                                                                                                     mnCorrectedReference(0), mnBAGlobalForKF(0), mpRefKF(pRefKF), mnVisible(1), mnFound(1), mbBad(false),
                                                                                                                     mpReplaced(static_cast<MapPoint *>(NULL)), mfMinDistance(0), mfMaxDistance(0), mpMap(pMap),
                                                                                                                     mnOriginMapId(pMap->GetId())
    {
        mInvDepth = invDepth;
        mInitU = (double)uv_init.x;
        mInitV = (double)uv_init.y;
        mpHostKF = pHostKF;

        mNormalVector.setZero();

        // Worldpos is not set
        // MapPoints can be created from Tracking and Local Mapping. This mutex avoid conflicts with id.
        unique_lock<mutex> lock(mpMap->mMutexPointCreation);
        mnId = nNextId++;
    }

    // ps：构造函数4
    /*
     * @brief 给定坐标与 frame 构造 MapPoint
     *
     * 双目：UpdateLastFrame()
     * @param Pos    MapPoint的坐标（世界坐标系）
     * @param pMap   Map
     * @param pFrame Frame
     * @param idxF   MapPoint在Frame中的索引，即对应的特征点的编号
     */
    MapPoint::MapPoint(const Eigen::Vector3f &Pos, Map *pMap, Frame *pFrame, const int &idxF) : mnFirstKFid(-1), mnFirstFrame(pFrame->mnId), nObs(0), mnTrackReferenceForFrame(0), mnLastFrameSeen(0),
                                                                                                mnBALocalForKF(0), mnFuseCandidateForKF(0), mnLoopPointForKF(0), mnCorrectedByKF(0),
                                                                                                mnCorrectedReference(0), mnBAGlobalForKF(0), mpRefKF(static_cast<KeyFrame *>(NULL)), mnVisible(1),
                                                                                                mnFound(1), mbBad(false), mpReplaced(NULL), mpMap(pMap), mnOriginMapId(pMap->GetId())
    {
        SetWorldPos(Pos);

        Eigen::Vector3f Ow;
        if (pFrame->Nleft == -1 || idxF < pFrame->Nleft)
        {
            Ow = pFrame->GetCameraCenter();
        }
        else
        {
            Eigen::Matrix3f Rwl = pFrame->GetRwc();
            Eigen::Vector3f tlr = pFrame->GetRelativePoseTlr().translation();
            Eigen::Vector3f twl = pFrame->GetOw();

            Ow = Rwl * tlr + twl;
        }
        mNormalVector = mWorldPos - Ow;                       // 世界坐标系下相机到 3D 点的向量 (当前关键帧的观测方向)
        mNormalVector = mNormalVector / mNormalVector.norm(); // 单位化

        // 这个算重了吧
        Eigen::Vector3f PC = mWorldPos - Ow;
        const float dist = PC.norm(); // 到相机的距离
        const int level = (pFrame->Nleft == -1)    ? pFrame->mvKeysUn[idxF].octave
                          : (idxF < pFrame->Nleft) ? pFrame->mvKeys[idxF].octave
                                                   : pFrame->mvKeysRight[idxF].octave;
        const float levelScaleFactor = pFrame->mvScaleFactors[level];
        const int nLevels = pFrame->mnScaleLevels;

        // 另见 PredictScale 函数前的注释
        /* 666，因为在提取特征点的时候, 考虑到了图像的尺度问题，因此在不同图层上提取得到的特征点，对应着特征点距离相机的远近
           不同, 所以在这里生成地图点的时候，也要再对其进行确认。
           虽然我们拿不到每个图层之间确定的尺度信息，但是我们有缩放比例这个相对的信息哇。
        */
        mfMaxDistance = dist * levelScaleFactor;                             // 当前图层的"深度"
        mfMinDistance = mfMaxDistance / pFrame->mvScaleFactors[nLevels - 1]; // 该特征点上一个图层的"深度"

        // 见 mDescriptor 在 MapPoint.h 中的注释 ==> 其实就是获取这个地图点的描述子
        pFrame->mDescriptors.row(idxF).copyTo(mDescriptor);

        // MapPoints can be created from Tracking and Local Mapping. This mutex avoid conflicts with id.
        // ? 不太懂，为什么会冲突?
        unique_lock<mutex> lock(mpMap->mMutexPointCreation);
        mnId = nNextId++;
    }

    // 设置地图点在世界坐标系下的坐标
    void MapPoint::SetWorldPos(const Eigen::Vector3f &Pos)
    {
        unique_lock<mutex> lock2(mGlobalMutex);
        unique_lock<mutex> lock(mMutexPos);
        mWorldPos = Pos;
    }

    Eigen::Vector3f MapPoint::GetWorldPos()
    {
        unique_lock<mutex> lock(mMutexPos);
        return mWorldPos;
    }

    Eigen::Vector3f MapPoint::GetNormal()
    {
        unique_lock<mutex> lock(mMutexPos);
        return mNormalVector;
    }

    KeyFrame *MapPoint::GetReferenceKeyFrame()
    {
        unique_lock<mutex> lock(mMutexFeatures);
        return mpRefKF;
    }

    /**
     * @brief 给地图点添加观测
     *
     * 记录哪些 KeyFrame 的那个特征点能观测到该 地图点
     * 并增加观测的相机数目 nObs，单目+1，双目或者rgbd+2
     * 这个函数是建立关键帧共视关系的核心函数，能共同观测到某些地图点的关键帧是共视关键帧
     * @param pKF KeyFrame
     * @param idx MapPoint 在 KeyFrame 中的索引
     */
    void MapPoint::AddObservation(KeyFrame *pKF, int idx)
    {
        unique_lock<mutex> lock(mMutexFeatures);
        tuple<int, int> indexes;

        if (mObservations.count(pKF))
        {
            indexes = mObservations[pKF];
        }
        else
        {
            indexes = tuple<int, int>(-1, -1);
        }

        if (pKF->NLeft != -1 && idx >= pKF->NLeft)
        {
            get<1>(indexes) = idx;
        }
        else
        {
            get<0>(indexes) = idx;
        }

        // 如果没有添加过观测，记录下能观测到该MapPoint的KF和该MapPoint在KF中的索引
        mObservations[pKF] = indexes;

        if (!pKF->mpCamera2 && pKF->mvuRight[idx] >= 0)
            nObs += 2;
        else
            nObs++;
    }

    void MapPoint::EraseObservation(KeyFrame *pKF)
    {
        bool bBad = false;
        {
            unique_lock<mutex> lock(mMutexFeatures);
            if (mObservations.count(pKF))
            {
                tuple<int, int> indexes = mObservations[pKF];
                int leftIndex = get<0>(indexes), rightIndex = get<1>(indexes);

                if (leftIndex != -1)
                {
                    if (!pKF->mpCamera2 && pKF->mvuRight[leftIndex] >= 0)
                        nObs -= 2;
                    else
                        nObs--;
                }
                if (rightIndex != -1)
                {
                    nObs--;
                }

                mObservations.erase(pKF);

                if (mpRefKF == pKF)
                    mpRefKF = mObservations.begin()->first;

                // If only 2 observations or less, discard point
                if (nObs <= 2)
                    bBad = true;
            }
        }

        if (bBad)
            SetBadFlag();
    }

    std::map<KeyFrame *, std::tuple<int, int>> MapPoint::GetObservations()
    {
        unique_lock<mutex> lock(mMutexFeatures);
        return mObservations;
    }

    // ps：被观测到的相机数目，单目+1，双目或RGB-D则+2
    /**
     * @brief 返回被观测次数，双目一帧算两次，左右目各算各的
     * @return nObs
     */
    int MapPoint::Observations()
    {
        unique_lock<mutex> lock(mMutexFeatures);
        return nObs;
    }

    void MapPoint::SetBadFlag()
    {
        map<KeyFrame *, tuple<int, int>> obs;
        {
            unique_lock<mutex> lock1(mMutexFeatures);
            unique_lock<mutex> lock2(mMutexPos);
            mbBad = true;
            obs = mObservations;
            mObservations.clear();
        }
        for (map<KeyFrame *, tuple<int, int>>::iterator mit = obs.begin(), mend = obs.end(); mit != mend; mit++)
        {
            KeyFrame *pKF = mit->first;
            int leftIndex = get<0>(mit->second), rightIndex = get<1>(mit->second);
            if (leftIndex != -1)
            {
                pKF->EraseMapPointMatch(leftIndex);
            }
            if (rightIndex != -1)
            {
                pKF->EraseMapPointMatch(rightIndex);
            }
        }

        mpMap->EraseMapPoint(this);
    }

    MapPoint *MapPoint::GetReplaced()
    {
        unique_lock<mutex> lock1(mMutexFeatures);
        unique_lock<mutex> lock2(mMutexPos);

        return mpReplaced;
    }

    void MapPoint::Replace(MapPoint *pMP)
    {
        if (pMP->mnId == this->mnId)
            return;

        int nvisible, nfound;
        map<KeyFrame *, tuple<int, int>> obs;
        {
            unique_lock<mutex> lock1(mMutexFeatures);
            unique_lock<mutex> lock2(mMutexPos);
            obs = mObservations;
            mObservations.clear();
            mbBad = true;
            nvisible = mnVisible;
            nfound = mnFound;
            mpReplaced = pMP;
        }

        for (map<KeyFrame *, tuple<int, int>>::iterator mit = obs.begin(), mend = obs.end(); mit != mend; mit++)
        {
            // Replace measurement in keyframe
            KeyFrame *pKF = mit->first;

            tuple<int, int> indexes = mit->second;
            int leftIndex = get<0>(indexes), rightIndex = get<1>(indexes);

            if (!pMP->IsInKeyFrame(pKF))
            {
                if (leftIndex != -1)
                {
                    pKF->ReplaceMapPointMatch(leftIndex, pMP);
                    pMP->AddObservation(pKF, leftIndex);
                }
                if (rightIndex != -1)
                {
                    pKF->ReplaceMapPointMatch(rightIndex, pMP);
                    pMP->AddObservation(pKF, rightIndex);
                }
            }
            else
            {
                if (leftIndex != -1)
                {
                    pKF->EraseMapPointMatch(leftIndex);
                }
                if (rightIndex != -1)
                {
                    pKF->EraseMapPointMatch(rightIndex);
                }
            }
        }
        pMP->IncreaseFound(nfound);
        pMP->IncreaseVisible(nvisible);
        pMP->ComputeDistinctiveDescriptors();

        mpMap->EraseMapPoint(this);
    }

    bool MapPoint::isBad()
    {
        unique_lock<mutex> lock1(mMutexFeatures, std::defer_lock);
        unique_lock<mutex> lock2(mMutexPos, std::defer_lock);
        lock(lock1, lock2);

        return mbBad;
    }

    /**
     * @brief Increase Visible
     *
     * Visible表示：
     * 1. 该MapPoint在某些帧的视野范围内，通过Frame::isInFrustum()函数判断
     * 2. 该MapPoint被这些帧观测到，但并不一定能和这些帧的特征点匹配上
     *    例如：有一个MapPoint（记为M），在某一帧F的视野范围内，
     *    但并不表明该点M可以和F这一帧的某个特征点能匹配上
     */
    void MapPoint::IncreaseVisible(int n)
    {
        unique_lock<mutex> lock(mMutexFeatures);
        mnVisible += n;
    }

    /**
     * @brief Increase Found
     *
     * 能找到该点的帧数+n，n默认为1
     * @see Tracking::TrackLocalMap()
     */
    void MapPoint::IncreaseFound(int n)
    {
        unique_lock<mutex> lock(mMutexFeatures);
        mnFound += n;
    }

    float MapPoint::GetFoundRatio()
    {
        unique_lock<mutex> lock(mMutexFeatures);
        return static_cast<float>(mnFound) / mnVisible;
    }

    // TODO 计算地图点具有代表性的描述子
    // 由于一个 MapPoint 会被许多相机观测到，而每张照片可能对这个地图点生成不一样的描述子，因为观测的角度不同啥的。
    // 因此在插入关键帧后，需要判断是否更新当前点的最适合的描述子，即从这些描述子中选出来最能代表这个地图点特征的描述子。
    // 先获得当前点的所有描述子，然后计算描述子之间的两两距离，最好的描述子与其他描述子应该具有【最小的距离中值】
    void MapPoint::ComputeDistinctiveDescriptors()
    {
        // Retrieve all observed descriptors
        vector<cv::Mat> vDescriptors;

        // Step 1 获取所有观测，跳过坏点
        map<KeyFrame *, tuple<int, int>> observations;
        {
            unique_lock<mutex> lock1(mMutexFeatures);

            if (mbBad)
                return;

            // 将地图点的所有观测记录（mObservations）复制到局部变量 observations 中。
            // ps：由于地图点的观测记录 mObservations 是多线程共享资源，为了保证数据一致性，访问时需要加锁。用局部变量复制出来后，不需要一直保持锁定，可以更高效地处理这些数据。
            observations = mObservations;
        }

        if (observations.empty())
            return;

        vDescriptors.reserve(observations.size());

        // Step 2 遍历观测到 3d 点的所有关键帧，获得 orb 描述子，并插入到 vDescriptors中
        for (map<KeyFrame *, tuple<int, int>>::iterator mit = observations.begin(), mend = observations.end(); mit != mend; mit++)
        {

            // mit->first 取观测到该地图点的关键帧
            // mit->second 取该地图点在关键帧中的索引
            KeyFrame *pKF = mit->first;

            if (!pKF->isBad())
            {
                tuple<int, int> indexes = mit->second;

                // leftIndex 和 rightIndex 分别表示该地图点在图像左视图和右视图中的对应的特征点索引
                int leftIndex = get<0>(indexes), rightIndex = get<1>(indexes); // * 访问 tuple 元素的方式

                if (leftIndex != -1)
                {
                    // pKF->mDescriptors.row(leftIndex)：关键帧中左视图特征点的描述子
                    vDescriptors.push_back(pKF->mDescriptors.row(leftIndex));
                }
                if (rightIndex != -1)
                {
                    vDescriptors.push_back(pKF->mDescriptors.row(rightIndex));
                }
            }
        }

        if (vDescriptors.empty())
            return;

        // Step 3 获得这些描述子两两之间的距离
        const size_t N = vDescriptors.size();

        // 将 Distances 表达成一个对称的矩阵
        float Distances[N][N];
        // ! 书上是下面的两行代码，但是有错误！
        // std::vector<std::vector<float>> Distances;
        // Distances.reserve(N, std::vector<float>(N, 0));
        for (size_t i = 0; i < N; i++)
        {
            // 和自己的距离当然是 0
            Distances[i][i] = 0;
            for (size_t j = i + 1; j < N; j++)
            {
                int distij = ORBmatcher::DescriptorDistance(vDescriptors[i], vDescriptors[j]);

                // ? 下面是啥意思？答：就是正对称矩阵阿！对角线元素都是 0，[0][1] = [1][0] 那样式的
                Distances[i][j] = distij;
                Distances[j][i] = distij;
            }
        }

        // Step 4 选择最有代表性的描述子，它与其他描述子应该具有最小的距离中值
        // Take the descriptor with least median distance to the rest
        int BestMedian = INT_MAX;
        int BestIdx = 0;

        // 遍历每个描述子 i，计算它与其他所有描述子的距离中值
        for (size_t i = 0; i < N; i++)
        {
            // 第 i 个描述子到其它所有描述子之间的距离，vDists 是一个向量，表示第 i 个描述子到其他所有描述子的距离，如 [1, 5, 8, 12, 3, 4,,,]
            // Distances[i] 是第 i 行的起始地址（即第i行的第一个元素）；Distances[i] + N 是第 i 行的结束地址（即第i行的最后一个元素的后一个地址）；
            vector<int> vDists(Distances[i], Distances[i] + N);
            sort(vDists.begin(), vDists.end());

            // 获得中值
            int median = vDists[0.5 * (N - 1)];

            // 寻找最小的中值
            // ps1：如果有两个描述子的距离中值是一样都是最小的，选哪一个作为最具代表性的描述子？
            // 答：按照代码逻辑，当两个描述子的中值相同，代码会继续沿着遍历顺序找到第一个符合条件的描述子。也就是说，会优先选择索引较小的描述子。
            // 改进建议：加入其他决策规则：计算与其他描述子的距离总和，选择总和更小的描述子；记录多个描述子的候选列表，最后随机挑选一个。代码如下：
            // if (median < BestMedian || (median == BestMedian && new_criterion))
            // {
            //     BestMedian = median;
            //     BestIdx = i;
            // }
            if (median < BestMedian)
            {
                BestMedian = median;
                BestIdx = i;
            }
        }

        // ps2：如果 N = 偶数，还能计算出来吗？
        // 答：不用管！ORB描述子是二值化特征，距离计算是汉明距离（整数），不会引入小数。严格中值的“平均值”没有意义，直接取中间偏小位置的值即可。选取偏小位置的值通常对描述子相似性的影响很小，可以忽略不计。
        {
            unique_lock<mutex> lock(mMutexFeatures);

            // 最好的描述子，该描述子相对于其他描述子有最小的距离中值
            // 简化来讲，中值代表了这个描述子到其它描述子的平均距离
            // 最好的描述子就是和其它描述子的平均距离最小
            mDescriptor = vDescriptors[BestIdx].clone();
        }
    }

    cv::Mat MapPoint::GetDescriptor()
    {
        unique_lock<mutex> lock(mMutexFeatures);
        return mDescriptor.clone();
    }

    tuple<int, int> MapPoint::GetIndexInKeyFrame(KeyFrame *pKF)
    {
        unique_lock<mutex> lock(mMutexFeatures);
        if (mObservations.count(pKF))
            return mObservations[pKF];
        else
            return tuple<int, int>(-1, -1);
    }

    bool MapPoint::IsInKeyFrame(KeyFrame *pKF)
    {
        unique_lock<mutex> lock(mMutexFeatures);
        return (mObservations.count(pKF));
    }

    // TODO 更新平均观测方向以及观测距离范围
    // 由于一个 MapPoint 会被许多相机观测到，因此在插入关键帧后，需要更新相应变量
    // ps：创建新的关键帧的时候会调用
    void MapPoint::UpdateNormalAndDepth()
    {
        // step 1 获得观测到该地图点的所有关键帧、坐标等信息
        map<KeyFrame *, tuple<int, int>> observations;
        KeyFrame *pRefKF;
        Eigen::Vector3f Pos;
        {
            unique_lock<mutex> lock1(mMutexFeatures);
            unique_lock<mutex> lock2(mMutexPos);
            if (mbBad)
                return;

            observations = mObservations; // 获得观测到该地图点的所有关键帧
            pRefKF = mpRefKF;             // 观测到该地图点的参考关键帧（第一次创建时的关键帧，也是最早观测到该点的关键帧）
            Pos = mWorldPos;              // 地图点在世界坐标系中的位置
        }

        if (observations.empty())
            return;

        // Step 2 计算该地图点的法线方向，法线方向指的是地图点相对于相机的平均观测方向，也就是朝向等信息。
        // 能观测到该地图点的所有关键帧，对该点的观测方向归一化为单位向量，然后进行求和，得到该地图点的朝向
        // 初始值为 0 向量，累加为归一化向量，最后除以总数 n
        Eigen::Vector3f normal; // 法线方向
        normal.setZero();       // 初始化为 [0, 0, 0]
        int n = 0;              // 记录观测的相机数量
        for (map<KeyFrame *, tuple<int, int>>::iterator mit = observations.begin(), mend = observations.end(); mit != mend; mit++)
        {
            KeyFrame *pKF = mit->first;

            tuple<int, int> indexes = mit->second;
            int leftIndex = get<0>(indexes), rightIndex = get<1>(indexes);

            if (leftIndex != -1)
            {
                // 获取相机的光心（世界坐标系下）
                Eigen::Vector3f Owi = pKF->GetCameraCenter();
                // 从相机光心到地图点的向量
                Eigen::Vector3f normali = Pos - Owi;
                // 累加单位向量
                normal = normal + normali / normali.norm(); // normali / normali.norm()：把向量归一化成长度为1的单位向量，表示纯粹的方向信息，不考虑距离大小。normali.norm()：向量的长度（模）
                n++;
            }
            if (rightIndex != -1)
            {
                Eigen::Vector3f Owi = pKF->GetRightCameraCenter();
                Eigen::Vector3f normali = Pos - Owi;
                normal = normal + normali / normali.norm();
                n++;
            }
        }

        // step 3 计算深度范围。深度范围是地图点从【参考关键帧】可以被观测到的距离上下限，取决于金字塔层次的缩放倍数。
        // 参考关键帧相机指向地图点的向量（在世界坐标系下的表示）
        Eigen::Vector3f PC = Pos - pRefKF->GetCameraCenter();
        // 该点到参考关键帧相机的距离（深度）
        const float dist = PC.norm();

        tuple<int, int> indexes = observations[pRefKF];
        int leftIndex = get<0>(indexes), rightIndex = get<1>(indexes);

        // 观测到该地图点的参考关键帧的特征点所在的金字塔层级，在上面定义了
        // const int level = pRefKF->mvKeysUn[observations[pRefKF]].octave;
        int level;
        // case1：如果是单目图像，直接从未校正关键点 mvKeysUn 中取出金字塔层级 octave
        if (pRefKF->NLeft == -1)
        {
            level = pRefKF->mvKeysUn[leftIndex].octave;
        }
        // case2：如果是双目图像，优先从左目 mvKeys 中取出层级
        else if (leftIndex != -1)
        {
            level = pRefKF->mvKeys[leftIndex].octave;
        }
        // case3：如果仅右目中观测到，取右目关键点的层级
        else
        {
            level = pRefKF->mvKeysRight[rightIndex - pRefKF->NLeft].octave;
        }

        // 该点的金字塔层对应的缩放因子 scale^n, scale = 1.2，n 为层级数
        const float levelScaleFactor = pRefKF->mvScaleFactors[level];

        // 金字塔总层数，默认为 8
        const int nLevels = pRefKF->mnScaleLevels;

        {
            unique_lock<mutex> lock3(mMutexPos);

            // 地图点可以被观测到的最大距离，公式：最大距离 = 点到相机的实际深度值 × 当前金字塔层级的缩放倍数。
            mfMaxDistance = dist * levelScaleFactor;
            // 地图点可以被观测到的最小距离，公式：最小距离 = 最大距离 / 最小缩放倍数（对应金字塔最底层的倍数）。
            mfMinDistance = mfMaxDistance / pRefKF->mvScaleFactors[nLevels - 1];

            // 保存地图点平均观测方向向量
            mNormalVector = normal / n;
        }
    }

    void MapPoint::SetNormalVector(const Eigen::Vector3f &normal)
    {
        unique_lock<mutex> lock3(mMutexPos);
        mNormalVector = normal;
    }

    float MapPoint::GetMinDistanceInvariance()
    {
        unique_lock<mutex> lock(mMutexPos);
        return 0.8f * mfMinDistance;
    }

    float MapPoint::GetMaxDistanceInvariance()
    {
        unique_lock<mutex> lock(mMutexPos);
        return 1.2f * mfMaxDistance;
    }

    int MapPoint::PredictScale(const float &currentDist, KeyFrame *pKF)
    {
        float ratio;
        {
            unique_lock<mutex> lock(mMutexPos);
            ratio = mfMaxDistance / currentDist;
        }

        int nScale = ceil(log(ratio) / pKF->mfLogScaleFactor);
        if (nScale < 0)
            nScale = 0;
        else if (nScale >= pKF->mnScaleLevels)
            nScale = pKF->mnScaleLevels - 1;

        return nScale;
    }

    int MapPoint::PredictScale(const float &currentDist, Frame *pF)
    {
        float ratio;
        {
            unique_lock<mutex> lock(mMutexPos);
            ratio = mfMaxDistance / currentDist;
        }

        int nScale = ceil(log(ratio) / pF->mfLogScaleFactor);
        if (nScale < 0)
            nScale = 0;
        else if (nScale >= pF->mnScaleLevels)
            nScale = pF->mnScaleLevels - 1;

        return nScale;
    }

    void MapPoint::PrintObservations()
    {
        cout << "MP_OBS: MP " << mnId << endl;
        for (map<KeyFrame *, tuple<int, int>>::iterator mit = mObservations.begin(), mend = mObservations.end(); mit != mend; mit++)
        {
            KeyFrame *pKFi = mit->first;
            tuple<int, int> indexes = mit->second;
            int leftIndex = get<0>(indexes), rightIndex = get<1>(indexes);
            cout << "--OBS in KF " << pKFi->mnId << " in map " << pKFi->GetMap()->GetId() << endl;
        }
    }

    Map *MapPoint::GetMap()
    {
        unique_lock<mutex> lock(mMutexMap);
        return mpMap;
    }

    void MapPoint::UpdateMap(Map *pMap)
    {
        unique_lock<mutex> lock(mMutexMap);
        mpMap = pMap;
    }

    void MapPoint::PreSave(set<KeyFrame *> &spKF, set<MapPoint *> &spMP)
    {
        mBackupReplacedId = -1;
        if (mpReplaced && spMP.find(mpReplaced) != spMP.end())
            mBackupReplacedId = mpReplaced->mnId;

        mBackupObservationsId1.clear();
        mBackupObservationsId2.clear();
        // Save the id and position in each KF who view it
        for (std::map<KeyFrame *, std::tuple<int, int>>::const_iterator it = mObservations.begin(), end = mObservations.end(); it != end; ++it)
        {
            KeyFrame *pKFi = it->first;
            if (spKF.find(pKFi) != spKF.end())
            {
                mBackupObservationsId1[it->first->mnId] = get<0>(it->second);
                mBackupObservationsId2[it->first->mnId] = get<1>(it->second);
            }
            else
            {
                EraseObservation(pKFi);
            }
        }

        // Save the id of the reference KF
        if (spKF.find(mpRefKF) != spKF.end())
        {
            mBackupRefKFId = mpRefKF->mnId;
        }
    }

    void MapPoint::PostLoad(map<long unsigned int, KeyFrame *> &mpKFid, map<long unsigned int, MapPoint *> &mpMPid)
    {
        mpRefKF = mpKFid[mBackupRefKFId];
        if (!mpRefKF)
        {
            cout << "ERROR: MP without KF reference " << mBackupRefKFId << "; Num obs: " << nObs << endl;
        }
        mpReplaced = static_cast<MapPoint *>(NULL);
        if (mBackupReplacedId >= 0)
        {
            map<long unsigned int, MapPoint *>::iterator it = mpMPid.find(mBackupReplacedId);
            if (it != mpMPid.end())
                mpReplaced = it->second;
        }

        mObservations.clear();

        for (map<long unsigned int, int>::const_iterator it = mBackupObservationsId1.begin(), end = mBackupObservationsId1.end(); it != end; ++it)
        {
            KeyFrame *pKFi = mpKFid[it->first];
            map<long unsigned int, int>::const_iterator it2 = mBackupObservationsId2.find(it->first);
            std::tuple<int, int> indexes = tuple<int, int>(it->second, it2->second);
            if (pKFi)
            {
                mObservations[pKFi] = indexes;
            }
        }

        mBackupObservationsId1.clear();
        mBackupObservationsId2.clear();
    }

} // namespace ORB_SLAM
