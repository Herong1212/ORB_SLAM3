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

#include "KeyFrameDatabase.h"

#include "KeyFrame.h"
#include "Thirdparty/DBoW2/DBoW2/BowVector.h"

#include <mutex>

using namespace std;

namespace ORB_SLAM3
{
    // 构造函数
    KeyFrameDatabase::KeyFrameDatabase(const ORBVocabulary &voc) : mpVoc(&voc)
    {
        // 倒排索引表，大小等于词袋词典的总词数，每个词的索引中存储了包含该词的所有关键帧。
        mvInvertedFile.resize(voc.size());
    }

    void KeyFrameDatabase::add(KeyFrame *pKF)
    {
        unique_lock<mutex> lock(mMutex);

        for (DBoW2::BowVector::const_iterator vit = pKF->mBowVec.begin(), vend = pKF->mBowVec.end(); vit != vend; vit++)
            // mvInvertedFile[vit->first] ：获取所有包含当前单词的关键帧列表
            // vit->first 对应的是 pKF->mBowVec 中每个单词的 ID（比如 1, 2, 5）
            mvInvertedFile[vit->first].push_back(pKF);
    }

    // todo 作用：当删除关键帧数据库中的某个关键帧时，也需要更新【倒排索引】👇
    /**
     * @brief 关键帧被删除后，更新数据库的倒排索引
     *
     * @param[in] pKF   删除的关键帧
     */
    void KeyFrameDatabase::erase(KeyFrame *pKF)
    {
        // 线程锁，保护共享数据
        unique_lock<mutex> lock(mMutex);

        // 每个关键帧包含多个单词，遍历倒排索引中的这些单词，然后在单词对应的关键帧列表中删除该关键帧
        for (DBoW2::BowVector::const_iterator vit = pKF->mBowVec.begin(), vend = pKF->mBowVec.end(); vit != vend; vit++)
        {
            // 取出包含该单词的所有关键帧列表
            list<KeyFrame *> &lKFs = mvInvertedFile[vit->first];

            // 如果包含待删除的关键帧，则把该关键帧从列表里删除
            for (list<KeyFrame *>::iterator lit = lKFs.begin(), lend = lKFs.end(); lit != lend; lit++)
            {
                if (pKF == *lit)
                {
                    lKFs.erase(lit);
                    break;
                }
            }
        }
    }

    // 清空关键帧数据库
    void KeyFrameDatabase::clear()
    {
        mvInvertedFile.clear();               // mvInvertedFile[i] 表示包含了第 i 个 word id 的所有关键帧
        mvInvertedFile.resize(mpVoc->size()); // 重新调整 mvInvertedFile 的大小，使其大小与词汇表中单词的数量一致。
    }

    void KeyFrameDatabase::clearMap(Map *pMap)
    {
        unique_lock<mutex> lock(mMutex);

        // Erase elements in the Inverse File for the entry
        for (std::vector<list<KeyFrame *>>::iterator vit = mvInvertedFile.begin(), vend = mvInvertedFile.end(); vit != vend; vit++)
        {
            // List of keyframes that share the word
            list<KeyFrame *> &lKFs = *vit;

            for (list<KeyFrame *>::iterator lit = lKFs.begin(), lend = lKFs.end(); lit != lend;)
            {
                KeyFrame *pKFi = *lit;
                if (pMap == pKFi->GetMap())
                {
                    lit = lKFs.erase(lit);
                    // Dont delete the KF because the class Map clean all the KF when it is destroyed
                }
                else
                {
                    ++lit;
                }
            }
        }
    }

    // note：检测闭环候选关键帧
    vector<KeyFrame *> KeyFrameDatabase::DetectLoopCandidates(KeyFrame *pKF, float minScore)
    {
        set<KeyFrame *> spConnectedKeyFrames = pKF->GetConnectedKeyFrames();
        list<KeyFrame *> lKFsSharingWords;

        // Step 1：找出和当前帧具有公共单词的所有关键帧，不包括与当前帧连接的关键帧
        {
            unique_lock<mutex> lock(mMutex);

            // todo 遍历词袋向量中的每个词
            for (DBoW2::BowVector::const_iterator vit = pKF->mBowVec.begin(), vend = pKF->mBowVec.end(); vit != vend; vit++)
            {
                // 根据倒排索引，提取所有包含该 word 的 KeyFrame
                list<KeyFrame *> &lKFs = mvInvertedFile[vit->first];

                // 然后对这些关键帧展开遍历
                for (list<KeyFrame *>::iterator lit = lKFs.begin(), lend = lKFs.end(); lit != lend; lit++)
                {
                    KeyFrame *pKFi = *lit;

                    if (pKFi->GetMap() == pKF->GetMap()) // For consider a loop candidate it a candidate it must be in the same map
                    {
                        if (pKFi->mnLoopQuery != pKF->mnId)
                        {
                            pKFi->mnLoopWords = 0;

                            // 和当前关键帧共视的话不作为闭环候选帧
                            if (!spConnectedKeyFrames.count(pKFi))
                            {
                                // 没有共视就标记作为闭环候选关键帧，放到 lKFsSharingWords 里
                                pKFi->mnLoopQuery = pKF->mnId;
                                lKFsSharingWords.push_back(pKFi);
                            }
                        }
                        // 记录pKFi与pKF具有相同word的个数
                        pKFi->mnLoopWords++;
                    }
                }
            }
        }

        // 如果没有关键帧和这个关键帧具有相同的单词，那么就返回空
        if (lKFsSharingWords.empty())
            return vector<KeyFrame *>();

        list<pair<float, KeyFrame *>> lScoreAndMatch;

        // Step 2：统计上述所有闭环候选帧中与当前帧具有共同单词最多的单词数，用来决定相对阈值
        int maxCommonWords = 0;

        for (list<KeyFrame *>::iterator lit = lKFsSharingWords.begin(), lend = lKFsSharingWords.end(); lit != lend; lit++)
        {
            if ((*lit)->mnLoopWords > maxCommonWords)
                maxCommonWords = (*lit)->mnLoopWords;
        }

        int minCommonWords = maxCommonWords * 0.8f;

        int nscores = 0;

        // Step 3：遍历上述所有闭环候选帧，挑选出共有单词数大于minCommonWords且单词匹配度大于minScore存入lScoreAndMatch
        for (list<KeyFrame *>::iterator lit = lKFsSharingWords.begin(), lend = lKFsSharingWords.end(); lit != lend; lit++)
        {
            KeyFrame *pKFi = *lit;

            // pKF只和具有共同单词较多（大于minCommonWords）的关键帧进行比较
            if (pKFi->mnLoopWords > minCommonWords)
            {
                nscores++;

                // 用词袋模型的 score() 函数来计算两者的相似度得分
                float si = mpVoc->score(pKF->mBowVec, pKFi->mBowVec);

                pKFi->mLoopScore = si;
                // ! 注意这里区别于 DetectLoopCandidates()，还有个审核条件！
                if (si >= minScore) // 重定位的时候直接加进去的！
                    lScoreAndMatch.push_back(make_pair(si, pKFi));
            }
        }

        // 如果没有超过指定相似度阈值的，那么也就直接跳过去
        if (lScoreAndMatch.empty())
            return vector<KeyFrame *>();

        // Step 4：计算上述候选帧对应的共视关键帧组的总得分，得到最高组得分 bestAccScore，并以此决定阈值 minScoreToRetain
        list<pair<float, KeyFrame *>> lAccScoreAndMatch;
        float bestAccScore = minScore;

        // ps1：首先，遍历候选关键帧
        for (list<pair<float, KeyFrame *>>::iterator it = lScoreAndMatch.begin(), itend = lScoreAndMatch.end(); it != itend; it++)
        {
            KeyFrame *pKFi = it->second;
            vector<KeyFrame *> vpNeighs = pKFi->GetBestCovisibilityKeyFrames(10);

            // todo1、当前共视组中得分最高的关键帧的相似度得分。
            float bestScore = it->first;
            // todo2、累计得分，表示当前候选关键帧及其共视关键帧组的总相似度得分。
            float accScore = it->first;
            // todo3、当前共视关键帧组中得分最高的关键帧。
            KeyFrame *pBestKF = pKFi;

            // 单单计算当前帧和某一关键帧的相似性是不够的，这里将与关键帧相连（权值最高，共视程度最高）的前十个关键帧归为一组，计算累计得分
            for (vector<KeyFrame *>::iterator vit = vpNeighs.begin(), vend = vpNeighs.end(); vit != vend; vit++)
            {
                KeyFrame *pKF2 = *vit;
                // ps2：其次，遍历每一个候选关键帧的共视关键帧组，并计算组的总得分
                if (pKF2->mnLoopQuery == pKF->mnId && pKF2->mnLoopWords > minCommonWords)
                {
                    // 1、统计每个组的累计得分
                    accScore += pKF2->mLoopScore;
                    // 2、统计得到组里分数最高的 KeyFrame 及其得分
                    if (pKF2->mLoopScore > bestScore)
                    {
                        pBestKF = pKF2;
                        bestScore = pKF2->mLoopScore;
                    }
                }
            }

            // first -- 每个候选组的累计得分；second -- 组内最佳关键帧
            lAccScoreAndMatch.push_back(make_pair(accScore, pBestKF));

            // 记录所有组中最高的得分
            if (accScore > bestAccScore)
                bestAccScore = accScore;
        }

        // Step 5：只取组得分大于阈值的组，得到组中分数最高的关键帧们作为闭环候选关键帧
        float minScoreToRetain = 0.75f * bestAccScore; // notice：【阈值2】---所有组中最高组得分的 0.75 倍

        // 用于检查是否有重复的关键帧，防止重复添加相同的关键帧到候选结果中
        set<KeyFrame *> spAlreadyAddedKF;
        // ps：存储的是所有符合条件的重定位候选关键帧指针！
        vector<KeyFrame *> vpLoopCandidates;
        // lAccScoreAndMatch 的大小是所有候选关键帧组的总数，理论上 vpRelocCandidates 的最终大小不会超过这个值。
        vpLoopCandidates.reserve(lAccScoreAndMatch.size());

        for (list<pair<float, KeyFrame *>>::iterator it = lAccScoreAndMatch.begin(), itend = lAccScoreAndMatch.end(); it != itend; it++)
        {
            // 只返回累计 得分大于【阈值 2】的组中分数最高的关键帧
            if (it->first > minScoreToRetain)
            {
                KeyFrame *pKFi = it->second;

                // 判断该 pKFi 是否已经添加在队列中了
                if (!spAlreadyAddedKF.count(pKFi))
                {
                    vpLoopCandidates.push_back(pKFi);
                    spAlreadyAddedKF.insert(pKFi);
                }
            }
        }

        return vpLoopCandidates; // 最终得到的闭环候选关键帧组
    }

    void KeyFrameDatabase::DetectCandidates(KeyFrame *pKF, float minScore, vector<KeyFrame *> &vpLoopCand, vector<KeyFrame *> &vpMergeCand)
    {
        set<KeyFrame *> spConnectedKeyFrames = pKF->GetConnectedKeyFrames();
        list<KeyFrame *> lKFsSharingWordsLoop, lKFsSharingWordsMerge;

        // Search all keyframes that share a word with current keyframes
        // Discard keyframes connected to the query keyframe
        {
            unique_lock<mutex> lock(mMutex);

            for (DBoW2::BowVector::const_iterator vit = pKF->mBowVec.begin(), vend = pKF->mBowVec.end(); vit != vend; vit++)
            {
                list<KeyFrame *> &lKFs = mvInvertedFile[vit->first];

                for (list<KeyFrame *>::iterator lit = lKFs.begin(), lend = lKFs.end(); lit != lend; lit++)
                {
                    KeyFrame *pKFi = *lit;
                    if (pKFi->GetMap() == pKF->GetMap()) // For consider a loop candidate it a candidate it must be in the same map
                    {
                        if (pKFi->mnLoopQuery != pKF->mnId)
                        {
                            pKFi->mnLoopWords = 0;
                            if (!spConnectedKeyFrames.count(pKFi))
                            {
                                pKFi->mnLoopQuery = pKF->mnId;
                                lKFsSharingWordsLoop.push_back(pKFi);
                            }
                        }
                        pKFi->mnLoopWords++;
                    }
                    else if (!pKFi->GetMap()->IsBad())
                    {
                        if (pKFi->mnMergeQuery != pKF->mnId)
                        {
                            pKFi->mnMergeWords = 0;
                            if (!spConnectedKeyFrames.count(pKFi))
                            {
                                pKFi->mnMergeQuery = pKF->mnId;
                                lKFsSharingWordsMerge.push_back(pKFi);
                            }
                        }
                        pKFi->mnMergeWords++;
                    }
                }
            }
        }

        if (lKFsSharingWordsLoop.empty() && lKFsSharingWordsMerge.empty())
            return;

        if (!lKFsSharingWordsLoop.empty())
        {
            list<pair<float, KeyFrame *>> lScoreAndMatch;

            // Only compare against those keyframes that share enough words
            int maxCommonWords = 0;
            for (list<KeyFrame *>::iterator lit = lKFsSharingWordsLoop.begin(), lend = lKFsSharingWordsLoop.end(); lit != lend; lit++)
            {
                if ((*lit)->mnLoopWords > maxCommonWords)
                    maxCommonWords = (*lit)->mnLoopWords;
            }

            int minCommonWords = maxCommonWords * 0.8f;

            int nscores = 0;

            // Compute similarity score. Retain the matches whose score is higher than minScore
            for (list<KeyFrame *>::iterator lit = lKFsSharingWordsLoop.begin(), lend = lKFsSharingWordsLoop.end(); lit != lend; lit++)
            {
                KeyFrame *pKFi = *lit;

                if (pKFi->mnLoopWords > minCommonWords)
                {
                    nscores++;

                    float si = mpVoc->score(pKF->mBowVec, pKFi->mBowVec);

                    pKFi->mLoopScore = si;
                    if (si >= minScore)
                        lScoreAndMatch.push_back(make_pair(si, pKFi));
                }
            }

            if (!lScoreAndMatch.empty())
            {
                list<pair<float, KeyFrame *>> lAccScoreAndMatch;
                float bestAccScore = minScore;

                // Lets now accumulate score by covisibility
                for (list<pair<float, KeyFrame *>>::iterator it = lScoreAndMatch.begin(), itend = lScoreAndMatch.end(); it != itend; it++)
                {
                    KeyFrame *pKFi = it->second;
                    vector<KeyFrame *> vpNeighs = pKFi->GetBestCovisibilityKeyFrames(10);

                    float bestScore = it->first;
                    float accScore = it->first;
                    KeyFrame *pBestKF = pKFi;
                    for (vector<KeyFrame *>::iterator vit = vpNeighs.begin(), vend = vpNeighs.end(); vit != vend; vit++)
                    {
                        KeyFrame *pKF2 = *vit;
                        if (pKF2->mnLoopQuery == pKF->mnId && pKF2->mnLoopWords > minCommonWords)
                        {
                            accScore += pKF2->mLoopScore;
                            if (pKF2->mLoopScore > bestScore)
                            {
                                pBestKF = pKF2;
                                bestScore = pKF2->mLoopScore;
                            }
                        }
                    }

                    lAccScoreAndMatch.push_back(make_pair(accScore, pBestKF));
                    if (accScore > bestAccScore)
                        bestAccScore = accScore;
                }

                // Return all those keyframes with a score higher than 0.75*bestScore
                float minScoreToRetain = 0.75f * bestAccScore;

                set<KeyFrame *> spAlreadyAddedKF;
                vpLoopCand.reserve(lAccScoreAndMatch.size());

                for (list<pair<float, KeyFrame *>>::iterator it = lAccScoreAndMatch.begin(), itend = lAccScoreAndMatch.end(); it != itend; it++)
                {
                    if (it->first > minScoreToRetain)
                    {
                        KeyFrame *pKFi = it->second;
                        if (!spAlreadyAddedKF.count(pKFi))
                        {
                            vpLoopCand.push_back(pKFi);
                            spAlreadyAddedKF.insert(pKFi);
                        }
                    }
                }
            }
        }

        if (!lKFsSharingWordsMerge.empty())
        {
            list<pair<float, KeyFrame *>> lScoreAndMatch;

            // Only compare against those keyframes that share enough words
            int maxCommonWords = 0;
            for (list<KeyFrame *>::iterator lit = lKFsSharingWordsMerge.begin(), lend = lKFsSharingWordsMerge.end(); lit != lend; lit++)
            {
                if ((*lit)->mnMergeWords > maxCommonWords)
                    maxCommonWords = (*lit)->mnMergeWords;
            }

            int minCommonWords = maxCommonWords * 0.8f;

            int nscores = 0;

            // Compute similarity score. Retain the matches whose score is higher than minScore
            for (list<KeyFrame *>::iterator lit = lKFsSharingWordsMerge.begin(), lend = lKFsSharingWordsMerge.end(); lit != lend; lit++)
            {
                KeyFrame *pKFi = *lit;

                if (pKFi->mnMergeWords > minCommonWords)
                {
                    nscores++;

                    float si = mpVoc->score(pKF->mBowVec, pKFi->mBowVec);

                    pKFi->mMergeScore = si;
                    if (si >= minScore)
                        lScoreAndMatch.push_back(make_pair(si, pKFi));
                }
            }

            if (!lScoreAndMatch.empty())
            {
                list<pair<float, KeyFrame *>> lAccScoreAndMatch;
                float bestAccScore = minScore;

                // Lets now accumulate score by covisibility
                for (list<pair<float, KeyFrame *>>::iterator it = lScoreAndMatch.begin(), itend = lScoreAndMatch.end(); it != itend; it++)
                {
                    KeyFrame *pKFi = it->second;
                    vector<KeyFrame *> vpNeighs = pKFi->GetBestCovisibilityKeyFrames(10);

                    float bestScore = it->first;
                    float accScore = it->first;
                    KeyFrame *pBestKF = pKFi;
                    for (vector<KeyFrame *>::iterator vit = vpNeighs.begin(), vend = vpNeighs.end(); vit != vend; vit++)
                    {
                        KeyFrame *pKF2 = *vit;
                        if (pKF2->mnMergeQuery == pKF->mnId && pKF2->mnMergeWords > minCommonWords)
                        {
                            accScore += pKF2->mMergeScore;
                            if (pKF2->mMergeScore > bestScore)
                            {
                                pBestKF = pKF2;
                                bestScore = pKF2->mMergeScore;
                            }
                        }
                    }

                    lAccScoreAndMatch.push_back(make_pair(accScore, pBestKF));
                    if (accScore > bestAccScore)
                        bestAccScore = accScore;
                }

                // Return all those keyframes with a score higher than 0.75*bestScore
                float minScoreToRetain = 0.75f * bestAccScore;

                set<KeyFrame *> spAlreadyAddedKF;
                vpMergeCand.reserve(lAccScoreAndMatch.size());

                for (list<pair<float, KeyFrame *>>::iterator it = lAccScoreAndMatch.begin(), itend = lAccScoreAndMatch.end(); it != itend; it++)
                {
                    if (it->first > minScoreToRetain)
                    {
                        KeyFrame *pKFi = it->second;
                        if (!spAlreadyAddedKF.count(pKFi))
                        {
                            vpMergeCand.push_back(pKFi);
                            spAlreadyAddedKF.insert(pKFi);
                        }
                    }
                }
            }
        }

        for (DBoW2::BowVector::const_iterator vit = pKF->mBowVec.begin(), vend = pKF->mBowVec.end(); vit != vend; vit++)
        {
            list<KeyFrame *> &lKFs = mvInvertedFile[vit->first];

            for (list<KeyFrame *>::iterator lit = lKFs.begin(), lend = lKFs.end(); lit != lend; lit++)
            {
                KeyFrame *pKFi = *lit;
                pKFi->mnLoopQuery = -1;
                pKFi->mnMergeQuery = -1;
            }
        }
    }

    void KeyFrameDatabase::DetectBestCandidates(KeyFrame *pKF, vector<KeyFrame *> &vpLoopCand, vector<KeyFrame *> &vpMergeCand, int nMinWords)
    {
        list<KeyFrame *> lKFsSharingWords;
        set<KeyFrame *> spConnectedKF;

        // Search all keyframes that share a word with current frame
        {
            unique_lock<mutex> lock(mMutex);

            spConnectedKF = pKF->GetConnectedKeyFrames();

            for (DBoW2::BowVector::const_iterator vit = pKF->mBowVec.begin(), vend = pKF->mBowVec.end(); vit != vend; vit++)
            {
                list<KeyFrame *> &lKFs = mvInvertedFile[vit->first];

                for (list<KeyFrame *>::iterator lit = lKFs.begin(), lend = lKFs.end(); lit != lend; lit++)
                {
                    KeyFrame *pKFi = *lit;
                    if (spConnectedKF.find(pKFi) != spConnectedKF.end())
                    {
                        continue;
                    }
                    if (pKFi->mnPlaceRecognitionQuery != pKF->mnId)
                    {
                        pKFi->mnPlaceRecognitionWords = 0;
                        pKFi->mnPlaceRecognitionQuery = pKF->mnId;
                        lKFsSharingWords.push_back(pKFi);
                    }
                    pKFi->mnPlaceRecognitionWords++;
                }
            }
        }
        if (lKFsSharingWords.empty())
            return;

        // Only compare against those keyframes that share enough words
        int maxCommonWords = 0;
        for (list<KeyFrame *>::iterator lit = lKFsSharingWords.begin(), lend = lKFsSharingWords.end(); lit != lend; lit++)
        {
            if ((*lit)->mnPlaceRecognitionWords > maxCommonWords)
                maxCommonWords = (*lit)->mnPlaceRecognitionWords;
        }

        int minCommonWords = maxCommonWords * 0.8f;

        if (minCommonWords < nMinWords)
        {
            minCommonWords = nMinWords;
        }

        list<pair<float, KeyFrame *>> lScoreAndMatch;

        int nscores = 0;

        // Compute similarity score.
        for (list<KeyFrame *>::iterator lit = lKFsSharingWords.begin(), lend = lKFsSharingWords.end(); lit != lend; lit++)
        {
            KeyFrame *pKFi = *lit;

            if (pKFi->mnPlaceRecognitionWords > minCommonWords)
            {
                nscores++;
                float si = mpVoc->score(pKF->mBowVec, pKFi->mBowVec);
                pKFi->mPlaceRecognitionScore = si;
                lScoreAndMatch.push_back(make_pair(si, pKFi));
            }
        }

        if (lScoreAndMatch.empty())
            return;

        list<pair<float, KeyFrame *>> lAccScoreAndMatch;
        float bestAccScore = 0;

        // Lets now accumulate score by covisibility
        for (list<pair<float, KeyFrame *>>::iterator it = lScoreAndMatch.begin(), itend = lScoreAndMatch.end(); it != itend; it++)
        {
            KeyFrame *pKFi = it->second;
            vector<KeyFrame *> vpNeighs = pKFi->GetBestCovisibilityKeyFrames(10);

            float bestScore = it->first;
            float accScore = bestScore;
            KeyFrame *pBestKF = pKFi;
            for (vector<KeyFrame *>::iterator vit = vpNeighs.begin(), vend = vpNeighs.end(); vit != vend; vit++)
            {
                KeyFrame *pKF2 = *vit;
                if (pKF2->mnPlaceRecognitionQuery != pKF->mnId)
                    continue;

                accScore += pKF2->mPlaceRecognitionScore;
                if (pKF2->mPlaceRecognitionScore > bestScore)
                {
                    pBestKF = pKF2;
                    bestScore = pKF2->mPlaceRecognitionScore;
                }
            }
            lAccScoreAndMatch.push_back(make_pair(accScore, pBestKF));
            if (accScore > bestAccScore)
                bestAccScore = accScore;
        }

        // Return all those keyframes with a score higher than 0.75*bestScore
        float minScoreToRetain = 0.75f * bestAccScore;
        set<KeyFrame *> spAlreadyAddedKF;
        vpLoopCand.reserve(lAccScoreAndMatch.size());
        vpMergeCand.reserve(lAccScoreAndMatch.size());
        for (list<pair<float, KeyFrame *>>::iterator it = lAccScoreAndMatch.begin(), itend = lAccScoreAndMatch.end(); it != itend; it++)
        {
            const float &si = it->first;
            if (si > minScoreToRetain)
            {
                KeyFrame *pKFi = it->second;
                if (!spAlreadyAddedKF.count(pKFi))
                {
                    if (pKF->GetMap() == pKFi->GetMap())
                    {
                        vpLoopCand.push_back(pKFi);
                    }
                    else
                    {
                        vpMergeCand.push_back(pKFi);
                    }
                    spAlreadyAddedKF.insert(pKFi);
                }
            }
        }
    }

    bool compFirst(const pair<float, KeyFrame *> &a, const pair<float, KeyFrame *> &b)
    {
        return a.first > b.first;
    }

    void KeyFrameDatabase::DetectNBestCandidates(KeyFrame *pKF, vector<KeyFrame *> &vpLoopCand, vector<KeyFrame *> &vpMergeCand, int nNumCandidates)
    {
        list<KeyFrame *> lKFsSharingWords;
        set<KeyFrame *> spConnectedKF;

        // Search all keyframes that share a word with current frame
        {
            unique_lock<mutex> lock(mMutex);

            spConnectedKF = pKF->GetConnectedKeyFrames();

            for (DBoW2::BowVector::const_iterator vit = pKF->mBowVec.begin(), vend = pKF->mBowVec.end(); vit != vend; vit++)
            {
                list<KeyFrame *> &lKFs = mvInvertedFile[vit->first];

                for (list<KeyFrame *>::iterator lit = lKFs.begin(), lend = lKFs.end(); lit != lend; lit++)
                {
                    KeyFrame *pKFi = *lit;

                    if (pKFi->mnPlaceRecognitionQuery != pKF->mnId)
                    {
                        pKFi->mnPlaceRecognitionWords = 0;
                        if (!spConnectedKF.count(pKFi))
                        {

                            pKFi->mnPlaceRecognitionQuery = pKF->mnId;
                            lKFsSharingWords.push_back(pKFi);
                        }
                    }
                    pKFi->mnPlaceRecognitionWords++;
                }
            }
        }
        if (lKFsSharingWords.empty())
            return;

        // Only compare against those keyframes that share enough words
        int maxCommonWords = 0;
        for (list<KeyFrame *>::iterator lit = lKFsSharingWords.begin(), lend = lKFsSharingWords.end(); lit != lend; lit++)
        {
            if ((*lit)->mnPlaceRecognitionWords > maxCommonWords)
                maxCommonWords = (*lit)->mnPlaceRecognitionWords;
        }

        int minCommonWords = maxCommonWords * 0.8f;

        list<pair<float, KeyFrame *>> lScoreAndMatch;

        int nscores = 0;

        // Compute similarity score.
        for (list<KeyFrame *>::iterator lit = lKFsSharingWords.begin(), lend = lKFsSharingWords.end(); lit != lend; lit++)
        {
            KeyFrame *pKFi = *lit;

            if (pKFi->mnPlaceRecognitionWords > minCommonWords)
            {
                nscores++;
                float si = mpVoc->score(pKF->mBowVec, pKFi->mBowVec);
                pKFi->mPlaceRecognitionScore = si;
                lScoreAndMatch.push_back(make_pair(si, pKFi));
            }
        }

        if (lScoreAndMatch.empty())
            return;

        list<pair<float, KeyFrame *>> lAccScoreAndMatch;
        float bestAccScore = 0;

        // Lets now accumulate score by covisibility
        for (list<pair<float, KeyFrame *>>::iterator it = lScoreAndMatch.begin(), itend = lScoreAndMatch.end(); it != itend; it++)
        {
            KeyFrame *pKFi = it->second;
            vector<KeyFrame *> vpNeighs = pKFi->GetBestCovisibilityKeyFrames(10);

            float bestScore = it->first;
            float accScore = bestScore;
            KeyFrame *pBestKF = pKFi;
            for (vector<KeyFrame *>::iterator vit = vpNeighs.begin(), vend = vpNeighs.end(); vit != vend; vit++)
            {
                KeyFrame *pKF2 = *vit;
                if (pKF2->mnPlaceRecognitionQuery != pKF->mnId)
                    continue;

                accScore += pKF2->mPlaceRecognitionScore;
                if (pKF2->mPlaceRecognitionScore > bestScore)
                {
                    pBestKF = pKF2;
                    bestScore = pKF2->mPlaceRecognitionScore;
                }
            }
            lAccScoreAndMatch.push_back(make_pair(accScore, pBestKF));
            if (accScore > bestAccScore)
                bestAccScore = accScore;
        }

        lAccScoreAndMatch.sort(compFirst);

        vpLoopCand.reserve(nNumCandidates);
        vpMergeCand.reserve(nNumCandidates);
        set<KeyFrame *> spAlreadyAddedKF;
        int i = 0;
        list<pair<float, KeyFrame *>>::iterator it = lAccScoreAndMatch.begin();
        while (i < lAccScoreAndMatch.size() && (vpLoopCand.size() < nNumCandidates || vpMergeCand.size() < nNumCandidates))
        {
            KeyFrame *pKFi = it->second;
            if (pKFi->isBad())
                continue;

            if (!spAlreadyAddedKF.count(pKFi))
            {
                if (pKF->GetMap() == pKFi->GetMap() && vpLoopCand.size() < nNumCandidates)
                {
                    vpLoopCand.push_back(pKFi);
                }
                else if (pKF->GetMap() != pKFi->GetMap() && vpMergeCand.size() < nNumCandidates && !pKFi->GetMap()->IsBad())
                {
                    vpMergeCand.push_back(pKFi);
                }
                spAlreadyAddedKF.insert(pKFi);
            }
            i++;
            it++;
        }
    }

    // note：检测重定位候选关键帧
    vector<KeyFrame *> KeyFrameDatabase::DetectRelocalizationCandidates(Frame *F, Map *pMap)
    {
        list<KeyFrame *> lKFsSharingWords;

        // Step 1：找出和当前帧具有共同单词 (word) 的所有关键帧
        {
            unique_lock<mutex> lock(mMutex);

            for (DBoW2::BowVector::const_iterator vit = F->mBowVec.begin(), vend = F->mBowVec.end(); vit != vend; vit++)
            {
                list<KeyFrame *> &lKFs = mvInvertedFile[vit->first];

                for (list<KeyFrame *>::iterator lit = lKFs.begin(), lend = lKFs.end(); lit != lend; lit++)
                {
                    KeyFrame *pKFi = *lit;
                    if (pKFi->mnRelocQuery != F->mnId)
                    {
                        pKFi->mnRelocWords = 0;
                        pKFi->mnRelocQuery = F->mnId;
                        lKFsSharingWords.push_back(pKFi);
                    }
                    pKFi->mnRelocWords++;
                }
            }
        }

        // 如果和当前帧具有公共单词的关键帧数目为 0，无法进行重定位，返回空
        if (lKFsSharingWords.empty())
            return vector<KeyFrame *>();

        // Step 2：统计上述关键帧中与当前帧 F 具有共同单词最多的单词数 maxCommonWords，用来设定 【阈值1】
        int maxCommonWords = 0;

        for (list<KeyFrame *>::iterator lit = lKFsSharingWords.begin(), lend = lKFsSharingWords.end(); lit != lend; lit++)
        {
            if ((*lit)->mnRelocWords > maxCommonWords)
                maxCommonWords = (*lit)->mnRelocWords;
        }

        // Step 3：遍历上述关键帧，挑选出共有单词数大于【阈值1】的及【其和当前帧单词匹配的得分】存入 lScoreAndMatch
        int minCommonWords = maxCommonWords * 0.8f; // notice：【阈值1】---最小公共单词数为最大公共单词数目的 0.8 倍

        // ! lScoreAndMatch的数据结构：first -- 相似度得分；second -- 对应的候选关键帧，如：lScoreAndMatch = [{0.8, KF_1}, {0.9, KF_3},,,];
        list<pair<float, KeyFrame *>> lScoreAndMatch;

        int nscores = 0;

        // Compute similarity score.
        for (list<KeyFrame *>::iterator lit = lKFsSharingWords.begin(), lend = lKFsSharingWords.end(); lit != lend; lit++)
        {
            KeyFrame *pKFi = *lit;

            // 当前帧 F 只和具有共同单词较多（大于 minCommonWords）的关键帧进行比较
            if (pKFi->mnRelocWords > minCommonWords)
            {
                nscores++; // 这个变量后面没有用到

                // 用词袋模型的 score() 函数来计算两者的相似度得分
                float si = mpVoc->score(F->mBowVec, pKFi->mBowVec);
                pKFi->mRelocScore = si;
                lScoreAndMatch.push_back(make_pair(si, pKFi)); // ? 直接就加进去了？
            }
        } // ps：结果---筛选出与当前帧相似的关键帧集合 lScoreAndMatch。

        if (lScoreAndMatch.empty())
            return vector<KeyFrame *>();

        // Step 4：计算 lScoreAndMatch 中每个关键帧的共视关键帧组的总得分，得到最高组得分 bestAccScore，并以此决定【阈值2】

        list<pair<float, KeyFrame *>> lAccScoreAndMatch;
        float bestAccScore = 0; // todo4、所有共视组中累计得分的最高值，用于设定筛选【阈值2】

        // ps1：首先，遍历候选关键帧
        for (list<pair<float, KeyFrame *>>::iterator it = lScoreAndMatch.begin(), itend = lScoreAndMatch.end(); it != itend; it++)
        {
            KeyFrame *pKFi = it->second;

            // 单单计算当前帧和某一关键帧的相似性是不够的，这里将与候选关键帧 pKFi 共视程度最高的【前 10 个】关键帧归为一组，计算累计得分
            vector<KeyFrame *> vpNeighs = pKFi->GetBestCovisibilityKeyFrames(10);

            // todo1、当前共视组中得分最高的关键帧的相似度得分。
            float bestScore = it->first;
            // todo2、累计得分，表示当前候选关键帧及其共视关键帧组的总相似度得分。
            float accScore = bestScore;
            // todo3、当前共视关键帧组中得分最高的关键帧。
            KeyFrame *pBestKF = pKFi;

            // ps2：其次，遍历每一个候选关键帧的共视关键帧组，并计算组的总得分
            for (vector<KeyFrame *>::iterator vit = vpNeighs.begin(), vend = vpNeighs.end(); vit != vend; vit++)
            {
                KeyFrame *pKF2 = *vit;
                // pKF2 = *vit; // 这样以后就不用对迭代器进行解引用了！

                // 只有 pKF2 也在重定位候选帧中，才能贡献分数
                if (pKF2->mnRelocQuery != F->mnId)
                    continue;

                // 1、统计每个组的累计得分
                accScore += pKF2->mRelocScore;

                // 2、统计得到组里分数最高的 KeyFrame 及其得分
                if (pKF2->mRelocScore > bestScore)
                {
                    pBestKF = pKF2;
                    bestScore = pKF2->mRelocScore;
                }
            }

            // ! lAccScoreAndMatch 的数据结构：first -- 每个候选组的累计得分；second -- 组内最佳关键帧
            lAccScoreAndMatch.push_back(make_pair(accScore, pBestKF));

            // 记录所有组中最高的得分
            if (accScore > bestAccScore)
                bestAccScore = accScore;
        }

        // Step 5：得到所有组中总得分大于【阈值 2】的，组内得分最高的关键帧，作为候选关键帧组
        float minScoreToRetain = 0.75f * bestAccScore; // notice：【阈值2】---最高组得分的 0.75 倍

        // 用于检查是否有重复的关键帧，防止重复添加相同的关键帧到候选结果中
        set<KeyFrame *> spAlreadyAddedKF;
        // * 存储的是所有符合条件的重定位候选关键帧指针
        vector<KeyFrame *> vpRelocCandidates;
        // lAccScoreAndMatch 的大小是所有候选关键帧组的总数，理论上 vpRelocCandidates 的最终大小不会超过这个值。
        vpRelocCandidates.reserve(lAccScoreAndMatch.size());

        for (list<pair<float, KeyFrame *>>::iterator it = lAccScoreAndMatch.begin(), itend = lAccScoreAndMatch.end(); it != itend; it++)
        {
            const float &si = it->first;
            if (si > minScoreToRetain)
            {
                KeyFrame *pKFi = it->second;

                if (pKFi->GetMap() != pMap)
                    continue;

                // 判断该 pKFi 是否已经添加在队列中了
                if (!spAlreadyAddedKF.count(pKFi))
                {
                    vpRelocCandidates.push_back(pKFi);
                    spAlreadyAddedKF.insert(pKFi);
                }
            }
        }

        return vpRelocCandidates; // 最终得到的重定位候选关键帧组，有好几个嗷~
    }

    void KeyFrameDatabase::SetORBVocabulary(ORBVocabulary *pORBVoc)
    {
        ORBVocabulary **ptr;
        ptr = (ORBVocabulary **)(&mpVoc);
        *ptr = pORBVoc;

        mvInvertedFile.clear();
        mvInvertedFile.resize(mpVoc->size());
    }

} // namespace ORB_SLAM
