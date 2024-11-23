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

#ifndef VIEWER_H
#define VIEWER_H

#include "FrameDrawer.h"
#include "MapDrawer.h"
#include "Tracking.h"
#include "System.h"
#include "Settings.h"

#include <mutex>

namespace ORB_SLAM3
{

    class Tracking;
    class FrameDrawer;
    class MapDrawer;
    class System;
    class Settings;

    class Viewer
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Viewer(System *pSystem, FrameDrawer *pFrameDrawer, MapDrawer *pMapDrawer, Tracking *pTracking, const string &strSettingPath, Settings *settings);

        void newParameterLoader(Settings *settings);

        // Main thread function. Draw points, keyframes, the current camera pose and the last processed
        // frame. Drawing is refreshed according to the camera fps. We use Pangolin.
        void Run();

        void RequestFinish();

        void RequestStop();

        bool isFinished();

        bool isStopped();

        bool isStepByStep();

        void Release();

        // void SetTrackingPause();

        bool both;

        // todo--Yolo
        std::vector<cv::Rect2i> mvPersonArea;
        // * 存储目标检测的结果：
        // first---表示检测到的类别名称（如 "person"、"car"）；second---是一个 std::vector，存储该类别对应的所有检测区域（cv::Rect）
        // cv::Revt ：OpenCV 定义的矩形框类型，包含四个整数值（x、y、width、height），表示矩形框的位置和大小
        std::map<std::string, std::vector<cv::Rect2i>> mmDetectMap; 
        std::mutex mMutexPAFinsh;

    private:
        bool ParseViewerParamFile(cv::FileStorage &fSettings);

        bool Stop();

        System *mpSystem;
        FrameDrawer *mpFrameDrawer;
        MapDrawer *mpMapDrawer;
        Tracking *mpTracker;

        // 1/fps in ms
        double mT;
        float mImageWidth, mImageHeight;
        float mImageViewerScale;

        float mViewpointX, mViewpointY, mViewpointZ, mViewpointF;

        bool CheckFinish();
        void SetFinish();
        bool mbFinishRequested;
        bool mbFinished;
        std::mutex mMutexFinish;

        bool mbStopped;
        bool mbStopRequested;
        std::mutex mMutexStop;

        bool mbStopTrack;
    };

}

#endif // VIEWER_H
