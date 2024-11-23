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

#include "Viewer.h"
#include <pangolin/pangolin.h>

#include <mutex>

namespace ORB_SLAM3
{

    // 查看器的构造函数
    Viewer::Viewer(System *pSystem, FrameDrawer *pFrameDrawer, MapDrawer *pMapDrawer, Tracking *pTracking, const string &strSettingPath, Settings *settings) : both(false), mpSystem(pSystem), mpFrameDrawer(pFrameDrawer), mpMapDrawer(pMapDrawer), mpTracker(pTracking),
                                                                                                                                                               mbFinishRequested(false), mbFinished(true), mbStopped(true), mbStopRequested(false)
    {
        if (settings)
        {
            newParameterLoader(settings);
        }
        else
        {

            cv::FileStorage fSettings(strSettingPath, cv::FileStorage::READ);

            bool is_correct = ParseViewerParamFile(fSettings);

            if (!is_correct)
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

        mbStopTrack = false;
    }

    void Viewer::newParameterLoader(Settings *settings)
    {
        mImageViewerScale = 1.f;

        float fps = settings->fps();
        if (fps < 1)
            fps = 30;
        mT = 1e3 / fps;

        cv::Size imSize = settings->newImSize();
        mImageHeight = imSize.height;
        mImageWidth = imSize.width;

        mImageViewerScale = settings->imageViewerScale();
        mViewpointX = settings->viewPointX();
        mViewpointY = settings->viewPointY();
        mViewpointZ = settings->viewPointZ();
        mViewpointF = settings->viewPointF();
    }

    bool Viewer::ParseViewerParamFile(cv::FileStorage &fSettings)
    {
        bool b_miss_params = false;
        mImageViewerScale = 1.f;

        float fps = fSettings["Camera.fps"];
        if (fps < 1)
            fps = 30;
        mT = 1e3 / fps;

        cv::FileNode node = fSettings["Camera.width"];
        if (!node.empty())
        {
            mImageWidth = node.real();
        }
        else
        {
            std::cerr << "*Camera.width parameter doesn't exist or is not a real number*" << std::endl;
            b_miss_params = true;
        }

        node = fSettings["Camera.height"];
        if (!node.empty())
        {
            mImageHeight = node.real();
        }
        else
        {
            std::cerr << "*Camera.height parameter doesn't exist or is not a real number*" << std::endl;
            b_miss_params = true;
        }

        node = fSettings["Viewer.imageViewScale"];
        if (!node.empty())
        {
            mImageViewerScale = node.real();
        }

        node = fSettings["Viewer.ViewpointX"];
        if (!node.empty())
        {
            mViewpointX = node.real();
        }
        else
        {
            std::cerr << "*Viewer.ViewpointX parameter doesn't exist or is not a real number*" << std::endl;
            b_miss_params = true;
        }

        node = fSettings["Viewer.ViewpointY"];
        if (!node.empty())
        {
            mViewpointY = node.real();
        }
        else
        {
            std::cerr << "*Viewer.ViewpointY parameter doesn't exist or is not a real number*" << std::endl;
            b_miss_params = true;
        }

        node = fSettings["Viewer.ViewpointZ"];
        if (!node.empty())
        {
            mViewpointZ = node.real();
        }
        else
        {
            std::cerr << "*Viewer.ViewpointZ parameter doesn't exist or is not a real number*" << std::endl;
            b_miss_params = true;
        }

        node = fSettings["Viewer.ViewpointF"];
        if (!node.empty())
        {
            mViewpointF = node.real();
        }
        else
        {
            std::cerr << "*Viewer.ViewpointF parameter doesn't exist or is not a real number*" << std::endl;
            b_miss_params = true;
        }

        return !b_miss_params;
    }

    // note：主函数！查看器的主进程看来是外部函数所调用的
    // pangolin库 的文档：http://docs.ros.org/fuerte/api/pangolin_wrapper/html/namespacepangolin.html
    void Viewer::Run()
    {
        // 这个变量配合 SetFinish 函数用于指示该函数是否执行完毕
        mbFinished = false;
        mbStopped = false;

        pangolin::CreateWindowAndBind("ORB-SLAM3-GHR: Map Viewer", 1024, 768);

        // 启动深度测试，OpenGL 只绘制最前面的一层，绘制时检查当前像素前面是否有别的像素，如果别的像素挡住了它，那它就不会绘制
        glEnable(GL_DEPTH_TEST);

        // 在OpenGL中使用颜色混合
        glEnable(GL_BLEND);
        // 选择混合选项
        glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

        // 新建按钮和选择框，第一个参数为按钮的名字，第二个为默认状态，第三个为是否有选择框。
        pangolin::CreatePanel("menu").SetBounds(0.0, 1.0, 0.0, pangolin::Attach::Pix(175));
        pangolin::Var<bool> menuFollowCamera("menu.Follow Camera", false, true);
        pangolin::Var<bool> menuCamView("menu.Camera View", false, false);
        pangolin::Var<bool> menuTopView("menu.Top View", false, false);
        // pangolin::Var<bool> menuSideView("menu.Side View",false,false);
        pangolin::Var<bool> menuShowPoints("menu.Show Points", true, true);
        pangolin::Var<bool> menuShowKeyFrames("menu.Show KeyFrames", true, true);
        pangolin::Var<bool> menuShowGraph("menu.Show Graph", false, true);
        pangolin::Var<bool> menuShowInertialGraph("menu.Show Inertial Graph", true, true);
        pangolin::Var<bool> menuLocalizationMode("menu.Localization Mode", false, true);
        pangolin::Var<bool> menuReset("menu.Reset", false, false);
        pangolin::Var<bool> menuStop("menu.Stop", false, false);
        pangolin::Var<bool> menuStepByStep("menu.Step By Step", false, true); // false, true
        pangolin::Var<bool> menuStep("menu.Step", false, false);

        pangolin::Var<bool> menuShowOptLba("menu.Show LBA opt", false, true);

        // Define Camera Render Object (for view / scene browsing)
        // 定义相机投影模型：ProjectionMatrix(w, h, fu, fv, u0, v0, zNear, zFar)
        // 定义观测方位向量：观测点位置：(mViewpointX mViewpointY mViewpointZ)
        //                观测目标位置：(0, 0, 0)
        //                观测的方位向量：(0.0,-1.0, 0.0)
        pangolin::OpenGlRenderState s_cam(
            pangolin::ProjectionMatrix(1024, 768, mViewpointF, mViewpointF, 512, 389, 0.1, 1000),
            pangolin::ModelViewLookAt(mViewpointX, mViewpointY, mViewpointZ, 0, 0, 0, 0.0, -1.0, 0.0));

        // Add named OpenGL viewport to window and provide 3D Handler
        // 定义显示面板大小，orbslam中有左右两个面板，昨天显示一些按钮，右边显示图形
        // 前两个参数（0.0, 1.0）表明宽度和面板纵向宽度和窗口大小相同
        // 中间两个参数（pangolin::Attach::Pix(175), 1.0）表明右边所有部分用于显示图形
        // 最后一个参数（-1024.0f/768.0f）为显示长宽比
        pangolin::View &d_cam = pangolin::CreateDisplay()
                                    .SetBounds(0.0, 1.0, pangolin::Attach::Pix(175), 1.0, -1024.0f / 768.0f)
                                    .SetHandler(new pangolin::Handler3D(s_cam));

        // 创建一个欧式变换矩阵，存储当前的相机位姿
        pangolin::OpenGlMatrix Twc, Twr;
        Twc.SetIdentity();
        pangolin::OpenGlMatrix Ow; // Oriented with g in the z axis
        Ow.SetIdentity();
        // 创建当前帧图像查看器，谢晓佳在泡泡机器人的第35课中讲过这个；需要先声明窗口，再创建；否则就容易出现窗口无法刷新的情况
        cv::namedWindow("ORB-SLAM3-GHR: Current Frame");

        // ui设置
        bool bFollow = true;
        bool bLocalizationMode = false;
        bool bStepByStep = false;
        bool bCameraView = true;

        if (mpTracker->mSensor == mpSystem->MONOCULAR || mpTracker->mSensor == mpSystem->STEREO || mpTracker->mSensor == mpSystem->RGBD)
        {
            menuShowGraph = true;
        }

        float trackedImageScale = mpTracker->GetImageScale(); // ? 为什么要追踪尺度？

        cout << "------------------------------Starting the Viewer------------------------------ " << endl;

        // 更新绘制的内容
        while (1)
        {
            // 清除缓冲区中的当前可写的颜色缓冲 和 深度缓冲
            glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

            // step1：得到最新的相机位姿
            mpMapDrawer->GetCurrentOpenGLCameraMatrix(Twc, Ow);

            if (mbStopTrack)
            {
                menuStepByStep = true;
                mbStopTrack = false;
            }

            // step2：根据相机的位姿调整视角
            // menuFollowCamera 为按钮的状态，bFollow 为真实的状态
            // case1： 当之前也在跟踪相机时
            if (menuFollowCamera && bFollow)
            {
                if (bCameraView)
                    // 当之前也在跟踪相机时
                    s_cam.Follow(Twc);
                else
                    s_cam.Follow(Ow);
            }
            // case2：当之前没有在跟踪相机时
            else if (menuFollowCamera && !bFollow)
            {
                if (bCameraView)
                {
                    s_cam.SetProjectionMatrix(pangolin::ProjectionMatrix(1024, 768, mViewpointF, mViewpointF, 512, 389, 0.1, 1000));
                    s_cam.SetModelViewMatrix(pangolin::ModelViewLookAt(mViewpointX, mViewpointY, mViewpointZ, 0, 0, 0, 0.0, -1.0, 0.0));
                    s_cam.Follow(Twc);
                }
                else
                {
                    s_cam.SetProjectionMatrix(pangolin::ProjectionMatrix(1024, 768, 3000, 3000, 512, 389, 0.1, 1000));
                    s_cam.SetModelViewMatrix(pangolin::ModelViewLookAt(0, 0.01, 10, 0, 0, 0, 0.0, 0.0, 1.0));
                    s_cam.Follow(Ow);
                }
                bFollow = true;
            }
            // case3：之前跟踪相机,但是现在菜单命令不要跟踪相机时
            else if (!menuFollowCamera && bFollow)
            {
                bFollow = false;
            }

            if (menuCamView)
            {
                menuCamView = false;
                bCameraView = true;
                s_cam.SetProjectionMatrix(pangolin::ProjectionMatrix(1024, 768, mViewpointF, mViewpointF, 512, 389, 0.1, 10000));
                s_cam.SetModelViewMatrix(pangolin::ModelViewLookAt(mViewpointX, mViewpointY, mViewpointZ, 0, 0, 0, 0.0, -1.0, 0.0));
                s_cam.Follow(Twc);
            }

            if (menuTopView && mpMapDrawer->mpAtlas->isImuInitialized())
            {
                menuTopView = false;
                bCameraView = false;
                s_cam.SetProjectionMatrix(pangolin::ProjectionMatrix(1024, 768, 3000, 3000, 512, 389, 0.1, 10000));
                s_cam.SetModelViewMatrix(pangolin::ModelViewLookAt(0, 0.01, 50, 0, 0, 0, 0.0, 0.0, 1.0));
                s_cam.Follow(Ow);
            }

            // 更新定位模式，或者是 SLAM 模式
            if (menuLocalizationMode && !bLocalizationMode)
            {
                mpSystem->ActivateLocalizationMode();
                bLocalizationMode = true;
            }
            else if (!menuLocalizationMode && bLocalizationMode)
            {
                mpSystem->DeactivateLocalizationMode();
                bLocalizationMode = false;
            }

            if (menuStepByStep && !bStepByStep)
            {
                // cout << "Viewer: step by step" << endl;
                mpTracker->SetStepByStep(true);
                bStepByStep = true;
            }
            else if (!menuStepByStep && bStepByStep)
            {
                mpTracker->SetStepByStep(false);
                bStepByStep = false;
            }

            if (menuStep)
            {
                mpTracker->mbStep = true;
                menuStep = false;
            }

            d_cam.Activate(s_cam);

            // step 3：绘制地图和图像(3D部分)
            // 设置为白色，glClearColor(red, green, blue, alpha），数值范围(0, 1)
            glClearColor(1.0f, 1.0f, 1.0f, 1.0f);
            // 绘制当前相机
            mpMapDrawer->DrawCurrentCamera(Twc);
            // 绘制关键帧和共视图
            if (menuShowKeyFrames || menuShowGraph || menuShowInertialGraph || menuShowOptLba)
                mpMapDrawer->DrawKeyFrames(menuShowKeyFrames, menuShowGraph, menuShowInertialGraph, menuShowOptLba);
            // 绘制地图点
            if (menuShowPoints)
                mpMapDrawer->DrawMapPoints();

            pangolin::FinishFrame();

            // step 4:绘制当前帧图像和特征点提取匹配结果
            cv::Mat toShow;
            cv::Mat im = mpFrameDrawer->DrawFrame(trackedImageScale);

            if (both)
            {
                cv::Mat imRight = mpFrameDrawer->DrawRightFrame(trackedImageScale);
                cv::hconcat(im, imRight, toShow);
            }
            else
            {
                toShow = im;
            }

            if (mImageViewerScale != 1.f)
            {
                int width = toShow.cols * mImageViewerScale;
                int height = toShow.rows * mImageViewerScale;
                cv::resize(toShow, toShow, cv::Size(width, height));
            }

            // TODO Yolo
            {
                // step1. 锁住资源：保证线程安全
                std::unique_lock<std::mutex> lock(mMutexPAFinsh);

                // step2. 遍历检测结果容器
                for (auto vit = mmDetectMap.begin(); vit != mmDetectMap.end(); vit++)
                {
                    if (vit->second.size() != 0)
                    {
                        for (auto area : vit->second)
                        {
                            // step3. 绘制矩形框
                            cv::rectangle(toShow,                // 待显示的图像
                                          area,                  // 检测区域，cv::Rect 类型，定义矩形框的位置和大小
                                          cv::Scalar(0, 255, 0), // 矩形框的颜色，表示绿色（BGR 格式）
                                          1);                    // 矩形框的线条宽度
                            // step4. 在矩形框上方添加文本
                            cv::putText(toShow,
                                        vit->first,                // 类别名称
                                        cv::Point(area.x, area.y), // 文本起始位置，设置为矩形框的左上角
                                        cv::FONT_HERSHEY_SIMPLEX,  // 文本字体
                                        1,                         // 字体大小
                                        cv::Scalar(0, 0, 0),       // 文本颜色，黑色（BGR 格式）
                                        2);                        // 文本粗细
                        }
                    }
                }
            }

            // {
            //     std::unique_lock<std::mutex> lock(mMutexPAFinsh);

            //     for (const auto &[className, areas] : mmDetectMap)
            //     {
            //         if (!areas.empty())
            //         {
            //             for (const auto &area : areas)
            //             {
            //                 // 绘制矩形框
            //                 cv::rectangle(toShow, area, cv::Scalar(0, 255, 0), 1);
            //                 // 绘制类别名称
            //                 cv::putText(toShow, className, cv::Point(area.x, area.y), cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(0, 0, 0), 2);
            //             }
            //         }
            //     }
            // }

            cv::imshow("ORB-SLAM3-GHR: Current Frame", toShow);
            // NOTICE 注意对于我所遇到的问题，ORB-SLAM2是这样子来处理的
            cv::waitKey(mT);

            // step 5 相应其他请求，如复位按钮、停止按钮
            if (menuReset)
            {
                // 将所有的GUI控件恢复初始状态
                menuShowGraph = true;
                menuShowInertialGraph = true;
                menuShowKeyFrames = true;
                menuShowPoints = true;
                menuLocalizationMode = false;
                if (bLocalizationMode)
                    mpSystem->DeactivateLocalizationMode();

                // 相关变量也恢复到初始状态
                bLocalizationMode = false;
                bFollow = true;
                menuFollowCamera = true;
                // 告知系统复位
                mpSystem->ResetActiveMap();
                // 按钮本身状态复位
                menuReset = false;
            }

            if (menuStop)
            {
                if (bLocalizationMode)
                    mpSystem->DeactivateLocalizationMode();

                // Stop all threads
                mpSystem->Shutdown();

                // Save camera trajectory
                mpSystem->SaveTrajectoryEuRoC("CameraTrajectory.txt");
                mpSystem->SaveKeyFrameTrajectoryEuRoC("KeyFrameTrajectory.txt");
                menuStop = false;
            }

            // 如果有停止更新的请求
            if (Stop())
            {
                // 就不再绘图了，并且在这里每隔三秒检查一下是否结束
                while (isStopped())
                {
                    // usleep(3000);
                    std::this_thread::sleep_for(std::chrono::milliseconds(3));
                }
            }

            // 满足的时候退出这个线程循环，这里应该是查看终止请求
            if (CheckFinish())
                break;
        }

        // 终止查看器，主要是设置状态，执行完成退出这个函数后，查看器进程就已经被销毁了
        SetFinish();
    }

    void Viewer::RequestFinish()
    {
        unique_lock<mutex> lock(mMutexFinish);
        mbFinishRequested = true;
    }

    bool Viewer::CheckFinish()
    {
        unique_lock<mutex> lock(mMutexFinish);
        return mbFinishRequested;
    }

    void Viewer::SetFinish()
    {
        unique_lock<mutex> lock(mMutexFinish);
        mbFinished = true;
    }

    bool Viewer::isFinished()
    {
        unique_lock<mutex> lock(mMutexFinish);
        return mbFinished;
    }

    void Viewer::RequestStop()
    {
        unique_lock<mutex> lock(mMutexStop);
        if (!mbStopped)
            mbStopRequested = true;
    }

    bool Viewer::isStopped()
    {
        unique_lock<mutex> lock(mMutexStop);
        return mbStopped;
    }

    bool Viewer::Stop()
    {
        unique_lock<mutex> lock(mMutexStop);
        unique_lock<mutex> lock2(mMutexFinish);

        if (mbFinishRequested)
            return false;
        else if (mbStopRequested)
        {
            mbStopped = true;
            mbStopRequested = false;
            return true;
        }

        return false;
    }

    void Viewer::Release()
    {
        unique_lock<mutex> lock(mMutexStop);
        mbStopped = false;
    }

    /*void Viewer::SetTrackingPause()
    {
        mbStopTrack = true;
    }*/
}
