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

#include "System.h"
#include "Converter.h"
#include <thread>
#include <pangolin/pangolin.h>
#include <iomanip>
#include <openssl/md5.h>
#include <boost/serialization/base_object.hpp>
#include <boost/serialization/string.hpp>
#include <boost/archive/text_iarchive.hpp>
#include <boost/archive/text_oarchive.hpp>
#include <boost/archive/binary_iarchive.hpp>
#include <boost/archive/binary_oarchive.hpp>
#include <boost/archive/xml_iarchive.hpp>
#include <boost/archive/xml_oarchive.hpp>

// TODO
#include "YoloDetect.h"

namespace ORB_SLAM3
{

    Verbose::eLevel Verbose::th = Verbose::VERBOSITY_NORMAL;

    // PS 作用：系统的构造函数，将会启动其他的线程，灰常重要嗷！！！
    System::System(const string &strVocFile,                                           // 词袋文件所在路径
                   const string &strSettingsFile,                                      // 配置文件所在路径
                   const eSensor sensor,                                               // 传感器类型
                   const bool bUseViewer,                                              // 是否使用可视化界面
                   const int initFr,                                                   // initFr 表示初始化帧的 id, 开始设置为 0
                   const string &strSequence) :                                        // 序列名，在【跟踪线程】和【局部建图线程】用得到
                                                mSensor(sensor),                       // 初始化传感器类型
                                                mpViewer(static_cast<Viewer *>(NULL)), // TODO 空。。。对象指针？
                                                mbReset(false),                        // 无复位标志
                                                mbResetActiveMap(false),
                                                mbActivateLocalizationMode(false),   // 没有这个模式转换标志
                                                mbDeactivateLocalizationMode(false), // 没有这个模式转换标志
                                                mbShutDown(false)
    {
        // Output welcome message
        cout << endl
             << "ORB-SLAM3 Copyright (C) 2017-2020 Carlos Campos, Richard Elvira, Juan J. Gómez, José M.M. Montiel and Juan D. Tardós, University of Zaragoza." << endl
             << "ORB-SLAM2 Copyright (C) 2014-2016 Raúl Mur-Artal, José M.M. Montiel and Juan D. Tardós, University of Zaragoza." << endl
             << "This program comes with ABSOLUTELY NO WARRANTY;" << endl
             << "This is free software, and you are welcome to redistribute it" << endl
             << "under certain conditions. See LICENSE.txt." << endl
             << endl;

        cout << "Input sensor was set to: ";

        if (mSensor == MONOCULAR)
            cout << "Monocular" << endl; // 单目
        else if (mSensor == STEREO)
            cout << "Stereo" << endl; // 双目
        else if (mSensor == RGBD)
            cout << "RGB-D" << endl; // RGBD相机
        else if (mSensor == IMU_MONOCULAR)
            cout << "Monocular-Inertial" << endl; // 单目 + imu
        else if (mSensor == IMU_STEREO)
            cout << "Stereo-Inertial" << endl; // 双目 + imu
        else if (mSensor == IMU_RGBD)
            cout << "RGB-D-Inertial" << endl; // RGBD相机 + imu

        //  Step 2 读取配置文件（Check settings file）
        cv::FileStorage fsSettings(strSettingsFile.c_str(), cv::FileStorage::READ);

        // 如果打开失败，就输出错误信息
        if (!fsSettings.isOpened())
        {
            cerr << "Failed to open settings file at: " << strSettingsFile << endl;
            exit(-1);
        }

        // 查看配置文件版本，不同版本有不同处理方法
        cv::FileNode node = fsSettings["File.version"]; // FileNode 是 OpenCV 中的类，表示一个 YAML 或 XML 文件的节点。用于获取文件中的具体参数。

        // case1：配置文件是 1.0 版本
        if (!node.empty() && node.isString() && node.string() == "1.0")
        {
            // 创建一个 Settings 对象，并将配置文件的路径和传感器类型（mSensor）传递给它。
            settings_ = new Settings(strSettingsFile, mSensor);

            // 保存及加载地图的名字
            mStrLoadAtlasFromFile = settings_->atlasLoadFile();
            mStrSaveAtlasToFile = settings_->atlasSaveFile();

            // ps：这里是为了输出配置文件的相关参数，方便调试或检查设置。
            cout << (*settings_) << endl;
        }
        // case2：配置文件不是 1.0 版本，大多都是 1.0 版本的，所以这个略过即可。
        else
        {
            settings_ = nullptr;
            cv::FileNode node = fsSettings["System.LoadAtlasFromFile"];
            if (!node.empty() && node.isString())
            {
                mStrLoadAtlasFromFile = (string)node;
            }

            node = fsSettings["System.SaveAtlasToFile"];
            if (!node.empty() && node.isString())
            {
                mStrSaveAtlasToFile = (string)node;
            }
        }

        // 是否激活回环，默认是开着的
        node = fsSettings["loopClosing"];
        bool activeLC = true;
        if (!node.empty())
        {
            // 如果配置文件中没有定义 loopClosing 键，则回环检测功能默认为激活状态
            activeLC = static_cast<int>(fsSettings["loopClosing"]) != 0;
        }

        // 初始化词汇表文件路径
        mStrVocabularyFilePath = strVocFile;

        bool loadedAtlas = false;

        // case1：如果 mStrLoadAtlasFromFile 是空的，表示没有提供先验的 Atlas 文件路径，系统需要从头创建一个新的 Atlas。
        if (mStrLoadAtlasFromFile.empty())
        {
            // Step 3 加载 ORB 字典
            cout << endl
                 << "Loading ORB Vocabulary. This could take a while..." << endl;

            mpVocabulary = new ORBVocabulary();
            bool bVocLoad = mpVocabulary->loadFromTextFile(strVocFile);
            if (!bVocLoad)
            {
                cerr << "Wrong path to vocabulary. " << endl;
                cerr << "Falied to open at: " << strVocFile << endl;
                exit(-1);
            }

            std::cout << "Vocabulary loaded!" << std::endl
                      << std::endl;

            //  Step 4 创建关键帧数据库
            mpKeyFrameDatabase = new KeyFrameDatabase(*mpVocabulary);

            //  Step 5 创建多地图，参数 0 表示初始化关键帧 id 为 0
            // 如果没有提供 Atlas 文件路径：创建新 Atlas
            cout << "Initialization of Atlas from scratch " << endl;
            mpAtlas = new Atlas(0); // 构造地图集 Atlas 类，参数 0 表示初始化关键帧 ID 为 0
        }

        // case2：如果路径不为空，则尝试从指定的文件中加载 Atlas。一般就是从头开始新建地图吧！【所以下面代码不用看】
        else
        {
            // Load ORB Vocabulary
            cout << endl
                 << "Loading ORB Vocabulary. This could take a while..." << endl;

            mpVocabulary = new ORBVocabulary();
            bool bVocLoad = mpVocabulary->loadFromTextFile(strVocFile);
            if (!bVocLoad)
            {
                cerr << "Wrong path to vocabulary. " << endl;
                cerr << "Falied to open at: " << strVocFile << endl;
                exit(-1);
            }

            std::cout << "Vocabulary loaded!" << std::endl
                      << std::endl;

            // Create KeyFrame Database
            mpKeyFrameDatabase = new KeyFrameDatabase(*mpVocabulary);

            cout << "Load File" << endl;

            // Load the file with an earlier session
            // clock_t start = clock();
            cout << "Initialization of Atlas from file: " << mStrLoadAtlasFromFile << endl;
            bool isRead = LoadAtlas(FileType::BINARY_FILE);

            if (!isRead)
            {
                cout << "Error to load the file, please try with other session file or vocabulary file" << endl;
                exit(-1);
            }
            // mpKeyFrameDatabase = new KeyFrameDatabase(*mpVocabulary);

            // cout << "KF in DB: " << mpKeyFrameDatabase->mnNumKFs << "; words: " << mpKeyFrameDatabase->mnNumWords << endl;

            loadedAtlas = true;

            mpAtlas->CreateNewMap();

            // clock_t timeElapsed = clock() - start;
            // unsigned msElapsed = timeElapsed / (CLOCKS_PER_SEC / 1000);
            // cout << "Binary file read in " << msElapsed << " ms" << endl;

            // usleep(10*1000*1000);
        }

        // 如果是有 imu 的传感器类型，设置 mbIsInertial = true; 以后的【跟踪】和【预积分】将和这个标志有关。
        // 根据传感器类型（mSensor）决定是否调用 mpAtlas->SetInertialSensor() 方法，目的是在地图（Atlas）中启用惯性传感器的支持。
        // ps：这在使用 IMU 数据的 SLAM 系统中是常见的设置！
        if (mSensor == IMU_STEREO || mSensor == IMU_MONOCULAR || mSensor == IMU_RGBD)
        {
            mpAtlas->SetInertialSensor();
            std::cout << "Inertial sensor support enabled in Atlas." << std::endl; // 调试用
        }
        // 扩展：如果未来需要支持更多类型的 IMU 或传感器，可以将条件判断改为更灵活的结构。例如，定义一个通用的标志位：
        // bool isInertialSensor = (mSensor == IMU_STEREO || mSensor == IMU_MONOCULAR || mSensor == IMU_RGBD);
        // if (isInertialSensor)
        // {
        //     mpAtlas->SetInertialSensor();
        // }

        // TODO：Yolo
        mpDetector = new YoloDetection();

        // Step 6 依次创建跟踪、局部建图、闭环、显示线程
        // 创建用于显示帧和地图的类，由 Viewer 调用
        mpFrameDrawer = new FrameDrawer(mpAtlas); // new 后面要跟类名，表示创建一个类的对象
        mpMapDrawer = new MapDrawer(mpAtlas, strSettingsFile, settings_);

        // ps1：初始化跟踪线程（主线程），不会立刻开启，会在对图像和 imu 预处理后在 main 主线程中执行
        cout << "Seq. Name: " << strSequence << endl;
        mpTracker = new Tracking(this,                    // 表示当前 SLAM 系统实例，允许 Tracking 访问整个系统
                                 mpVocabulary,            // 汇表，用于特征点描述符的匹配
                                 mpFrameDrawer,           // 负责当前帧的可视化
                                 mpMapDrawer,             // 负责地图的可视化
                                 mpAtlas,                 // 地图管理模块，保存关键帧和地图点
                                 mpKeyFrameDatabase,      // 关键帧数据库，用于回环检测
                                 strSettingsFile,         // 配置文件路径
                                 mSensor,                 // 传感器类型（如单目、立体、RGB-D）
                                 settings_, strSequence); // 其他配置信息

        // TODO Yolo 句柄
        mpTracker->SetDetector(mpDetector);

        // TODO 点云
        mpPointCloudMapper = new PointCloudMapper();
        mpTracker->mpPointCloudMapper = mpPointCloudMapper; // 如果启用，这一步将 YOLO 检测器与跟踪线程关联，使其支持目标检测与跟踪任务
        //  TODO 创建并启动点云处理线程
        mptPointCloudMapping = new thread(&PointCloudMapper::Run, mpTracker->mpPointCloudMapper); // 将点云处理器 mpPointCloudMapper 关联到跟踪线程 mpTracker，便于跟踪模块调用点云相关功能

        // ps2：创建并启动局部建图线程（Local Mapping）
        mpLocalMapper = new LocalMapping(this,                                                                     // 当前 SLAM 系统实例
                                         mpAtlas,                                                                  // 地图管理模块
                                         mSensor == MONOCULAR || mSensor == IMU_MONOCULAR,                         // 是否为单目传感器
                                         mSensor == IMU_MONOCULAR || mSensor == IMU_STEREO || mSensor == IMU_RGBD, // 是否支持惯性传感器
                                         strSequence);                                                             // 序列名称
        mptLocalMapping = new thread(&ORB_SLAM3::LocalMapping::Run, mpLocalMapper);
        mpLocalMapper->mInitFr = initFr;

        // 设置最远 3D 地图点的深度值，如果超过阈值，说明可能三角化不太准确，丢弃
        // 即：设置一个阈值 mThFarPoints，用于过滤那些距离相机超过指定范围的 3D 地图点
        // ? 这里有个疑问，settings_ 和 fsSettings 访问 thFarPoints 变量的值有什么不一样，不都是读取的一个配置文件（手动传入命令行的那个）？
        if (settings_)
            mpLocalMapper->mThFarPoints = settings_->thFarPoints();
        else
            mpLocalMapper->mThFarPoints = fsSettings["thFarPoints"];

        // ? 这里有个疑问，C++ 中浮点型跟 0 比较是否用精确?
        // if (mpLocalMapper->mThFarPoints != 0)
        // {
        //     cout << "Discard points further than " << mpLocalMapper->mThFarPoints << " m from current camera" << endl;
        //     mpLocalMapper->mbFarPoints = true;
        // }
        // else
        //     mpLocalMapper->mbFarPoints = false;

        // ! 如果 ThFarPoints 是 int 类型，这里可以直接与 0 作比较，如果是 float 建议采用 epsion 方法
        const double epsion = 1e-6;
        if (std::abs(mpLocalMapper->mThFarPoints) > epsion)
        {
            // 深度阈值不为 0 -- 表示剔除远点，也就是启用剔除逻辑
            std::cout << "Discard points further than " << mpLocalMapper->mThFarPoints << " m from current camera!" << std::endl;
            mpLocalMapper->mbFarPoints = true;
        }
        else
        {
            // 深度阈值为 0 -- 表示不剔除任何点，不就是禁用剔除逻辑
            mpLocalMapper->mbFarPoints = false;
        }

        // ps3：创建并启动回环检测线程（Loop Closing）
        // mSensor != MONOCULAR && mSensor != IMU_MONOCULAR
        mpLoopCloser = new LoopClosing(mpAtlas,              // 地图管理模块
                                       mpKeyFrameDatabase,   // 关键帧数据库，用于高效回环检测
                                       mpVocabulary,         // 词汇表，用于回环检测中的关键帧相似性计算
                                       mSensor != MONOCULAR, // 是否使用立体或 RGB-D 传感器（单目通常不执行回环检测）
                                       activeLC);            // 是否启用回环检测
        mptLoopClosing = new thread(&ORB_SLAM3::LoopClosing::Run, mpLoopCloser);

        // 设置线程间的指针 --- 作用：建立各模块之间的关联，使它们能够协同工作。
        mpTracker->SetLocalMapper(mpLocalMapper);
        mpTracker->SetLoopClosing(mpLoopCloser);

        mpLocalMapper->SetTracker(mpTracker);
        mpLocalMapper->SetLoopCloser(mpLoopCloser);

        mpLoopCloser->SetTracker(mpTracker);
        mpLoopCloser->SetLocalMapper(mpLocalMapper);

        // usleep(10*1000*1000);

        // ps4：如果启用了 Viewer，则创建并启动可视化线程
        if (bUseViewer)
        // if(false) // TODO
        {
            mpViewer = new Viewer(this, mpFrameDrawer, mpMapDrawer, mpTracker, strSettingsFile, settings_);
            mptViewer = new thread(&Viewer::Run, mpViewer);
            mpTracker->SetViewer(mpViewer);
            mpLoopCloser->mpViewer = mpViewer;
            mpViewer->both = mpFrameDrawer->both;
        }

        // 打印输出中间的信息，设置为安静模式（Fix verbosity)
        Verbose::SetTh(Verbose::VERBOSITY_QUIET);
    }

    // --------------------------------------Track---------------------------------------------

    // TODO1 双目输入时的追踪器接口---将左、右相机的图像输入到系统中，完成图像预处理、IMU 数据处理、系统状态检查，并最终调用跟踪模块处理输入数据
    // 返回：Sophus::SE3f --- 当前帧的相机位姿，描述相机在世界坐标系中的旋转和平移
    Sophus::SE3f System::TrackStereo(const cv::Mat &imLeft, const cv::Mat &imRight, // 左右相机的图像帧
                                     const double &timestamp,                       // 当前图像帧的时间戳，用于同步 IMU 和图像数据
                                     const vector<IMU::Point> &vImuMeas,            // 当前帧的 IMU 数据列表
                                     string filename)                               // 当前图像帧对应的文件名
    {
        // step1.检查传感器模式
        if (mSensor != STEREO && mSensor != IMU_STEREO)
        {
            cerr << "ERROR: you called TrackStereo but input sensor was not set to Stereo nor Stereo-Inertial." << endl;
            exit(-1);
        }

        // step2. 图像预处理
        cv::Mat imLeftToFeed, imRightToFeed;

        // case1：若需要畸变校正
        if (settings_ && settings_->needToRectify())
        {
            cv::Mat M1l = settings_->M1l();
            cv::Mat M2l = settings_->M2l();
            cv::Mat M1r = settings_->M1r();
            cv::Mat M2r = settings_->M2r();

            cv::remap(imLeft, imLeftToFeed, M1l, M2l, cv::INTER_LINEAR);
            cv::remap(imRight, imRightToFeed, M1r, M2r, cv::INTER_LINEAR);
        }
        // case2：若需要尺寸调整
        else if (settings_ && settings_->needToResize())
        {
            cv::resize(imLeft, imLeftToFeed, settings_->newImSize());
            cv::resize(imRight, imRightToFeed, settings_->newImSize());
        }
        // case3：若不需要任何处理
        else
        {
            imLeftToFeed = imLeft.clone();
            imRightToFeed = imRight.clone();
        }

        // step3. 检查是否有运行模式的动态改变
        {
            // TODO 锁住这个变量？防止其他的线程对它的更改？
            // 独占锁，主要是为了 mbActivateLocalizationMode 和 mbDeactivateLocalizationMode 不会发生混乱
            unique_lock<mutex> lock(mMutexMode);

            // case1：仅定位模式
            if (mbActivateLocalizationMode)
            {
                // 调用局部建图器的请求停止函数
                mpLocalMapper->RequestStop();

                // Wait until Local Mapping has effectively stopped
                while (!mpLocalMapper->isStopped())
                {
                    usleep(1000);
                }

                // 运行到这里的时候，局部建图部分就真正地停止了。告知追踪器，现在 仅进行跟踪而不更新地图
                // 局部地图关闭以后，只进行追踪的线程，只计算相机的位姿，没有对局部地图进行更新
                mpTracker->InformOnlyTracking(true); // 定位时，只跟踪
                // 关闭线程可以使得别的线程得到更多的资源
                mbActivateLocalizationMode = false; // 同时清除定位标记，防止重复执行
            }
            // case2：SLAM 模式
            if (mbDeactivateLocalizationMode)
            {
                // 如果取消定位模式。告知追踪器，现在地图构建部分也要开始工作了！
                mpTracker->InformOnlyTracking(false);
                // 重新启动局部建图线程
                mpLocalMapper->Release();
                mbDeactivateLocalizationMode = false; // 清除标志，防止重复执行
            }
        }

        // step4. 检查是否需要重置
        {
            unique_lock<mutex> lock(mMutexReset);

            // * 通过检查变量 mbReset 和 mbResetActiveMap，决定是否执行全局或局部重置
            // case1：全局重置！调用 mpTracker->Reset()，清除所有跟踪状态和地图
            if (mbReset)
            {
                // 追踪器复位
                mpTracker->Reset();
                // 清除标志
                mbReset = false;
                mbResetActiveMap = false;
            }

            // case2：局部重置！调用 mpTracker->ResetActiveMap()，只清除当前活动地图，保留系统状态
            else if (mbResetActiveMap)
            {
                mpTracker->ResetActiveMap();
                mbResetActiveMap = false;
            }
        }

        // step5. 处理 IMU 数据
        if (mSensor == System::IMU_STEREO)
            for (size_t i_imu = 0; i_imu < vImuMeas.size(); i_imu++)
                mpTracker->GrabImuData(vImuMeas[i_imu]);

        std::cout << "start GrabImageStereo" << std::endl;

        // step6. 主跟踪逻辑。用矩阵 Tcw 来保存估计的相机 位姿，运动追踪器的【GrabImageStereo函数】才是真正进行运动估计的函数！
        Sophus::SE3f Tcw = mpTracker->GrabImageStereo(imLeftToFeed, imRightToFeed, timestamp, filename);

        std::cout << "out grabber" << std::endl;

        // step7. 更新系统状态
        unique_lock<mutex> lock2(mMutexState);
        // 1、获取运动追踪状态
        mTrackingState = mpTracker->mState;
        // 2、获取当前帧追踪到的地图点向量指针
        mTrackedMapPoints = mpTracker->mCurrentFrame.mvpMapPoints;
        // 3、获取当前帧追踪到的关键帧特征点向量的指针
        mTrackedKeyPointsUn = mpTracker->mCurrentFrame.mvKeysUn;

        // 返回获得的相机运动估计
        return Tcw;
    }

    // TODO2 当输入图像为 RGBD 时进行的追踪，参数就不在一一说明了------追踪深度相机数据，返回 mpTracker->GrabImageRGBD(im,depthmap,timestamp)
    Sophus::SE3f System::TrackRGBD(const cv::Mat &im,                  // RGB 彩色图像帧，类型为 cv::Mat，作为当前帧的输入，用于特征提取与匹配
                                   const cv::Mat &depthmap,            // 深度图帧，类型为 cv::Mat，包含像素的深度信息（通常单位为米）
                                   const double &timestamp,            // 当前帧的时间戳，用于与 IMU 数据同步
                                   const vector<IMU::Point> &vImuMeas, // 当前帧的 IMU 数据列表，类型为 vector<IMU::Point>
                                   string filename)                    // 当前帧的图像文件名，可能用于调试或日志记录
    {
        // step1. 检查传感器模式
        if (mSensor != RGBD && mSensor != IMU_RGBD)
        {
            cerr << "ERROR: you called TrackRGBD but input sensor was not set to RGBD." << endl;
            exit(-1);
        }

        // step2. 图像预处理
        cv::Mat imToFeed = im.clone();
        cv::Mat imDepthToFeed = depthmap.clone();

        if (settings_ && settings_->needToResize())
        {
            // cv::Mat resizedIm;
            // cv::resize(im, resizedIm, settings_->newImSize());
            // imToFeed = resizedIm;

            cv::resize(im, imToFeed, settings_->newImSize()); // 上面太过冗余，中间变量 resizedIm 没必要阿！

            cv::resize(depthmap, imDepthToFeed, settings_->newImSize());
        }

        // step3. 检查运行模式的动态切换
        {
            unique_lock<mutex> lock(mMutexMode);

            if (mbActivateLocalizationMode)
            {
                mpLocalMapper->RequestStop();

                // Wait until Local Mapping has effectively stopped
                while (!mpLocalMapper->isStopped())
                {
                    usleep(1000);
                }

                mpTracker->InformOnlyTracking(true);
                mbActivateLocalizationMode = false;
            }
            if (mbDeactivateLocalizationMode)
            {
                mpTracker->InformOnlyTracking(false);
                mpLocalMapper->Release();
                mbDeactivateLocalizationMode = false;
            }
        }

        // step4. 检查是否需要重置
        {
            unique_lock<mutex> lock(mMutexReset);

            if (mbReset)
            {
                mpTracker->Reset();
                mbReset = false;
                mbResetActiveMap = false;
            }
            else if (mbResetActiveMap)
            {
                mpTracker->ResetActiveMap();
                mbResetActiveMap = false;
            }
        }

        // step5. 处理 IMU 数据
        if (mSensor == System::IMU_RGBD)
            for (size_t i_imu = 0; i_imu < vImuMeas.size(); i_imu++)
                mpTracker->GrabImuData(vImuMeas[i_imu]);

        // step6. 跟踪 RGB-D 数据，处理当前帧的 RGB 图像和深度图，计算当前帧的位姿 Tcw
        Sophus::SE3f Tcw = mpTracker->GrabImageRGBD(imToFeed, imDepthToFeed, timestamp, filename);

        // step7. 更新跟踪状态
        unique_lock<mutex> lock2(mMutexState);
        mTrackingState = mpTracker->mState;
        mTrackedMapPoints = mpTracker->mCurrentFrame.mvpMapPoints;
        mTrackedKeyPointsUn = mpTracker->mCurrentFrame.mvKeysUn;
        return Tcw;
    }

    // TODO3 同理，输入为单目图像时的追踪器接口------追踪单目相机数据返回 mpTracker->GrabImageMonocular(im,timestamp)
    Sophus::SE3f System::TrackMonocular(const cv::Mat &im,                  // 灰度图像
                                        const double &timestamp,            // 图像时间戳
                                        const vector<IMU::Point> &vImuMeas, // 上一帧到当前帧图像之间的IMU测量值
                                        string filename)                    // 调试用的文件名
    {

        {
            unique_lock<mutex> lock(mMutexReset);
            if (mbShutDown)
                return Sophus::SE3f();
        }

        if (mSensor != MONOCULAR && mSensor != IMU_MONOCULAR)
        {
            cerr << "ERROR: you called TrackMonocular but input sensor was not set to Monocular nor Monocular-Inertial." << endl;
            exit(-1);
        }

        cv::Mat imToFeed = im.clone();
        if (settings_ && settings_->needToResize())
        {
            // cv::Mat resizedIm;
            // cv::resize(im, resizedIm, settings_->newImSize());
            // imToFeed = resizedIm;

            cv::resize(im, imToFeed, settings_->newImSize());
        }

        {
            unique_lock<mutex> lock(mMutexMode);

            if (mbActivateLocalizationMode)
            {
                mpLocalMapper->RequestStop();

                while (!mpLocalMapper->isStopped())
                {
                    usleep(1000);
                }

                mpTracker->InformOnlyTracking(true);
                mbActivateLocalizationMode = false;
            }
            if (mbDeactivateLocalizationMode)
            {
                mpTracker->InformOnlyTracking(false);
                mpLocalMapper->Release();
                mbDeactivateLocalizationMode = false;
            }
        }

        {
            unique_lock<mutex> lock(mMutexReset);

            if (mbReset)
            {
                mpTracker->Reset();
                mbReset = false;
                mbResetActiveMap = false;
            }
            else if (mbResetActiveMap)
            {
                cout << "SYSTEM-> Reseting active map in monocular case" << endl;
                mpTracker->ResetActiveMap();
                mbResetActiveMap = false;
            }
        }

        if (mSensor == System::IMU_MONOCULAR)
            for (size_t i_imu = 0; i_imu < vImuMeas.size(); i_imu++)
                mpTracker->GrabImuData(vImuMeas[i_imu]);

        Sophus::SE3f Tcw = mpTracker->GrabImageMonocular(imToFeed, timestamp, filename);

        unique_lock<mutex> lock2(mMutexState);
        mTrackingState = mpTracker->mState;
        mTrackedMapPoints = mpTracker->mCurrentFrame.mvpMapPoints;
        mTrackedKeyPointsUn = mpTracker->mCurrentFrame.mvKeysUn;

        return Tcw;
    }

    // ---------------------------------Mode select--------------------------------------------------

    void System::ActivateLocalizationMode()
    {
        unique_lock<mutex> lock(mMutexMode);
        mbActivateLocalizationMode = true;
    }

    void System::DeactivateLocalizationMode()
    {
        unique_lock<mutex> lock(mMutexMode);
        mbDeactivateLocalizationMode = true;
    }

    // -----------------------------------------------------------------------------------

    bool System::MapChanged()
    {
        static int n = 0;
        int curn = mpAtlas->GetLastBigChangeIdx();
        if (n < curn)
        {
            n = curn;
            return true;
        }
        else
            return false;
    }

    // ------------------------------Reset Mode-----------------------------------------------------

    void System::Reset()
    {
        unique_lock<mutex> lock(mMutexReset);
        mbReset = true;
    }

    void System::ResetActiveMap()
    {
        unique_lock<mutex> lock(mMutexReset);
        mbResetActiveMap = true;
    }

    // -----------------------------------------------------------------------------------

    void System::Shutdown()
    {
        {
            unique_lock<mutex> lock(mMutexReset);
            mbShutDown = true;
        }

        cout << "Shutdown" << endl;

        mpLocalMapper->RequestFinish();
        mpLoopCloser->RequestFinish();
        /*if(mpViewer)
        {
            mpViewer->RequestFinish();
            while(!mpViewer->isFinished())
                usleep(5000);
        }*/

        // Wait until all thread have effectively stopped
        /*while(!mpLocalMapper->isFinished() || !mpLoopCloser->isFinished() || mpLoopCloser->isRunningGBA())
        {
            if(!mpLocalMapper->isFinished())
                cout << "mpLocalMapper is not finished" << endl;*/
        /*if(!mpLoopCloser->isFinished())
            cout << "mpLoopCloser is not finished" << endl;
        if(mpLoopCloser->isRunningGBA()){
            cout << "mpLoopCloser is running GBA" << endl;
            cout << "break anyway..." << endl;
            break;
        }*/
        /*usleep(5000);
    }*/

        if (!mStrSaveAtlasToFile.empty())
        {
            Verbose::PrintMess("Atlas saving to file " + mStrSaveAtlasToFile, Verbose::VERBOSITY_NORMAL);
            SaveAtlas(FileType::BINARY_FILE);
        }

        /*if(mpViewer)
            pangolin::BindToContext("ORB-SLAM2: Map Viewer");*/

#ifdef REGISTER_TIMES
        mpTracker->PrintTimeStats();
#endif
    }

    bool System::isShutDown()
    {
        unique_lock<mutex> lock(mMutexReset);
        return mbShutDown;
    }

    // ----------------------------------Save Trajectory-------------------------------------------------

    void System::SaveTrajectoryTUM(const string &filename)
    {
        cout << endl
             << "Saving camera trajectory to " << filename << " ..." << endl;
        if (mSensor == MONOCULAR)
        {
            cerr << "ERROR: SaveTrajectoryTUM cannot be used for monocular." << endl;
            return;
        }

        vector<KeyFrame *> vpKFs = mpAtlas->GetAllKeyFrames();
        sort(vpKFs.begin(), vpKFs.end(), KeyFrame::lId);

        // Transform all keyframes so that the first keyframe is at the origin.
        // After a loop closure the first keyframe might not be at the origin.
        Sophus::SE3f Two = vpKFs[0]->GetPoseInverse();

        ofstream f;
        f.open(filename.c_str());
        f << fixed;

        // Frame pose is stored relative to its reference keyframe (which is optimized by BA and pose graph).
        // We need to get first the keyframe pose and then concatenate the relative transformation.
        // Frames not localized (tracking failure) are not saved.

        // For each frame we have a reference keyframe (lRit), the timestamp (lT) and a flag
        // which is true when tracking failed (lbL).
        list<ORB_SLAM3::KeyFrame *>::iterator lRit = mpTracker->mlpReferences.begin();
        list<double>::iterator lT = mpTracker->mlFrameTimes.begin();
        list<bool>::iterator lbL = mpTracker->mlbLost.begin();
        for (list<Sophus::SE3f>::iterator lit = mpTracker->mlRelativeFramePoses.begin(),
                                          lend = mpTracker->mlRelativeFramePoses.end();
             lit != lend; lit++, lRit++, lT++, lbL++)
        {
            if (*lbL)
                continue;

            KeyFrame *pKF = *lRit;

            Sophus::SE3f Trw;

            // If the reference keyframe was culled, traverse the spanning tree to get a suitable keyframe.
            while (pKF->isBad())
            {
                Trw = Trw * pKF->mTcp;
                pKF = pKF->GetParent();
            }

            Trw = Trw * pKF->GetPose() * Two;

            Sophus::SE3f Tcw = (*lit) * Trw;
            Sophus::SE3f Twc = Tcw.inverse();

            Eigen::Vector3f twc = Twc.translation();
            Eigen::Quaternionf q = Twc.unit_quaternion();

            f << setprecision(6) << *lT << " " << setprecision(9) << twc(0) << " " << twc(1) << " " << twc(2) << " " << q.x() << " " << q.y() << " " << q.z() << " " << q.w() << endl;
        }
        f.close();
        // cout << endl << "trajectory saved!" << endl;
    }

    void System::SaveKeyFrameTrajectoryTUM(const string &filename)
    {
        cout << endl
             << "Saving keyframe trajectory to " << filename << " ..." << endl;

        vector<KeyFrame *> vpKFs = mpAtlas->GetAllKeyFrames();
        sort(vpKFs.begin(), vpKFs.end(), KeyFrame::lId);

        // Transform all keyframes so that the first keyframe is at the origin.
        // After a loop closure the first keyframe might not be at the origin.
        ofstream f;
        f.open(filename.c_str());
        f << fixed;

        for (size_t i = 0; i < vpKFs.size(); i++)
        {
            KeyFrame *pKF = vpKFs[i];

            // pKF->SetPose(pKF->GetPose()*Two);

            if (pKF->isBad())
                continue;

            Sophus::SE3f Twc = pKF->GetPoseInverse();
            Eigen::Quaternionf q = Twc.unit_quaternion();
            Eigen::Vector3f t = Twc.translation();
            f << setprecision(6) << pKF->mTimeStamp << setprecision(7) << " " << t(0) << " " << t(1) << " " << t(2)
              << " " << q.x() << " " << q.y() << " " << q.z() << " " << q.w() << endl;
        }

        f.close();
    }

    void System::SaveTrajectoryEuRoC(const string &filename)
    {

        cout << endl
             << "Saving trajectory to " << filename << " ..." << endl;
        /*if(mSensor==MONOCULAR)
        {
            cerr << "ERROR: SaveTrajectoryEuRoC cannot be used for monocular." << endl;
            return;
        }*/

        vector<Map *> vpMaps = mpAtlas->GetAllMaps();
        int numMaxKFs = 0;
        Map *pBiggerMap;
        std::cout << "There are " << std::to_string(vpMaps.size()) << " maps in the atlas" << std::endl;
        for (Map *pMap : vpMaps)
        {
            std::cout << "  Map " << std::to_string(pMap->GetId()) << " has " << std::to_string(pMap->GetAllKeyFrames().size()) << " KFs" << std::endl;
            if (pMap->GetAllKeyFrames().size() > numMaxKFs)
            {
                numMaxKFs = pMap->GetAllKeyFrames().size();
                pBiggerMap = pMap;
            }
        }

        vector<KeyFrame *> vpKFs = pBiggerMap->GetAllKeyFrames();
        sort(vpKFs.begin(), vpKFs.end(), KeyFrame::lId);

        // Transform all keyframes so that the first keyframe is at the origin.
        // After a loop closure the first keyframe might not be at the origin.
        Sophus::SE3f Twb; // Can be word to cam0 or world to b depending on IMU or not.
        if (mSensor == IMU_MONOCULAR || mSensor == IMU_STEREO || mSensor == IMU_RGBD)
            Twb = vpKFs[0]->GetImuPose();
        else
            Twb = vpKFs[0]->GetPoseInverse();

        ofstream f;
        f.open(filename.c_str());
        // cout << "file open" << endl;
        f << fixed;

        // Frame pose is stored relative to its reference keyframe (which is optimized by BA and pose graph).
        // We need to get first the keyframe pose and then concatenate the relative transformation.
        // Frames not localized (tracking failure) are not saved.

        // For each frame we have a reference keyframe (lRit), the timestamp (lT) and a flag
        // which is true when tracking failed (lbL).
        list<ORB_SLAM3::KeyFrame *>::iterator lRit = mpTracker->mlpReferences.begin();
        list<double>::iterator lT = mpTracker->mlFrameTimes.begin();
        list<bool>::iterator lbL = mpTracker->mlbLost.begin();

        // cout << "size mlpReferences: " << mpTracker->mlpReferences.size() << endl;
        // cout << "size mlRelativeFramePoses: " << mpTracker->mlRelativeFramePoses.size() << endl;
        // cout << "size mpTracker->mlFrameTimes: " << mpTracker->mlFrameTimes.size() << endl;
        // cout << "size mpTracker->mlbLost: " << mpTracker->mlbLost.size() << endl;

        for (auto lit = mpTracker->mlRelativeFramePoses.begin(),
                  lend = mpTracker->mlRelativeFramePoses.end();
             lit != lend; lit++, lRit++, lT++, lbL++)
        {
            // cout << "1" << endl;
            if (*lbL)
                continue;

            KeyFrame *pKF = *lRit;
            // cout << "KF: " << pKF->mnId << endl;

            Sophus::SE3f Trw;

            // If the reference keyframe was culled, traverse the spanning tree to get a suitable keyframe.
            if (!pKF)
                continue;

            // cout << "2.5" << endl;

            while (pKF->isBad())
            {
                // cout << " 2.bad" << endl;
                Trw = Trw * pKF->mTcp;
                pKF = pKF->GetParent();
                // cout << "--Parent KF: " << pKF->mnId << endl;
            }

            if (!pKF || pKF->GetMap() != pBiggerMap)
            {
                // cout << "--Parent KF is from another map" << endl;
                continue;
            }

            // cout << "3" << endl;

            Trw = Trw * pKF->GetPose() * Twb; // Tcp*Tpw*Twb0=Tcb0 where b0 is the new world reference

            // cout << "4" << endl;

            if (mSensor == IMU_MONOCULAR || mSensor == IMU_STEREO || mSensor == IMU_RGBD)
            {
                Sophus::SE3f Twb = (pKF->mImuCalib.mTbc * (*lit) * Trw).inverse();
                Eigen::Quaternionf q = Twb.unit_quaternion();
                Eigen::Vector3f twb = Twb.translation();
                f << setprecision(6) << 1e9 * (*lT) << " " << setprecision(9) << twb(0) << " " << twb(1) << " " << twb(2) << " " << q.x() << " " << q.y() << " " << q.z() << " " << q.w() << endl;
            }
            else
            {
                Sophus::SE3f Twc = ((*lit) * Trw).inverse();
                Eigen::Quaternionf q = Twc.unit_quaternion();
                Eigen::Vector3f twc = Twc.translation();
                f << setprecision(6) << 1e9 * (*lT) << " " << setprecision(9) << twc(0) << " " << twc(1) << " " << twc(2) << " " << q.x() << " " << q.y() << " " << q.z() << " " << q.w() << endl;
            }

            // cout << "5" << endl;
        }
        // cout << "end saving trajectory" << endl;
        f.close();
        cout << endl
             << "End of saving trajectory to " << filename << " ..." << endl;
    }

    void System::SaveTrajectoryEuRoC(const string &filename, Map *pMap)
    {

        cout << endl
             << "Saving trajectory of map " << pMap->GetId() << " to " << filename << " ..." << endl;
        /*if(mSensor==MONOCULAR)
        {
            cerr << "ERROR: SaveTrajectoryEuRoC cannot be used for monocular." << endl;
            return;
        }*/

        int numMaxKFs = 0;

        vector<KeyFrame *> vpKFs = pMap->GetAllKeyFrames();
        sort(vpKFs.begin(), vpKFs.end(), KeyFrame::lId);

        // Transform all keyframes so that the first keyframe is at the origin.
        // After a loop closure the first keyframe might not be at the origin.
        Sophus::SE3f Twb; // Can be word to cam0 or world to b dependingo on IMU or not.
        if (mSensor == IMU_MONOCULAR || mSensor == IMU_STEREO || mSensor == IMU_RGBD)
            Twb = vpKFs[0]->GetImuPose();
        else
            Twb = vpKFs[0]->GetPoseInverse();

        ofstream f;
        f.open(filename.c_str());
        // cout << "file open" << endl;
        f << fixed;

        // Frame pose is stored relative to its reference keyframe (which is optimized by BA and pose graph).
        // We need to get first the keyframe pose and then concatenate the relative transformation.
        // Frames not localized (tracking failure) are not saved.

        // For each frame we have a reference keyframe (lRit), the timestamp (lT) and a flag
        // which is true when tracking failed (lbL).
        list<ORB_SLAM3::KeyFrame *>::iterator lRit = mpTracker->mlpReferences.begin();
        list<double>::iterator lT = mpTracker->mlFrameTimes.begin();
        list<bool>::iterator lbL = mpTracker->mlbLost.begin();

        // cout << "size mlpReferences: " << mpTracker->mlpReferences.size() << endl;
        // cout << "size mlRelativeFramePoses: " << mpTracker->mlRelativeFramePoses.size() << endl;
        // cout << "size mpTracker->mlFrameTimes: " << mpTracker->mlFrameTimes.size() << endl;
        // cout << "size mpTracker->mlbLost: " << mpTracker->mlbLost.size() << endl;

        for (auto lit = mpTracker->mlRelativeFramePoses.begin(),
                  lend = mpTracker->mlRelativeFramePoses.end();
             lit != lend; lit++, lRit++, lT++, lbL++)
        {
            // cout << "1" << endl;
            if (*lbL)
                continue;

            KeyFrame *pKF = *lRit;
            // cout << "KF: " << pKF->mnId << endl;

            Sophus::SE3f Trw;

            // If the reference keyframe was culled, traverse the spanning tree to get a suitable keyframe.
            if (!pKF)
                continue;

            // cout << "2.5" << endl;

            while (pKF->isBad())
            {
                // cout << " 2.bad" << endl;
                Trw = Trw * pKF->mTcp;
                pKF = pKF->GetParent();
                // cout << "--Parent KF: " << pKF->mnId << endl;
            }

            if (!pKF || pKF->GetMap() != pMap)
            {
                // cout << "--Parent KF is from another map" << endl;
                continue;
            }

            // cout << "3" << endl;

            Trw = Trw * pKF->GetPose() * Twb; // Tcp*Tpw*Twb0=Tcb0 where b0 is the new world reference

            // cout << "4" << endl;

            if (mSensor == IMU_MONOCULAR || mSensor == IMU_STEREO || mSensor == IMU_RGBD)
            {
                Sophus::SE3f Twb = (pKF->mImuCalib.mTbc * (*lit) * Trw).inverse();
                Eigen::Quaternionf q = Twb.unit_quaternion();
                Eigen::Vector3f twb = Twb.translation();
                f << setprecision(6) << 1e9 * (*lT) << " " << setprecision(9) << twb(0) << " " << twb(1) << " " << twb(2) << " " << q.x() << " " << q.y() << " " << q.z() << " " << q.w() << endl;
            }
            else
            {
                Sophus::SE3f Twc = ((*lit) * Trw).inverse();
                Eigen::Quaternionf q = Twc.unit_quaternion();
                Eigen::Vector3f twc = Twc.translation();
                f << setprecision(6) << 1e9 * (*lT) << " " << setprecision(9) << twc(0) << " " << twc(1) << " " << twc(2) << " " << q.x() << " " << q.y() << " " << q.z() << " " << q.w() << endl;
            }

            // cout << "5" << endl;
        }
        // cout << "end saving trajectory" << endl;
        f.close();
        cout << endl
             << "End of saving trajectory to " << filename << " ..." << endl;
    }

    /*void System::SaveTrajectoryEuRoC(const string &filename)
    {

        cout << endl << "Saving trajectory to " << filename << " ..." << endl;
        if(mSensor==MONOCULAR)
        {
            cerr << "ERROR: SaveTrajectoryEuRoC cannot be used for monocular." << endl;
            return;
        }

        vector<Map*> vpMaps = mpAtlas->GetAllMaps();
        Map* pBiggerMap;
        int numMaxKFs = 0;
        for(Map* pMap :vpMaps)
        {
            if(pMap->GetAllKeyFrames().size() > numMaxKFs)
            {
                numMaxKFs = pMap->GetAllKeyFrames().size();
                pBiggerMap = pMap;
            }
        }

        vector<KeyFrame*> vpKFs = pBiggerMap->GetAllKeyFrames();
        sort(vpKFs.begin(),vpKFs.end(),KeyFrame::lId);

        // Transform all keyframes so that the first keyframe is at the origin.
        // After a loop closure the first keyframe might not be at the origin.
        Sophus::SE3f Twb; // Can be word to cam0 or world to b dependingo on IMU or not.
        if (mSensor==IMU_MONOCULAR || mSensor==IMU_STEREO || mSensor==IMU_RGBD)
            Twb = vpKFs[0]->GetImuPose_();
        else
            Twb = vpKFs[0]->GetPoseInverse_();

        ofstream f;
        f.open(filename.c_str());
        // cout << "file open" << endl;
        f << fixed;

        // Frame pose is stored relative to its reference keyframe (which is optimized by BA and pose graph).
        // We need to get first the keyframe pose and then concatenate the relative transformation.
        // Frames not localized (tracking failure) are not saved.

        // For each frame we have a reference keyframe (lRit), the timestamp (lT) and a flag
        // which is true when tracking failed (lbL).
        list<ORB_SLAM3::KeyFrame*>::iterator lRit = mpTracker->mlpReferences.begin();
        list<double>::iterator lT = mpTracker->mlFrameTimes.begin();
        list<bool>::iterator lbL = mpTracker->mlbLost.begin();

        //cout << "size mlpReferences: " << mpTracker->mlpReferences.size() << endl;
        //cout << "size mlRelativeFramePoses: " << mpTracker->mlRelativeFramePoses.size() << endl;
        //cout << "size mpTracker->mlFrameTimes: " << mpTracker->mlFrameTimes.size() << endl;
        //cout << "size mpTracker->mlbLost: " << mpTracker->mlbLost.size() << endl;


        for(list<Sophus::SE3f>::iterator lit=mpTracker->mlRelativeFramePoses.begin(),
            lend=mpTracker->mlRelativeFramePoses.end();lit!=lend;lit++, lRit++, lT++, lbL++)
        {
            //cout << "1" << endl;
            if(*lbL)
                continue;


            KeyFrame* pKF = *lRit;
            //cout << "KF: " << pKF->mnId << endl;

            Sophus::SE3f Trw;

            // If the reference keyframe was culled, traverse the spanning tree to get a suitable keyframe.
            if (!pKF)
                continue;

            //cout << "2.5" << endl;

            while(pKF->isBad())
            {
                //cout << " 2.bad" << endl;
                Trw = Trw * pKF->mTcp;
                pKF = pKF->GetParent();
                //cout << "--Parent KF: " << pKF->mnId << endl;
            }

            if(!pKF || pKF->GetMap() != pBiggerMap)
            {
                //cout << "--Parent KF is from another map" << endl;
                continue;
            }

            //cout << "3" << endl;

            Trw = Trw * pKF->GetPose()*Twb; // Tcp*Tpw*Twb0=Tcb0 where b0 is the new world reference

            // cout << "4" << endl;


            if (mSensor == IMU_MONOCULAR || mSensor == IMU_STEREO || mSensor==IMU_RGBD)
            {
                Sophus::SE3f Tbw = pKF->mImuCalib.Tbc_ * (*lit) * Trw;
                Sophus::SE3f Twb = Tbw.inverse();

                Eigen::Vector3f twb = Twb.translation();
                Eigen::Quaternionf q = Twb.unit_quaternion();
                f << setprecision(6) << 1e9*(*lT) << " " <<  setprecision(9) << twb(0) << " " << twb(1) << " " << twb(2) << " " << q.x() << " " << q.y() << " " << q.z() << " " << q.w() << endl;
            }
            else
            {
                Sophus::SE3f Tcw = (*lit) * Trw;
                Sophus::SE3f Twc = Tcw.inverse();

                Eigen::Vector3f twc = Twc.translation();
                Eigen::Quaternionf q = Twc.unit_quaternion();
                f << setprecision(6) << 1e9*(*lT) << " " <<  setprecision(9) << twc(0) << " " << twc(1) << " " << twc(2) << " " << q.x() << " " << q.y() << " " << q.z() << " " << q.w() << endl;
            }

            // cout << "5" << endl;
        }
        //cout << "end saving trajectory" << endl;
        f.close();
        cout << endl << "End of saving trajectory to " << filename << " ..." << endl;
    }*/

    /*void System::SaveKeyFrameTrajectoryEuRoC_old(const string &filename)
    {
        cout << endl << "Saving keyframe trajectory to " << filename << " ..." << endl;

        vector<Map*> vpMaps = mpAtlas->GetAllMaps();
        Map* pBiggerMap;
        int numMaxKFs = 0;
        for(Map* pMap :vpMaps)
        {
            if(pMap->GetAllKeyFrames().size() > numMaxKFs)
            {
                numMaxKFs = pMap->GetAllKeyFrames().size();
                pBiggerMap = pMap;
            }
        }

        vector<KeyFrame*> vpKFs = pBiggerMap->GetAllKeyFrames();
        sort(vpKFs.begin(),vpKFs.end(),KeyFrame::lId);

        // Transform all keyframes so that the first keyframe is at the origin.
        // After a loop closure the first keyframe might not be at the origin.
        ofstream f;
        f.open(filename.c_str());
        f << fixed;

        for(size_t i=0; i<vpKFs.size(); i++)
        {
            KeyFrame* pKF = vpKFs[i];

           // pKF->SetPose(pKF->GetPose()*Two);

            if(pKF->isBad())
                continue;
            if (mSensor == IMU_MONOCULAR || mSensor == IMU_STEREO || mSensor==IMU_RGBD)
            {
                cv::Mat R = pKF->GetImuRotation().t();
                vector<float> q = Converter::toQuaternion(R);
                cv::Mat twb = pKF->GetImuPosition();
                f << setprecision(6) << 1e9*pKF->mTimeStamp  << " " <<  setprecision(9) << twb.at<float>(0) << " " << twb.at<float>(1) << " " << twb.at<float>(2) << " " << q[0] << " " << q[1] << " " << q[2] << " " << q[3] << endl;

            }
            else
            {
                cv::Mat R = pKF->GetRotation();
                vector<float> q = Converter::toQuaternion(R);
                cv::Mat t = pKF->GetCameraCenter();
                f << setprecision(6) << 1e9*pKF->mTimeStamp << " " <<  setprecision(9) << t.at<float>(0) << " " << t.at<float>(1) << " " << t.at<float>(2) << " " << q[0] << " " << q[1] << " " << q[2] << " " << q[3] << endl;
            }
        }
        f.close();
    }*/

    void System::SaveKeyFrameTrajectoryEuRoC(const string &filename)
    {
        cout << endl
             << "Saving keyframe trajectory to " << filename << " ..." << endl;

        vector<Map *> vpMaps = mpAtlas->GetAllMaps();
        Map *pBiggerMap;
        int numMaxKFs = 0;
        for (Map *pMap : vpMaps)
        {
            if (pMap && pMap->GetAllKeyFrames().size() > numMaxKFs)
            {
                numMaxKFs = pMap->GetAllKeyFrames().size();
                pBiggerMap = pMap;
            }
        }

        if (!pBiggerMap)
        {
            std::cout << "There is not a map!!" << std::endl;
            return;
        }

        vector<KeyFrame *> vpKFs = pBiggerMap->GetAllKeyFrames();
        sort(vpKFs.begin(), vpKFs.end(), KeyFrame::lId);

        // Transform all keyframes so that the first keyframe is at the origin.
        // After a loop closure the first keyframe might not be at the origin.
        ofstream f;
        f.open(filename.c_str());
        f << fixed;

        for (size_t i = 0; i < vpKFs.size(); i++)
        {
            KeyFrame *pKF = vpKFs[i];

            // pKF->SetPose(pKF->GetPose()*Two);

            if (!pKF || pKF->isBad())
                continue;
            if (mSensor == IMU_MONOCULAR || mSensor == IMU_STEREO || mSensor == IMU_RGBD)
            {
                Sophus::SE3f Twb = pKF->GetImuPose();
                Eigen::Quaternionf q = Twb.unit_quaternion();
                Eigen::Vector3f twb = Twb.translation();
                f << setprecision(6) << 1e9 * pKF->mTimeStamp << " " << setprecision(9) << twb(0) << " " << twb(1) << " " << twb(2) << " " << q.x() << " " << q.y() << " " << q.z() << " " << q.w() << endl;
            }
            else
            {
                Sophus::SE3f Twc = pKF->GetPoseInverse();
                Eigen::Quaternionf q = Twc.unit_quaternion();
                Eigen::Vector3f t = Twc.translation();
                f << setprecision(6) << 1e9 * pKF->mTimeStamp << " " << setprecision(9) << t(0) << " " << t(1) << " " << t(2) << " " << q.x() << " " << q.y() << " " << q.z() << " " << q.w() << endl;
            }
        }
        f.close();
    }

    void System::SaveKeyFrameTrajectoryEuRoC(const string &filename, Map *pMap)
    {
        cout << endl
             << "Saving keyframe trajectory of map " << pMap->GetId() << " to " << filename << " ..." << endl;

        vector<KeyFrame *> vpKFs = pMap->GetAllKeyFrames();
        sort(vpKFs.begin(), vpKFs.end(), KeyFrame::lId);

        // Transform all keyframes so that the first keyframe is at the origin.
        // After a loop closure the first keyframe might not be at the origin.
        ofstream f;
        f.open(filename.c_str());
        f << fixed;

        for (size_t i = 0; i < vpKFs.size(); i++)
        {
            KeyFrame *pKF = vpKFs[i];

            if (!pKF || pKF->isBad())
                continue;
            if (mSensor == IMU_MONOCULAR || mSensor == IMU_STEREO || mSensor == IMU_RGBD)
            {
                Sophus::SE3f Twb = pKF->GetImuPose();
                Eigen::Quaternionf q = Twb.unit_quaternion();
                Eigen::Vector3f twb = Twb.translation();
                f << setprecision(6) << 1e9 * pKF->mTimeStamp << " " << setprecision(9) << twb(0) << " " << twb(1) << " " << twb(2) << " " << q.x() << " " << q.y() << " " << q.z() << " " << q.w() << endl;
            }
            else
            {
                Sophus::SE3f Twc = pKF->GetPoseInverse();
                Eigen::Quaternionf q = Twc.unit_quaternion();
                Eigen::Vector3f t = Twc.translation();
                f << setprecision(6) << 1e9 * pKF->mTimeStamp << " " << setprecision(9) << t(0) << " " << t(1) << " " << t(2) << " " << q.x() << " " << q.y() << " " << q.z() << " " << q.w() << endl;
            }
        }
        f.close();
    }

    /*void System::SaveTrajectoryKITTI(const string &filename)
    {
        cout << endl << "Saving camera trajectory to " << filename << " ..." << endl;
        if(mSensor==MONOCULAR)
        {
            cerr << "ERROR: SaveTrajectoryKITTI cannot be used for monocular." << endl;
            return;
        }

        vector<KeyFrame*> vpKFs = mpAtlas->GetAllKeyFrames();
        sort(vpKFs.begin(),vpKFs.end(),KeyFrame::lId);

        // Transform all keyframes so that the first keyframe is at the origin.
        // After a loop closure the first keyframe might not be at the origin.
        cv::Mat Two = vpKFs[0]->GetPoseInverse();

        ofstream f;
        f.open(filename.c_str());
        f << fixed;

        // Frame pose is stored relative to its reference keyframe (which is optimized by BA and pose graph).
        // We need to get first the keyframe pose and then concatenate the relative transformation.
        // Frames not localized (tracking failure) are not saved.

        // For each frame we have a reference keyframe (lRit), the timestamp (lT) and a flag
        // which is true when tracking failed (lbL).
        list<ORB_SLAM3::KeyFrame*>::iterator lRit = mpTracker->mlpReferences.begin();
        list<double>::iterator lT = mpTracker->mlFrameTimes.begin();
        for(list<cv::Mat>::iterator lit=mpTracker->mlRelativeFramePoses.begin(), lend=mpTracker->mlRelativeFramePoses.end();lit!=lend;lit++, lRit++, lT++)
        {
            ORB_SLAM3::KeyFrame* pKF = *lRit;

            cv::Mat Trw = cv::Mat::eye(4,4,CV_32F);

            while(pKF->isBad())
            {
                Trw = Trw * Converter::toCvMat(pKF->mTcp.matrix());
                pKF = pKF->GetParent();
            }

            Trw = Trw * pKF->GetPoseCv() * Two;

            cv::Mat Tcw = (*lit)*Trw;
            cv::Mat Rwc = Tcw.rowRange(0,3).colRange(0,3).t();
            cv::Mat twc = -Rwc*Tcw.rowRange(0,3).col(3);

            f << setprecision(9) << Rwc.at<float>(0,0) << " " << Rwc.at<float>(0,1)  << " " << Rwc.at<float>(0,2) << " "  << twc.at<float>(0) << " " <<
                 Rwc.at<float>(1,0) << " " << Rwc.at<float>(1,1)  << " " << Rwc.at<float>(1,2) << " "  << twc.at<float>(1) << " " <<
                 Rwc.at<float>(2,0) << " " << Rwc.at<float>(2,1)  << " " << Rwc.at<float>(2,2) << " "  << twc.at<float>(2) << endl;
        }
        f.close();
    }*/

    void System::SaveTrajectoryKITTI(const string &filename)
    {
        cout << endl
             << "Saving camera trajectory to " << filename << " ..." << endl;
        if (mSensor == MONOCULAR)
        {
            cerr << "ERROR: SaveTrajectoryKITTI cannot be used for monocular." << endl;
            return;
        }

        vector<KeyFrame *> vpKFs = mpAtlas->GetAllKeyFrames();
        sort(vpKFs.begin(), vpKFs.end(), KeyFrame::lId);

        // Transform all keyframes so that the first keyframe is at the origin.
        // After a loop closure the first keyframe might not be at the origin.
        Sophus::SE3f Tow = vpKFs[0]->GetPoseInverse();

        ofstream f;
        f.open(filename.c_str());
        f << fixed;

        // Frame pose is stored relative to its reference keyframe (which is optimized by BA and pose graph).
        // We need to get first the keyframe pose and then concatenate the relative transformation.
        // Frames not localized (tracking failure) are not saved.

        // For each frame we have a reference keyframe (lRit), the timestamp (lT) and a flag
        // which is true when tracking failed (lbL).
        list<ORB_SLAM3::KeyFrame *>::iterator lRit = mpTracker->mlpReferences.begin();
        list<double>::iterator lT = mpTracker->mlFrameTimes.begin();
        for (list<Sophus::SE3f>::iterator lit = mpTracker->mlRelativeFramePoses.begin(),
                                          lend = mpTracker->mlRelativeFramePoses.end();
             lit != lend; lit++, lRit++, lT++)
        {
            ORB_SLAM3::KeyFrame *pKF = *lRit;

            Sophus::SE3f Trw;

            if (!pKF)
                continue;

            while (pKF->isBad())
            {
                Trw = Trw * pKF->mTcp;
                pKF = pKF->GetParent();
            }

            Trw = Trw * pKF->GetPose() * Tow;

            Sophus::SE3f Tcw = (*lit) * Trw;
            Sophus::SE3f Twc = Tcw.inverse();
            Eigen::Matrix3f Rwc = Twc.rotationMatrix();
            Eigen::Vector3f twc = Twc.translation();

            f << setprecision(9) << Rwc(0, 0) << " " << Rwc(0, 1) << " " << Rwc(0, 2) << " " << twc(0) << " " << Rwc(1, 0) << " " << Rwc(1, 1) << " " << Rwc(1, 2) << " " << twc(1) << " " << Rwc(2, 0) << " " << Rwc(2, 1) << " " << Rwc(2, 2) << " " << twc(2) << endl;
        }
        f.close();
    }

    // -------------------------------------------------------------------------------------------------

    void System::SaveDebugData(const int &initIdx)
    {
        // 0. Save initialization trajectory
        SaveTrajectoryEuRoC("init_FrameTrajectoy_" + to_string(mpLocalMapper->mInitSect) + "_" + to_string(initIdx) + ".txt");

        // 1. Save scale
        ofstream f;
        f.open("init_Scale_" + to_string(mpLocalMapper->mInitSect) + ".txt", ios_base::app);
        f << fixed;
        f << mpLocalMapper->mScale << endl;
        f.close();

        // 2. Save gravity direction
        f.open("init_GDir_" + to_string(mpLocalMapper->mInitSect) + ".txt", ios_base::app);
        f << fixed;
        f << mpLocalMapper->mRwg(0, 0) << "," << mpLocalMapper->mRwg(0, 1) << "," << mpLocalMapper->mRwg(0, 2) << endl;
        f << mpLocalMapper->mRwg(1, 0) << "," << mpLocalMapper->mRwg(1, 1) << "," << mpLocalMapper->mRwg(1, 2) << endl;
        f << mpLocalMapper->mRwg(2, 0) << "," << mpLocalMapper->mRwg(2, 1) << "," << mpLocalMapper->mRwg(2, 2) << endl;
        f.close();

        // 3. Save computational cost
        f.open("init_CompCost_" + to_string(mpLocalMapper->mInitSect) + ".txt", ios_base::app);
        f << fixed;
        f << mpLocalMapper->mCostTime << endl;
        f.close();

        // 4. Save biases
        f.open("init_Biases_" + to_string(mpLocalMapper->mInitSect) + ".txt", ios_base::app);
        f << fixed;
        f << mpLocalMapper->mbg(0) << "," << mpLocalMapper->mbg(1) << "," << mpLocalMapper->mbg(2) << endl;
        f << mpLocalMapper->mba(0) << "," << mpLocalMapper->mba(1) << "," << mpLocalMapper->mba(2) << endl;
        f.close();

        // 5. Save covariance matrix
        f.open("init_CovMatrix_" + to_string(mpLocalMapper->mInitSect) + "_" + to_string(initIdx) + ".txt", ios_base::app);
        f << fixed;
        for (int i = 0; i < mpLocalMapper->mcovInertial.rows(); i++)
        {
            for (int j = 0; j < mpLocalMapper->mcovInertial.cols(); j++)
            {
                if (j != 0)
                    f << ",";
                f << setprecision(15) << mpLocalMapper->mcovInertial(i, j);
            }
            f << endl;
        }
        f.close();

        // 6. Save initialization time
        f.open("init_Time_" + to_string(mpLocalMapper->mInitSect) + ".txt", ios_base::app);
        f << fixed;
        f << mpLocalMapper->mInitTime << endl;
        f.close();
    }

    int System::GetTrackingState()
    {
        unique_lock<mutex> lock(mMutexState);
        return mTrackingState;
    }

    vector<MapPoint *> System::GetTrackedMapPoints()
    {
        unique_lock<mutex> lock(mMutexState);
        return mTrackedMapPoints;
    }

    vector<cv::KeyPoint> System::GetTrackedKeyPointsUn()
    {
        unique_lock<mutex> lock(mMutexState);
        return mTrackedKeyPointsUn;
    }

    double System::GetTimeFromIMUInit()
    {
        double aux = mpLocalMapper->GetCurrKFTime() - mpLocalMapper->mFirstTs;
        if ((aux > 0.) && mpAtlas->isImuInitialized())
            return mpLocalMapper->GetCurrKFTime() - mpLocalMapper->mFirstTs;
        else
            return 0.f;
    }

    bool System::isLost()
    {
        if (!mpAtlas->isImuInitialized())
            return false;
        else
        {
            if ((mpTracker->mState == Tracking::LOST)) //||(mpTracker->mState==Tracking::RECENTLY_LOST))
                return true;
            else
                return false;
        }
    }

    bool System::isFinished()
    {
        return (GetTimeFromIMUInit() > 0.1);
    }

    void System::ChangeDataset()
    {
        if (mpAtlas->GetCurrentMap()->KeyFramesInMap() < 12)
        {
            mpTracker->ResetActiveMap();
        }
        else
        {
            mpTracker->CreateMapInAtlas();
        }

        mpTracker->NewDataset();
    }

    float System::GetImageScale()
    {
        return mpTracker->GetImageScale();
    }

#ifdef REGISTER_TIMES
    void System::InsertRectTime(double &time)
    {
        mpTracker->vdRectStereo_ms.push_back(time);
    }

    void System::InsertResizeTime(double &time)
    {
        mpTracker->vdResizeImage_ms.push_back(time);
    }

    void System::InsertTrackTime(double &time)
    {
        mpTracker->vdTrackTotal_ms.push_back(time);
    }
#endif

    void System::SaveAtlas(int type)
    {
        if (!mStrSaveAtlasToFile.empty())
        {
            // clock_t start = clock();

            // Save the current session
            mpAtlas->PreSave();

            string pathSaveFileName = "./";
            pathSaveFileName = pathSaveFileName.append(mStrSaveAtlasToFile);
            pathSaveFileName = pathSaveFileName.append(".osa");

            string strVocabularyChecksum = CalculateCheckSum(mStrVocabularyFilePath, TEXT_FILE);
            std::size_t found = mStrVocabularyFilePath.find_last_of("/\\");
            string strVocabularyName = mStrVocabularyFilePath.substr(found + 1);

            if (type == TEXT_FILE) // File text
            {
                cout << "Starting to write the save text file " << endl;
                std::remove(pathSaveFileName.c_str());
                std::ofstream ofs(pathSaveFileName, std::ios::binary);
                boost::archive::text_oarchive oa(ofs);

                oa << strVocabularyName;
                oa << strVocabularyChecksum;
                oa << mpAtlas;
                cout << "End to write the save text file" << endl;
            }
            else if (type == BINARY_FILE) // File binary
            {
                cout << "Starting to write the save binary file" << endl;
                std::remove(pathSaveFileName.c_str());
                std::ofstream ofs(pathSaveFileName, std::ios::binary);
                boost::archive::binary_oarchive oa(ofs);
                oa << strVocabularyName;
                oa << strVocabularyChecksum;
                oa << mpAtlas;
                cout << "End to write save binary file" << endl;
            }
        }
    }

    bool System::LoadAtlas(int type)
    {
        string strFileVoc, strVocChecksum;
        bool isRead = false;

        string pathLoadFileName = "./";
        pathLoadFileName = pathLoadFileName.append(mStrLoadAtlasFromFile);
        pathLoadFileName = pathLoadFileName.append(".osa");

        if (type == TEXT_FILE) // File text
        {
            cout << "Starting to read the save text file " << endl;
            std::ifstream ifs(pathLoadFileName, std::ios::binary);
            if (!ifs.good())
            {
                cout << "Load file not found" << endl;
                return false;
            }
            boost::archive::text_iarchive ia(ifs);
            ia >> strFileVoc;
            ia >> strVocChecksum;
            ia >> mpAtlas;
            cout << "End to load the save text file " << endl;
            isRead = true;
        }
        else if (type == BINARY_FILE) // File binary
        {
            cout << "Starting to read the save binary file" << endl;
            std::ifstream ifs(pathLoadFileName, std::ios::binary);
            if (!ifs.good())
            {
                cout << "Load file not found" << endl;
                return false;
            }
            boost::archive::binary_iarchive ia(ifs);
            ia >> strFileVoc;
            ia >> strVocChecksum;
            ia >> mpAtlas;
            cout << "End to load the save binary file" << endl;
            isRead = true;
        }

        if (isRead)
        {
            // Check if the vocabulary is the same
            string strInputVocabularyChecksum = CalculateCheckSum(mStrVocabularyFilePath, TEXT_FILE);

            if (strInputVocabularyChecksum.compare(strVocChecksum) != 0)
            {
                cout << "The vocabulary load isn't the same which the load session was created " << endl;
                cout << "-Vocabulary name: " << strFileVoc << endl;
                return false; // Both are differents
            }

            mpAtlas->SetKeyFrameDababase(mpKeyFrameDatabase);
            mpAtlas->SetORBVocabulary(mpVocabulary);
            mpAtlas->PostLoad();

            return true;
        }
        return false;
    }

    string System::CalculateCheckSum(string filename, int type)
    {
        string checksum = "";

        unsigned char c[MD5_DIGEST_LENGTH];

        std::ios_base::openmode flags = std::ios::in;
        if (type == BINARY_FILE) // Binary file
            flags = std::ios::in | std::ios::binary;

        ifstream f(filename.c_str(), flags);
        if (!f.is_open())
        {
            cout << "[E] Unable to open the in file " << filename << " for Md5 hash." << endl;
            return checksum;
        }

        MD5_CTX md5Context;
        char buffer[1024];

        MD5_Init(&md5Context);
        while (int count = f.readsome(buffer, sizeof(buffer)))
        {
            MD5_Update(&md5Context, buffer, count);
        }

        f.close();

        MD5_Final(c, &md5Context);

        for (int i = 0; i < MD5_DIGEST_LENGTH; i++)
        {
            char aux[10];
            sprintf(aux, "%02x", c[i]);
            checksum = checksum + aux;
        }

        return checksum;
    }

} // namespace ORB_SLAM
