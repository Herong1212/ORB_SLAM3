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

#include "Settings.h"

#include "CameraModels/Pinhole.h"
#include "CameraModels/KannalaBrandt8.h"

#include "System.h"

#include <opencv2/core/persistence.hpp>
#include <opencv2/core/eigen.hpp>

#include <iostream>

using namespace std;

namespace ORB_SLAM3
{

    template <>
    float Settings::readParameter<float>(cv::FileStorage &fSettings, const std::string &name, bool &found, const bool required)
    {
        cv::FileNode node = fSettings[name];
        if (node.empty())
        {
            if (required)
            {
                std::cerr << name << " required parameter does not exist, aborting..." << std::endl;
                exit(-1);
            }
            else
            {
                std::cerr << name << " optional parameter does not exist..." << std::endl;
                found = false;
                return 0.0f;
            }
        }
        else if (!node.isReal())
        {
            std::cerr << name << " parameter must be a real number, aborting..." << std::endl;
            exit(-1);
        }
        else
        {
            found = true;
            return node.real();
        }
    }

    template <>
    int Settings::readParameter<int>(cv::FileStorage &fSettings, const std::string &name, bool &found, const bool required)
    {
        cv::FileNode node = fSettings[name];
        if (node.empty())
        {
            if (required)
            {
                std::cerr << name << " required parameter does not exist, aborting..." << std::endl;
                exit(-1);
            }
            else
            {
                std::cerr << name << " optional parameter does not exist..." << std::endl;
                found = false;
                return 0;
            }
        }
        else if (!node.isInt())
        {
            std::cerr << name << " parameter must be an integer number, aborting..." << std::endl;
            exit(-1);
        }
        else
        {
            found = true;
            return node.operator int();
        }
    }

    template <>
    string Settings::readParameter<string>(cv::FileStorage &fSettings, const std::string &name, bool &found, const bool required)
    {
        cv::FileNode node = fSettings[name];
        if (node.empty())
        {
            if (required)
            {
                std::cerr << name << " required parameter does not exist, aborting..." << std::endl;
                exit(-1);
            }
            else
            {
                std::cerr << name << " optional parameter does not exist..." << std::endl;
                found = false;
                return string();
            }
        }
        else if (!node.isString())
        {
            std::cerr << name << " parameter must be a string, aborting..." << std::endl;
            exit(-1);
        }
        else
        {
            found = true;
            return node.string();
        }
    }

    template <>
    cv::Mat Settings::readParameter<cv::Mat>(cv::FileStorage &fSettings, const std::string &name, bool &found, const bool required)
    {
        cv::FileNode node = fSettings[name];
        if (node.empty())
        {
            if (required)
            {
                std::cerr << name << " required parameter does not exist, aborting..." << std::endl;
                exit(-1);
            }
            else
            {
                std::cerr << name << " optional parameter does not exist..." << std::endl;
                found = false;
                return cv::Mat();
            }
        }
        else
        {
            found = true;
            return node.mat();
        }
    }

    // ps：该构造函数用于初始化 Settings 类的对象
    /**
     * @brief 该构造函数用于初始化 Settings 类的对象
     *
     * @param configFile            常量引用，指向一个 std::string 类型的变量 configFile。它通常用于传递一个配置文件路径或名称，用于配置对象的初始化。
     * @param sensor                常量引用，指向一个 int 类型的变量 sensor，通常代表传感器的编号或标识。
     * @param bNeedToUndistort_     被初始化为 false，表示是否需要进行去畸变操作。
     * @param bNeedToRectify_       被初始化为 false，表示是否需要进行图像矫正操作。
     * @param bNeedToResize1_       下面两个都被初始化为 false，表示是否需要调整图像尺寸（可能是两个不同的尺寸调整操作）。
     * @param bNeedToResize2_
     */
    Settings::Settings(const std::string &configFile, const int &sensor) : bNeedToUndistort_(false), bNeedToRectify_(false), bNeedToResize1_(false), bNeedToResize2_(false)
    {
        sensor_ = sensor;

        // Open settings file
        cv::FileStorage fSettings(configFile, cv::FileStorage::READ);
        if (!fSettings.isOpened())
        {
            cerr << "[ERROR]: could not open configuration file at: " << configFile << endl;
            cerr << "Aborting..." << endl;

            exit(-1);
        }
        else
        {
            cout << "Loading settings from " << configFile << endl;
        }

        // 读取第一个相机
        readCamera1(fSettings);
        cout << "\t-Loaded camera 1" << endl;

        // 如果是双目相机，就读取第二个相机（未校正的）
        if (sensor_ == System::STEREO || sensor_ == System::IMU_STEREO)
        {
            readCamera2(fSettings);
            cout << "\t-Loaded camera 2" << endl;
        }

        // Read image info
        readImageInfo(fSettings);
        cout << "\t-Loaded image info" << endl;

        if (sensor_ == System::IMU_MONOCULAR || sensor_ == System::IMU_STEREO || sensor_ == System::IMU_RGBD)
        {
            readIMU(fSettings);
            cout << "\t-Loaded IMU calibration" << endl;
        }

        if (sensor_ == System::RGBD || sensor_ == System::IMU_RGBD)
        {
            readRGBD(fSettings);
            cout << "\t-Loaded RGB-D calibration" << endl;
        }

        readORB(fSettings);
        cout << "\t-Loaded ORB settings" << endl;
        readViewer(fSettings);
        cout << "\t-Loaded viewer settings" << endl;
        readLoadAndSave(fSettings);
        cout << "\t-Loaded Atlas settings" << endl;
        readOtherParameters(fSettings);
        cout << "\t-Loaded misc parameters" << endl;

        if (bNeedToRectify_)
        {
            precomputeRectificationMaps();
            cout << "\t-Computed rectification maps" << endl;
        }

        cout << "----------------------------------" << endl;
    }

    // 读取第一个相机
    void Settings::readCamera1(cv::FileStorage &fSettings)
    {
        bool found;

        // Read camera model
        string cameraModel = readParameter<string>(fSettings, "Camera.type", found);

        vector<float> vCalibration;
        if (cameraModel == "PinHole")
        {
            cameraType_ = PinHole;

            // Read intrinsic parameters
            float fx = readParameter<float>(fSettings, "Camera1.fx", found);
            float fy = readParameter<float>(fSettings, "Camera1.fy", found);
            float cx = readParameter<float>(fSettings, "Camera1.cx", found);
            float cy = readParameter<float>(fSettings, "Camera1.cy", found);

            vCalibration = {fx, fy, cx, cy};

            calibration1_ = new Pinhole(vCalibration);
            originalCalib1_ = new Pinhole(vCalibration);

            // Check if it is a distorted PinHole
            readParameter<float>(fSettings, "Camera1.k1", found, false);
            if (found)
            {
                readParameter<float>(fSettings, "Camera1.k3", found, false);
                if (found)
                {
                    vPinHoleDistorsion1_.resize(5);
                    vPinHoleDistorsion1_[4] = readParameter<float>(fSettings, "Camera1.k3", found);
                }
                else
                {
                    vPinHoleDistorsion1_.resize(4);
                }
                vPinHoleDistorsion1_[0] = readParameter<float>(fSettings, "Camera1.k1", found);
                vPinHoleDistorsion1_[1] = readParameter<float>(fSettings, "Camera1.k2", found);
                vPinHoleDistorsion1_[2] = readParameter<float>(fSettings, "Camera1.p1", found);
                vPinHoleDistorsion1_[3] = readParameter<float>(fSettings, "Camera1.p2", found);
            }

            // Check if we need to correct distortion from the images
            if ((sensor_ == System::MONOCULAR || sensor_ == System::IMU_MONOCULAR) && vPinHoleDistorsion1_.size() != 0)
            {
                bNeedToUndistort_ = true;
            }
        }
        else if (cameraModel == "Rectified")
        {
            cameraType_ = Rectified;

            // Read intrinsic parameters
            float fx = readParameter<float>(fSettings, "Camera1.fx", found);
            float fy = readParameter<float>(fSettings, "Camera1.fy", found);
            float cx = readParameter<float>(fSettings, "Camera1.cx", found);
            float cy = readParameter<float>(fSettings, "Camera1.cy", found);

            vCalibration = {fx, fy, cx, cy};

            calibration1_ = new Pinhole(vCalibration);
            originalCalib1_ = new Pinhole(vCalibration);

            // Rectified images are assumed to be ideal PinHole images (no distortion)
        }
        else if (cameraModel == "KannalaBrandt8")
        {
            cameraType_ = KannalaBrandt;

            // Read intrinsic parameters
            float fx = readParameter<float>(fSettings, "Camera1.fx", found);
            float fy = readParameter<float>(fSettings, "Camera1.fy", found);
            float cx = readParameter<float>(fSettings, "Camera1.cx", found);
            float cy = readParameter<float>(fSettings, "Camera1.cy", found);

            float k0 = readParameter<float>(fSettings, "Camera1.k1", found);
            float k1 = readParameter<float>(fSettings, "Camera1.k2", found);
            float k2 = readParameter<float>(fSettings, "Camera1.k3", found);
            float k3 = readParameter<float>(fSettings, "Camera1.k4", found);

            vCalibration = {fx, fy, cx, cy, k0, k1, k2, k3};

            calibration1_ = new KannalaBrandt8(vCalibration);
            originalCalib1_ = new KannalaBrandt8(vCalibration);

            if (sensor_ == System::STEREO || sensor_ == System::IMU_STEREO)
            {
                int colBegin = readParameter<int>(fSettings, "Camera1.overlappingBegin", found);
                int colEnd = readParameter<int>(fSettings, "Camera1.overlappingEnd", found);
                vector<int> vOverlapping = {colBegin, colEnd};

                static_cast<KannalaBrandt8 *>(calibration1_)->mvLappingArea = vOverlapping;
            }
        }
        else
        {
            cerr << "Error: " << cameraModel << " not known" << endl;
            exit(-1);
        }
    }

    // 读取第二个相机（双目相机时）
    void Settings::readCamera2(cv::FileStorage &fSettings)
    {
        bool found;
        vector<float> vCalibration;
        if (cameraType_ == PinHole)
        {
            bNeedToRectify_ = true;

            // Read intrinsic parameters
            float fx = readParameter<float>(fSettings, "Camera2.fx", found);
            float fy = readParameter<float>(fSettings, "Camera2.fy", found);
            float cx = readParameter<float>(fSettings, "Camera2.cx", found);
            float cy = readParameter<float>(fSettings, "Camera2.cy", found);

            vCalibration = {fx, fy, cx, cy};

            calibration2_ = new Pinhole(vCalibration);
            originalCalib2_ = new Pinhole(vCalibration);

            // Check if it is a distorted PinHole
            readParameter<float>(fSettings, "Camera2.k1", found, false);
            if (found)
            {
                readParameter<float>(fSettings, "Camera2.k3", found, false);
                if (found)
                {
                    vPinHoleDistorsion2_.resize(5);
                    vPinHoleDistorsion2_[4] = readParameter<float>(fSettings, "Camera2.k3", found);
                }
                else
                {
                    vPinHoleDistorsion2_.resize(4);
                }
                vPinHoleDistorsion2_[0] = readParameter<float>(fSettings, "Camera2.k1", found);
                vPinHoleDistorsion2_[1] = readParameter<float>(fSettings, "Camera2.k2", found);
                vPinHoleDistorsion2_[2] = readParameter<float>(fSettings, "Camera2.p1", found);
                vPinHoleDistorsion2_[3] = readParameter<float>(fSettings, "Camera2.p2", found);
            }
        }
        else if (cameraType_ == KannalaBrandt)
        {
            // Read intrinsic parameters
            float fx = readParameter<float>(fSettings, "Camera2.fx", found);
            float fy = readParameter<float>(fSettings, "Camera2.fy", found);
            float cx = readParameter<float>(fSettings, "Camera2.cx", found);
            float cy = readParameter<float>(fSettings, "Camera2.cy", found);

            float k0 = readParameter<float>(fSettings, "Camera1.k1", found);
            float k1 = readParameter<float>(fSettings, "Camera1.k2", found);
            float k2 = readParameter<float>(fSettings, "Camera1.k3", found);
            float k3 = readParameter<float>(fSettings, "Camera1.k4", found);

            vCalibration = {fx, fy, cx, cy, k0, k1, k2, k3};

            calibration2_ = new KannalaBrandt8(vCalibration);
            originalCalib2_ = new KannalaBrandt8(vCalibration);

            int colBegin = readParameter<int>(fSettings, "Camera2.overlappingBegin", found);
            int colEnd = readParameter<int>(fSettings, "Camera2.overlappingEnd", found);
            vector<int> vOverlapping = {colBegin, colEnd};

            static_cast<KannalaBrandt8 *>(calibration2_)->mvLappingArea = vOverlapping;
        }

        // Load stereo extrinsic calibration
        if (cameraType_ == Rectified)
        {
            b_ = readParameter<float>(fSettings, "Stereo.b", found);
            bf_ = b_ * calibration1_->getParameter(0);
        }
        else
        {
            cv::Mat cvTlr = readParameter<cv::Mat>(fSettings, "Stereo.T_c1_c2", found);
            Tlr_ = Converter::toSophus(cvTlr);

            // TODO: also search for Trl and invert if necessary

            b_ = Tlr_.translation().norm();
            bf_ = b_ * calibration1_->getParameter(0);
        }

        thDepth_ = readParameter<float>(fSettings, "Stereo.ThDepth", found);
    }

    // 读取与相机相关的图像信息，并根据读取的数据调整内部参数
    void Settings::readImageInfo(cv::FileStorage &fSettings)
    {
        bool found;
        // Read original and desired image dimensions
        int originalRows = readParameter<int>(fSettings, "Camera.height", found);
        int originalCols = readParameter<int>(fSettings, "Camera.width", found);
        originalImSize_.width = originalCols;
        originalImSize_.height = originalRows;

        newImSize_ = originalImSize_;
        int newHeigh = readParameter<int>(fSettings, "Camera.newHeight", found, false);
        if (found)
        {
            bNeedToResize1_ = true;
            newImSize_.height = newHeigh;

            if (!bNeedToRectify_)
            {
                // Update calibration
                float scaleRowFactor = (float)newImSize_.height / (float)originalImSize_.height;
                calibration1_->setParameter(calibration1_->getParameter(1) * scaleRowFactor, 1);
                calibration1_->setParameter(calibration1_->getParameter(3) * scaleRowFactor, 3);

                if ((sensor_ == System::STEREO || sensor_ == System::IMU_STEREO) && cameraType_ != Rectified)
                {
                    calibration2_->setParameter(calibration2_->getParameter(1) * scaleRowFactor, 1);
                    calibration2_->setParameter(calibration2_->getParameter(3) * scaleRowFactor, 3);
                }
            }
        }

        int newWidth = readParameter<int>(fSettings, "Camera.newWidth", found, false);
        if (found)
        {
            bNeedToResize1_ = true;
            newImSize_.width = newWidth;

            if (!bNeedToRectify_)
            {
                // Update calibration
                float scaleColFactor = (float)newImSize_.width / (float)originalImSize_.width;
                calibration1_->setParameter(calibration1_->getParameter(0) * scaleColFactor, 0);
                calibration1_->setParameter(calibration1_->getParameter(2) * scaleColFactor, 2);

                if ((sensor_ == System::STEREO || sensor_ == System::IMU_STEREO) && cameraType_ != Rectified)
                {
                    calibration2_->setParameter(calibration2_->getParameter(0) * scaleColFactor, 0);
                    calibration2_->setParameter(calibration2_->getParameter(2) * scaleColFactor, 2);

                    if (cameraType_ == KannalaBrandt)
                    {
                        static_cast<KannalaBrandt8 *>(calibration1_)->mvLappingArea[0] *= scaleColFactor;
                        static_cast<KannalaBrandt8 *>(calibration1_)->mvLappingArea[1] *= scaleColFactor;

                        static_cast<KannalaBrandt8 *>(calibration2_)->mvLappingArea[0] *= scaleColFactor;
                        static_cast<KannalaBrandt8 *>(calibration2_)->mvLappingArea[1] *= scaleColFactor;
                    }
                }
            }
        }

        fps_ = readParameter<int>(fSettings, "Camera.fps", found);
        bRGB_ = (bool)readParameter<int>(fSettings, "Camera.RGB", found);
    }

    // 从配置文件中读取与 IMU（惯性测量单元）相关的参数，并将这些参数存储到类的相应成员变量中。
    void Settings::readIMU(cv::FileStorage &fSettings)
    {
        bool found;
        noiseGyro_ = readParameter<float>(fSettings, "IMU.NoiseGyro", found);
        noiseAcc_ = readParameter<float>(fSettings, "IMU.NoiseAcc", found);
        gyroWalk_ = readParameter<float>(fSettings, "IMU.GyroWalk", found);
        accWalk_ = readParameter<float>(fSettings, "IMU.AccWalk", found);
        imuFrequency_ = readParameter<float>(fSettings, "IMU.Frequency", found);

        cv::Mat cvTbc = readParameter<cv::Mat>(fSettings, "IMU.T_b_c1", found);
        Tbc_ = Converter::toSophus(cvTbc);

        readParameter<int>(fSettings, "IMU.InsertKFsWhenLost", found, false);
        if (found)
        {
            insertKFsWhenLost_ = (bool)readParameter<int>(fSettings, "IMU.InsertKFsWhenLost", found, false);
        }
        else
        {
            insertKFsWhenLost_ = true;
        }
    }

    // 从配置文件中读取与 RGBD 图像相关的参数，并将这些参数存储到类的相应成员变量中。
    void Settings::readRGBD(cv::FileStorage &fSettings)
    {
        bool found;

        depthMapFactor_ = readParameter<float>(fSettings, "RGBD.DepthMapFactor", found);
        thDepth_ = readParameter<float>(fSettings, "Stereo.ThDepth", found);
        b_ = readParameter<float>(fSettings, "Stereo.b", found);
        bf_ = b_ * calibration1_->getParameter(0);
    }

    // 从的配置文件（cv::FileStorage 类型）中读取 ORB 提取器相关的参数
    void Settings::readORB(cv::FileStorage &fSettings)
    {
        bool found;

        nFeatures_ = readParameter<int>(fSettings, "ORBextractor.nFeatures", found);
        scaleFactor_ = readParameter<float>(fSettings, "ORBextractor.scaleFactor", found);
        nLevels_ = readParameter<int>(fSettings, "ORBextractor.nLevels", found);
        // 初始 FAST 阈值，用于特征点检测。
        initThFAST_ = readParameter<int>(fSettings, "ORBextractor.iniThFAST", found);
        // 最小 FAST 阈值，降低阈值后可以检测更多点
        minThFAST_ = readParameter<int>(fSettings, "ORBextractor.minThFAST", found);
    }

    // 从配置文件中读取了一组用于控制 Viewer 外观和行为
    // 的参数，涵盖了关键帧、图形线条、相机、视点以及图像查看比例等。
    void Settings::readViewer(cv::FileStorage &fSettings)
    {
        bool found;
        // ps：keyFrameSize_ 中命名方式中的下划线 _ 是一种编程习惯，通常用于表示该变量是类的 成员变量

        // 设置关键帧的尺寸，控制在 Viewer 中关键帧的可视化大小。
        keyFrameSize_ = readParameter<float>(fSettings, "Viewer.KeyFrameSize", found);
        keyFrameLineWidth_ = readParameter<float>(fSettings, "Viewer.KeyFrameLineWidth", found);
        graphLineWidth_ = readParameter<float>(fSettings, "Viewer.GraphLineWidth", found);
        pointSize_ = readParameter<float>(fSettings, "Viewer.PointSize", found);
        cameraSize_ = readParameter<float>(fSettings, "Viewer.CameraSize", found);
        cameraLineWidth_ = readParameter<float>(fSettings, "Viewer.CameraLineWidth", found);
        viewPointX_ = readParameter<float>(fSettings, "Viewer.ViewpointX", found);
        viewPointY_ = readParameter<float>(fSettings, "Viewer.ViewpointY", found);
        viewPointZ_ = readParameter<float>(fSettings, "Viewer.ViewpointZ", found);
        viewPointF_ = readParameter<float>(fSettings, "Viewer.ViewpointF", found);

        // ps：在读取 imageViewScale 这个参数时，代码添加了一个缺省值机制：如果未找到该参数（即 found 为 false），它将使用默认值 1.0f。
        imageViewerScale_ = readParameter<float>(fSettings, "Viewer.imageViewScale", found, false);

        if (!found)
            imageViewerScale_ = 1.0f;
    }

    // 从配置文件中读取与“加载”和“保存”功能相关的文件路径信息，并存储在类的成员变量中
    void Settings::readLoadAndSave(cv::FileStorage &fSettings)
    {
        bool found;

        sLoadFrom_ = readParameter<string>(fSettings, "System.LoadAtlasFromFile", found, false);
        sSaveto_ = readParameter<string>(fSettings, "System.SaveAtlasToFile", found, false);
    }

    void Settings::readOtherParameters(cv::FileStorage &fSettings)
    {
        bool found;

        thFarPoints_ = readParameter<float>(fSettings, "System.thFarPoints", found, false);
    }

    // ps：为立体相机的图像对进行预处理，具体包括计算立体校正映射、更新相机
    // 参数和内外参矩阵，以便后续图像处理（如立体匹配和深度计算）能够在校正后的图像中进行
    void Settings::precomputeRectificationMaps()
    {
        // Precompute rectification maps, new calibrations, ...
        // step1. 立体相机的校正矩阵计算
        cv::Mat K1 = static_cast<Pinhole *>(calibration1_)->toK();
        K1.convertTo(K1, CV_64F);
        cv::Mat K2 = static_cast<Pinhole *>(calibration2_)->toK();
        K2.convertTo(K2, CV_64F);

        // step2. 转换相机之间的位姿关系
        cv::Mat cvTlr; // Tlr --- 表示从右相机到左相机的变换矩阵（通常是 4x4 的齐次矩阵）
        cv::eigen2cv(Tlr_.inverse().matrix3x4(), cvTlr);

        cv::Mat R12 = cvTlr.rowRange(0, 3).colRange(0, 3);
        R12.convertTo(R12, CV_64F);
        cv::Mat t12 = cvTlr.rowRange(0, 3).col(3);
        t12.convertTo(t12, CV_64F);

        // step3. 计算立体校正参数
        cv::Mat R_r1_u1, R_r2_u2; // 用于校正左右相机图像的旋转矩阵
        cv::Mat P1, P2;           // 校正后的投影矩阵
        cv::Mat Q;                // 重投影矩阵，用于将视差图转换为深度图

        cv::stereoRectify(K1, camera1DistortionCoef(), K2, camera2DistortionCoef(), newImSize_,
                          R12, t12,
                          R_r1_u1, R_r2_u2, P1, P2, Q,
                          cv::CALIB_ZERO_DISPARITY, -1, newImSize_);

        // step4. 计算图像的校正映射，用于将输入的原始图像矫正为无畸变且对齐的图像
        cv::initUndistortRectifyMap(K1, camera1DistortionCoef(), R_r1_u1, P1.rowRange(0, 3).colRange(0, 3),
                                    newImSize_, CV_32F, M1l_, M2l_);
        cv::initUndistortRectifyMap(K2, camera2DistortionCoef(), R_r2_u2, P2.rowRange(0, 3).colRange(0, 3),
                                    newImSize_, CV_32F, M1r_, M2r_);

        // step5. 使用校正后的投影矩阵 P1 更新左相机的内参参数
        calibration1_->setParameter(P1.at<double>(0, 0), 0); // 焦距 fx
        calibration1_->setParameter(P1.at<double>(1, 1), 1); // 焦距 fy
        calibration1_->setParameter(P1.at<double>(0, 2), 2); // 主点 cx
        calibration1_->setParameter(P1.at<double>(1, 2), 3); // 主点 cy

        // step6. 更新基线乘以焦距
        bf_ = b_ * P1.at<double>(0, 0); // 基线长度（b_）乘以焦距（fx）

        // step7. 更新 IMU 和相机的相对姿态
        if (sensor_ == System::IMU_STEREO)
        {
            Eigen::Matrix3f eigenR_r1_u1;
            cv::cv2eigen(R_r1_u1, eigenR_r1_u1);
            Sophus::SE3f T_r1_u1(eigenR_r1_u1, Eigen::Vector3f::Zero()); // T_r1_u1 是校正后的旋转变换，将其逆应用于现有的 Tbc_
            Tbc_ = Tbc_ * T_r1_u1.inverse();
        }
    }

    // ps：定义了一个 << 操作符的重载函数，目的是以人类可读的形式打印 Settings 类的内容
    ostream &operator<<(std::ostream &output, const Settings &settings)
    {
        output << "SLAM settings: " << endl;

        output << "\t-Camera 1 parameters (";

        if (settings.cameraType_ == Settings::PinHole || settings.cameraType_ == Settings::Rectified)
        {
            output << "Pinhole";
        }
        else
        {
            output << "Kannala-Brandt";
        }

        output << ")" << ": [";

        for (size_t i = 0; i < settings.originalCalib1_->size(); i++)
        {
            output << " " << settings.originalCalib1_->getParameter(i);
        }

        output << " ]" << endl;

        // 畸变参数
        if (!settings.vPinHoleDistorsion1_.empty())
        {
            output << "\t-Camera 1 distortion parameters: [ ";
            for (float d : settings.vPinHoleDistorsion1_)
            {
                output << " " << d;
            }
            output << " ]" << endl;
        }

        if (settings.sensor_ == System::STEREO || settings.sensor_ == System::IMU_STEREO)
        {
            output << "\t-Camera 2 parameters (";
            if (settings.cameraType_ == Settings::PinHole || settings.cameraType_ == Settings::Rectified)
            {
                output << "Pinhole";
            }
            else
            {
                output << "Kannala-Brandt";
            }

            output << "" << ": [";

            for (size_t i = 0; i < settings.originalCalib2_->size(); i++)
            {
                output << " " << settings.originalCalib2_->getParameter(i);
            }
            output << " ]" << endl;

            if (!settings.vPinHoleDistorsion2_.empty())
            {
                output << "\t-Camera 1 distortion parameters: [ ";
                for (float d : settings.vPinHoleDistorsion2_)
                {
                    output << " " << d;
                }
                output << " ]" << endl;
            }
        }

        // 打印原始图像尺寸和当前图像尺寸（可能因缩放或校正而改变）
        output << "\t-Original image size: [ " << settings.originalImSize_.width << " , " << settings.originalImSize_.height << " ]" << endl;
        output << "\t-Current image size: [ " << settings.newImSize_.width << " , " << settings.newImSize_.height << " ]" << endl;

        if (settings.bNeedToRectify_)
        {
            output << "\t-Camera 1 parameters after rectification: [ ";
            for (size_t i = 0; i < settings.calibration1_->size(); i++)
            {
                output << " " << settings.calibration1_->getParameter(i);
            }
            output << " ]" << endl;
        }
        else if (settings.bNeedToResize1_)
        {
            output << "\t-Camera 1 parameters after resize: [ ";
            for (size_t i = 0; i < settings.calibration1_->size(); i++)
            {
                output << " " << settings.calibration1_->getParameter(i);
            }
            output << " ]" << endl;

            if ((settings.sensor_ == System::STEREO || settings.sensor_ == System::IMU_STEREO) &&
                settings.cameraType_ == Settings::KannalaBrandt)
            {
                output << "\t-Camera 2 parameters after resize: [ ";
                for (size_t i = 0; i < settings.calibration2_->size(); i++)
                {
                    output << " " << settings.calibration2_->getParameter(i);
                }
                output << " ]" << endl;
            }
        }

        output << "\t-Sequence FPS: " << settings.fps_ << endl;

        // ps：如果是立体相机（STEREO 或 IMU_STEREO），打印立体相机的基线长度、深度阈值等信息
        if (settings.sensor_ == System::STEREO || settings.sensor_ == System::IMU_STEREO)
        {
            output << "\t-Stereo baseline: " << settings.b_ << endl;
            output << "\t-Stereo depth threshold : " << settings.thDepth_ << endl;

            if (settings.cameraType_ == Settings::KannalaBrandt)
            {
                auto vOverlapping1 = static_cast<KannalaBrandt8 *>(settings.calibration1_)->mvLappingArea;
                auto vOverlapping2 = static_cast<KannalaBrandt8 *>(settings.calibration2_)->mvLappingArea;
                output << "\t-Camera 1 overlapping area: [ " << vOverlapping1[0] << " , " << vOverlapping1[1] << " ]" << endl;
                output << "\t-Camera 2 overlapping area: [ " << vOverlapping2[0] << " , " << vOverlapping2[1] << " ]" << endl;
            }
        }

        // ps：如果传感器包含 IMU，打印陀螺仪和加速度计的噪声、漂移（walk）参数，以及 IMU 的频率
        if (settings.sensor_ == System::IMU_MONOCULAR || settings.sensor_ == System::IMU_STEREO || settings.sensor_ == System::IMU_RGBD)
        {
            output << "\t-Gyro noise: " << settings.noiseGyro_ << endl;
            output << "\t-Accelerometer noise: " << settings.noiseAcc_ << endl;
            output << "\t-Gyro walk: " << settings.gyroWalk_ << endl;
            output << "\t-Accelerometer walk: " << settings.accWalk_ << endl;
            output << "\t-IMU frequency: " << settings.imuFrequency_ << endl;
        }

        // ps：如果是 RGB-D 传感器，打印深度图的比例因子（depthMapFactor_），该因子通常用于将原始深度值转换为实际距离
        if (settings.sensor_ == System::RGBD || settings.sensor_ == System::IMU_RGBD)
        {
            output << "\t-RGB-D depth map factor: " << settings.depthMapFactor_ << endl;
        }

        output << "\t-Features per image: " << settings.nFeatures_ << endl;
        output << "\t-ORB scale factor: " << settings.scaleFactor_ << endl;
        output << "\t-ORB number of scales: " << settings.nLevels_ << endl;
        output << "\t-Initial FAST threshold: " << settings.initThFAST_ << endl;
        output << "\t-Min FAST threshold: " << settings.minThFAST_ << endl;

        return output;
    }
};
