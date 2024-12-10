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

#ifndef IMUTYPES_H
#define IMUTYPES_H

#include <vector>
#include <utility>
#include <opencv2/core/core.hpp>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <Eigen/Dense>
#include <sophus/se3.hpp>
#include <mutex>

#include "SerializationUtils.h"

#include <boost/serialization/serialization.hpp>
#include <boost/serialization/vector.hpp>

namespace ORB_SLAM3
{

    namespace IMU
    {

        const float GRAVITY_VALUE = 9.81;

        // IMU measurement (gyro, accelerometer and timestamp)
        class Point
        {
        public:
            Point(const float &acc_x, const float &acc_y, const float &acc_z,
                  const float &ang_vel_x, const float &ang_vel_y, const float &ang_vel_z,
                  const double &timestamp) : a(acc_x, acc_y, acc_z), w(ang_vel_x, ang_vel_y, ang_vel_z), t(timestamp) {}
            Point(const cv::Point3f Acc, const cv::Point3f Gyro, const double &timestamp) : a(Acc.x, Acc.y, Acc.z), w(Gyro.x, Gyro.y, Gyro.z), t(timestamp) {}

        public:
            Eigen::Vector3f a;
            Eigen::Vector3f w;
            double t; // 每个IMU数据点的时间戳，表示该数据点被采集的时间
            EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        };

        // IMU biases (gyro and accelerometer)
        class Bias
        {
            friend class boost::serialization::access;
            template <class Archive>
            void serialize(Archive &ar, const unsigned int version)
            {
                ar & bax;
                ar & bay;
                ar & baz;

                ar & bwx;
                ar & bwy;
                ar & bwz;
            }

        public:
            Bias() : bax(0), bay(0), baz(0), bwx(0), bwy(0), bwz(0) {}
            Bias(const float &b_acc_x, const float &b_acc_y, const float &b_acc_z,
                 const float &b_ang_vel_x, const float &b_ang_vel_y, const float &b_ang_vel_z) : bax(b_acc_x), bay(b_acc_y), baz(b_acc_z), bwx(b_ang_vel_x), bwy(b_ang_vel_y), bwz(b_ang_vel_z) {}
            void CopyFrom(Bias &b);
            friend std::ostream &operator<<(std::ostream &out, const Bias &b);

        public:
            float bax, bay, baz;
            float bwx, bwy, bwz;
            EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        };

        // IMU calibration (Tbc, Tcb, noise)
        class Calib
        {
            friend class boost::serialization::access;
            template <class Archive>
            void serialize(Archive &ar, const unsigned int version)
            {
                serializeSophusSE3(ar, mTcb, version);
                serializeSophusSE3(ar, mTbc, version);

                ar &boost::serialization::make_array(Cov.diagonal().data(), Cov.diagonal().size());
                ar &boost::serialization::make_array(CovWalk.diagonal().data(), CovWalk.diagonal().size());

                ar & mbIsSet;
            }

        public:
            Calib(const Sophus::SE3<float> &Tbc, const float &ng, const float &na, const float &ngw, const float &naw)
            {
                Set(Tbc, ng, na, ngw, naw);
            }

            Calib(const Calib &calib);   // Calib 类的拷贝构造函数，用于创建当前标定对象的副本。
            Calib() { mbIsSet = false; } // Calib 类的默认构造函数，将 mbIsSet 设置为 false，表示初始时标定数据尚未设置。

            // void Set(const cv::Mat &cvTbc, const float &ng, const float &na, const float &ngw, const float &naw);
            void Set(const Sophus::SE3<float> &sophTbc, const float &ng, const float &na, const float &ngw, const float &naw);

        public:
            // Sophus/Eigen implementation
            Sophus::SE3<float> mTcb;                 // 从 IMU 坐标系到相机坐标系的变换（旋转和平移），是一个 4x4 的齐次变换矩阵
            Sophus::SE3<float> mTbc;                 // 从相机坐标系到 IMU 坐标系的变换。它与 Tcb 是相反的变换：Tbc = Tcb.inverse()。
            Eigen::DiagonalMatrix<float, 6> Cov;     // 加速度计的噪声协方差矩阵，通常是一个对角矩阵，描述了加速度计的测量误差。协方差矩阵的对角元素通常表示每个轴（例如 X、Y、Z 三轴）的噪声水平
            Eigen::DiagonalMatrix<float, 6> CovWalk; // 加速度计的随机游走噪声协方差，描述了加速度计随时间推移产生的漂移或累积误差（通常加速度计会出现随机游走，导致长期内有漂移现象）
            bool mbIsSet;                            // 标记是否已经正确设置了 IMU 的标定数据
        };

        // Integration of 1 gyro measurement
        class IntegratedRotation
        {
        public:
            IntegratedRotation() {}
            IntegratedRotation(const Eigen::Vector3f &angVel, const Bias &imuBias, const float &time);

        public:
            float deltaT; // integration time
            Eigen::Matrix3f deltaR;
            Eigen::Matrix3f rightJ; // right jacobian
            EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        };

        // Preintegration of Imu Measurements
        class Preintegrated
        {
            friend class boost::serialization::access;
            template <class Archive>
            void serialize(Archive &ar, const unsigned int version)
            {
                ar & dT;
                ar &boost::serialization::make_array(C.data(), C.size());
                ar &boost::serialization::make_array(Info.data(), Info.size());
                ar &boost::serialization::make_array(Nga.diagonal().data(), Nga.diagonal().size());
                ar &boost::serialization::make_array(NgaWalk.diagonal().data(), NgaWalk.diagonal().size());
                ar & b;
                ar &boost::serialization::make_array(dR.data(), dR.size());
                ar &boost::serialization::make_array(dV.data(), dV.size());
                ar &boost::serialization::make_array(dP.data(), dP.size());
                ar &boost::serialization::make_array(JRg.data(), JRg.size());
                ar &boost::serialization::make_array(JVg.data(), JVg.size());
                ar &boost::serialization::make_array(JVa.data(), JVa.size());
                ar &boost::serialization::make_array(JPg.data(), JPg.size());
                ar &boost::serialization::make_array(JPa.data(), JPa.size());
                ar &boost::serialization::make_array(avgA.data(), avgA.size());
                ar &boost::serialization::make_array(avgW.data(), avgW.size());

                ar & bu;
                ar &boost::serialization::make_array(db.data(), db.size());
                ar & mvMeasurements;
            }

        public:
            EIGEN_MAKE_ALIGNED_OPERATOR_NEW
            Preintegrated(const Bias &b_, const Calib &calib);
            Preintegrated(Preintegrated *pImuPre);
            Preintegrated() {}
            ~Preintegrated() {}
            void CopyFrom(Preintegrated *pImuPre);
            void Initialize(const Bias &b_);
            void IntegrateNewMeasurement(const Eigen::Vector3f &acceleration, const Eigen::Vector3f &angVel, const float &dt);
            void Reintegrate();
            void MergePrevious(Preintegrated *pPrev);
            void SetNewBias(const Bias &bu_);
            IMU::Bias GetDeltaBias(const Bias &b_);

            Eigen::Matrix3f GetDeltaRotation(const Bias &b_);
            Eigen::Vector3f GetDeltaVelocity(const Bias &b_);
            Eigen::Vector3f GetDeltaPosition(const Bias &b_);

            Eigen::Matrix3f GetUpdatedDeltaRotation();
            Eigen::Vector3f GetUpdatedDeltaVelocity();
            Eigen::Vector3f GetUpdatedDeltaPosition();

            Eigen::Matrix3f GetOriginalDeltaRotation();
            Eigen::Vector3f GetOriginalDeltaVelocity();
            Eigen::Vector3f GetOriginalDeltaPosition();

            Eigen::Matrix<float, 6, 1> GetDeltaBias();

            Bias GetOriginalBias();
            Bias GetUpdatedBias();

            void printMeasurements() const
            {
                std::cout << "pint meas:\n";
                for (int i = 0; i < mvMeasurements.size(); i++)
                {
                    std::cout << "meas " << mvMeasurements[i].t << std::endl;
                }
                std::cout << "end pint meas:\n";
            }

        public:
            float dT;
            Eigen::Matrix<float, 15, 15> C;
            Eigen::Matrix<float, 15, 15> Info;
            Eigen::DiagonalMatrix<float, 6> Nga;     // 加速度计的噪声协方差矩阵
            Eigen::DiagonalMatrix<float, 6> NgaWalk; // 加速度计 随时间变化的噪声协方差矩阵（通常是加速度计的随机游走噪声）

            // Values for the original bias (when integration was computed)
            Bias b;
            Eigen::Matrix3f dR;
            Eigen::Vector3f dV, dP;
            Eigen::Matrix3f JRg, JVg, JVa, JPg, JPa;
            Eigen::Vector3f avgA, avgW; // avgA 和 avgW 分别是当前帧的平均加速度和平均角速度（通常是从 IMU 数据中计算得到的）

        private:
            // Updated bias
            Bias bu;
            // Dif between original and updated bias
            // This is used to compute the updated values of the preintegration
            Eigen::Matrix<float, 6, 1> db;

            struct integrable
            {
                template <class Archive>
                void serialize(Archive &ar, const unsigned int version)
                {
                    ar &boost::serialization::make_array(a.data(), a.size());
                    ar &boost::serialization::make_array(w.data(), w.size());
                    ar & t;
                }

                EIGEN_MAKE_ALIGNED_OPERATOR_NEW
                integrable() {}
                integrable(const Eigen::Vector3f &a_, const Eigen::Vector3f &w_, const float &t_) : a(a_), w(w_), t(t_) {}
                Eigen::Vector3f a, w;
                float t;
            };

            std::vector<integrable> mvMeasurements;

            std::mutex mMutex;
        };

        // Lie Algebra Functions
        Eigen::Matrix3f RightJacobianSO3(const float &x, const float &y, const float &z);
        Eigen::Matrix3f RightJacobianSO3(const Eigen::Vector3f &v);

        Eigen::Matrix3f InverseRightJacobianSO3(const float &x, const float &y, const float &z);
        Eigen::Matrix3f InverseRightJacobianSO3(const Eigen::Vector3f &v);

        Eigen::Matrix3f NormalizeRotation(const Eigen::Matrix3f &R);

    }

} // namespace ORB_SLAM2

#endif // IMUTYPES_H
