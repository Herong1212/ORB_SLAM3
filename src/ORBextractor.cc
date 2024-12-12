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

/**
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2009, Willow Garage, Inc.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the Willow Garage nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 */

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <vector>
#include <iostream>

#include "ORBextractor.h"

using namespace cv;
using namespace std;

namespace ORB_SLAM3
{
    // 理解下面代码的过程中需要用到一些知识：
    // 高斯图像金字塔，参考[https://blog.csdn.net/xbcreal/article/details/52629465]
    //
    // 另外有一篇对这个部分进行简介的帖子，具有很大的参考价值：[https://blog.csdn.net/saber_altolia/article/details/52623513]
    //

    const int PATCH_SIZE = 31;      ///< 使用灰度质心法计算特征点的方向信息时，图像块的大小，或者说是直径
    const int HALF_PATCH_SIZE = 15; ///< 上面这个大小的一半，或者说是半径
    const int EDGE_THRESHOLD = 19;  ///< 算法生成的图像边
    // 生成这个边的目的是进行图像金子塔的生成时，需要对图像进行高斯滤波处理，为了考虑到使滤波后的图像边界处的像素也能够携带有正确的图像信息，
    // 这里作者就将原图像扩大了一个边。

    // ps：这个函数用于计算【某个特征点】的方向，这里是返回角度作为方向，在其他函数里调用
    /**
     * @brief 这个函数用于计算特征点的方向，这里是返回角度作为方向。
     * 计算特征点方向是为了使得提取的特征点具有旋转不变性。
     * 方法是灰度质心法：以【几何中心】和【灰度质心】的连线作为该特征点方向
     * @param[in] image     要进行操作的某层金字塔图像
     * @param[in] pt        当前特征点在这张图像中的坐标
     * @param[in] u_max     图像块的每一行的坐标边界 u_max
     * @return float        返回特征点的角度，范围为[0,360)角度，精度为0.3°
     */
    static float IC_Angle(const Mat &image, Point2f pt, const vector<int> &u_max)
    {
        // 图像的矩，前者是按照图像块的y坐标加权，后者是按照图像块的x坐标加权
        int m_01 = 0, m_10 = 0;

        // 获得这个特征点所在的图像块的中心点坐标灰度值的指针center
        const uchar *center = &image.at<uchar>(cvRound(pt.y), cvRound(pt.x));

        // Treat the center line differently, v=0
        // 这条v=0中心线的计算需要特殊对待
        // 由于是中心行+若干行对，所以PATCH_SIZE应该是个奇数
        for (int u = -HALF_PATCH_SIZE; u <= HALF_PATCH_SIZE; ++u)
            // 注意这里的center下标u可以是负的！中心水平线上的像素按x坐标（也就是u坐标）加权
            m_10 += u * center[u];

        // Go line by line in the circular patch
        // 这里的step1表示这个图像一行包含的字节总数。参考[https://blog.csdn.net/qianqing13579/article/details/45318279]
        int step = (int)image.step1();
        // 注意这里是以v=0中心线为对称轴，然后对称地每成对的两行之间进行遍历，这样处理加快了计算速度
        for (int v = 1; v <= HALF_PATCH_SIZE; ++v)
        {
            // Proceed over the two lines
            // 本来m_01应该是一列一列地计算的，但是由于对称以及坐标x,y正负的原因，可以一次计算两行
            int v_sum = 0;
            // 获取某行像素横坐标的最大范围，注意这里的图像块是圆形的！
            int d = u_max[v];
            // 在坐标范围内挨个像素遍历，实际是一次遍历2个
            //  假设每次处理的两个点坐标，中心线下方为(x,y),中心线上方为(x,-y)
            //  对于某次待处理的两个点：m_10 = Σ x*I(x,y) =  x*I(x,y) + x*I(x,-y) = x*(I(x,y) + I(x,-y))
            //  对于某次待处理的两个点：m_01 = Σ y*I(x,y) =  y*I(x,y) - y*I(x,-y) = y*(I(x,y) - I(x,-y))
            for (int u = -d; u <= d; ++u)
            {
                // 得到需要进行加运算和减运算的像素灰度值
                // val_plus：在中心线下方x=u时的的像素灰度值
                // val_minus：在中心线上方x=u时的像素灰度值
                int val_plus = center[u + v * step], val_minus = center[u - v * step];
                // 在v（y轴）上，2行所有像素灰度值之差
                v_sum += (val_plus - val_minus);
                // u轴（也就是x轴）方向上用u坐标加权和（u坐标也有正负符号），相当于同时计算两行
                m_10 += u * (val_plus + val_minus);
            }
            // 将这一行上的和按照y坐标加权
            m_01 += v * v_sum;
        }

        // 为了加快速度还使用了fastAtan2()函数，输出为[0,360)角度，精度为0.3°
        return fastAtan2((float)m_01, (float)m_10);
    }

    /// 乘数因子，一度对应着多少弧度
    const float factorPI = (float)(CV_PI / 180.f);

    // ps：计算 ORB 特征点的描述子。注意这个是全局的静态函数，只能是在本文件内被调用
    /**
     * @brief 计算 ORB 特征点的描述子。注意这个是全局的静态函数，只能是在本文件内被调用
     * @param[in] kpt       特征点对象
     * @param[in] img       提取出特征点的图像
     * @param[in] pattern   预定义好的随机采样点集
     * @param[out] desc     用作输出变量，保存计算好的描述子，长度为32*8bit
     */
    static void computeOrbDescriptor(const KeyPoint &kpt,
                                     const Mat &img, const Point *pattern,
                                     uchar *desc)
    {
        // 得到特征点的角度，用弧度制表示。kpt.angle 是角度制，范围为[0,360)度
        float angle = (float)kpt.angle * factorPI;
        // 然后计算这个角度的余弦值和正弦值
        float a = (float)cos(angle), b = (float)sin(angle);

        // 获得图像中心指针
        const uchar *center = &img.at<uchar>(cvRound(kpt.pt.y), cvRound(kpt.pt.x));
        // 获得图像的每行的字节数
        const int step = (int)img.step;

// 原始的BRIEF描述子不具有方向信息，通过加入特征点的方向来计算描述子，称之为Steer BRIEF，具有较好旋转不变特性
// 具体地，在计算的时候需要将这里选取的随机点点集的x轴方向旋转到特征点的方向。
// 获得随机“相对点集”中某个idx所对应的点的灰度,这里旋转前坐标为(x,y), 旋转后坐标(x',y')推导:
//  x'= xcos(θ) - ysin(θ),  y'= xsin(θ) + ycos(θ)
#define GET_VALUE(idx)                                               \
    center[cvRound(pattern[idx].x * b + pattern[idx].y * a) * step + \
           cvRound(pattern[idx].x * a - pattern[idx].y * b)]

        // y'* step
        // x'
        // brief描述子由32*8位组成
        // 其中每一位是来自于两个像素点灰度的直接比较，所以每比较出8bit结果，需要16个随机点，这也就是为什么pattern需要+=16的原因
        for (int i = 0; i < 32; ++i, pattern += 16)
        {
            int t0,  // 参与比较的一个特征点的灰度值
                t1,  // 参与比较的另一个特征点的灰度值
                val; // 描述子这个字节的比较结果

            t0 = GET_VALUE(0);
            t1 = GET_VALUE(1);
            val = t0 < t1; // 描述子本字节的 bit0
            t0 = GET_VALUE(2);
            t1 = GET_VALUE(3);
            val |= (t0 < t1) << 1; // 描述子本字节的bit1
            t0 = GET_VALUE(4);
            t1 = GET_VALUE(5);
            val |= (t0 < t1) << 2; // 描述子本字节的bit2
            t0 = GET_VALUE(6);
            t1 = GET_VALUE(7);
            val |= (t0 < t1) << 3; // 描述子本字节的bit3
            t0 = GET_VALUE(8);
            t1 = GET_VALUE(9);
            val |= (t0 < t1) << 4; // 描述子本字节的bit4
            t0 = GET_VALUE(10);
            t1 = GET_VALUE(11);
            val |= (t0 < t1) << 5; // 描述子本字节的bit5
            t0 = GET_VALUE(12);
            t1 = GET_VALUE(13);
            val |= (t0 < t1) << 6; // 描述子本字节的bit6
            t0 = GET_VALUE(14);
            t1 = GET_VALUE(15);
            val |= (t0 < t1) << 7; // 描述子本字节的bit7

            // 保存当前比较的出来的描述子的这个字节
            desc[i] = (uchar)val;
        } // 通过对随机点像素灰度的比较，得出BRIEF描述子，一共是 32 * 8 = 256位

// 为了避免和程序中的其他部分冲突在，在使用完成之后就取消这个宏定义
#undef GET_VALUE
    }

    // 下面就是预先定义好的随机点集，256是指可以提取出256bit的描述子信息，每个bit由一对点比较得来；4=2*2，前面的2是需要两个点（一对点）进行比较，后面的2是一个点有两个坐标
    static int bit_pattern_31_[256 * 4] =
        {
            8, -3, 9, 5 /*mean (0), correlation (0)*/,
            4, 2, 7, -12 /*mean (1.12461e-05), correlation (0.0437584)*/,
            -11, 9, -8, 2 /*mean (3.37382e-05), correlation (0.0617409)*/,
            7, -12, 12, -13 /*mean (5.62303e-05), correlation (0.0636977)*/,
            2, -13, 2, 12 /*mean (0.000134953), correlation (0.085099)*/,
            1, -7, 1, 6 /*mean (0.000528565), correlation (0.0857175)*/,
            -2, -10, -2, -4 /*mean (0.0188821), correlation (0.0985774)*/,
            -13, -13, -11, -8 /*mean (0.0363135), correlation (0.0899616)*/,
            -13, -3, -12, -9 /*mean (0.121806), correlation (0.099849)*/,
            10, 4, 11, 9 /*mean (0.122065), correlation (0.093285)*/,
            -13, -8, -8, -9 /*mean (0.162787), correlation (0.0942748)*/,
            -11, 7, -9, 12 /*mean (0.21561), correlation (0.0974438)*/,
            7, 7, 12, 6 /*mean (0.160583), correlation (0.130064)*/,
            -4, -5, -3, 0 /*mean (0.228171), correlation (0.132998)*/,
            -13, 2, -12, -3 /*mean (0.00997526), correlation (0.145926)*/,
            -9, 0, -7, 5 /*mean (0.198234), correlation (0.143636)*/,
            12, -6, 12, -1 /*mean (0.0676226), correlation (0.16689)*/,
            -3, 6, -2, 12 /*mean (0.166847), correlation (0.171682)*/,
            -6, -13, -4, -8 /*mean (0.101215), correlation (0.179716)*/,
            11, -13, 12, -8 /*mean (0.200641), correlation (0.192279)*/,
            4, 7, 5, 1 /*mean (0.205106), correlation (0.186848)*/,
            5, -3, 10, -3 /*mean (0.234908), correlation (0.192319)*/,
            3, -7, 6, 12 /*mean (0.0709964), correlation (0.210872)*/,
            -8, -7, -6, -2 /*mean (0.0939834), correlation (0.212589)*/,
            -2, 11, -1, -10 /*mean (0.127778), correlation (0.20866)*/,
            -13, 12, -8, 10 /*mean (0.14783), correlation (0.206356)*/,
            -7, 3, -5, -3 /*mean (0.182141), correlation (0.198942)*/,
            -4, 2, -3, 7 /*mean (0.188237), correlation (0.21384)*/,
            -10, -12, -6, 11 /*mean (0.14865), correlation (0.23571)*/,
            5, -12, 6, -7 /*mean (0.222312), correlation (0.23324)*/,
            5, -6, 7, -1 /*mean (0.229082), correlation (0.23389)*/,
            1, 0, 4, -5 /*mean (0.241577), correlation (0.215286)*/,
            9, 11, 11, -13 /*mean (0.00338507), correlation (0.251373)*/,
            4, 7, 4, 12 /*mean (0.131005), correlation (0.257622)*/,
            2, -1, 4, 4 /*mean (0.152755), correlation (0.255205)*/,
            -4, -12, -2, 7 /*mean (0.182771), correlation (0.244867)*/,
            -8, -5, -7, -10 /*mean (0.186898), correlation (0.23901)*/,
            4, 11, 9, 12 /*mean (0.226226), correlation (0.258255)*/,
            0, -8, 1, -13 /*mean (0.0897886), correlation (0.274827)*/,
            -13, -2, -8, 2 /*mean (0.148774), correlation (0.28065)*/,
            -3, -2, -2, 3 /*mean (0.153048), correlation (0.283063)*/,
            -6, 9, -4, -9 /*mean (0.169523), correlation (0.278248)*/,
            8, 12, 10, 7 /*mean (0.225337), correlation (0.282851)*/,
            0, 9, 1, 3 /*mean (0.226687), correlation (0.278734)*/,
            7, -5, 11, -10 /*mean (0.00693882), correlation (0.305161)*/,
            -13, -6, -11, 0 /*mean (0.0227283), correlation (0.300181)*/,
            10, 7, 12, 1 /*mean (0.125517), correlation (0.31089)*/,
            -6, -3, -6, 12 /*mean (0.131748), correlation (0.312779)*/,
            10, -9, 12, -4 /*mean (0.144827), correlation (0.292797)*/,
            -13, 8, -8, -12 /*mean (0.149202), correlation (0.308918)*/,
            -13, 0, -8, -4 /*mean (0.160909), correlation (0.310013)*/,
            3, 3, 7, 8 /*mean (0.177755), correlation (0.309394)*/,
            5, 7, 10, -7 /*mean (0.212337), correlation (0.310315)*/,
            -1, 7, 1, -12 /*mean (0.214429), correlation (0.311933)*/,
            3, -10, 5, 6 /*mean (0.235807), correlation (0.313104)*/,
            2, -4, 3, -10 /*mean (0.00494827), correlation (0.344948)*/,
            -13, 0, -13, 5 /*mean (0.0549145), correlation (0.344675)*/,
            -13, -7, -12, 12 /*mean (0.103385), correlation (0.342715)*/,
            -13, 3, -11, 8 /*mean (0.134222), correlation (0.322922)*/,
            -7, 12, -4, 7 /*mean (0.153284), correlation (0.337061)*/,
            6, -10, 12, 8 /*mean (0.154881), correlation (0.329257)*/,
            -9, -1, -7, -6 /*mean (0.200967), correlation (0.33312)*/,
            -2, -5, 0, 12 /*mean (0.201518), correlation (0.340635)*/,
            -12, 5, -7, 5 /*mean (0.207805), correlation (0.335631)*/,
            3, -10, 8, -13 /*mean (0.224438), correlation (0.34504)*/,
            -7, -7, -4, 5 /*mean (0.239361), correlation (0.338053)*/,
            -3, -2, -1, -7 /*mean (0.240744), correlation (0.344322)*/,
            2, 9, 5, -11 /*mean (0.242949), correlation (0.34145)*/,
            -11, -13, -5, -13 /*mean (0.244028), correlation (0.336861)*/,
            -1, 6, 0, -1 /*mean (0.247571), correlation (0.343684)*/,
            5, -3, 5, 2 /*mean (0.000697256), correlation (0.357265)*/,
            -4, -13, -4, 12 /*mean (0.00213675), correlation (0.373827)*/,
            -9, -6, -9, 6 /*mean (0.0126856), correlation (0.373938)*/,
            -12, -10, -8, -4 /*mean (0.0152497), correlation (0.364237)*/,
            10, 2, 12, -3 /*mean (0.0299933), correlation (0.345292)*/,
            7, 12, 12, 12 /*mean (0.0307242), correlation (0.366299)*/,
            -7, -13, -6, 5 /*mean (0.0534975), correlation (0.368357)*/,
            -4, 9, -3, 4 /*mean (0.099865), correlation (0.372276)*/,
            7, -1, 12, 2 /*mean (0.117083), correlation (0.364529)*/,
            -7, 6, -5, 1 /*mean (0.126125), correlation (0.369606)*/,
            -13, 11, -12, 5 /*mean (0.130364), correlation (0.358502)*/,
            -3, 7, -2, -6 /*mean (0.131691), correlation (0.375531)*/,
            7, -8, 12, -7 /*mean (0.160166), correlation (0.379508)*/,
            -13, -7, -11, -12 /*mean (0.167848), correlation (0.353343)*/,
            1, -3, 12, 12 /*mean (0.183378), correlation (0.371916)*/,
            2, -6, 3, 0 /*mean (0.228711), correlation (0.371761)*/,
            -4, 3, -2, -13 /*mean (0.247211), correlation (0.364063)*/,
            -1, -13, 1, 9 /*mean (0.249325), correlation (0.378139)*/,
            7, 1, 8, -6 /*mean (0.000652272), correlation (0.411682)*/,
            1, -1, 3, 12 /*mean (0.00248538), correlation (0.392988)*/,
            9, 1, 12, 6 /*mean (0.0206815), correlation (0.386106)*/,
            -1, -9, -1, 3 /*mean (0.0364485), correlation (0.410752)*/,
            -13, -13, -10, 5 /*mean (0.0376068), correlation (0.398374)*/,
            7, 7, 10, 12 /*mean (0.0424202), correlation (0.405663)*/,
            12, -5, 12, 9 /*mean (0.0942645), correlation (0.410422)*/,
            6, 3, 7, 11 /*mean (0.1074), correlation (0.413224)*/,
            5, -13, 6, 10 /*mean (0.109256), correlation (0.408646)*/,
            2, -12, 2, 3 /*mean (0.131691), correlation (0.416076)*/,
            3, 8, 4, -6 /*mean (0.165081), correlation (0.417569)*/,
            2, 6, 12, -13 /*mean (0.171874), correlation (0.408471)*/,
            9, -12, 10, 3 /*mean (0.175146), correlation (0.41296)*/,
            -8, 4, -7, 9 /*mean (0.183682), correlation (0.402956)*/,
            -11, 12, -4, -6 /*mean (0.184672), correlation (0.416125)*/,
            1, 12, 2, -8 /*mean (0.191487), correlation (0.386696)*/,
            6, -9, 7, -4 /*mean (0.192668), correlation (0.394771)*/,
            2, 3, 3, -2 /*mean (0.200157), correlation (0.408303)*/,
            6, 3, 11, 0 /*mean (0.204588), correlation (0.411762)*/,
            3, -3, 8, -8 /*mean (0.205904), correlation (0.416294)*/,
            7, 8, 9, 3 /*mean (0.213237), correlation (0.409306)*/,
            -11, -5, -6, -4 /*mean (0.243444), correlation (0.395069)*/,
            -10, 11, -5, 10 /*mean (0.247672), correlation (0.413392)*/,
            -5, -8, -3, 12 /*mean (0.24774), correlation (0.411416)*/,
            -10, 5, -9, 0 /*mean (0.00213675), correlation (0.454003)*/,
            8, -1, 12, -6 /*mean (0.0293635), correlation (0.455368)*/,
            4, -6, 6, -11 /*mean (0.0404971), correlation (0.457393)*/,
            -10, 12, -8, 7 /*mean (0.0481107), correlation (0.448364)*/,
            4, -2, 6, 7 /*mean (0.050641), correlation (0.455019)*/,
            -2, 0, -2, 12 /*mean (0.0525978), correlation (0.44338)*/,
            -5, -8, -5, 2 /*mean (0.0629667), correlation (0.457096)*/,
            7, -6, 10, 12 /*mean (0.0653846), correlation (0.445623)*/,
            -9, -13, -8, -8 /*mean (0.0858749), correlation (0.449789)*/,
            -5, -13, -5, -2 /*mean (0.122402), correlation (0.450201)*/,
            8, -8, 9, -13 /*mean (0.125416), correlation (0.453224)*/,
            -9, -11, -9, 0 /*mean (0.130128), correlation (0.458724)*/,
            1, -8, 1, -2 /*mean (0.132467), correlation (0.440133)*/,
            7, -4, 9, 1 /*mean (0.132692), correlation (0.454)*/,
            -2, 1, -1, -4 /*mean (0.135695), correlation (0.455739)*/,
            11, -6, 12, -11 /*mean (0.142904), correlation (0.446114)*/,
            -12, -9, -6, 4 /*mean (0.146165), correlation (0.451473)*/,
            3, 7, 7, 12 /*mean (0.147627), correlation (0.456643)*/,
            5, 5, 10, 8 /*mean (0.152901), correlation (0.455036)*/,
            0, -4, 2, 8 /*mean (0.167083), correlation (0.459315)*/,
            -9, 12, -5, -13 /*mean (0.173234), correlation (0.454706)*/,
            0, 7, 2, 12 /*mean (0.18312), correlation (0.433855)*/,
            -1, 2, 1, 7 /*mean (0.185504), correlation (0.443838)*/,
            5, 11, 7, -9 /*mean (0.185706), correlation (0.451123)*/,
            3, 5, 6, -8 /*mean (0.188968), correlation (0.455808)*/,
            -13, -4, -8, 9 /*mean (0.191667), correlation (0.459128)*/,
            -5, 9, -3, -3 /*mean (0.193196), correlation (0.458364)*/,
            -4, -7, -3, -12 /*mean (0.196536), correlation (0.455782)*/,
            6, 5, 8, 0 /*mean (0.1972), correlation (0.450481)*/,
            -7, 6, -6, 12 /*mean (0.199438), correlation (0.458156)*/,
            -13, 6, -5, -2 /*mean (0.211224), correlation (0.449548)*/,
            1, -10, 3, 10 /*mean (0.211718), correlation (0.440606)*/,
            4, 1, 8, -4 /*mean (0.213034), correlation (0.443177)*/,
            -2, -2, 2, -13 /*mean (0.234334), correlation (0.455304)*/,
            2, -12, 12, 12 /*mean (0.235684), correlation (0.443436)*/,
            -2, -13, 0, -6 /*mean (0.237674), correlation (0.452525)*/,
            4, 1, 9, 3 /*mean (0.23962), correlation (0.444824)*/,
            -6, -10, -3, -5 /*mean (0.248459), correlation (0.439621)*/,
            -3, -13, -1, 1 /*mean (0.249505), correlation (0.456666)*/,
            7, 5, 12, -11 /*mean (0.00119208), correlation (0.495466)*/,
            4, -2, 5, -7 /*mean (0.00372245), correlation (0.484214)*/,
            -13, 9, -9, -5 /*mean (0.00741116), correlation (0.499854)*/,
            7, 1, 8, 6 /*mean (0.0208952), correlation (0.499773)*/,
            7, -8, 7, 6 /*mean (0.0220085), correlation (0.501609)*/,
            -7, -4, -7, 1 /*mean (0.0233806), correlation (0.496568)*/,
            -8, 11, -7, -8 /*mean (0.0236505), correlation (0.489719)*/,
            -13, 6, -12, -8 /*mean (0.0268781), correlation (0.503487)*/,
            2, 4, 3, 9 /*mean (0.0323324), correlation (0.501938)*/,
            10, -5, 12, 3 /*mean (0.0399235), correlation (0.494029)*/,
            -6, -5, -6, 7 /*mean (0.0420153), correlation (0.486579)*/,
            8, -3, 9, -8 /*mean (0.0548021), correlation (0.484237)*/,
            2, -12, 2, 8 /*mean (0.0616622), correlation (0.496642)*/,
            -11, -2, -10, 3 /*mean (0.0627755), correlation (0.498563)*/,
            -12, -13, -7, -9 /*mean (0.0829622), correlation (0.495491)*/,
            -11, 0, -10, -5 /*mean (0.0843342), correlation (0.487146)*/,
            5, -3, 11, 8 /*mean (0.0929937), correlation (0.502315)*/,
            -2, -13, -1, 12 /*mean (0.113327), correlation (0.48941)*/,
            -1, -8, 0, 9 /*mean (0.132119), correlation (0.467268)*/,
            -13, -11, -12, -5 /*mean (0.136269), correlation (0.498771)*/,
            -10, -2, -10, 11 /*mean (0.142173), correlation (0.498714)*/,
            -3, 9, -2, -13 /*mean (0.144141), correlation (0.491973)*/,
            2, -3, 3, 2 /*mean (0.14892), correlation (0.500782)*/,
            -9, -13, -4, 0 /*mean (0.150371), correlation (0.498211)*/,
            -4, 6, -3, -10 /*mean (0.152159), correlation (0.495547)*/,
            -4, 12, -2, -7 /*mean (0.156152), correlation (0.496925)*/,
            -6, -11, -4, 9 /*mean (0.15749), correlation (0.499222)*/,
            6, -3, 6, 11 /*mean (0.159211), correlation (0.503821)*/,
            -13, 11, -5, 5 /*mean (0.162427), correlation (0.501907)*/,
            11, 11, 12, 6 /*mean (0.16652), correlation (0.497632)*/,
            7, -5, 12, -2 /*mean (0.169141), correlation (0.484474)*/,
            -1, 12, 0, 7 /*mean (0.169456), correlation (0.495339)*/,
            -4, -8, -3, -2 /*mean (0.171457), correlation (0.487251)*/,
            -7, 1, -6, 7 /*mean (0.175), correlation (0.500024)*/,
            -13, -12, -8, -13 /*mean (0.175866), correlation (0.497523)*/,
            -7, -2, -6, -8 /*mean (0.178273), correlation (0.501854)*/,
            -8, 5, -6, -9 /*mean (0.181107), correlation (0.494888)*/,
            -5, -1, -4, 5 /*mean (0.190227), correlation (0.482557)*/,
            -13, 7, -8, 10 /*mean (0.196739), correlation (0.496503)*/,
            1, 5, 5, -13 /*mean (0.19973), correlation (0.499759)*/,
            1, 0, 10, -13 /*mean (0.204465), correlation (0.49873)*/,
            9, 12, 10, -1 /*mean (0.209334), correlation (0.49063)*/,
            5, -8, 10, -9 /*mean (0.211134), correlation (0.503011)*/,
            -1, 11, 1, -13 /*mean (0.212), correlation (0.499414)*/,
            -9, -3, -6, 2 /*mean (0.212168), correlation (0.480739)*/,
            -1, -10, 1, 12 /*mean (0.212731), correlation (0.502523)*/,
            -13, 1, -8, -10 /*mean (0.21327), correlation (0.489786)*/,
            8, -11, 10, -6 /*mean (0.214159), correlation (0.488246)*/,
            2, -13, 3, -6 /*mean (0.216993), correlation (0.50287)*/,
            7, -13, 12, -9 /*mean (0.223639), correlation (0.470502)*/,
            -10, -10, -5, -7 /*mean (0.224089), correlation (0.500852)*/,
            -10, -8, -8, -13 /*mean (0.228666), correlation (0.502629)*/,
            4, -6, 8, 5 /*mean (0.22906), correlation (0.498305)*/,
            3, 12, 8, -13 /*mean (0.233378), correlation (0.503825)*/,
            -4, 2, -3, -3 /*mean (0.234323), correlation (0.476692)*/,
            5, -13, 10, -12 /*mean (0.236392), correlation (0.475462)*/,
            4, -13, 5, -1 /*mean (0.236842), correlation (0.504132)*/,
            -9, 9, -4, 3 /*mean (0.236977), correlation (0.497739)*/,
            0, 3, 3, -9 /*mean (0.24314), correlation (0.499398)*/,
            -12, 1, -6, 1 /*mean (0.243297), correlation (0.489447)*/,
            3, 2, 4, -8 /*mean (0.00155196), correlation (0.553496)*/,
            -10, -10, -10, 9 /*mean (0.00239541), correlation (0.54297)*/,
            8, -13, 12, 12 /*mean (0.0034413), correlation (0.544361)*/,
            -8, -12, -6, -5 /*mean (0.003565), correlation (0.551225)*/,
            2, 2, 3, 7 /*mean (0.00835583), correlation (0.55285)*/,
            10, 6, 11, -8 /*mean (0.00885065), correlation (0.540913)*/,
            6, 8, 8, -12 /*mean (0.0101552), correlation (0.551085)*/,
            -7, 10, -6, 5 /*mean (0.0102227), correlation (0.533635)*/,
            -3, -9, -3, 9 /*mean (0.0110211), correlation (0.543121)*/,
            -1, -13, -1, 5 /*mean (0.0113473), correlation (0.550173)*/,
            -3, -7, -3, 4 /*mean (0.0140913), correlation (0.554774)*/,
            -8, -2, -8, 3 /*mean (0.017049), correlation (0.55461)*/,
            4, 2, 12, 12 /*mean (0.01778), correlation (0.546921)*/,
            2, -5, 3, 11 /*mean (0.0224022), correlation (0.549667)*/,
            6, -9, 11, -13 /*mean (0.029161), correlation (0.546295)*/,
            3, -1, 7, 12 /*mean (0.0303081), correlation (0.548599)*/,
            11, -1, 12, 4 /*mean (0.0355151), correlation (0.523943)*/,
            -3, 0, -3, 6 /*mean (0.0417904), correlation (0.543395)*/,
            4, -11, 4, 12 /*mean (0.0487292), correlation (0.542818)*/,
            2, -4, 2, 1 /*mean (0.0575124), correlation (0.554888)*/,
            -10, -6, -8, 1 /*mean (0.0594242), correlation (0.544026)*/,
            -13, 7, -11, 1 /*mean (0.0597391), correlation (0.550524)*/,
            -13, 12, -11, -13 /*mean (0.0608974), correlation (0.55383)*/,
            6, 0, 11, -13 /*mean (0.065126), correlation (0.552006)*/,
            0, -1, 1, 4 /*mean (0.074224), correlation (0.546372)*/,
            -13, 3, -9, -2 /*mean (0.0808592), correlation (0.554875)*/,
            -9, 8, -6, -3 /*mean (0.0883378), correlation (0.551178)*/,
            -13, -6, -8, -2 /*mean (0.0901035), correlation (0.548446)*/,
            5, -9, 8, 10 /*mean (0.0949843), correlation (0.554694)*/,
            2, 7, 3, -9 /*mean (0.0994152), correlation (0.550979)*/,
            -1, -6, -1, -1 /*mean (0.10045), correlation (0.552714)*/,
            9, 5, 11, -2 /*mean (0.100686), correlation (0.552594)*/,
            11, -3, 12, -8 /*mean (0.101091), correlation (0.532394)*/,
            3, 0, 3, 5 /*mean (0.101147), correlation (0.525576)*/,
            -1, 4, 0, 10 /*mean (0.105263), correlation (0.531498)*/,
            3, -6, 4, 5 /*mean (0.110785), correlation (0.540491)*/,
            -13, 0, -10, 5 /*mean (0.112798), correlation (0.536582)*/,
            5, 8, 12, 11 /*mean (0.114181), correlation (0.555793)*/,
            8, 9, 9, -6 /*mean (0.117431), correlation (0.553763)*/,
            7, -4, 8, -12 /*mean (0.118522), correlation (0.553452)*/,
            -10, 4, -10, 9 /*mean (0.12094), correlation (0.554785)*/,
            7, 3, 12, 4 /*mean (0.122582), correlation (0.555825)*/,
            9, -7, 10, -2 /*mean (0.124978), correlation (0.549846)*/,
            7, 0, 12, -2 /*mean (0.127002), correlation (0.537452)*/,
            -1, -6, 0, -11 /*mean (0.127148), correlation (0.547401)*/
    };

    // notice：特征点提取器的构造函数
    ORBextractor::ORBextractor(int _nfeatures, float _scaleFactor, int _nlevels,
                               int _iniThFAST, int _minThFAST) : nfeatures(_nfeatures), scaleFactor(_scaleFactor), nlevels(_nlevels),
                                                                 iniThFAST(_iniThFAST), minThFAST(_minThFAST)
    {
        mvScaleFactor.resize(nlevels);
        mvLevelSigma2.resize(nlevels);
        mvScaleFactor[0] = 1.0f;
        mvLevelSigma2[0] = 1.0f;
        for (int i = 1; i < nlevels; i++)
        {
            mvScaleFactor[i] = mvScaleFactor[i - 1] * scaleFactor;
            mvLevelSigma2[i] = mvScaleFactor[i] * mvScaleFactor[i];
        }

        mvInvScaleFactor.resize(nlevels);
        mvInvLevelSigma2.resize(nlevels);
        for (int i = 0; i < nlevels; i++)
        {
            mvInvScaleFactor[i] = 1.0f / mvScaleFactor[i];
            mvInvLevelSigma2[i] = 1.0f / mvLevelSigma2[i];
        }

        mvImagePyramid.resize(nlevels);

        mnFeaturesPerLevel.resize(nlevels);
        float factor = 1.0f / scaleFactor;
        float nDesiredFeaturesPerScale = nfeatures * (1 - factor) / (1 - (float)pow((double)factor, (double)nlevels));

        int sumFeatures = 0;
        for (int level = 0; level < nlevels - 1; level++)
        {
            mnFeaturesPerLevel[level] = cvRound(nDesiredFeaturesPerScale);
            sumFeatures += mnFeaturesPerLevel[level];
            nDesiredFeaturesPerScale *= factor;
        }
        mnFeaturesPerLevel[nlevels - 1] = std::max(nfeatures - sumFeatures, 0);

        const int npoints = 512;
        const Point *pattern0 = (const Point *)bit_pattern_31_;
        std::copy(pattern0, pattern0 + npoints, std::back_inserter(pattern));

        // This is for orientation
        //  pre-compute the end of a row in a circular patch
        umax.resize(HALF_PATCH_SIZE + 1);

        int v, v0, vmax = cvFloor(HALF_PATCH_SIZE * sqrt(2.f) / 2 + 1);
        int vmin = cvCeil(HALF_PATCH_SIZE * sqrt(2.f) / 2);
        const double hp2 = HALF_PATCH_SIZE * HALF_PATCH_SIZE;
        for (v = 0; v <= vmax; ++v)
            umax[v] = cvRound(sqrt(hp2 - v * v));

        // Make sure we are symmetric
        for (v = HALF_PATCH_SIZE, v0 = 0; v >= vmin; --v)
        {
            while (umax[v0] == umax[v0 + 1])
                ++v0;
            umax[v] = v0;
            ++v0;
        }
    }

    // ps：计算特征点的方向
    static void computeOrientation(const Mat &image, vector<KeyPoint> &keypoints, const vector<int> &umax)
    {
        // 遍历所有的特征点
        for (vector<KeyPoint>::iterator keypoint = keypoints.begin(), keypointEnd = keypoints.end(); keypoint != keypointEnd; ++keypoint)
        {
            // 调用上面的 IC_Angle() 函数计算这个特征点的方向
            keypoint->angle = IC_Angle(image, keypoint->pt, umax);
        }
    }

    // ps：将提取器节点分成4个子节点，同时也完成图像区域的划分、特征点归属的划分，以及相关标志位的置位
    void ExtractorNode::DivideNode(ExtractorNode &n1, // 提取器节点1：左上
                                   ExtractorNode &n2, // 提取器节点1：右上
                                   ExtractorNode &n3, // 提取器节点1：左下
                                   ExtractorNode &n4) // 提取器节点1：右下
    {
        // 得到当前提取器节点所在图像区域的一半长宽，当然结果需要取整
        const int halfX = ceil(static_cast<float>(UR.x - UL.x) / 2);
        const int halfY = ceil(static_cast<float>(BR.y - UL.y) / 2);

        // 下面的操作大同小异，将一个图像区域再细分成为四个小图像区块
        // n1 存储左上区域的边界
        n1.UL = UL;
        n1.UR = cv::Point2i(UL.x + halfX, UL.y);
        n1.BL = cv::Point2i(UL.x, UL.y + halfY);
        n1.BR = cv::Point2i(UL.x + halfX, UL.y + halfY);
        // 用来存储在该节点对应的图像网格中提取出来的特征点的 vector
        n1.vKeys.reserve(vKeys.size());

        // n2 存储右上区域的边界
        n2.UL = n1.UR;
        n2.UR = UR;
        n2.BL = n1.BR;
        n2.BR = cv::Point2i(UR.x, UL.y + halfY);
        n2.vKeys.reserve(vKeys.size());

        // n3 存储左下区域的边界
        n3.UL = n1.BL;
        n3.UR = n1.BR;
        n3.BL = BL;
        n3.BR = cv::Point2i(n1.BR.x, BL.y);
        n3.vKeys.reserve(vKeys.size());

        // n4 存储右下区域的边界
        n4.UL = n3.UR;
        n4.UR = n2.BR;
        n4.BL = n3.BR;
        n4.BR = BR;
        n4.vKeys.reserve(vKeys.size());

        // 遍历当前提取器节点的vkeys中存储的特征点
        // Associate points to childs
        for (size_t i = 0; i < vKeys.size(); i++)
        {
            // 获取这个特征点对象
            const cv::KeyPoint &kp = vKeys[i];

            // 判断这个特征点在当前特征点提取器节点图像的哪个区域，更严格地说是属于那个子图像区块
            // 然后就将这个特征点追加到那个特征点提取器节点的vkeys中
            // NOTICE BUG REVIEW 这里也是直接进行比较的，但是特征点的坐标是在“半径扩充图像”坐标系下的，而节点区域的坐标则是在“边缘扩充图像”坐标系下的
            if (kp.pt.x < n1.UR.x)
            {
                if (kp.pt.y < n1.BR.y)
                    n1.vKeys.push_back(kp);
                else
                    n3.vKeys.push_back(kp);
            }
            else if (kp.pt.y < n1.BR.y)
                n2.vKeys.push_back(kp);
            else
                n4.vKeys.push_back(kp);
        }

        // 遍历当前提取器节点的vkeys中存储的特征点
        // 判断每个子特征点提取器节点所在的图像中特征点的数目（就是分配给子节点的特征点数目），然后做标记
        // 这里判断是否数目等于1的目的是确定这个节点还能不能再向下进行分裂
        if (n1.vKeys.size() == 1)
            n1.bNoMore = true;
        if (n2.vKeys.size() == 1)
            n2.bNoMore = true;
        if (n3.vKeys.size() == 1)
            n3.bNoMore = true;
        if (n4.vKeys.size() == 1)
            n4.bNoMore = true;
    }

    static bool compareNodes(pair<int, ExtractorNode *> &e1, pair<int, ExtractorNode *> &e2)
    {
        if (e1.first < e2.first)
        {
            return true;
        }
        else if (e1.first > e2.first)
        {
            return false;
        }
        else
        {
            if (e1.second->UL.x < e2.second->UL.x)
            {
                return true;
            }
            else
            {
                return false;
            }
        }
    }

    // ps：使用四叉树法对一个图像金字塔图层中的特征点进行平均和分发
    /**
     * @brief 使用四叉树法对一个图像金字塔图层中的特征点进行平均和分发
     *
     * @param[in] vToDistributeKeys     等待进行分配到四叉树中的特征点
     * @param[in] minX                  当前图层的图像的边界，坐标都是在“半径扩充图像”坐标系下的坐标
     * @param[in] maxX
     * @param[in] minY
     * @param[in] maxY
     * @param[in] N                     希望提取出的特征点个数
     * @param[in] level                 指定的金字塔图层，并未使用
     * @return vector<cv::KeyPoint>     已经均匀分散好的特征点vector容器
     */
    vector<cv::KeyPoint> ORBextractor::DistributeOctTree(const vector<cv::KeyPoint> &vToDistributeKeys, const int &minX,
                                                         const int &maxX, const int &minY, const int &maxY, const int &N, const int &level)
    {
        // Compute how many initial nodes
        const int nIni = round(static_cast<float>(maxX - minX) / (maxY - minY));

        const float hX = static_cast<float>(maxX - minX) / nIni;

        list<ExtractorNode> lNodes;

        vector<ExtractorNode *> vpIniNodes;
        vpIniNodes.resize(nIni);

        for (int i = 0; i < nIni; i++)
        {
            ExtractorNode ni;
            ni.UL = cv::Point2i(hX * static_cast<float>(i), 0);
            ni.UR = cv::Point2i(hX * static_cast<float>(i + 1), 0);
            ni.BL = cv::Point2i(ni.UL.x, maxY - minY);
            ni.BR = cv::Point2i(ni.UR.x, maxY - minY);
            ni.vKeys.reserve(vToDistributeKeys.size());

            lNodes.push_back(ni);
            vpIniNodes[i] = &lNodes.back();
        }

        // Associate points to childs
        for (size_t i = 0; i < vToDistributeKeys.size(); i++)
        {
            const cv::KeyPoint &kp = vToDistributeKeys[i];
            vpIniNodes[kp.pt.x / hX]->vKeys.push_back(kp);
        }

        list<ExtractorNode>::iterator lit = lNodes.begin();

        while (lit != lNodes.end())
        {
            if (lit->vKeys.size() == 1)
            {
                lit->bNoMore = true;
                lit++;
            }
            else if (lit->vKeys.empty())
                lit = lNodes.erase(lit);
            else
                lit++;
        }

        bool bFinish = false;

        int iteration = 0;

        vector<pair<int, ExtractorNode *>> vSizeAndPointerToNode;
        vSizeAndPointerToNode.reserve(lNodes.size() * 4);

        while (!bFinish)
        {
            iteration++;

            int prevSize = lNodes.size();

            lit = lNodes.begin();

            int nToExpand = 0;

            vSizeAndPointerToNode.clear();

            while (lit != lNodes.end())
            {
                if (lit->bNoMore)
                {
                    // If node only contains one point do not subdivide and continue
                    lit++;
                    continue;
                }
                else
                {
                    // If more than one point, subdivide
                    ExtractorNode n1, n2, n3, n4;
                    lit->DivideNode(n1, n2, n3, n4);

                    // Add childs if they contain points
                    if (n1.vKeys.size() > 0)
                    {
                        lNodes.push_front(n1);
                        if (n1.vKeys.size() > 1)
                        {
                            nToExpand++;
                            vSizeAndPointerToNode.push_back(make_pair(n1.vKeys.size(), &lNodes.front()));
                            lNodes.front().lit = lNodes.begin();
                        }
                    }
                    if (n2.vKeys.size() > 0)
                    {
                        lNodes.push_front(n2);
                        if (n2.vKeys.size() > 1)
                        {
                            nToExpand++;
                            vSizeAndPointerToNode.push_back(make_pair(n2.vKeys.size(), &lNodes.front()));
                            lNodes.front().lit = lNodes.begin();
                        }
                    }
                    if (n3.vKeys.size() > 0)
                    {
                        lNodes.push_front(n3);
                        if (n3.vKeys.size() > 1)
                        {
                            nToExpand++;
                            vSizeAndPointerToNode.push_back(make_pair(n3.vKeys.size(), &lNodes.front()));
                            lNodes.front().lit = lNodes.begin();
                        }
                    }
                    if (n4.vKeys.size() > 0)
                    {
                        lNodes.push_front(n4);
                        if (n4.vKeys.size() > 1)
                        {
                            nToExpand++;
                            vSizeAndPointerToNode.push_back(make_pair(n4.vKeys.size(), &lNodes.front()));
                            lNodes.front().lit = lNodes.begin();
                        }
                    }

                    lit = lNodes.erase(lit);
                    continue;
                }
            }

            // Finish if there are more nodes than required features or all nodes contain just one point
            if ((int)lNodes.size() >= N || (int)lNodes.size() == prevSize)
            {
                bFinish = true;
            }
            else if (((int)lNodes.size() + nToExpand * 3) > N)
            {

                while (!bFinish)
                {

                    prevSize = lNodes.size();

                    vector<pair<int, ExtractorNode *>> vPrevSizeAndPointerToNode = vSizeAndPointerToNode;
                    vSizeAndPointerToNode.clear();

                    sort(vPrevSizeAndPointerToNode.begin(), vPrevSizeAndPointerToNode.end(), compareNodes);
                    for (int j = vPrevSizeAndPointerToNode.size() - 1; j >= 0; j--)
                    {
                        ExtractorNode n1, n2, n3, n4;
                        vPrevSizeAndPointerToNode[j].second->DivideNode(n1, n2, n3, n4);

                        // Add childs if they contain points
                        if (n1.vKeys.size() > 0)
                        {
                            lNodes.push_front(n1);
                            if (n1.vKeys.size() > 1)
                            {
                                vSizeAndPointerToNode.push_back(make_pair(n1.vKeys.size(), &lNodes.front()));
                                lNodes.front().lit = lNodes.begin();
                            }
                        }
                        if (n2.vKeys.size() > 0)
                        {
                            lNodes.push_front(n2);
                            if (n2.vKeys.size() > 1)
                            {
                                vSizeAndPointerToNode.push_back(make_pair(n2.vKeys.size(), &lNodes.front()));
                                lNodes.front().lit = lNodes.begin();
                            }
                        }
                        if (n3.vKeys.size() > 0)
                        {
                            lNodes.push_front(n3);
                            if (n3.vKeys.size() > 1)
                            {
                                vSizeAndPointerToNode.push_back(make_pair(n3.vKeys.size(), &lNodes.front()));
                                lNodes.front().lit = lNodes.begin();
                            }
                        }
                        if (n4.vKeys.size() > 0)
                        {
                            lNodes.push_front(n4);
                            if (n4.vKeys.size() > 1)
                            {
                                vSizeAndPointerToNode.push_back(make_pair(n4.vKeys.size(), &lNodes.front()));
                                lNodes.front().lit = lNodes.begin();
                            }
                        }

                        lNodes.erase(vPrevSizeAndPointerToNode[j].second->lit);

                        if ((int)lNodes.size() >= N)
                            break;
                    }

                    if ((int)lNodes.size() >= N || (int)lNodes.size() == prevSize)
                        bFinish = true;
                }
            }
        }

        // Retain the best point in each node
        vector<cv::KeyPoint> vResultKeys;
        vResultKeys.reserve(nfeatures);
        for (list<ExtractorNode>::iterator lit = lNodes.begin(); lit != lNodes.end(); lit++)
        {
            vector<cv::KeyPoint> &vNodeKeys = lit->vKeys;
            cv::KeyPoint *pKP = &vNodeKeys[0];
            float maxResponse = pKP->response;

            for (size_t k = 1; k < vNodeKeys.size(); k++)
            {
                if (vNodeKeys[k].response > maxResponse)
                {
                    pKP = &vNodeKeys[k];
                    maxResponse = vNodeKeys[k].response;
                }
            }

            vResultKeys.push_back(*pKP);
        }

        return vResultKeys;
    }

    // ps：计算四叉树的特征点，函数名字后面的 OctTree 只是说明了在过滤和分配特征点时所使用的方式
    // 所有的特征点，这里第一层 vector 存储的是某图层里面的所有特征点，第二层 vector 存储的是整个图像金字塔中的所有图层里面的所有特征点，一般是 1000 左右
    void ORBextractor::ComputeKeyPointsOctTree(vector<vector<KeyPoint>> &allKeypoints)
    {
        // 重新调整图像层数
        allKeypoints.resize(nlevels);

        // 图像 cell 的尺寸，是个正方形，可以理解为边长 in 像素坐标
        const float W = 35;

        // 遍历每一层图像，对每一层图像做处理
        for (int level = 0; level < nlevels; ++level)
        {
            // 计算这层图像的坐标边界，注意这里是坐标边界，EDGE_THRESHOLD 指的应该是可以提取特征点的有效图像边界，后面会一直使用“有效图像边界“这个自创名词
            const int minBorderX = EDGE_THRESHOLD - 3; // 这里的 3 是因为在计算 FAST 特征点的时候，需要建立一个半径为 3 的圆
            const int minBorderY = minBorderX;         // minY 的计算就可以直接拷贝上面的计算结果了
            const int maxBorderX = mvImagePyramid[level].cols - EDGE_THRESHOLD + 3;
            const int maxBorderY = mvImagePyramid[level].rows - EDGE_THRESHOLD + 3;

            // 存储需要进行平均分配的特征点
            vector<cv::KeyPoint> vToDistributeKeys;
            // 一般地都是过量采集，所以这里预分配的空间大小是 nfeatures * 10
            vToDistributeKeys.reserve(nfeatures * 10);

            // 计算进行特征点提取的图像区域尺寸
            const float width = (maxBorderX - minBorderX);
            const float height = (maxBorderY - minBorderY);

            // 计算网格在当前层的图像有的行数和列数
            const int nCols = width / W;
            const int nRows = height / W;
            // 计算每个图像网格所占的像素行数和列数
            const int wCell = ceil(width / nCols);
            const int hCell = ceil(height / nRows);

            // 开始遍历图像网格，还是以行开始遍历的
            for (int i = 0; i < nRows; i++)
            {
                // 计算当前网格初始行坐标
                const float iniY = minBorderY + i * hCell;
                // 计算当前网格最大的行坐标，这里的+6=+3+3，即考虑到了多出来3是为了cell边界像素进行FAST特征点提取用
                // 前面的EDGE_THRESHOLD指的应该是提取后的特征点所在的边界，所以minBorderY是考虑了计算半径时候的图像边界
                // 目测一个图像网格的大小是25*25啊
                float maxY = iniY + hCell + 6;

                // 如果初始的行坐标就已经超过了有效的图像边界了，这里的“有效图像”是指原始的、可以提取FAST特征点的图像区域
                if (iniY >= maxBorderY - 3)
                    // 那么就跳过这一行
                    continue;
                // 如果图像的大小导致不能够正好划分出来整齐的图像网格，那么就要委屈最后一行了
                if (maxY > maxBorderY)
                    maxY = maxBorderY;

                // 开始列的遍历
                for (int j = 0; j < nCols; j++)
                {
                    // 计算初始的列坐标
                    const float iniX = minBorderX + j * wCell;
                    // 计算这列网格的最大列坐标，+6 的含义和前面相同
                    float maxX = iniX + wCell + 6;
                    // 判断坐标是否在图像中
                    // TODO 不太能够明白为什么要-6，前面不都是 -3 吗？
                    // ! BUG  正确应该是 maxBorderX-3
                    // if (iniX >= maxBorderX - 6) 修改后👇
                    if (iniX >= maxBorderX - 3)
                        continue;
                    // 如果最大坐标越界那么委屈一下
                    if (maxX > maxBorderX)
                        maxX = maxBorderX;

                    // FAST提取兴趣点, 自适应阈值
                    // 这个向量存储这个cell中的特征点
                    vector<cv::KeyPoint> vKeysCell;

                    // 调用opencv的库函数来检测FAST角点
                    FAST(mvImagePyramid[level].rowRange(iniY, maxY).colRange(iniX, maxX), // 待检测的图像，这里就是当前遍历到的图像块
                         vKeysCell,                                                       // 存储角点位置的容器
                         iniThFAST,                                                       // 检测阈值
                         true);                                                           // 是否使用非极大值抑制

                    // 如果这个图像块中使用【默认的 FAST 检测阈值】没有能够检测到角点
                    if (vKeysCell.empty())
                    {
                        // 那么就使用【更低的阈值】来进行重新检测
                        FAST(mvImagePyramid[level].rowRange(iniY, maxY).colRange(iniX, maxX), vKeysCell, minThFAST, true);
                    }

                    // 当图像cell中检测到FAST角点的时候，执行下面的语句
                    if (!vKeysCell.empty())
                    {
                        // 遍历其中的所有FAST角点
                        for (vector<cv::KeyPoint>::iterator vit = vKeysCell.begin(); vit != vKeysCell.end(); vit++)
                        {
                            // notice 到目前为止，这些角点的坐标都是基于图像cell的，现在我们要先将其恢复到当前的【坐标边界】下的坐标
                            // 这样做是因为在下面使用八叉树法整理特征点的时候将会使用得到这个坐标
                            // 在后面将会被继续转换成为在当前图层的扩充图像坐标系下的坐标
                            (*vit).pt.x += j * wCell;
                            (*vit).pt.y += i * hCell;

                            // 然后将其加入到“等待被分配”的特征点容器中
                            vToDistributeKeys.push_back(*vit);
                        }
                    }

                    // TODO Delete kp---新增部分好像是
                    float scale = mvScaleFactor[level];
                    for (auto vit = vToDistributeKeys.begin(); vit != vToDistributeKeys.end(); vit++)
                    {
                        vit->pt.x += minBorderX;
                        vit->pt.y += minBorderY;
                        vit->pt *= scale;
                    }

                    bool Find = false;
                    for (auto vit_kp = vToDistributeKeys.begin(); vit_kp != vToDistributeKeys.end();)
                    {
                        for (auto vit_area = mvDynamicArea.begin(); vit_area != mvDynamicArea.end(); vit_area++)
                        {
                            Find = false;
                            if (vit_area->contains(vit_kp->pt))
                            {
                                Find = true;
                                vit_kp = vToDistributeKeys.erase(vit_kp);
                                break;
                            }
                        }

                        if (!Find)
                        {
                            ++vit_kp;
                            Find = false;
                        }
                    }

                    float scale_inverse = 1 / scale;
                    for (auto vit = vToDistributeKeys.begin(); vit != vToDistributeKeys.end(); vit++)
                    {
                        vit->pt *= scale_inverse;
                        vit->pt.x -= minBorderX;
                        vit->pt.y -= minBorderY;
                    }
                }
            }

            // 声明一个对当前图层的特征点的容器的引用
            vector<KeyPoint> &keypoints = allKeypoints[level];
            // 并且调整其大小为欲提取出来的特征点个数（当然这里也是扩大了的，因为不可能所有的特征点都是在这一个图层中提取出来的）
            keypoints.reserve(nfeatures);

            // 根据mnFeatuvector<KeyPoint> & keypoints = allKeypoints[level];resPerLevel，即该层的兴趣点数，对特征点进行剔除
            // 返回值是一个保存有特征点的vector容器，含有剔除后的保留下来的特征点
            // 得到的特征点的坐标，依旧是在当前图层下来讲的
            keypoints = DistributeOctTree(vToDistributeKeys,                              // 当前图层提取出来的特征点，也即是等待剔除的特征点
                                          minBorderX, maxBorderX, minBorderY, maxBorderY, // 当前图层图像的边界，而这里的坐标却都是在“边缘扩充图像”下的
                                          mnFeaturesPerLevel[level],                      // 希望保留下来的当前层图像的特征点个数
                                          level);                                         // 当前层图像所在的图层

            // PATCH_SIZE 是对于底层的初始图像来说的，现在要根据当前图层的尺度缩放倍数进行缩放得到缩放后的PATCH大小 和特征点的方向计算有关
            const int scaledPatchSize = PATCH_SIZE * mvScaleFactor[level];

            // 获取剔除过程后保留下来的特征点数目
            const int nkps = keypoints.size();
            for (int i = 0; i < nkps; i++)
            {
                // 对每一个保留下来的特征点，恢复到相对于当前图层“边缘扩充图像下”的坐标系的坐标
                keypoints[i].pt.x += minBorderX;
                keypoints[i].pt.y += minBorderY;
                // 记录特征点来源的图像金字塔图层
                keypoints[i].octave = level;
                // 记录计算方向的patch，缩放后对应的大小， 又被称作为特征点半径
                keypoints[i].size = scaledPatchSize;
            }
        }

        // 然后计算这些特征点的方向信息，注意这里还是分层计算的
        for (int level = 0; level < nlevels; ++level)
        {
            computeOrientation(mvImagePyramid[level], // 对应的图层的图像
                               allKeypoints[level],   // 这个图层中提取并保留下来的特征点容器
                               umax);                 // 以及 PATCH 的横坐标边界
        }
    }

    // 这个函数应该是使用老办法来计算特征点，不用看了，没用到
    void ORBextractor::ComputeKeyPointsOld(std::vector<std::vector<KeyPoint>> &allKeypoints)
    {
        allKeypoints.resize(nlevels);

        float imageRatio = (float)mvImagePyramid[0].cols / mvImagePyramid[0].rows;

        for (int level = 0; level < nlevels; ++level)
        {
            const int nDesiredFeatures = mnFeaturesPerLevel[level];

            const int levelCols = sqrt((float)nDesiredFeatures / (5 * imageRatio));
            const int levelRows = imageRatio * levelCols;

            const int minBorderX = EDGE_THRESHOLD;
            const int minBorderY = minBorderX;
            const int maxBorderX = mvImagePyramid[level].cols - EDGE_THRESHOLD;
            const int maxBorderY = mvImagePyramid[level].rows - EDGE_THRESHOLD;

            const int W = maxBorderX - minBorderX;
            const int H = maxBorderY - minBorderY;
            const int cellW = ceil((float)W / levelCols);
            const int cellH = ceil((float)H / levelRows);

            const int nCells = levelRows * levelCols;
            const int nfeaturesCell = ceil((float)nDesiredFeatures / nCells);

            vector<vector<vector<KeyPoint>>> cellKeyPoints(levelRows, vector<vector<KeyPoint>>(levelCols));

            vector<vector<int>> nToRetain(levelRows, vector<int>(levelCols, 0));
            vector<vector<int>> nTotal(levelRows, vector<int>(levelCols, 0));
            vector<vector<bool>> bNoMore(levelRows, vector<bool>(levelCols, false));
            vector<int> iniXCol(levelCols);
            vector<int> iniYRow(levelRows);
            int nNoMore = 0;
            int nToDistribute = 0;

            float hY = cellH + 6;

            for (int i = 0; i < levelRows; i++)
            {
                const float iniY = minBorderY + i * cellH - 3;
                iniYRow[i] = iniY;

                if (i == levelRows - 1)
                {
                    hY = maxBorderY + 3 - iniY;
                    if (hY <= 0)
                        continue;
                }

                float hX = cellW + 6;

                for (int j = 0; j < levelCols; j++)
                {
                    float iniX;

                    if (i == 0)
                    {
                        iniX = minBorderX + j * cellW - 3;
                        iniXCol[j] = iniX;
                    }
                    else
                    {
                        iniX = iniXCol[j];
                    }

                    if (j == levelCols - 1)
                    {
                        hX = maxBorderX + 3 - iniX;
                        if (hX <= 0)
                            continue;
                    }

                    Mat cellImage = mvImagePyramid[level].rowRange(iniY, iniY + hY).colRange(iniX, iniX + hX);

                    cellKeyPoints[i][j].reserve(nfeaturesCell * 5);

                    FAST(cellImage, cellKeyPoints[i][j], iniThFAST, true);

                    if (cellKeyPoints[i][j].size() <= 3)
                    {
                        cellKeyPoints[i][j].clear();

                        FAST(cellImage, cellKeyPoints[i][j], minThFAST, true);
                    }

                    const int nKeys = cellKeyPoints[i][j].size();
                    nTotal[i][j] = nKeys;

                    if (nKeys > nfeaturesCell)
                    {
                        nToRetain[i][j] = nfeaturesCell;
                        bNoMore[i][j] = false;
                    }
                    else
                    {
                        nToRetain[i][j] = nKeys;
                        nToDistribute += nfeaturesCell - nKeys;
                        bNoMore[i][j] = true;
                        nNoMore++;
                    }
                }
            }

            // Retain by score

            while (nToDistribute > 0 && nNoMore < nCells)
            {
                int nNewFeaturesCell = nfeaturesCell + ceil((float)nToDistribute / (nCells - nNoMore));
                nToDistribute = 0;

                for (int i = 0; i < levelRows; i++)
                {
                    for (int j = 0; j < levelCols; j++)
                    {
                        if (!bNoMore[i][j])
                        {
                            if (nTotal[i][j] > nNewFeaturesCell)
                            {
                                nToRetain[i][j] = nNewFeaturesCell;
                                bNoMore[i][j] = false;
                            }
                            else
                            {
                                nToRetain[i][j] = nTotal[i][j];
                                nToDistribute += nNewFeaturesCell - nTotal[i][j];
                                bNoMore[i][j] = true;
                                nNoMore++;
                            }
                        }
                    }
                }
            }

            vector<KeyPoint> &keypoints = allKeypoints[level];
            keypoints.reserve(nDesiredFeatures * 2);

            const int scaledPatchSize = PATCH_SIZE * mvScaleFactor[level];

            // Retain by score and transform coordinates
            for (int i = 0; i < levelRows; i++)
            {
                for (int j = 0; j < levelCols; j++)
                {
                    vector<KeyPoint> &keysCell = cellKeyPoints[i][j];
                    KeyPointsFilter::retainBest(keysCell, nToRetain[i][j]);
                    if ((int)keysCell.size() > nToRetain[i][j])
                        keysCell.resize(nToRetain[i][j]);

                    for (size_t k = 0, kend = keysCell.size(); k < kend; k++)
                    {
                        keysCell[k].pt.x += iniXCol[j];
                        keysCell[k].pt.y += iniYRow[i];
                        keysCell[k].octave = level;
                        keysCell[k].size = scaledPatchSize;
                        keypoints.push_back(keysCell[k]);
                    }
                }
            }

            if ((int)keypoints.size() > nDesiredFeatures)
            {
                KeyPointsFilter::retainBest(keypoints, nDesiredFeatures);
                keypoints.resize(nDesiredFeatures);
            }
        }

        // and compute orientations
        for (int level = 0; level < nlevels; ++level)
            computeOrientation(mvImagePyramid[level], allKeypoints[level], umax);
    }

    // ps：注意这是一个不属于任何类的全局静态函数，static修饰符限定其只能够被本文件中的函数调用
    /**
     * @brief 计算某层金字塔图像上特征点的描述子
     *
     * @param[in] image                 某层金字塔图像
     * @param[in] keypoints             特征点vector容器
     * @param[out] descriptors          描述子
     * @param[in] pattern               计算描述子使用的固定随机点集
     */
    static void computeDescriptors(const Mat &image, vector<KeyPoint> &keypoints, Mat &descriptors,
                                   const vector<Point> &pattern)
    {
        // 清空保存描述子信息的容器
        descriptors = Mat::zeros((int)keypoints.size(), 32, CV_8UC1);

        // 开始遍历特征点
        for (size_t i = 0; i < keypoints.size(); i++)
            // 计算这个特征点的描述子
            computeOrbDescriptor(keypoints[i], image, &pattern[0], descriptors.ptr((int)i));
    }

    // note：用仿函数（重载括号运算符）方法来计算图像特征点
    /**
     * @brief 用仿函数（重载括号运算符）方法来计算图像特征点
     *
     * @param[in] _image                    输入原始图的图像
     * @param[in] _mask                     掩膜 mask 矩阵
     * @param[in & out] _keypoints          存储特征点关键点的向量
     * @param[in & out] _descriptors        存储特征点描述子的矩阵
     */
    int ORBextractor::operator()(InputArray _image, InputArray _mask, vector<KeyPoint> &_keypoints,
                                 OutputArray _descriptors, std::vector<int> &vLappingArea)
    {
        std::cout << "[ORBextractor]: Max Features: " << nfeatures << std::endl;

        // Step 1 检查图像有效性。如果图像为空，那么就直接返回
        if (_image.empty())
            return -1;

        // 获取图像的大小
        Mat image = _image.getMat();
        // 判断图像的格式是否正确，要求是单通道灰度值
        assert(image.type() == CV_8UC1);

        // Step 2 构建图像金字塔
        // Pre-compute the scale pyramid
        ComputePyramid(image);

        // Step 3 计算图像的特征点，并且将特征点进行均匀化。均匀的特征点可以提高位姿计算精度
        // 存储所有的特征点，注意此处为二维的 vector，第一维存储的是金字塔的层数，第二维存储的是那一层金字塔图像里提取的所有特征点
        vector<vector<KeyPoint>> allKeypoints;
        // 使用四叉树的方式计算每层图像的特征点并进行分配
        ComputeKeyPointsOctTree(allKeypoints);

        // 使用传统的方法提取并平均分配图像的特征点，作者并未使用
        // ComputeKeyPointsOld(allKeypoints);

        // Step 4 拷贝图像描述子到新的矩阵 descriptors
        Mat descriptors;

        // 统计整个图像金字塔中的特征点
        int nkeypoints = 0;
        // 开始遍历每层图像金字塔，并且累加每层的特征点个数
        for (int level = 0; level < nlevels; ++level)
            nkeypoints += (int)allKeypoints[level].size();
        // 如果本图像金字塔中没有任何的特征点
        if (nkeypoints == 0)
            // 通过调用cv::mat类的.realse方法，强制清空矩阵的引用计数，这样就可以强制释放矩阵的数据了
            // 参考[https://blog.csdn.net/giantchen547792075/article/details/9107877]
            _descriptors.release();
        else
        {
            // 如果图像金字塔中有特征点，那么就创建这个存储描述子的矩阵，注意这个矩阵是存储整个图像金字塔中特征点的描述子的
            _descriptors.create(nkeypoints, // 矩阵的行数，对应为特征点的总个数
                                32,         // 矩阵的列数，对应为使用32*8=256位描述子
                                CV_8U);     // 矩阵元素的格式

            // 获取这个描述子的矩阵信息
            // ? 为什么不是直接在参数 _descriptors 上对矩阵内容进行修改，而是重新新建了一个变量，复制矩阵后，在这个新建变量的基础上进行修改？
            descriptors = _descriptors.getMat();
        }

        //_keypoints.clear();
        //_keypoints.reserve(nkeypoints);
        _keypoints = vector<cv::KeyPoint>(nkeypoints);

        // 因为遍历是一层一层进行的，但是描述子那个矩阵是存储整个图像金字塔中特征点的描述子，所以在这里设置了Offset变量来保存“寻址”时的偏移量，
        // 辅助进行在总描述子mat中的定位
        int offset = 0;
        // Modified for speeding up stereo fisheye matching
        int monoIndex = 0, stereoIndex = nkeypoints - 1;
        for (int level = 0; level < nlevels; ++level)
        {
            // 获取在 allKeypoints 中当前层特征点容器的句柄
            vector<KeyPoint> &keypoints = allKeypoints[level];
            // 本层的特征点数
            int nkeypointsLevel = (int)keypoints.size();

            if (nkeypointsLevel == 0)
                continue;

            //  Step 5 对图像进行高斯模糊
            // 深拷贝当前金字塔所在层级的图像
            Mat workingMat = mvImagePyramid[level].clone();

            // 注意：提取特征点的时候，使用的是清晰的原图像；这里计算描述子的时候，为了避免图像噪声的影响，使用了高斯模糊
            GaussianBlur(workingMat,          // 源图像
                         workingMat,          // 输出图像
                         Size(7, 7),          // 高斯滤波器kernel大小，必须为正的奇数
                         2, 2,                // 高斯滤波在 x方向 和 y方向 的标准差
                         BORDER_REFLECT_101); // 边缘拓展点插值类型

            // 开始计算描述子，desc 存储当前图层的描述子
            // Mat desc = descriptors.rowRange(offset, offset + nkeypointsLevel);
            Mat desc = cv::Mat(nkeypointsLevel, 32, CV_8U);
            // Step 6 计算高斯模糊后图像的描述子
            computeDescriptors(workingMat, // 高斯模糊之后的图层图像
                               keypoints,  // 当前图层中的特征点集合
                               desc,       // 存储计算之后的描述子
                               pattern);   // 随机采样点集

            // 更新偏移量的值
            offset += nkeypointsLevel;

            // Step 6 对非第 0 层图像中的特征点的坐标恢复到第 0 层图像（原图像）的坐标系下
            // ? 得到所有层特征点在第0层里的坐标放到_keypoints里面
            // 对于第0层的图像特征点，他们的坐标就不需要再进行恢复了
            float scale = mvScaleFactor[level]; // getScale(level, firstLevel, scaleFactor);
            int i = 0;
            for (vector<KeyPoint>::iterator keypoint = keypoints.begin(),
                                            keypointEnd = keypoints.end();
                 keypoint != keypointEnd; ++keypoint)
            {

                if (level != 0)
                {
                    // 特征点本身直接乘缩放倍数就可以了
                    keypoint->pt *= scale;
                }

                if (keypoint->pt.x >= vLappingArea[0] && keypoint->pt.x <= vLappingArea[1])
                {
                    _keypoints.at(stereoIndex) = (*keypoint);
                    desc.row(i).copyTo(descriptors.row(stereoIndex));
                    stereoIndex--;
                }
                else
                {
                    _keypoints.at(monoIndex) = (*keypoint);
                    desc.row(i).copyTo(descriptors.row(monoIndex));
                    monoIndex++;
                }
                i++;
            }
        }

        std::cout << "[ORBextractor]: extracted " << _keypoints.size() << " KeyPoints" << std::endl;
        return monoIndex;
    }

    // ps：构建图像金字塔，image 是输入的原图像，这个输入图像所有像素都是有效的，也就是说都是可以在其上提取出 FAST 角点的
    void ORBextractor::ComputePyramid(cv::Mat image)
    {
        for (int level = 0; level < nlevels; ++level)
        {
            // 计算当前层的尺寸
            float scale = mvInvScaleFactor[level]; // 1/1.2
            Size sz(cvRound((float)image.cols * scale), cvRound((float)image.rows * scale));

            Size wholeSize(sz.width + EDGE_THRESHOLD * 2, sz.height + EDGE_THRESHOLD * 2);
            Mat temp(wholeSize, image.type()), masktemp;
            mvImagePyramid[level] = temp(Rect(EDGE_THRESHOLD, EDGE_THRESHOLD, sz.width, sz.height));

            // 计算第 0 层以上 resize 后的图像
            // Compute the resized image
            if (level != 0)
            {
                // 将上一层金字塔图像根据设定 sz 缩放到当前层级
                resize(mvImagePyramid[level - 1], // 输入图像
                       mvImagePyramid[level],     // 输出图像
                       sz,                        // 输出图像的尺寸
                       0,                         // 水平方向上的缩放系数，留0表示自动计算
                       0,                         // 垂直方向上的缩放系数，留0表示自动计算
                       INTER_LINEAR);             // 图像缩放的差值算法类型，这里的是【线性插值算法】

                // 把源图像拷贝到目的图像的中央，四面填充指定的像素。图片如果已经拷贝到中间，只填充边界
                // TODO 貌似这样做是因为在计算描述子前，进行高斯滤波的时候，图像边界会导致一些问题，说不明白
                // EDGE_THRESHOLD 指的这个边界的宽度，由于这个边界之外的像素不是原图像素而是算法生成出来的，所以不能够在 EDGE_THRESHOLD 之外提取特征点
                copyMakeBorder(mvImagePyramid[level],                 // 源图像
                               temp,                                  // 目标图像（此时其实就已经有大了一圈的尺寸了）
                               EDGE_THRESHOLD, EDGE_THRESHOLD,        // top & bottom 需要扩展的border大小
                               EDGE_THRESHOLD, EDGE_THRESHOLD,        // left & right 需要扩展的border大小
                               BORDER_REFLECT_101 + BORDER_ISOLATED); // 扩充方式，opencv给出的解释：

                /*Various border types, image boundaries are denoted with '|'
                 * BORDER_REPLICATE:     aaaaaa|abcdefgh|hhhhhhh
                 * BORDER_REFLECT:       fedcba|abcdefgh|hgfedcb
                 * BORDER_REFLECT_101:   gfedcb|abcdefgh|gfedcba
                 * BORDER_WRAP:          cdefgh|abcdefgh|abcdefg
                 * BORDER_CONSTANT:      iiiiii|abcdefgh|iiiiiii  with some specified 'i'
                 */

                // BORDER_ISOLATED	表示对整个图像进行操作
                //  https://docs.opencv.org/3.4.4/d2/de8/group__core__array.html#ga2ac1049c2c3dd25c2b41bffe17658a36

                std::cout << "Level: " << level << ": Size" << sz.width << " x " << sz.height << std::endl;
            }
            else
            {
                // 对于第 0 层原始图像，就直接扩展边界了，无需缩放
                // ? temp 是在循环内部新定义的，在该函数里又作为输出，并没有使用啊！
                copyMakeBorder(image, temp, EDGE_THRESHOLD, EDGE_THRESHOLD, EDGE_THRESHOLD, EDGE_THRESHOLD,
                               BORDER_REFLECT_101);
            }
        }
    }

} // namespace ORB_SLAM
