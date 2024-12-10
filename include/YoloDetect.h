//
// Created by yuwenlu on 2022/3/14.
//
#ifndef YOLO_DETECT_H
#define YOLO_DETECT_H

#include <opencv2/opencv.hpp>
#include <torch/script.h>
#include <algorithm>
#include <iostream>
#include <utility>
#include <time.h>

using namespace std;

class YoloDetection
{
public:
    YoloDetection();
    ~YoloDetection();
    void GetImage(cv::Mat &RGB);
    void ClearImage();
    bool Detect();
    void ClearArea();
    vector<cv::Rect2i> mvPersonArea = {};
    vector<torch::Tensor> non_max_suppression(torch::Tensor preds, float score_thresh = 0.5, float iou_thresh = 0.5);

public:
    cv::Mat mRGB;
    torch::jit::script::Module mModule;   // mModule 是 YOLO 模型的一个实例，通常是一个已训练的神经网络模型（例如 YOLOv5）
    std::vector<std::string> mClassnames; // mClassnames[classID] 是目标类别的名称

    // 6-28
    vector<string> mvDynamicNames;               // 一个用来存储动态物体的 类别名。如 "person"，"car"
    vector<cv::Rect2i> mvDynamicArea;            // 一个用来保存 动态物体区域 的变量。它存储的是图像中被检测到的动态物体的位置。
    map<string, vector<cv::Rect2i>> mmDetectMap; // 一个 物体类别和对应检测区域 的映射关系，first -- string 的类别名称；second -- vector<cv::Rect2i> 的检测框列表，它记录了每个检测到的物体（比如“行人”、“车子”）的 位置区域。
};

#endif // YOLO_DETECT_H
