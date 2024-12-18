//
// Created by yuwenlu on 2022/3/14.
//
#include "YoloDetect.h"

// 构造函数：加载 YOLOv5 模型并准备与之相关的一些数据结构
YoloDetection::YoloDetection()
{
    mModule = torch::jit::load("yolov5s.torchscript.pt");

    std::ifstream f("coco.names");
    std::string name = "";
    while (std::getline(f, name))
    {
        mClassnames.push_back(name);
    }
    mvDynamicNames = {"person", "car", "motorbike", "bus", "train", "truck", "boat", "bird", "cat",
                      "dog", "horse", "sheep", "crow", "bear"};
}

YoloDetection::~YoloDetection()
{
}

bool YoloDetection::Detect()
{
    // step1：输入图像
    cv::Mat img;

    if (mRGB.empty())
    {
        std::cout << "Read RGB failed!" << std::endl;
        return false;
    }

    // step2：预处理图像（resize 和转换颜色）
    //  Preparing input tensor
    cv::resize(mRGB, img, cv::Size(640, 380));
    cv::cvtColor(img, img, cv::COLOR_BGR2RGB); // YOLO 通常处理 RGB 图像，而 OpenCV 默认读取的是 BGR 图像。

    // step3：OpenCV 图像 img 转换为一个 PyTorch 张量（Tensor）
    torch::Tensor imgTensor = torch::from_blob(img.data,                // 图像数据指针
                                               {img.rows, img.cols, 3}, // 指定张量的形状为 [高度, 宽度, 通道数]，即 (380, 640, 3)
                                               torch::kByte);           // 指定数据类型为 uint8（图像通常是 8 位无符号整数）

    imgTensor = imgTensor.permute({2, 0, 1});    // PyTorch 的图像张量通常是 (C, H, W) 的形状，这里我们将通道维度从最后移到最前，变成 (3, 380, 640)，以符合 YOLO 模型的输入格式。
    imgTensor = imgTensor.toType(torch::kFloat); // 将数据类型转换为 float，这是 YOLO 模型所需要的。
    imgTensor = imgTensor.div(255);              // 将像素值归一化到 [0, 1] 之间，YOLO 模型通常会处理归一化后的图像。
    imgTensor = imgTensor.unsqueeze(0);          // 在张量的最前面增加一个维度，形成批次维度。YOLO 模型要求输入是一个批次，即使是单张图像，也需要一个批次维度，形状变为 [1, 3, 380, 640]。

    // step4：目标检测推理（模型前向传播）
    // preds: [?, 15120, 9]
    torch::Tensor preds = mModule.forward({imgTensor}).toTuple()->elements()[0].toTensor();

    // step5：非极大值抑制：目标检测中的后处理操作，用来抑制冗余的框，只保留最有可能是目标的边界框。s
    std::vector<torch::Tensor> dets = YoloDetection::non_max_suppression(preds, 0.4, // 置信度阈值，低于该阈值的框将被忽略
                                                                         0.5);       // 重叠阈值（IoU 阈值），两个框的重叠度大于该值时，保留置信度更高的框，抑制重叠框
    // step6：处理检测结果
    if (dets.size() > 0)
    {
        // Visualize result
        for (size_t i = 0; i < dets[0].sizes()[0]; ++i)
        {
            // 检测框的坐标被按比例缩放回原图的大小（原图尺寸为 mRGB.cols 和 mRGB.rows）
            float left = dets[0][i][0].item().toFloat() * mRGB.cols / 640;
            float top = dets[0][i][1].item().toFloat() * mRGB.rows / 384;
            float right = dets[0][i][2].item().toFloat() * mRGB.cols / 640;
            float bottom = dets[0][i][3].item().toFloat() * mRGB.rows / 384;
            int classID = dets[0][i][5].item().toInt();

            // 对于每个检测到的目标，提取其左上角和右下角的坐标以及类别信息，构造出一个矩形框（cv::Rect2i）
            cv::Rect2i DetectArea(left, top, (right - left), (bottom - top));

            mmDetectMap[mClassnames[classID]].push_back(DetectArea); // 将每个检测到的区域存储到 mmDetectMap 中，按照类别名称存储检测框

            // 动态区域检测：如果该目标属于动态物体（通过名字 mClassnames[classID] 来判断），则将其添加到 mvDynamicArea 中
            if (count(mvDynamicNames.begin(), mvDynamicNames.end(), mClassnames[classID]))
            {
                cv::Rect2i DynamicArea(left, top, (right - left), (bottom - top));
                mvDynamicArea.push_back(DynamicArea);
            }
        }

        // step7：处理没有动态物体的情况
        if (mvDynamicArea.size() == 0)
        {
            // 如果没有检测到任何动态物体，就在 mvDynamicArea 中插入一个默认的空矩形（(1, 1, 1, 1)），这可能是为了避免后续处理中的空数据问题。
            cv::Rect2i tDynamicArea(1, 1, 1, 1);
            mvDynamicArea.push_back(tDynamicArea);
        }
    }
    return true;
}

// 非极大值抑制
vector<torch::Tensor> YoloDetection::non_max_suppression(torch::Tensor preds, float score_thresh, float iou_thresh)
{
    std::vector<torch::Tensor> output;
    for (size_t i = 0; i < preds.sizes()[0]; ++i)
    {
        torch::Tensor pred = preds.select(0, i);

        // Filter by scores
        torch::Tensor scores = pred.select(1, 4) * std::get<0>(torch::max(pred.slice(1, 5, pred.sizes()[1]), 1));
        pred = torch::index_select(pred, 0, torch::nonzero(scores > score_thresh).select(1, 0));
        if (pred.sizes()[0] == 0)
            continue;

        // (center_x, center_y, w, h) to (left, top, right, bottom)
        pred.select(1, 0) = pred.select(1, 0) - pred.select(1, 2) / 2;
        pred.select(1, 1) = pred.select(1, 1) - pred.select(1, 3) / 2;
        pred.select(1, 2) = pred.select(1, 0) + pred.select(1, 2);
        pred.select(1, 3) = pred.select(1, 1) + pred.select(1, 3);

        // Computing scores and classes
        std::tuple<torch::Tensor, torch::Tensor> max_tuple = torch::max(pred.slice(1, 5, pred.sizes()[1]), 1);
        pred.select(1, 4) = pred.select(1, 4) * std::get<0>(max_tuple);
        pred.select(1, 5) = std::get<1>(max_tuple);

        torch::Tensor dets = pred.slice(1, 0, 6);

        torch::Tensor keep = torch::empty({dets.sizes()[0]});
        torch::Tensor areas = (dets.select(1, 3) - dets.select(1, 1)) * (dets.select(1, 2) - dets.select(1, 0));
        std::tuple<torch::Tensor, torch::Tensor> indexes_tuple = torch::sort(dets.select(1, 4), 0, 1);
        torch::Tensor v = std::get<0>(indexes_tuple);
        torch::Tensor indexes = std::get<1>(indexes_tuple);
        int count = 0;
        while (indexes.sizes()[0] > 0)
        {
            keep[count] = (indexes[0].item().toInt());
            count += 1;

            // Computing overlaps
            torch::Tensor lefts = torch::empty(indexes.sizes()[0] - 1);
            torch::Tensor tops = torch::empty(indexes.sizes()[0] - 1);
            torch::Tensor rights = torch::empty(indexes.sizes()[0] - 1);
            torch::Tensor bottoms = torch::empty(indexes.sizes()[0] - 1);
            torch::Tensor widths = torch::empty(indexes.sizes()[0] - 1);
            torch::Tensor heights = torch::empty(indexes.sizes()[0] - 1);
            for (size_t i = 0; i < indexes.sizes()[0] - 1; ++i)
            {
                lefts[i] = std::max(dets[indexes[0]][0].item().toFloat(), dets[indexes[i + 1]][0].item().toFloat());
                tops[i] = std::max(dets[indexes[0]][1].item().toFloat(), dets[indexes[i + 1]][1].item().toFloat());
                rights[i] = std::min(dets[indexes[0]][2].item().toFloat(), dets[indexes[i + 1]][2].item().toFloat());
                bottoms[i] = std::min(dets[indexes[0]][3].item().toFloat(), dets[indexes[i + 1]][3].item().toFloat());
                widths[i] = std::max(float(0), rights[i].item().toFloat() - lefts[i].item().toFloat());
                heights[i] = std::max(float(0), bottoms[i].item().toFloat() - tops[i].item().toFloat());
            }
            torch::Tensor overlaps = widths * heights;

            // FIlter by IOUs
            torch::Tensor ious = overlaps / (areas.select(0, indexes[0].item().toInt()) + torch::index_select(areas, 0, indexes.slice(0, 1, indexes.sizes()[0])) - overlaps);
            indexes = torch::index_select(indexes, 0, torch::nonzero(ious <= iou_thresh).select(1, 0) + 1);
        }
        keep = keep.toType(torch::kInt64);
        output.push_back(torch::index_select(dets, 0, keep.slice(0, 0, count)));
    }
    return output;
}

// 将传入的 RGB 图像 RGB 存储到类的成员变量 mRGB 中
void YoloDetection::GetImage(cv::Mat &RGB)
{
    mRGB = RGB;
}

void YoloDetection::ClearImage()
{
    mRGB = 0;
}

void YoloDetection::ClearArea()
{
    mvPersonArea.clear();
}
