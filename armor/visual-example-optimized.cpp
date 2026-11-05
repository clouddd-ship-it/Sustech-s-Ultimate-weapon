#include <opencv2/opencv.hpp>
#include <iostream>
#include <vector>
#include <cmath>
#include <numeric>
#include <algorithm>

using namespace std;
using namespace cv;

// 定义装甲板的数据结构，方便后续存储和传递给 PnP
struct Armor {
    Point2f center;             // 装甲板中心点
    vector<Point2f> corners;    // 装甲板的四个角点 (左上, 右上, 右下, 左下)
    Rect bounding_box;          // 正外接矩形 (用于画框)
};

// ================= 模块 1：图像预处理 =================
// 改进：GaussianBlur 移到 threshold 之前，避免对二值图做无意义模糊
Mat preprocess(const Mat& frame, bool is_red) {
    Mat channels[3];
    split(frame, channels); // 分离 BGR 通道

    Mat diff;
    if (is_red) {
        subtract(channels[2], channels[0], diff);
    } else {
        subtract(channels[0], channels[2], diff);
    }

    // 先平滑再二值化，减少噪点导致的锯齿边缘
    Mat blurred;
    GaussianBlur(diff, blurred, Size(3, 3), 0);

    Mat binary;
    threshold(blurred, binary, 100, 255, THRESH_BINARY);

    // 形态学操作去噪
    Mat kernel = getStructuringElement(MORPH_RECT, Size(3, 3));
    morphologyEx(binary, binary, MORPH_OPEN, kernel);  // 开运算：去小白点
    morphologyEx(binary, binary, MORPH_CLOSE, kernel); // 闭运算：填灯条内部黑洞

    return binary;
}

// ================= 模块 2：灯条提取 =================
vector<RotatedRect> findLightBars(const Mat& binary_img) {
    vector<vector<Point>> contours;
    findContours(binary_img, contours, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE);

    vector<RotatedRect> light_bars;

    for (size_t i = 0; i < contours.size(); i++) {
        float area = contourArea(contours[i]);
        if (area < 50) continue; // 与上限筛选统一阈值

        RotatedRect rect = minAreaRect(contours[i]);

        float width = rect.size.width;
        float height = rect.size.height;
        if (width > height) swap(width, height);

        float aspect_ratio = height / width;
        float solidity = area / (width * height);

        if (aspect_ratio > 1.5f && aspect_ratio < 10.0f && solidity > 0.8f && area < 5000) {
            // 统一角度：确保 width <= height，角度归一化
            if (rect.size.width > rect.size.height) {
                rect.angle += 90.0f;
            }
            light_bars.push_back(rect);
        }
    }
    return light_bars;
}

// ================= 模块 3：装甲板配对 =================
// 改进：
//   1. 用 RotatedRect::points() 获取真实旋转角点，而非轴对齐近似
//   2. const 引用避免拷贝
//   3. 提取公共表达式减少重复计算
//   4. 按 Y 坐标排序后用滑动窗口缩小配对范围
vector<Armor> matchArmors(const vector<RotatedRect>& light_bars) {
    vector<Armor> armors;
    if (light_bars.size() < 2) return armors;

    // 按中心点 Y 坐标排序，减少无效配对
    vector<size_t> idx(light_bars.size());
    iota(idx.begin(), idx.end(), 0);
    sort(idx.begin(), idx.end(), [&](size_t a, size_t b) {
        return light_bars[a].center.y < light_bars[b].center.y;
    });

    const float kMaxHeightDiffRatio = 0.2f;
    const float kMinDistRatio = 1.5f;
    const float kMaxDistRatio = 4.0f;
    const float kMaxAngleDiff = 15.0f;

    for (size_t ii = 0; ii < light_bars.size(); ii++) {
        size_t i = idx[ii];
        const RotatedRect& bar1 = light_bars[i];

        for (size_t jj = ii + 1; jj < light_bars.size(); jj++) {
            size_t j = idx[jj];
            const RotatedRect& bar2 = light_bars[j];

            // Y 差太大，后续灯条也不可能配对（排序保证单调性）
            if (abs(bar1.center.y - bar2.center.y) > 100) break;

            // 确保 bar1 在左，bar2 在右
            const RotatedRect* left = &bar1;
            const RotatedRect* right = &bar2;
            if (bar1.center.x > bar2.center.x) swap(left, right);

            float h1 = max(left->size.width, left->size.height);
            float h2 = max(right->size.width, right->size.height);
            float max_h = max(h1, h2);

            // 1. 高度差限制
            if (abs(h1 - h2) / max_h > kMaxHeightDiffRatio) continue;

            // 2. 角度差限制
            if (abs(left->angle - right->angle) > kMaxAngleDiff) continue;

            // 3. 距离限制（提取公共表达式）
            float dist = norm(left->center - right->center);
            float ratio = dist / max_h;
            if (ratio < kMinDistRatio || ratio > kMaxDistRatio) continue;

            // 4. 配对成功，用 RotatedRect::points() 获取真实旋转角点
            Point2f ptsL[4], ptsR[4];
            left->points(ptsL);
            right->points(ptsR);

            // OpenCV points() 顺序：0=左下, 1=左上, 2=右上, 3=右下
            Armor armor;
            armor.center = (left->center + right->center) * 0.5f;
            armor.corners = { ptsL[1], ptsR[2], ptsR[3], ptsL[0] }; // 左上, 右上, 右下, 左下
            armor.bounding_box = boundingRect(armor.corners);

            armors.push_back(armor);
        }
    }
    return armors;
}

// ================= 主函数：运行与可视化 =================
int main() {
    VideoCapture cap(0);
    if (!cap.isOpened()) {
        cerr << "无法打开摄像头！" << endl;
        return -1;
    }

    cap.set(CAP_PROP_FRAME_WIDTH, 1280);
    cap.set(CAP_PROP_FRAME_HEIGHT, 720);

    // 验证实际分辨率，避免静默失败导致参数不匹配
    int actual_w = cap.get(CAP_PROP_FRAME_WIDTH);
    int actual_h = cap.get(CAP_PROP_FRAME_HEIGHT);
    cout << "实际分辨率: " << actual_w << "x" << actual_h << endl;

    Mat frame;
    while (true) {
        cap >> frame;
        if (frame.empty()) break;

        Mat binary_img = preprocess(frame, true);
        vector<RotatedRect> light_bars = findLightBars(binary_img);
        vector<Armor> armors = matchArmors(light_bars);

        // ================= 绘制结果 =================
        for (const auto& bar : light_bars) {
            Point2f pts[4];
            bar.points(pts);
            for (int k = 0; k < 4; k++) {
                line(frame, pts[k], pts[(k + 1) % 4], Scalar(0, 255, 0), 2);
            }
        }

        for (const auto& armor : armors) {
            rectangle(frame, armor.bounding_box, Scalar(255, 0, 0), 2);
            circle(frame, armor.center, 3, Scalar(0, 0, 255), -1);
            for (int k = 0; k < 4; k++) {
                line(frame, armor.corners[k], armor.corners[(k + 1) % 4], Scalar(0, 255, 255), 1);
            }
        }

        imshow("Original + Result", frame);
        // imshow("Binary (Mask)", binary_img);

        if (waitKey(1) == 27) break;
    }

    return 0;
}
