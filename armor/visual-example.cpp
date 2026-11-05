#include <opencv2/opencv.hpp>
#include <iostream>
#include <vector>
#include <cmath>

using namespace std;
using namespace cv;

// 定义装甲板的数据结构，方便后续存储和传递给 PnP
struct Armor {
    Point2f center;             // 装甲板中心点
    vector<Point2f> corners;    // 装甲板的四个角点 (左上, 右上, 右下, 左下)
    Rect bounding_box;          // 正外接矩形 (用于画框)
};

// ================= 模块 1：图像预处理 =================
Mat preprocess(const Mat& frame, bool is_red) {
    Mat channels[3];
    split(frame, channels); // 分离 BGR 通道
    
    Mat diff;
    if (is_red) {
        // 红色：R通道 (索引2) 减去 B通道 (索引0)
        // cv::subtract 会做饱和运算，相减小于0的会自动变成0，不会溢出
        subtract(channels[2], channels[0], diff); 
    } else {
        // 蓝色：B通道 (索引0) 减去 R通道 (索引2)
        subtract(channels[0], channels[2], diff);
    }

    Mat binary;
    // 二值化：大于100的变255(白)，否则变0(黑)
    threshold(diff, binary, 100, 255, THRESH_BINARY);

    // 形态学操作去噪
    Mat kernel = getStructuringElement(MORPH_RECT, Size(3, 3));
    GaussianBlur(binary, binary, Size(3, 3), 0); // 平滑边缘
    morphologyEx(binary, binary, MORPH_OPEN, kernel);  // 开运算：去小白点
    morphologyEx(binary, binary, MORPH_CLOSE, kernel); // 闭运算：填灯条内部黑洞

    return binary;
}

// ================= 模块 2：灯条提取 =================
vector<RotatedRect> findLightBars(const Mat& binary_img) {
    vector<vector<Point>> contours;
    // 寻找所有外轮廓
    findContours(binary_img, contours, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE);

    vector<RotatedRect> light_bars;

    for (size_t i = 0; i < contours.size(); i++) {
        float area = contourArea(contours[i]);
        if (area < 30) continue; // 面积太小的直接扔

        // 拟合最小外接旋转矩形
        RotatedRect rect = minAreaRect(contours[i]);
        
        // 确保 height 是最长边，width 是最短边
        float width = rect.size.width;
        float height = rect.size.height;
        if (width > height) swap(width, height); 

        float aspect_ratio = height / width; // 宽高比
        float solidity = area / (width * height); // 凸度

        // 筛选条件 (注意：这里的参数需要在你的实际画面中微调)
        // 宽高比通常在 1.5 到 10 之间，凸度 > 0.8，面积范围根据距离定
        if (aspect_ratio > 1.5 && aspect_ratio < 10.0 && solidity > 0.8 && area > 50 && area < 5000) {
            // 统一角度范围，防止角度正负导致配对失败
            if (rect.size.width > rect.size.height) {
                rect.angle += 90.0f;
            }
            light_bars.push_back(rect);
        }
    }
    return light_bars;
}

// ================= 模块 3：装甲板配对 =================
vector<Armor> matchArmors(const vector<RotatedRect>& light_bars) {
    vector<Armor> armors;

    // 两两遍历所有灯条
    for (size_t i = 0; i < light_bars.size(); i++) {
        for (size_t j = i + 1; j < light_bars.size(); j++) {
            RotatedRect bar1 = light_bars[i];
            RotatedRect bar2 = light_bars[j];

            // 确保 bar1 在左，bar2 在右 (根据中心点 X 坐标)
            if (bar1.center.x > bar2.center.x) {
                swap(bar1, bar2);
            }

            // 1. 高度差限制 (高度差不能超过最大高度的 20%)
            float h1 = max(bar1.size.width, bar1.size.height);
            float h2 = max(bar2.size.width, bar2.size.height);
            if (abs(h1 - h2) / max(h1, h2) > 0.2) continue;

            // 2. 角度差限制 (两根灯条倾斜角不能差太多，限制在 15 度内)
            if (abs(bar1.angle - bar2.angle) > 15.0) continue;

            // 3. 距离限制 (两中心点距离 / 灯条高度，通常比例在 1.5 到 4.0 之间)
            float dist = norm(bar1.center - bar2.center);
            if (dist / max(h1, h2) < 1.5 || dist / max(h1, h2) > 4.0) continue;

            // === 额外抗干扰：中间暗区检测 ===
            // 提取两根灯条中心区域，如果里面白色像素太多，说明连在一起了(反光)
            Point2f mid_center = (bar1.center + bar2.center) * 0.5f;
            // (这里省略了去 Mask 里采样的代码，有兴趣可以自己加)

            // 4. 配对成功，生成装甲板数据
            Armor armor;
            armor.center = mid_center;

            // 获取两个灯条的四个顶点
            Point2f pts1[4], pts2[4];
            bar1.points(pts1);
            bar2.points(pts2);

            // 获取真正的四个角点。因为是旋转矩形，需要自己根据 x, y 坐标去组合出 左上、右上、右下、左下
            // 简单找法：左侧灯条的左上和左下，右侧灯条的右上和右下。
            // 为了简单，直接取灯条端点组合：
            armor.corners.push_back(Point2f(bar1.center.x, bar1.center.y - h1/2)); // 假左上
            armor.corners.push_back(Point2f(bar2.center.x, bar2.center.y - h2/2)); // 假右上
            armor.corners.push_back(Point2f(bar2.center.x, bar2.center.y + h2/2)); // 假右下
            armor.corners.push_back(Point2f(bar1.center.x, bar1.center.y + h1/2)); // 假左下

            // 生成正外接矩形用于绘制
            armor.bounding_box = boundingRect(armor.corners);

            armors.push_back(armor);
        }
    }
    return armors;
}

// ================= 主函数：运行与可视化 =================
int main() {
    // 1. 打开摄像头 (如果想测试视频，把 0 改成 "你的视频路径.mp4")
    VideoCapture cap(0);
    if (!cap.isOpened()) {
        cerr << "无法打开摄像头！" << endl;
        return -1;
    }

    // 设置相机参数（不同摄像头支持度不一样，能设就设）
    cap.set(CAP_PROP_FRAME_WIDTH, 1280);
    cap.set(CAP_PROP_FRAME_HEIGHT, 720);

    Mat frame;
    while (true) {
        cap >> frame;
        if (frame.empty()) break;

        // 预处理 (这里选 true 检测红色，如果要测蓝色改成 false)
        Mat binary_img = preprocess(frame, true);

        // 提取灯条
        vector<RotatedRect> light_bars = findLightBars(binary_img);

        // 配对装甲板
        vector<Armor> armors = matchArmors(light_bars);

        // ================= 绘制结果 =================
        // 画灯条 (绿色)
        for (const auto& bar : light_bars) {
            Point2f pts[4];
            bar.points(pts);
            for (int k = 0; k < 4; k++) {
                line(frame, pts[k], pts[(k + 1) % 4], Scalar(0, 255, 0), 2);
            }
        }

        // 画装甲板 (蓝色)
        for (const auto& armor : armors) {
            // 画外接矩形
            rectangle(frame, armor.bounding_box, Scalar(255, 0, 0), 2);
            // 画中心点
            circle(frame, armor.center, 3, Scalar(0, 0, 255), -1);
            // 可以把四个角点连线看看（为 PnP 做准备）
            for (int k = 0; k < 4; k++) {
                line(frame, armor.corners[k], armor.corners[(k + 1) % 4], Scalar(0, 255, 255), 1);
            }
        }

        // 显示窗口
        imshow("Original + Result", frame);
        // imshow("Binary (Mask)", binary_img); // 调参时可以把这个注释放开，看看黑白图

        // 按 ESC 退出
        if (waitKey(1) == 27) break;
    }

    return 0;
}