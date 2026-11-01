// 把 ARTINX 视觉的装甲板检测接到 RoboMaster Aim Lab 仿真相机上。
//
// 与 visual-example-optimized.cpp 的关系：
//   preprocess / findLightBars / matchArmors 三个模块的算法一行没动，
//   只把“取图”从 VideoCapture cap(0) 换成了仿真的 /dev/shm 共享内存 IPC，
//   并额外保存 PNG 方便离线核对。
//
// 协议来源（MIT）：rm_aim_sim 的 examples/cpp/sim_client_demo.cpp。
// 启动顺序：先在另一个终端运行 rm_aim_sim 的 ./run.sh -- --remote
//
// 编译（在 ARTINX-visual-work 目录）：
//   g++ -std=c++17 -g -O2 -Wall sim-detect.cpp -o build/sim-detect \
//       $(pkg-config --cflags --libs opencv4)
//
// 用法：
//   ./build/sim-detect --team=blue --dump=/tmp/artinx --frames=60
//   ./build/sim-detect --strict        # 用原始阈值复现“近距离整定”的效果
//   ./build/sim-detect --scale=3       # 不调阈值，改成先放大 3 倍再走原算法

#include <nlohmann/json.hpp>
#include <opencv2/opencv.hpp>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <csignal>
#include <cstdint>
#include <cstdlib>
#include <cerrno>
#include <fcntl.h>
#include <iostream>
#include <numeric>
#include <regex>
#include <stdexcept>
#include <string>
#include <sys/file.h>
#include <sys/stat.h>
#include <thread>
#include <unistd.h>
#include <vector>

using namespace std;
using namespace cv;
using Json = nlohmann::json;
using Clock = std::chrono::steady_clock;
using namespace std::chrono_literals;

volatile std::sig_atomic_t exiting = 0;
void on_signal(int) { exiting = 1; }

// ================= 下面三个模块：算法结构未改动 =================
// 只把三处“按近距离大目标整定”的常量提成参数（默认值仍是原值），
// 以便在小目标场景下调参；--strict 就是原始行为。
struct LightBarTune {
    float area_min = 50.0f;
    float area_max = 5000.0f;
    float solidity_min = 0.8f;
    float aspect_min = 1.5f;
    float aspect_max = 10.0f;
    bool morphology = true;   // 原代码的 3x3 开+闭运算
};

struct Armor {
    Point2f center;
    vector<Point2f> corners;
    Rect bounding_box;
};

Mat preprocess(const Mat& frame, bool is_red, bool morphology = true) {
    Mat channels[3];
    split(frame, channels);

    Mat diff;
    if (is_red) {
        subtract(channels[2], channels[0], diff);
    } else {
        subtract(channels[0], channels[2], diff);
    }

    Mat blurred;
    GaussianBlur(diff, blurred, Size(3, 3), 0);

    Mat binary;
    threshold(blurred, binary, 100, 255, THRESH_BINARY);

    if (morphology) {
        Mat kernel = getStructuringElement(MORPH_RECT, Size(3, 3));
        morphologyEx(binary, binary, MORPH_OPEN, kernel);
        morphologyEx(binary, binary, MORPH_CLOSE, kernel);
    }

    return binary;
}

vector<RotatedRect> findLightBars(const Mat& binary_img, const LightBarTune& tune = {}) {
    vector<vector<Point>> contours;
    findContours(binary_img, contours, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE);

    vector<RotatedRect> light_bars;

    for (size_t i = 0; i < contours.size(); i++) {
        float area = contourArea(contours[i]);
        if (area < tune.area_min) continue;

        RotatedRect rect = minAreaRect(contours[i]);

        float width = rect.size.width;
        float height = rect.size.height;
        if (width > height) swap(width, height);

        float aspect_ratio = height / width;
        float solidity = area / (width * height);

        if (aspect_ratio > tune.aspect_min && aspect_ratio < tune.aspect_max &&
            solidity > tune.solidity_min && area < tune.area_max) {
            if (rect.size.width > rect.size.height) {
                rect.angle += 90.0f;
            }
            light_bars.push_back(rect);
        }
    }
    return light_bars;
}

vector<Armor> matchArmors(const vector<RotatedRect>& light_bars) {
    vector<Armor> armors;
    if (light_bars.size() < 2) return armors;

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

            if (abs(bar1.center.y - bar2.center.y) > 100) break;

            const RotatedRect* left = &bar1;
            const RotatedRect* right = &bar2;
            if (bar1.center.x > bar2.center.x) swap(left, right);

            float h1 = max(left->size.width, left->size.height);
            float h2 = max(right->size.width, right->size.height);
            float max_h = max(h1, h2);

            if (abs(h1 - h2) / max_h > kMaxHeightDiffRatio) continue;
            if (abs(left->angle - right->angle) > kMaxAngleDiff) continue;

            float dist = norm(left->center - right->center);
            float ratio = dist / max_h;
            if (ratio < kMinDistRatio || ratio > kMaxDistRatio) continue;

            Point2f ptsL[4], ptsR[4];
            left->points(ptsL);
            right->points(ptsR);

            Armor armor;
            armor.center = (left->center + right->center) * 0.5f;
            armor.corners = { ptsL[1], ptsR[2], ptsR[3], ptsL[0] };
            armor.bounding_box = boundingRect(armor.corners);

            armors.push_back(armor);
        }
    }
    return armors;
}

// ================= 取图适配层：仿真共享内存 -> cv::Mat BGR =================
struct SimImage {
    Mat bgr;
    Json metadata;
    Clock::time_point received;
};

class SimFrameClient {
public:
    explicit SimFrameClient(const string& channel) {
        if (!regex_match(channel, regex("[A-Za-z0-9_-]{1,80}")))
            throw runtime_error("通道名非法");
        const string path = "/dev/shm/rm_aim_sim_" + channel;
        directory_ = ::open(path.c_str(), O_RDONLY | O_DIRECTORY | O_CLOEXEC | O_NOFOLLOW);
        if (directory_ < 0)
            throw runtime_error("打不开 " + path + "，请先启动仿真：./run.sh -- --remote");
        struct stat info{};
        if (::fstat(directory_, &info) != 0 || info.st_uid != ::getuid() ||
            (info.st_mode & 0777) != 0700) {
            ::close(directory_);
            throw runtime_error("IPC 目录必须是当前用户且权限 0700");
        }
        lock_ = ::openat(directory_, "client.lock", O_CREAT | O_RDWR | O_CLOEXEC | O_NOFOLLOW, 0600);
        if (lock_ < 0 || ::flock(lock_, LOCK_EX | LOCK_NB) != 0) {
            if (lock_ >= 0) ::close(lock_);
            ::close(directory_);
            throw runtime_error("client.lock 已被占用：同一通道只允许一个客户端");
        }
        token_ = to_string(::getpid()) + "-" +
                 to_string(Clock::now().time_since_epoch().count());
    }

    ~SimFrameClient() {
        ::close(lock_);
        ::close(directory_);
    }

    SimFrameClient(const SimFrameClient&) = delete;
    SimFrameClient& operator=(const SimFrameClient&) = delete;

    bool poll_state() {
        Json value = read_json("state.json");
        if (value.value("protocol", "") != "rm_aim_sim/2" || value.value("type", "") != "state")
            return false;
        const double wall = std::chrono::duration<double>(
            std::chrono::system_clock::now().time_since_epoch()).count();
        const double age = wall - value.at("wall_unix_sec").get<double>();
        if (!std::isfinite(age) || age < -0.1 || age > 0.25) return false;
        if (!state_.empty() && value.at("publisher_id") == state_.at("publisher_id") &&
            value.at("publication") == state_.at("publication"))
            return false;
        if (state_.empty() || context(value) != context(state_)) request_.clear();
        state_ = std::move(value);
        last_state_ = Clock::now();
        return true;
    }

    bool fresh() const { return !state_.empty() && Clock::now() - last_state_ <= 200ms; }
    const Json& state() const { return state_; }

    void request_frame() {
        if (!request_.empty() || !fresh()) return;
        request_ = token_ + "-" + to_string(request_count_++);
        requested_ = Clock::now();
        write_json("request.json", {{"publisher_id", state_.at("publisher_id")},
                                    {"request_id", request_}});
    }

    bool try_frame(SimImage& result) {
        if (request_.empty()) return false;
        if (Clock::now() - requested_ > 5s)
            throw runtime_error("取图超时：仿真必须用图形模式（不能用 --headless）");
        Json frame = read_json("frame.json");
        if (frame.value("request_id", "") != request_) return false;
        if (!fresh() || frame.value("protocol", "") != "rm_aim_sim/2" ||
            frame.value("type", "") != "frame" || context(frame) != context(state_)) {
            request_.clear();
            return false;
        }
        const double age = state_.at("sim_time").get<double>() - frame.at("sim_time").get<double>();
        if (!std::isfinite(age) || age > 0.2 || age < -0.1) {
            request_.clear();
            return false;
        }
        const int width = frame.at("camera").at("width").get<int>();
        const int height = frame.at("camera").at("height").get<int>();
        const string encoding = frame.at("encoding").get<string>();
        const int channels = encoding == "rgb8" ? 3 : encoding == "rgba8" ? 4 : 0;
        const string filename = "frame-" + request_ + ".raw";
        if (!channels || width < 160 || width > 3840 || height < 120 || height > 2160 ||
            frame.at("frame_file") != filename)
            throw runtime_error("帧布局非法");
        const size_t bytes = size_t(width) * height * channels;
        if (frame.at("image_bytes").get<size_t>() != bytes)
            throw runtime_error("image_bytes 不一致");
        int fd = ::openat(directory_, filename.c_str(), O_RDONLY | O_CLOEXEC | O_NOFOLLOW);
        if (fd < 0) throw runtime_error("帧文件消失：仿真可能重启，请重建客户端");
        struct stat info{};
        Mat rgb;
        try {
            if (::fstat(fd, &info) != 0 || !S_ISREG(info.st_mode) ||
                info.st_size != static_cast<off_t>(bytes))
                throw runtime_error("帧文件大小/类型不符");
            rgb.create(height, width, CV_MAKETYPE(CV_8U, channels));
            read_all(fd, rgb.data, bytes);
        } catch (...) {
            ::close(fd);
            throw;
        }
        ::close(fd);
        // 仿真给的是 RGB(A)，OpenCV 用 BGR，必须显式转换，否则红蓝颠倒。
        cvtColor(rgb, result.bgr, channels == 3 ? COLOR_RGB2BGR : COLOR_RGBA2BGR);
        result.metadata = std::move(frame);
        result.received = Clock::now();
        request_.clear();
        return true;
    }

private:
    int directory_ = -1, lock_ = -1;
    Json state_ = Json::object();
    string request_, token_;
    uint64_t request_count_ = 0;
    Clock::time_point last_state_, requested_;

    static Json context(const Json& packet) {
        const Json& camera = packet.at("camera");
        return Json::array({packet.at("publisher_id"), packet.at("session_id"),
                            packet.at("appearance"), packet.at("paused"),
                            packet.at("ipc").at("enabled"), camera.at("width"),
                            camera.at("height"), camera.at("K"), camera.at("distortion"),
                            camera.at("R_camera2gimbal"), camera.at("t_camera2gimbal")});
    }

    static void read_all(int fd, void* output, size_t bytes) {
        auto data = static_cast<char*>(output);
        while (bytes) {
            const auto count = ::read(fd, data, bytes);
            if (count < 0 && errno == EINTR) continue;
            if (count <= 0) throw runtime_error("文件读入不完整");
            data += count;
            bytes -= count;
        }
    }

    Json read_json(const char* filename) const {
        int fd = ::openat(directory_, filename, O_RDONLY | O_CLOEXEC | O_NOFOLLOW);
        if (fd < 0) return Json::object();
        struct stat info{};
        string data;
        try {
            if (::fstat(fd, &info) != 0 || !S_ISREG(info.st_mode) || info.st_size <= 0 ||
                info.st_size > 65536)
                throw runtime_error("JSON 文件大小/类型非法");
            data.resize(info.st_size);
            read_all(fd, data.data(), data.size());
        } catch (...) {
            ::close(fd);
            throw;
        }
        ::close(fd);
        Json value = Json::parse(data, nullptr, false);
        return value.is_object() ? value : Json::object();
    }

    void write_json(const char* filename, const Json& value) const {
        const string temporary = string(filename) + ".client.tmp";
        int fd = ::openat(directory_, temporary.c_str(),
                          O_CREAT | O_TRUNC | O_WRONLY | O_CLOEXEC | O_NOFOLLOW, 0600);
        if (fd < 0) throw runtime_error("无法创建请求临时文件");
        try {
            const string data = value.dump();
            size_t written = 0;
            while (written < data.size()) {
                const auto count = ::write(fd, data.data() + written, data.size() - written);
                if (count < 0 && errno == EINTR) continue;
                if (count <= 0) throw runtime_error("文件写入不完整");
                written += count;
            }
        } catch (...) {
            ::close(fd);
            ::unlinkat(directory_, temporary.c_str(), 0);
            throw;
        }
        ::close(fd);
        if (::renameat(directory_, temporary.c_str(), directory_, filename) != 0)
            throw runtime_error("原子发布请求失败");
    }
};

// ================= 可视化与主循环 =================
static void draw_result(Mat& canvas, const vector<RotatedRect>& light_bars,
                        const vector<Armor>& armors) {
    for (const auto& bar : light_bars) {
        Point2f pts[4];
        bar.points(pts);
        for (int k = 0; k < 4; k++)
            line(canvas, pts[k], pts[(k + 1) % 4], Scalar(0, 255, 0), 2);
    }
    for (const auto& armor : armors) {
        rectangle(canvas, armor.bounding_box, Scalar(255, 0, 0), 2);
        circle(canvas, armor.center, 3, Scalar(0, 0, 255), -1);
        for (int k = 0; k < 4; k++)
            line(canvas, armor.corners[k], armor.corners[(k + 1) % 4], Scalar(0, 255, 255), 1);
    }
}

int main(int argc, char** argv) {
    try {
        string channel = "default", dump_dir;
        string team = "blue";
        bool display = true, strict = false;
        int frames = 0, scale = 1;
        double duration = 0;

        for (int i = 1; i < argc; ++i) {
            const string arg = argv[i];
            if (arg == "--help") {
                cout << "sim-detect [--channel=default] [--team=blue|red] [--no-display]\n"
                        "          [--frames=N] [--duration=SEC] [--dump=DIR]\n"
                        "          [--strict] [--scale=N]\n"
                        "先启动仿真：cd ~/rm_aim_sim && ./run.sh -- --remote\n"
                        "--strict 用 original 阈值（近距离整定，4 m 场景会有 0 命中）\n"
                        "--scale=N 先把图放大 N 倍再走原算法，N=1 默认不缩\n";
                return 0;
            }
            if (arg == "--no-display") { display = false; continue; }
            if (arg == "--strict") { strict = true; continue; }
            const auto equals = arg.find('=');
            if (equals == string::npos) throw runtime_error("参数需要 = 值: " + arg);
            const string key = arg.substr(0, equals), value = arg.substr(equals + 1);
            if (key == "--channel") channel = value;
            else if (key == "--team") team = value;
            else if (key == "--dump") dump_dir = value;
            else {
                size_t used = 0;
                const double number = stod(value, &used);
                if (used != value.size() || !isfinite(number))
                    throw runtime_error("数值参数非法: " + arg);
                if (key == "--frames" && number >= 0 && number == floor(number))
                    frames = static_cast<int>(number);
                else if (key == "--duration" && number >= 0) duration = number;
                else if (key == "--scale" && number >= 1 && number <= 4 && number == floor(number))
                    scale = static_cast<int>(number);
                else throw runtime_error("未知参数: " + arg);
            }
        }
        if (team != "blue" && team != "red") throw runtime_error("--team 只能是 blue 或 red");
        const bool is_red = team == "red";

        // 默认一套小目标参数：4 m / 1440x1080 下灯条只有 2~4 px 宽，
        // 实测原参数（area>=50 + 3x3 开运算 + solidity>0.8）会全部滤除。
        LightBarTune tune;
        if (!strict) {
            tune.area_min = 10.0f;
            tune.solidity_min = 0.55f;
            tune.morphology = false;
        }

        signal(SIGINT, on_signal);
        signal(SIGTERM, on_signal);

        SimFrameClient client(channel);
        const auto begin = Clock::now();
        auto next_report = begin;
        int count = 0, saved = 0;
        size_t last_bars = 0, last_armors = 0;

        while (!exiting && (duration == 0 ||
               std::chrono::duration<double>(Clock::now() - begin).count() < duration)) {
            client.poll_state();
            client.request_frame();
            SimImage sample;
            if (client.try_frame(sample)) {
                ++count;
                // 真实流水线在这里交出 sample.bgr + sample.metadata（同帧 sim_time/内参）
                Mat input = sample.bgr;
                if (scale > 1)
                    resize(sample.bgr, input, Size(), scale, scale, INTER_LINEAR);
                Mat binary = preprocess(input, is_red, tune.morphology);
                vector<RotatedRect> light_bars = findLightBars(binary, tune);
                vector<Armor> armors = matchArmors(light_bars);
                // 放大过的结果换算回原始像素坐标再画到原图上
                if (scale > 1) {
                    for (auto& bar : light_bars) {
                        bar.center /= static_cast<float>(scale);
                        bar.size = Size2f(bar.size.width / scale, bar.size.height / scale);
                    }
                    for (auto& armor : armors) {
                        armor.center /= static_cast<float>(scale);
                        for (auto& p : armor.corners) p /= static_cast<float>(scale);
                        armor.bounding_box = boundingRect(armor.corners);
                    }
                }
                last_bars = light_bars.size();
                last_armors = armors.size();

                Mat canvas = sample.bgr.clone();
                draw_result(canvas, light_bars, armors);

                if (!dump_dir.empty() && saved < 20) {
                    imwrite(dump_dir + "/mask_" + to_string(saved) + ".png", binary);
                    imwrite(dump_dir + "/raw_" + to_string(saved) + ".png", sample.bgr);
                    imwrite(dump_dir + "/draw_" + to_string(saved) + ".png", canvas);
                    ++saved;
                }

                if (display) {
                    imshow("sim-detect (ARTINX armor detector)", canvas);
                    const int key = waitKey(1);
                    if (key == 'q' || key == 27) break;
                }

                if (Clock::now() >= next_report) {
                    cout << Json{{"type", "artinx_detect"},
                                 {"frame", count},
                                 {"sim_time", sample.metadata.at("sim_time")},
                                 {"size", {sample.bgr.cols, sample.bgr.rows}},
                                 {"light_bars", last_bars},
                                 {"armors", last_armors},
                                 {"gimbal", client.state().at("gimbal")}}.dump()
                         << endl;
                    next_report = Clock::now() + 1s;
                }
            }
            if (frames && count >= frames) break;
            std::this_thread::sleep_for(2ms);
        }
        if (display) destroyAllWindows();
        if (!count)
            throw runtime_error("一帧都没收到：确认仿真是图形模式、通道一致、没有其它客户端占锁");
        cout << Json{{"type", "artinx_detect_result"}, {"frames", count},
                     {"light_bars", last_bars}, {"armors", last_armors}}.dump()
             << endl;
        return 0;
    } catch (const std::exception& error) {
        cerr << "sim-detect 错误: " << error.what() << endl;
        return 1;
    }
}
