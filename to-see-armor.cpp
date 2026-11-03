#include <algorithm>
#include <opencv2/opencv.hpp>//提供opencv的图像处理，矩阵运算，特征点提取
#include <iostream>
#include <vector>//引入动态数组，可以动态增删元素
#include <cmath>//引用常见的数学运算
#include <opencv2/videoio.hpp>//提供视频输入输出的功能
#include"opencv2/core.hpp"//提供opencv的核心功能，包括矩阵运算，图像处理，特征点提取等
//----------下面这些头文件是给“仿真共享内存取图”这一层用的，算法部分不需要它们----------
#include <chrono>//计时：判断仿真状态是否新鲜(200ms)、取图是否超时(5s)
#include <string>//通道名、文件名
#include <memory>//unique_ptr 管理仿真客户端，函数中途抛异常也能自动关文件
#include <regex>//校验通道名只允许字母数字下划线短横线
#include <thread>//仿真没给新帧时睡2ms，不要空转把CPU吃满
#include <cstdint>//uint64_t，请求计数器
#include <cstdio>//renameat：请求文件的原子发布
#include <csignal>//Ctrl+C 优雅退出
#include <stdexcept>//runtime_error：出错时带中文原因直接抛出
#include <cerrno>//errno：区分“目录已存在”
#include <nlohmann/json.hpp>//解析仿真发布的 state.json / frame.json
#include <fcntl.h>//open/openat/O_* 标志
#include <sys/stat.h>//fstat/mkdir：检查文件类型、大小、权限
#include <sys/file.h>//flock：一个通道只允许一个客户端
#include <unistd.h>//read/close/getuid/getpid

using namespace std ;
using namespace cv ;
using Json = nlohmann::json ;//仿真进程之间交换的报文都是JSON
using Clock = std::chrono::steady_clock ;//单调时钟，不受系统时间调整影响

struct Armor{
    Point2f center ;//point2f是opencv中的二维坐标点类型，包含x和y两个浮点数成员，用于描述装甲板的中心点坐标
    vector<Point2f> corners ;//是中心点的角点集合。表示一个存储类型为point2f的动态数组，存储这四个角点的目的是用来计算装甲板的倾斜角度，进行透视变换求出机器人的3D姿态
    Rect bounding_box  ;//装甲板的边界矩形框。rect是opencv中表示矩形的类
};

//preprocess of picture
Mat preprocess(const Mat& frame,bool is_red )//Mat 返回类型代表了一个矩阵，说明这个函数处理完图像后会返回一个矩阵。const Mat& frame是传入的原始图像，叫一帧frame,const表示不会修改原图。引用&避免了复制一整张巨大的图片提高效率
{Mat channels[3] ;
    split(frame ,channels);//用于分离bgr通道,channal[0]是蓝色，1是绿色，2是红色。

    Mat diff;
    if(is_red){
        subtract(channels[2] ,channels[0],diff);//对手是红方，红色通道减蓝色通道，红色值要比蓝色值大得多，所以diff在二值化时红色灯条会变白，其余部分就变黑。
    }
    else{
        subtract(channels[0],channels[2],diff);//对手是蓝方，蓝色通道减去红色通道，蓝色值要比红色值大得多，所以在二值化之后蓝色灯条会变成白色，其余部分变成黑色
    }

    GaussianBlur(diff, diff, Size(3,3),0);//先进行高斯模糊再进行二值化，避免二值化之后出现灰度过渡区域
    Mat binary;//二值化，大于100的变255否则变0,此处声明了一个名为binary的矩阵，用来存放二值化处理后的黑白图像
    threshold(diff,binary,100,255,THRESH_BINARY);//输入图像，输出图像，阈值，超过阈值时变成什么值，阈值类型。阈值类型可以理解为“你要用哪一种比较规则把图像分成目标和背景”
    //形态学操作去噪点
    Mat kernel = getStructuringElement(MORPH_RECT ,Size(3,3)) ;//getStructuringElement是一个opencv提供的用来生成一个特定形状刷子的函数，MORPH_RECT表示刷子的形状是矩形，尺寸为三像素×三像素
    //mediaBlur(Binary,Binary,3);中值滤波，不会产生灰色过渡带使得轮廓更加清晰，当你没有用高斯模糊时的补救，用在二值化之后。
    morphologyEx(binary , binary, MORPH_OPEN,kernel);//开运算，先腐蚀后膨胀，腐蚀的目的是把白色的区域变瘦，实则消除了噪点，膨胀会补救回来所以不急
    morphologyEx(binary ,binary,MORPH_CLOSE,kernel);//闭运算，先膨胀后腐蚀，膨胀是把白色区域变胖，实则可以把断开的两个白色区域连起来，补充出来完整的灯条
    //注意是先开后闭
    return binary;
}

    //灯条提取
    vector<RotatedRect> findLightBars(const Mat& binary_img)//vector<RotatedRect>是这个函数的返回值，，即这个函数结束后会返回一堆旋转矩形。
    {
        vector<vector<Point>> contours;//寻找外轮廓.vector是动态数组，point是数据类型表示存的是一个点，vector<point>表示存一系列点，就是一个轮廓。vector<vector<point>>就是一堆轮廓，可以共同构成装甲板
        findContours(binary_img,contours,RETR_EXTERNAL,CHAIN_APPROX_SIMPLE);//RETR_EXTERNAL表示只提取最外层（极端外部）的轮廓，它会把所有轮廓的层级关系（hierarchy）中的【2】
//和【3】（代表父轮廓和子轮廓索引）都设置为-1.也就是说RETR_EXTERNAL是只关注最外层的轮廓，内部的轮廓都会被忽略，好处是可以避免把噪点看成是一个单独的物体。
        vector<RotatedRect> light_bars;
        for (size_t i = 0 ;i < contours.size();i++) 
        {
        double area = contourArea(contours[i]);//contourArea的返回类型是double？
        if(area < 30)continue;
        RotatedRect rect = minAreaRect(contours[i]);//minAreaRect就是想象成一根皮筋去绑几个点，绑出来的图形面积就是这个函数的返回值。所以要小心的就是当点集很接近Mat的边缘的时候，可能会溢出即其顶点坐标可能出现负数。
        float width = rect.size.width;
        float height = rect.size.height;
        if (width > height) swap(width,height);//疑似和82，83行的作用一样，确保width是小边，height是大边，看作是双保险吧

        float aspect_ratio = height /width ;//高宽比
        float solidity = area/(height*width) ;//填充率。一个完美的矩形，当你用橡皮筋去包它的时候会完全贴合；但是如果它表面坑坑洼洼，或者有中空，area(实际面积）就会小于橡皮筋的底乘高
        
        if(aspect_ratio > 1.5 && aspect_ratio <10.0 && solidity >0.8 && area > 50 && area <5000){
        
            //OpenCV 4.5+ 的 minAreaRect 返回的 angle 落在 (0,90]，且宽高谁大谁小不稳定；
            //这里把角度归一到 (-45,45]，相当于“灯条相对竖直方向的倾斜”，两根平行灯条才会得到同一个值
            //注意：减 90° 时必须同时交换宽高，否则矩形会从“竖”变成“横”，
            //      barEnds 求出的上下端点就跑到左右两端，四角会连成自交的蝴蝶结。
            float a = rect.angle;
            if(a > 45.0f){
                a -= 90.0f;
                swap(rect.size.width, rect.size.height);
            }
            rect.angle = a;
            light_bars.push_back(rect);
        }
        }
        return light_bars;
    }

    //用灯条的真实角点求它的上下两端：四条边的中点里距离最远的那一对，就是灯条长边方向上的两个端点
    void barEnds(const Point2f corners[4], Point2f& top, Point2f& bottom){
        Point2f mids[4];//四条边的中点，顺序与 corners 一致：0-1、1-2、2-3、3-0
        for(int k = 0;k < 4;k++) mids[k] = (corners[k] + corners[(k+1)%4]) * 0.5f;

        int bi = 0 , bj = 1;//记下距离最远的那一对下标的编号
        float best = -1.0f;
        for(int m = 0;m < 4;m++){
            for(int n = m+1;n < 4;n++){
                float d = norm(mids[m] - mids[n]);
                if(d > best){ best = d ; bi = m ; bj = n ; }
            }
        }
        if(mids[bi].y <= mids[bj].y){ top = mids[bi] ; bottom = mids[bj] ; }//y小的在上面
        else{ top = mids[bj] ; bottom = mids[bi] ; }
    }

    //=========模块三：装甲板配对============
    vector<Armor> matchArmors(const vector<RotatedRect>& light_bars){
        vector<Armor> armors;
        //两两遍历所有灯条
        for(size_t i = 0;i < light_bars.size();i++){
            for(size_t j = i+1;j < light_bars.size();j++){
            RotatedRect bar1 = light_bars[i] ;
            RotatedRect bar2 = light_bars[j] ;
            //握手算法，将所有的灯条候选人两两配对，等待二次选择，因为你传进来的所谓灯条可能是天上的大灯，场地的反光，等等。所以这一步先到先配对，在慢慢去筛选。
            if(bar1.center.x > bar2.center.x)swap(bar1,bar2);//确保bar1在左边，bar2在右边。目的是在pnp解算时，需要严格顺序要求的左上右上左下右下，只有先在这里分清左右后面才能不烧心

            float h1 =max(bar1.size.width, bar1.size.height);
            float h2 =max(bar2.size.width , bar2.size.height);//为了找到这个灯条的实际高度
            if(abs(h1-h2)/max(h1,h2) > 0.2)continue;//abs的意思是绝对值，意思是如果两个灯条的高度差距超过20%，就不配对了，因为装甲板的两边灯条高度应该差不多，continue的意思是跳过本次循环，继续下一次循环。也就是如果高度差距超过20%，就不配对了，直接跳过本次循环，继续下一次循环。
            if(abs(bar1.angle - bar2.angle) > 15.0)continue;//同样的目的，同一块装甲板上的两根灯条一定是平行的，所以角度差距不能超过15度.但是你知道的阈值这东西说不准。。。

            float dist = norm(bar1.center - bar2.center);//norm是opencv提供的计算两点之间距离的函数，dist是两个灯条中心点之间的距离
            if(dist / max(h1,h2)<1.5||dist /max(h1,h2) > 4.0)continue;//同样是在筛选
            Point2f mid_center = (bar1.center + bar2.center) * 0.5f;//0.5f的意思是把0.5当作单精度浮点数来处理，只占用四个字节的内存，如果写成0.5，那么编译器会默认0.5是双精度浮点数，会占用八个字节的内存
//而且你的bar1.center和bar2.center都是Point2f类型的，里面的x和y都是float类型的，所以如果你写成0.5，那么编译器会把0.5当作double类型来处理，然后再把double类型转换成float类型，这样就会浪费内存空间，而且还会增加计算时间.
            Armor armor ;//这是一个结构体变量，里面有三个成员变量，分别是中心点，角点集合，边界矩形框
            armor.center = mid_center ;
            
            Point2f pts1[4] , pts2[4];//这一步是为了把旋转矩形的四个角点提取出来，存放在一个数组里，方便后面计算装甲板的四个角点
            bar1.points(pts1);//这种.points()之类的被称为类的成员。.读作“的”，后面的成员函数可以看作是前面的类的一个性质，属性。（）是动作，带上括号才会执行这个动作。注意不要搞混.size()和.size，前者是函数，后者是属性。.size()是动作，每次调用都要现算；而.size是存着一个值，直接使用
            bar2.points(pts2);
            
            //用灯条真实角点求出的上下端点来搭装甲板四角，不再是“中心 ± 高/2”的近似，灯条倾斜时角点才准
            Point2f top1 , bottom1 , top2 , bottom2;//左灯条的上/下端点，右灯条的上/下端点
            barEnds(pts1, top1, bottom1);
            barEnds(pts2, top2, bottom2);

            armor.corners.push_back(top1);//左上
            armor.corners.push_back(top2);//右上
            armor.corners.push_back(bottom2);//右下
            armor.corners.push_back(bottom1);//左下

            armor.bounding_box = boundingRect(armor.corners);//boundingRect是opencv提供的一个函数，用来计算一个点集的最小外接矩形。

            armors.push_back(armor);//push_back的本质是复制粘贴，它把armor这个结构体变量的值复制一份，然后粘贴到armors这个动态数组的末尾。注意这里是复制粘贴，而不是引用传递，所以即使armor这个变量在后面被修改了，armors里面的值也不会受到影响。
            //所以这里的armor是一个临时小工，一套四个点一套四个点地干活，干完一次就被清除记忆，每一次boundingrect来读取数据时都只会读到本次循环里的记忆。
            //那一堆armor都被放在了动态数组armors之中，编号和推进浑然一体。
            }
        }
        return armors ;//最终返回配对好的装甲板
    }
//回望你做的这些事情，总结一下，就是先传入图像和是红方还是蓝方，进行红蓝相减，高斯模糊，二值化图像，对其中可能存在噪点和灯条的黑白图像进行有固定阈值的筛选。最后筛出灯条轮廓。然后通过灯条轮廓反找装甲板，通过进行两个灯条的匹配性筛查，找到真正是一对的灯条，把他们匹配起来，然后利用灯条的成员函数下的中心点数据得到装甲板的四个角点，再利用角点包出一个矩形，至此，装甲板已成！

//=================== 取图适配层：仿真共享内存 -> cv::Mat BGR ===================
//说明：下面这一段只负责“把图取进来”，不参与任何装甲板识别逻辑。
//协议来源：rm_aim_sim 的 examples/cpp/sim_client_demo.cpp（MIT）
//启动顺序：先在另一个终端运行 rm_aim_sim 的 ./run.sh -- --remote，再运行本程序 --sim
struct SimImage{
    Mat bgr ;//已经转成BGR的一帧，可以直接喂给preprocess
    Json metadata ;//同帧元数据（sim_time、相机内参、分辨率），本次只用于排查问题
    Clock::time_point received ;//收到这一帧的时刻
};

class SimFrameClient{
public:
    explicit SimFrameClient(const string& channel){
        if(!regex_match(channel, regex("[A-Za-z0-9_-]{1,80}")))
            throw runtime_error("通道名非法，只允许字母、数字、下划线、短横线");
        const string path = "/dev/shm/rm_aim_sim_" + channel;
        directory_ = ::open(path.c_str(), O_RDONLY | O_DIRECTORY | O_CLOEXEC | O_NOFOLLOW);
        if(directory_ < 0)
            throw runtime_error("打不开 " + path + "，请先启动仿真：cd ~/rm_aim_sim && ./run.sh -- --remote");
        struct stat info{};
        if(::fstat(directory_, &info) != 0 || info.st_uid != ::getuid() ||
           (info.st_mode & 0777) != 0700){
            ::close(directory_);
            directory_ = -1;
            throw runtime_error("IPC 目录必须是当前用户所有且权限为 0700");
        }
        lock_ = ::openat(directory_, "client.lock", O_CREAT | O_RDWR | O_CLOEXEC | O_NOFOLLOW, 0600);
        if(lock_ < 0 || ::flock(lock_, LOCK_EX | LOCK_NB) != 0){
            if(lock_ >= 0) ::close(lock_);
            ::close(directory_);
            lock_ = -1;
            directory_ = -1;
            throw runtime_error("client.lock 已被占用：同一通道只允许一个客户端");
        }
        token_ = to_string(::getpid()) + "-" + to_string(Clock::now().time_since_epoch().count());
    }

    ~SimFrameClient(){
        if(lock_ >= 0) ::close(lock_);
        if(directory_ >= 0) ::close(directory_);
    }

    SimFrameClient(const SimFrameClient&) = delete ;
    SimFrameClient& operator=(const SimFrameClient&) = delete ;

    //读状态：确认仿真是活的、协议版本对、不是过期数据
    bool poll_state(){
        Json value = read_json("state.json");
        if(value.value("protocol", "") != "rm_aim_sim/2" || value.value("type", "") != "state")
            return false;
        const double wall = std::chrono::duration<double>(
            std::chrono::system_clock::now().time_since_epoch()).count();
        const double age = wall - value.at("wall_unix_sec").get<double>();
        if(!std::isfinite(age) || age < -0.1 || age > 0.25) return false;
        if(!state_.empty() && value.at("publisher_id") == state_.at("publisher_id") &&
           value.at("publication") == state_.at("publication"))
            return false;
        if(state_.empty() || context(value) != context(state_)) request_.clear();
        state_ = std::move(value);
        last_state_ = Clock::now();
        return true;
    }

    bool fresh() const { return !state_.empty() && Clock::now() - last_state_ <= std::chrono::milliseconds(200); }
    const Json& state() const { return state_ ; }

    //投递取图请求（request.json 原子发布）
    void request_frame(){
        if(!request_.empty() || !fresh()) return;
        request_ = token_ + "-" + to_string(request_count_++);
        requested_ = Clock::now();
        write_json("request.json", {{"publisher_id", state_.at("publisher_id")},
                                    {"request_id", request_}});
    }

    //取一帧：成功时把RGB(A)转成BGR放进result.bgr
    bool try_frame(SimImage& result){
        if(request_.empty()) return false;
        if(Clock::now() - requested_ > std::chrono::seconds(5))
            throw runtime_error("取图超时：仿真必须以图形模式运行（不能加 --headless）");
        Json frame = read_json("frame.json");
        if(frame.value("request_id", "") != request_) return false;
        if(!fresh() || frame.value("protocol", "") != "rm_aim_sim/2" ||
           frame.value("type", "") != "frame" || context(frame) != context(state_)){
            request_.clear();
            return false;
        }
        const double age = state_.at("sim_time").get<double>() - frame.at("sim_time").get<double>();
        if(!std::isfinite(age) || age > 0.2 || age < -0.1){
            request_.clear();
            return false;
        }
        const int width = frame.at("camera").at("width").get<int>();
        const int height = frame.at("camera").at("height").get<int>();
        const string encoding = frame.at("encoding").get<string>();
        const int channels = encoding == "rgb8" ? 3 : encoding == "rgba8" ? 4 : 0;
        const string filename = "frame-" + request_ + ".raw";
        if(!channels || width < 160 || width > 3840 || height < 120 || height > 2160 ||
           frame.at("frame_file") != filename)
            throw runtime_error("帧布局非法");
        const size_t bytes = size_t(width) * size_t(height) * size_t(channels);
        if(frame.at("image_bytes").get<size_t>() != bytes)
            throw runtime_error("image_bytes 与实际分辨率不一致");
        int fd = ::openat(directory_, filename.c_str(), O_RDONLY | O_CLOEXEC | O_NOFOLLOW);
        if(fd < 0) throw runtime_error("帧文件消失：仿真可能重启过，请重新运行本程序");
        struct stat info{};
        Mat rgb;
        try{
            if(::fstat(fd, &info) != 0 || !S_ISREG(info.st_mode) ||
               info.st_size != static_cast<off_t>(bytes))
                throw runtime_error("帧文件大小/类型不符");
            rgb.create(height, width, CV_MAKETYPE(CV_8U, channels));
            read_all(fd, rgb.data, bytes);
        }catch(...){
            ::close(fd);
            throw;
        }
        ::close(fd);
        //仿真发布的是RGB(A)，OpenCV内部用BGR，必须显式转换，否则红蓝颠倒
        cvtColor(rgb, result.bgr, channels == 3 ? COLOR_RGB2BGR : COLOR_RGBA2BGR);
        result.metadata = std::move(frame);
        result.received = Clock::now();
        request_.clear();
        return true;
    }

private:
    int directory_ = -1 , lock_ = -1 ;
    Json state_ = Json::object() ;
    string request_ , token_ ;
    uint64_t request_count_ = 0 ;
    Clock::time_point last_state_ , requested_ ;

    //上下文指纹：仿真重启/改分辨率/改内参/暂停时，旧请求必须作废
    static Json context(const Json& packet){
        const Json& camera = packet.at("camera");
        return Json::array({packet.at("publisher_id"), packet.at("session_id"),
                            packet.at("appearance"), packet.at("paused"),
                            packet.at("ipc").at("enabled"), camera.at("width"),
                            camera.at("height"), camera.at("K"), camera.at("distortion"),
                            camera.at("R_camera2gimbal"), camera.at("t_camera2gimbal")});
    }

    static void read_all(int fd, void* output, size_t bytes){
        auto data = static_cast<char*>(output);
        while(bytes){
            const auto count = ::read(fd, data, bytes);
            if(count < 0 && errno == EINTR) continue;
            if(count <= 0) throw runtime_error("文件读入不完整");
            data += count;
            bytes -= static_cast<size_t>(count);
        }
    }

    Json read_json(const char* filename) const{
        int fd = ::openat(directory_, filename, O_RDONLY | O_CLOEXEC | O_NOFOLLOW);
        if(fd < 0) return Json::object();
        struct stat info{};
        string data;
        try{
            if(::fstat(fd, &info) != 0 || !S_ISREG(info.st_mode) || info.st_size <= 0 ||
               info.st_size > 65536)
                throw runtime_error("JSON 文件大小/类型非法");
            data.resize(static_cast<size_t>(info.st_size));
            read_all(fd, data.data(), data.size());
        }catch(...){
            ::close(fd);
            throw;
        }
        ::close(fd);
        Json value = Json::parse(data, nullptr, false);
        return value.is_object() ? value : Json::object();
    }

    void write_json(const char* filename, const Json& value) const{
        const string temporary = string(filename) + ".client.tmp";
        int fd = ::openat(directory_, temporary.c_str(),
                          O_CREAT | O_TRUNC | O_WRONLY | O_CLOEXEC | O_NOFOLLOW, 0600);
        if(fd < 0) throw runtime_error("无法创建请求临时文件");
        try{
            const string data = value.dump();
            size_t written = 0;
            while(written < data.size()){
                const auto count = ::write(fd, data.data() + written, data.size() - written);
                if(count < 0 && errno == EINTR) continue;
                if(count <= 0) throw runtime_error("文件写入不完整");
                written += static_cast<size_t>(count);
            }
        }catch(...){
            ::close(fd);
            ::unlinkat(directory_, temporary.c_str(), 0);
            throw;
        }
        ::close(fd);
        if(::renameat(directory_, temporary.c_str(), directory_, filename) != 0)
            throw runtime_error("请求文件原子发布失败");
    }
};

//Ctrl+C / kill 时只置一个标志位，真正的收尾放在主循环之后
volatile std::sig_atomic_t exiting = 0 ;
static void on_signal(int){ exiting = 1 ; }

//统一取图入口：仿真走共享内存，否则走摄像头
//返回false：仿真这一轮还没给帧（继续轮询），或者摄像头已经读不到图（结束）
static bool grabFrame(SimFrameClient* sim, VideoCapture& cap, Mat& frame){
    if(sim == nullptr){
        cap >> frame ;
        return !frame.empty() ;
    }
    sim->poll_state();
    sim->request_frame();
    SimImage sample;
    if(!sim->try_frame(sample)) return false;
    frame = std::move(sample.bgr);
    return true;
}

    //=========终于到了主函数部分==========
int main(int argc, char** argv){
    try{
        //=========参数解析：一个参数都不写时，行为和以前完全一样（打开0号摄像头、红方、ESC退出）=========
        string channel = "default" ;//仿真通道名，对应 rm_aim_sim 的 [ipc].channel
        string dump_dir ;//--dump=DIR 时保存若干组原图/二值图/绘制图，方便离线核对
        bool use_sim = false ;//是否从仿真共享内存取图；false 就是原来的摄像头模式
        bool is_red = true ;//在实际情况下这个会由电控组的串口传过来
        bool display = true ;//--no-display 时不建窗口，只算不显示
        int max_frames = 0 ;//--frames=N 收满N帧退出，0表示不限
        int scale = 1 ;//--scale=N 先把图放大N倍再走原算法，结果坐标再除回来
        double duration = 0 ;//--duration=S 跑S秒退出，0表示不限

        for(int i = 1;i < argc;i++){
            const string arg = argv[i];
            if(arg == "--help"){
                cout << "to-see-armor 用法：（不带参数 = 打开0号摄像头，和以前完全一样）\n"
                        "  ./to-see-armor                      摄像头模式，红方，ESC退出\n"
                        "  ./to-see-armor --sim                从 rm_aim_sim 共享内存取图\n"
                        "  ./to-see-armor --sim=team_a         指定通道，等价 --channel=team_a\n"
                        "选项：\n"
                        "  --channel=NAME    仿真通道名（默认 default）\n"
                        "  --team=red|blue   识别哪一方装甲板（默认 red）\n"
                        "  --frames=N        收满N帧后退出（默认0=不限）\n"
                        "  --duration=S      跑S秒后退出（默认0=不限）\n"
                        "  --scale=N         先把图放大N倍(1~4)再走原算法，坐标自动还原（默认1）\n"
                        "  --no-display      不建窗口，只计算\n"
                        "  --dump=DIR        保存最多20组 raw_/mask_/draw_ PNG\n"
                        "  --help            显示本帮助\n"
                        "仿真必须先启动：cd ~/rm_aim_sim && ./run.sh -- --remote\n";
                return 0;
            }
            if(arg == "--sim"){ use_sim = true ; continue ; }
            if(arg == "--no-display"){ display = false ; continue ; }
            const auto equals = arg.find('=');
            if(equals == string::npos) throw runtime_error("参数需要写成 = 值 的形式: " + arg);
            const string key = arg.substr(0, equals);
            const string value = arg.substr(equals + 1);
            if(key == "--sim" || key == "--channel"){ use_sim = true ; channel = value ; }
            else if(key == "--team"){
                if(value != "blue" && value != "red") throw runtime_error("--team 只能是 blue 或 red");
                is_red = (value == "red");
            }
            else if(key == "--dump"){ dump_dir = value ; }
            else{
                size_t used = 0;
                double number = 0;
                try{ number = stod(value, &used); }
                catch(const std::exception&){ throw runtime_error("数值参数非法: " + arg); }
                if(used != value.size() || !std::isfinite(number)) throw runtime_error("数值参数非法: " + arg);
                if(key == "--frames" && number >= 0 && number == floor(number)) max_frames = static_cast<int>(number);
                else if(key == "--duration" && number >= 0) duration = number;
                else if(key == "--scale" && number >= 1 && number <= 4 && number == floor(number)) scale = static_cast<int>(number);
                else throw runtime_error("未知参数或取值越界: " + arg);
            }
        }

        if(!dump_dir.empty() && ::mkdir(dump_dir.c_str(), 0755) != 0 && errno != EEXIST)
            throw runtime_error("无法创建 dump 目录: " + dump_dir);

        std::signal(SIGINT, on_signal);
        std::signal(SIGTERM, on_signal);

        //=========初始化视频源========
        VideoCapture cap ;
        std::unique_ptr<SimFrameClient> client ;
        if(use_sim){
            client.reset(new SimFrameClient(channel));
        }else{
            cap.open(0);
            if(!cap.isOpened()){
                cout<<"不是哥们摄像头都不开啊"<<endl;
                return -1;
            }
        }

        Mat frame ;
        int count = 0 , saved = 0 ;
        size_t last_bars = 0 , last_armors = 0 ;
        const auto begin = Clock::now();

        //==========视觉主循环=========
        while(!exiting){
            if(duration > 0 && std::chrono::duration<double>(Clock::now() - begin).count() >= duration) break;

            if(!grabFrame(client.get(), cap, frame)){
                if(use_sim){//仿真这一轮还没渲染好，等2ms再问，别空转把CPU吃满
                    std::this_thread::sleep_for(std::chrono::milliseconds(2));
                    continue;
                }
                break;//摄像头按原来的逻辑：读不到图就结束
            }
            ++count;

            //--scale>1 时先放大再走原算法：仿真里灯条只有2~4px宽，原阈值会把它全部滤掉
            Mat input = frame ;
            if(scale > 1) resize(frame, input, Size(), scale, scale, INTER_LINEAR);

            Mat binary_img = preprocess(input, is_red);
            vector<RotatedRect> light_bars = findLightBars(binary_img);
            vector<Armor> armors = matchArmors(light_bars);

            //放大过的结果要除回原始像素坐标，才能在原图上画对位置
            if(scale > 1){
                const float s = static_cast<float>(scale);
                for(auto& bar : light_bars){
                    bar.center /= s;
                    bar.size = Size2f(bar.size.width / s, bar.size.height / s);
                }
                for(auto& armor : armors){
                    armor.center /= s;
                    for(auto& p : armor.corners) p /= s;
                    armor.bounding_box = boundingRect(armor.corners);
                }
            }
            last_bars = light_bars.size();
            last_armors = armors.size();

            Mat raw_for_dump ;//先留一份没画线的原图，方便和画过的图对比
            if(!dump_dir.empty() && saved < 20) raw_for_dump = frame.clone();

            for(size_t i = 0 ;i < armors.size();i++){
                Armor current_armor = armors[i];
                rectangle(frame,current_armor.bounding_box,Scalar(0,255,0),2);//在原图上画出装甲板的边界矩形框，颜色为绿色，(saclsr(0 , 255,0))这里就表明了是绿色，线宽为2个像素
                circle(frame,current_armor.center,5,Scalar(0,0,255),-1);//在原图上画出装甲板的中心点，颜色为蓝色，半径为5，线宽为-1的意思是填充圆形

                vector<Point> corner_points;
                for(const auto& pt : current_armor.corners){
                    corner_points.push_back(Point(pt.x,pt.y));//把四个角点的坐标转换成整数类型，存放在一个动态数组里，方便后面画出多边形
                }
                if(corner_points.size()== 4){
                    polylines(frame,corner_points,true,Scalar(0,255,0),2);//在原图上画出装甲板的四个角点，颜色为红色，线宽为2个像素
                }
                cout<<"发现了第"<<i+1<<"块装甲板，中心点坐标为("<<current_armor.center.x<<","<<current_armor.center.y<<")"<<endl;
            }

            if(!dump_dir.empty() && saved < 20){
                imwrite(dump_dir + "/raw_" + to_string(saved) + ".png", raw_for_dump);
                imwrite(dump_dir + "/mask_" + to_string(saved) + ".png", binary_img);
                imwrite(dump_dir + "/draw_" + to_string(saved) + ".png", frame);
                ++saved;
            }

            if(display){
                Mat binary_show = binary_img ;
                if(scale > 1) resize(binary_img, binary_show, frame.size(), 0, 0, INTER_NEAREST);//二值图放大过，缩回原尺寸方便对照
                imshow("最初图像",frame);
                imshow("二值化图像",binary_show);

                if(waitKey(1) == 27)break ;//表示按下ESC键退出循环，27是ESC键的ASCII码
            }

            if(max_frames > 0 && count >= max_frames) break;
        }

        cap.release();
        if(display) destroyAllWindows();

        if(use_sim && count == 0)
            throw runtime_error("一帧都没收到，按顺序检查：\n"
                                "  1) 仿真是否已启动：cd ~/rm_aim_sim && ./run.sh -- --remote\n"
                                "  2) 通道是否一致：本程序 --channel=? 与 configs/sim.conf 的 [ipc].channel\n"
                                "  3) 仿真是否用了 --headless：相机帧必须图形模式\n"
                                "  4) 是否有其它客户端正占着 client.lock");
        if(use_sim){
            const double seconds = std::chrono::duration<double>(Clock::now() - begin).count();
            cout << "共收到 " << count << " 帧（平均 " << (seconds > 0 ? count / seconds : 0.0)
                 << " fps）；最后灯条 " << last_bars << " 个，装甲板 " << last_armors << " 块" << endl;
        }
        return 0 ;
    }catch(const std::exception& error){
        cerr << "to-see-armor 错误: " << error.what() << endl;
        return 1;
    }
}
