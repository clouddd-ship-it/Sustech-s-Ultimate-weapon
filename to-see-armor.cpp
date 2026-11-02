#include <algorithm>
#include <opencv2/opencv.hpp>//提供opencv的图像处理，矩阵运算，特征点提取
#include <iostream>
#include <vector>//引入动态数组，可以动态增删元素
#include <cmath>//引用常见的数学运算
#include <opencv2/videoio.hpp>//提供视频输入输出的功能
#include"opencv2/core.hpp"//提供opencv的核心功能，包括矩阵运算，图像处理，特征点提取等

using namespace std ;
using namespace cv ;

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
            float a = rect.angle;
            if(a > 45.0f) a -= 90.0f;
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

    //=========终于到了主函数部分==========
int main(){
    //=========初始化视频源========
    cv::VideoCapture cap(0);
    if(!cap.isOpened()){
        cout<<"不是哥们摄像头都不开啊"<<endl;
        return -1;
    }

    bool is_red = true;//在实际情况下这个会由电控组的串口传过来。
    Mat frame;
    //==========视觉主循环=========
    while (true){
        cap >> frame ;
        if(frame.empty())break;

        //开始串联三个模块
        Mat binary_img = preprocess(frame,is_red);
        vector<RotatedRect> light_bars = findLightBars(binary_img);
        vector<Armor> armors = matchArmors(light_bars);

        for(size_t i = 0 ;i < armors.size();i++){
            Armor current_armor = armors[i];
            rectangle(frame,current_armor.bounding_box,Scalar(0,255,0),2);//在原图上画出装甲板的边界矩形框，颜色为绿色,(saclsr(0 , 255,0))这里就表明了是绿色，线宽为2个像素
            //我不是已经把装甲板的四个角点都算出来了，为什么还要画一个矩形框呢？因为这个矩形框是最小外接矩形，可能会比实际的装甲板大一点点，所以要再次画一个框来代表具体的装甲板。但是它的好处是可以把装甲板完整地包起来，方便后续的图像处理，比如说裁剪，透视变换等等。
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
        //显示和按键必须放在 for 外面、while 里面：没有装甲板时也要刷新画面，break 才能退出 while
        imshow("最初图像",frame);
        imshow("二值化图像",binary_img);

        if(waitKey(1) == 27)break ;//表示按下ESC键退出循环，27是ESC键的ASCII码
    }
    cap.release();
    destroyAllWindows();
    return 0 ;
}
