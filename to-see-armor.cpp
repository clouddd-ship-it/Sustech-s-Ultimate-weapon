#include <algorithm>
#include <opencv2/opencv.hpp>//提供opencv的图像处理，矩阵运算，特征点提取
#include <iostream>
#include <vector>//引入动态数组，可以动态增删元素
#include <cmath>//引用常见的数学运算

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
        if (width > height) swap(width,height);

        float aspect_ratio = height /width ;//高宽比
        float solidity = area/(height*width) ;//填充率。一个完美的矩形，当你用橡皮筋去包它的时候会完全贴合；但是如果它表面坑坑洼洼，或者有中空，area(实际面积）就会小于橡皮筋的底乘高
        
        if(aspect_ratio > 1.5 && aspect_ratio <10.0 && solidity >0.8 && area > 50 && area <5000){
        
            if(rect.size.width > rect.size.height) {
                rect.angle += 90.0f;
            }
            light_bars.push_back(rect);
        }//这里看不懂加90度的意思。。。。。
        }
        return light_bars;
    }

    //=========模块三：装甲板配对============
    vector<Armor> matchArmors(const vector<RotatedRect>& light_bars){
        vector<Armor> armors;
        for(size_t i = 0;i < light_bars.size();i++){
            for(size_t j = i+1;j < light_bars.size();j++){
            RotatedRect bar1 = light_bars[i] ;
            RotatedRect bar2 = light_bars[j] ;
            if(bar1.center.x > bar2.center.x)swap(bar1,bar2);

            float h1 =max(bar1.size.width, bar1.size.height);
            float h2 =max(bar2.size.width , bar2.size.height);
            if(abs(h1-h2)/max(h1,h2) > 0.2)continue;
            if(abs(bar1.angle - bar2.angle) > 15.0)continue;

            float dist = norm(bar1.center - bar2.center);
            if(dist / max(h1,h2)<1.5||dist /max(h1,h2) > 4.0)continue;
            }
        }

    }

