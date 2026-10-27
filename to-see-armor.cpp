#include <algorithm>
#include <opencv2/opencv.hpp>//提供opencv的图像处理，矩阵运算，特征点提取
#include <iostream>
#include <vector>//引入动态数组，可以动态增删元素
#include <cmath>//引用常见的数学运算

using namespace std ;
using namespace cv ;

struct Armor{
    Point2f center ;//point2f是opencv中的二维坐标点类型，包含x和y两个浮点数成员，用于描述装甲板的中心点坐标
    vector<Point2f> corners ;//是中心点的角点集合。表示一个存储point2f的动态数组，存储这四个角点的目的是用来计算装甲板的倾斜角度，进行透视变换求出机器人的3D姿态
    Rect bounding_box  ;//装甲板的边界矩形框。rect是opencv中表示矩形的类
};

//preprocess of picture
Mat preprocess(const Mat& frame,bool is_red )//Mat 返回类型代表了一个矩阵，说明这个函数处理完图像后会返回一个矩阵。const Mat& frame是传入的原始图像，叫一帧frame,const表示不会修改原图。引用&避免了复制一整张巨大的图片提高效率
{Mat channels[3] ;
    split(frame ,channels);//用于分离bgr通道,channal[0]是蓝色，1是绿色，2是红色。

    Mat diff;
    if(is_red){
        subtract(channels[2] ,channels[0],diff);
    }
    else{
        subtract(channels[0],channels[2],diff);
    }

    Mat binary;//二值化，大于100的变255否则变0,此处声明了一个名为binary的矩阵，用来存放二值化处理后的黑白图像
    threshold(diff,binary,100,255,THRESH_BINARY);//输入图像，输出图像，阈值，超过阈值时变成什么值，阈值类型。阈值类型可以理解为“你要用哪一种比较规则把图像分成目标和背景”
    //形态学操作去噪点
    Mat kernel = getStructuringElement(MORPH_RECT ,Size(3,3)) ;//getStructuringElement是一个opencv提供的用来生成一个特定形状刷子的函数，MORPH_RECT表示刷子的形状是矩形，尺寸为三像素×三像素
    GaussianBlur(binary,binary,Size(3,3),0);
    morphologyEx(binary , binary, MORPH_OPEN,kernel);//开运算，先腐蚀后膨胀
    morphologyEx(binary ,binary,MORPH_CLOSE,kernel);//闭运算，先膨胀后腐蚀

    return binary;
}

    //灯条提取
    vector<RotatedRect> findLightBars(const Mat& binary_img)//vector<RotatedRect>是这个函数的返回值，，即这个函数结束后会返回一堆旋转矩形。
    {
        vector<vector<Point>> contours;//寻找外轮廓.vector是动态数组，point是数据类型表示存的是一个点，vector<point>表示存一系列点，就是一个轮廓。vector<vector<point>>就是一堆轮廓，可以共同构成装甲板
        findContours(binary_img,contours,RETR_EXTERNAL,CHAIN_APPROX_SIMPLE);

        vector<RotatedRect> light_bars;
    }


