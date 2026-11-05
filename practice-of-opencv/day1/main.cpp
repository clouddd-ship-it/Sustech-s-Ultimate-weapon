#include<iostream>
#include<opencv2/opencv.hpp>
using namespace std;
using namespace cv;

int main(){
    Mat img = imread("../test.jpg",cv::IMREAD_COLOR);//如果实在不会写就写绝对路径。。。/home/lvshuang/ARTINX-visual-work/day1/(文件名)
    //..表示返回上一级目录
    if(img.empty()){
        cout<<"不是哥们图像给我传好了啊，必须压力"<<endl;
        return -1;//如果你不在这里加一个return -1那就会继续跑下去，继续执行下面的每一行代码，就相当于你的空矩阵检测没有任何效果。
        //在c++中所有非零数都是错误，常用-1而已。在这里触发-1就不会继续跑了。
    }

    cout<<"图像的高度为："<<img.rows<<endl;
    cout<<"图像的宽度为： "<<img.cols<<endl;
    cout<<"图像的通道数为： "<<img.channels()<<endl;
    cout<<"图像的格式为： "<<img.type()<<endl;

    imshow("窗口名",img);
    waitKey(0);
    destroyAllWindows();

    Mat rgb ;
    cvtColor(img , rgb,cv::COLOR_BGR2RGB);

    imshow("窗口名是必须要打的，不大会报错的",rgb);
    waitKey(0);
    destroyAllWindows();

    Mat gray;
    cvtColor(img,gray,COLOR_BGR2GRAY);

    cout<<"gray:rows"<<gray.rows<<endl;
    cout<<"gray:cols"<<gray.cols<<endl;
    cout<<"gray:channels"<<gray.channels()<<endl;
    cout<<"gray:type"<<gray.type()<<endl;

    imshow("窗口名是必须要打的，不打会报错的",gray);
    waitKey(0);
    destroyAllWindows();

    return 0;
}