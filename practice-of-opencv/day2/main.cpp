#include<opencv2/opencv.hpp>
#include<iostream>
#include<vector>
#include<algorithm>//用于std::max和std::min

using namespace cv;
using namespace std;

int main(){
    Mat img = imread("../robomaster-picture/Bluecar.jpg");
        if(img.empty()){
            cout<<"不是哥们图像都没有啊？？可以收拾东西走人了别在这里丢人了。"<<endl;//cerr是什么？
            return 325;
        }
        cout<<"width :"<<img.cols<<endl;
        cout<<"height:"<<img.rows<<endl;
        cout<<"channels"<<img.channels()<<endl;
        cout<<"type:"<<img.type()<<endl;

        int center_x = img.cols/2 - 100;
        int center_y = img.rows/2 - 100;

        Rect roi_rect(center_x,center_y,200,200);
        
        cout<<"任务一：浅拷贝验证"<<endl;
        
        if( roi_rect.x >=0 && roi_rect.y >=0 && 
            roi_rect.x + roi_rect.width <= img.cols && 
            roi_rect.y + roi_rect.height <= img.rows  ){
        cout<<"数据合法"<<endl;
        Mat roi_img = img(roi_rect);

        Point pt1(10,10);//注意这里是相对roi_img的坐标
        Point pt2(190,190);
        rectangle(roi_img,pt1,pt2,Scalar(0,255,0),1,LINE_AA);
        imshow("好好看看吧bro",img);
        waitKey(0);
        }else{
            cout<<"ROI越界了你懂吗?"<<endl;
        }
//Mat roi_img = img(roi_rect);//这里img的作用是什么？它是一个函数吗？


return 0;
}