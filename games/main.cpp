// C++ 标准库
#include <iostream>
#include <string>
using namespace std;
// OpenCV 库
#include <opencv2/opencv.hpp>


// 主函数 
int main(int argc, char** argv)
{
    // 图像矩阵
    cv::Mat rgb, depth;

    rgb = cv::imread("../data/Hero.png");
    // rgb 图像是8UC3的彩色图像
    cv::resize(rgb, depth, cv::Size(40, 60));
    cv::imwrite("../data/ret.jpg", depth);
    cv::imshow("Hero", depth);
    cv::waitKey();
    
    return 0;        

}
