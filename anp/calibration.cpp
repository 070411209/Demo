#include <iostream>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/opencv.hpp>
#include <vector>

using namespace std;
using namespace cv;

int main(int argc, char** argv) {
    cv::Mat imageL = cv::imread("../6_24.bmp", -1);
    double camD[9] = {  1818.24, 0, 973.382,
                        0, 1817.57, 563.979,
                        0, 0, 1};

    double distCoeffD[5] = {-0.365267, 0.03125, 0.0, 0.0, 0.0};
    
    Mat camera_matrix = Mat(3, 3, CV_64FC1, camD);
    Mat distortion_coefficients = Mat(5, 1, CV_64FC1, distCoeffD);

    vector<cv::Point3f> Points3D;
    Points3D.push_back(cv::Point3f(0,   0, 0));      // P1 三维坐标的单位是毫米
    Points3D.push_back(cv::Point3f(290, 0, 0));    // P2
    Points3D.push_back(cv::Point3f(580, 0, 0));    // P3

    Points3D.push_back(cv::Point3f(580, 290, 0));  // P4
    Points3D.push_back(cv::Point3f(290, 290, 0));      // P5 三维坐标的单位是毫米
    Points3D.push_back(cv::Point3f(0,   290, 0));    // P6

    Points3D.push_back(cv::Point3f(0,   580, 0));    // P7
    Points3D.push_back(cv::Point3f(290, 580, 0));  // P8
    Points3D.push_back(cv::Point3f(580, 580, 0));  // P9

    vector<cv::Point2f> Points2D;
    Points2D.push_back(cv::Point2f(861, 1049));  // P1 单位是像素
    Points2D.push_back(cv::Point2f(863, 1035));  // P2
    Points2D.push_back(cv::Point2f(865, 1024));  // P3

    Points2D.push_back(cv::Point2f(814, 1023));  // P4 单位是像素
    Points2D.push_back(cv::Point2f(811, 1034));  // P5
    Points2D.push_back(cv::Point2f(809, 1048));  // P6

    Points2D.push_back(cv::Point2f(756, 1046));  // P7 单位是像素
    Points2D.push_back(cv::Point2f(760, 1033));  // P8
    Points2D.push_back(cv::Point2f(765, 1021));  // P9

    //初始化输出矩阵
    cv::Mat rvec = cv::Mat::zeros(3, 1, CV_64FC1);
    cv::Mat tvec = cv::Mat::zeros(3, 1, CV_64FC1);

    //三种方法求解
    // solvePnP(Points3D, Points2D, camera_matrix, distortion_coefficients, rvec,
    //         tvec, false, CV_ITERATIVE);  //实测迭代法似乎只能用共面特征点求位置
    // solvePnP(Points3D, Points2D, camera_matrix, distortion_coefficients, rvec,
    // tvec, false, CV_P3P);        //Gao的方法可以使用任意四个特征点
    // The estimated pose is thus the rotation (rvec) and the translation (tvec) vectors that allow transforming a 3D point expressed in the world frame into the camera frame
    solvePnP(Points3D, Points2D, camera_matrix, distortion_coefficients, rvec, tvec, false, CV_EPNP);

    //旋转向量变旋转矩阵
    double rm[9];
    cv::Mat rotM(3, 3, CV_64FC1, rm);
    Rodrigues(rvec, rotM);


    //计算出相机坐标系的三轴旋转欧拉角，旋转后可以转出世界坐标系。
    //旋转顺序为z、y、x
    const double PI = 3.141592653;
    double thetaz = atan2(rotM.at<double>(1, 0), rotM.at<double>(0, 0)) / PI * 180;
    double thetay = atan2(-1 * rotM.at<double>(2, 0), sqrt(rotM.at<double>(2, 1)*rotM.at<double>(2, 1) + rotM.at<double>(2, 2)*rotM.at<double>(2, 2))) / PI * 180;
    double thetax = atan2(rotM.at<double>(2, 1), rotM.at<double>(2, 2)) / PI * 180;
    
	vector<cv::Point2f> projectedPoints;
    vector<cv::Point3f> objPM;
    objPM.push_back(cv::Point3f(0,   870, 0.0));      // P1 三维坐标的单位是毫米   
    objPM.push_back(cv::Point3f(290, 870, 0.0));      // P1 三维坐标的单位是毫米  
    objPM.push_back(cv::Point3f(570, 900, 0.0));      // P1 三维坐标的单位是毫米   
	projectPoints(objPM, rvec, tvec, camera_matrix, distortion_coefficients, projectedPoints);
		
	for(unsigned int i = 0; i < projectedPoints.size(); ++i)
	{
		circle(imageL, projectedPoints[i], 3, Scalar(0,255,0), -1, 8);
	}

    // Rodrigues(tvec, rotT);
    cout << "rotation matrix: " << endl << rotM << endl;
    cout << "translation matrix: " << endl
        << "x:"<< tvec.at<double>(0, 0)/10.0 << "cm,y:" << tvec.at<double>(1, 0)/10.0 << "cm,z:"
        << tvec.at<double>(2, 0)/10.0 << "cm."<< endl;

    cout << "eular argular: " << thetaz << "," << thetay << "," << thetax << "." << endl; 

    for(unsigned int i = 0; i < Points2D.size(); ++i)  
    {  
        circle(imageL, Points2D[i], 3, Scalar(255,i*20,0), 1, 8);  
    }    
    cv::imshow("show", imageL);
    char c = cv::waitKey();
    if( c == 27 || c == 'q' || c == 'Q' )
        return 0;  

    return 0;
}
