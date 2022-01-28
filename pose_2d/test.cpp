
#include <iostream>
#include <opencv2/core/core.hpp>
#include <ceres/ceres.h>
#include <chrono>

#include <opencv2/features2d/features2d.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <Eigen/SVD>
#include "ceres/rotation.h"

using namespace std;
using namespace cv;

struct cost_function_define
{
    cost_function_define(Point3f p1, Point3f p2) : _p1(p1), _p2(p2) {}
    template <typename T>
    bool operator()(const T *const cere_r, const T *const cere_t, T *residual) const
    {
        T p_1[3];
        T p_2[3];
        p_1[0] = T(_p1.x);
        p_1[1] = T(_p1.y);
        p_1[2] = T(_p1.z);

        ceres::AngleAxisRotatePoint(cere_r, p_1, p_2);
        p_2[0] = p_2[0] + cere_t[0];
        p_2[1] = p_2[1] + cere_t[1];
        p_2[2] = p_2[2] + cere_t[2];

        //观测的在图像坐标下的值
        T p2_x = T(_p2.x);
        T p2_y = T(_p2.y);
        T p2_z = T(_p2.z);

        T r2_r1 = (p_2[0] - p2_x) * (p_2[0] - p2_x) + (p_2[1] - p2_y) * (p_2[1] - p2_y) + (p_2[2] - p2_z) * (p_2[2] - p2_z);

        residual[0] = T(100.0) - r2_r1;

        return true;
    }
    Point3f _p1;
    Point3f _p2;
};

int main(int argc, char **argv)
{

    // 建立3D点
    vector<Point3f> r1_3d;    
    vector<Point3f> r2_3d;
    
r1_3d.push_back(Point3f(1.54300075112,-19.4569992489,0));
r1_3d.push_back(Point3f(3.31580109432,-18.6841989057,0));
r1_3d.push_back(Point3f(5.65649271543,-17.3435072846,0));
r1_3d.push_back(Point3f(7.12816216177,-16.8718378382,0));
r1_3d.push_back(Point3f(9.33342023501,-15.666579765,0));
r1_3d.push_back(Point3f(11.6803865518,-14.3196134482,0));
r1_3d.push_back(Point3f(13.2416298324,-13.7583701676,0));
r1_3d.push_back(Point3f(15.2772439141,-12.7227560859,0));
r1_3d.push_back(Point3f(17.9199455814,-11.0800544186,0));
r1_3d.push_back(Point3f(19.0363839899,-10.9636160101,0));
r1_3d.push_back(Point3f(21.1222494849,-9.87775051506,0));
r1_3d.push_back(Point3f(23.5554476215,-8.44455237847,0));
r1_3d.push_back(Point3f(25.3811756506,-7.61882434942,0));
r1_3d.push_back(Point3f(27.9011508206,-6.09884917938,0));
r1_3d.push_back(Point3f(29.4732609409,-5.52673905914,0));
r1_3d.push_back(Point3f(31.617104828,-4.38289517204,0));
r1_3d.push_back(Point3f(33.23929813,-3.76070187003,0));
r1_3d.push_back(Point3f(35.1978993213,-2.80210067866,0));
r1_3d.push_back(Point3f(37.3712934158,-1.62870658416,0));
r1_3d.push_back(Point3f(39.3579949815,-0.642005018468,0));
r2_3d.push_back(Point3f(20.1047214525,6.71610863402,0));
r2_3d.push_back(Point3f(24.0307060991,7.72849325268,0));
r2_3d.push_back(Point3f(20.9257716011,11.2886813212,0));
r2_3d.push_back(Point3f(24.6871575016,11.7816630913,0));
r2_3d.push_back(Point3f(19.5431696477,15.3457729133,0));
r2_3d.push_back(Point3f(24.6736492038,17.0397805843,0));
r2_3d.push_back(Point3f(24.4459240745,18.9123048114,0));
r2_3d.push_back(Point3f(20.5316769556,21.988319817,0));
r2_3d.push_back(Point3f(25.9577538797,23.5143568317,0));
r2_3d.push_back(Point3f(24.4565601295,25.5125616094,0));
r2_3d.push_back(Point3f(20.5145220639,28.4554538633,0));
r2_3d.push_back(Point3f(23.0046988684,31.1125042944,0));
r2_3d.push_back(Point3f(24.564276803,32.7396367517,0));
r2_3d.push_back(Point3f(23.6519896286,35.9365291269,0));
r2_3d.push_back(Point3f(27.9369454161,35.5532741693,0));
r2_3d.push_back(Point3f(20.2090959273,40.2119672556,0));
r2_3d.push_back(Point3f(24.7377120542,41.6076636923,0));
r2_3d.push_back(Point3f(27.993754742,42.2250396115,0));
r2_3d.push_back(Point3f(22.2706885974,46.668612841,0));
r2_3d.push_back(Point3f(25.7667644311,48.2395845935,0));

    cout << "3d-2d pairs: " << r2_3d.size() << "," << r1_3d.size() << endl;
    cout << "calling bundle adjustment" << endl;

    //给rot，和tranf初值
    double cere_rot[3], cere_tranf[3];
    cere_rot[0] = 0;
    cere_rot[1] = 0;
    cere_rot[2] = -0.0;

    cere_tranf[0] = 2.0;
    cere_tranf[1] = 1.0;
    cere_tranf[2] = 0.0;

    ceres::Problem problem;
    for (int i = 0; i < r2_3d.size(); i++)
    {
        ceres::CostFunction *costfunction = new ceres::AutoDiffCostFunction<cost_function_define, 1, 3, 3>(new cost_function_define(r1_3d[i], r2_3d[i]));
        problem.AddResidualBlock(costfunction, NULL, cere_rot, cere_tranf); //注意，cere_rot不能为Mat类型
    }

    ceres::Solver::Options option;
    option.linear_solver_type = ceres::DENSE_SCHUR;
    //输出迭代信息到屏幕
    option.minimizer_progress_to_stdout = true;
    //显示优化信息
    ceres::Solver::Summary summary;
    //开始求解
    ceres::Solve(option, &problem, &summary);
    //显示优化信息
    cout << summary.BriefReport() << endl;

    cout << "----------------optional after--------------------" << endl;

    Mat radar_recv = (Mat_<double>(3, 1) << cere_rot[0], cere_rot[1], cere_rot[2]);
    Mat radar_rotation;
    cv::Rodrigues(radar_recv, radar_rotation); // r为旋转向量形式，用Rodrigues公式转换为矩阵

    cout << "radar_recv:" << endl
         << radar_recv << endl;

    cout << "radar_rotation:" << endl
         << radar_rotation << endl;

    cout << "radar_t:" << cere_tranf[0] << "  " << cere_tranf[1] << "  " << cere_tranf[2] << endl;
}
