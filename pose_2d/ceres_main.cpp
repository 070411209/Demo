#include <iostream>
#include <opencv2/opencv.hpp>
#include <ceres/ceres.h>
#include <chrono>
#include <cmath>
#include "glog/logging.h"
#include <cstdio>
#include <vector>
#include <fstream>

using ceres::AutoDiffCostFunction;
using ceres::CauchyLoss;
using ceres::CostFunction;
using ceres::LossFunction;
using ceres::Problem;
using ceres::Solve;
using ceres::Solver;

using namespace std;

#define PI 3.14159265
#define PI_rad 180.0 / PI

DEFINE_double(robust_threshold,
              0.0,
              "Robust loss parameter. Set to 0 for normal squared error (no "
              "robustification).");

// The cost for a single sample. The returned residual is related to the
class DistanceFromCircleCost
{
public:
  DistanceFromCircleCost(cv::Point2d xx, cv::Point2d yy, double dist) : xx_(xx), yy_(yy), dist_(dist) {}

  template <typename T>
  bool operator()(const T *const x,
                  const T *const y,
                  const T *const r,
                  T *residual) const
  {
    // https://blog.csdn.net/Mynameisyournamewuyu/article/details/88650409
    // [cos@, sin@]
    // [-sin@, cos@]
    T new_x = xx_.x * cos(*r) + xx_.y * sin(*r) + (*x);
    T new_y = -xx_.x * sin(*r) + xx_.y * cos(*r) + (*y);

    T xp = yy_.x - new_x;
    T yp = yy_.y - new_y;

    // It is tempting to use the following cost:
    residual[0] = T(dist_) - sqrt(xp * xp + yp * yp);
    return true;
  }

private:
  // The measured x,y coordinate that should be on the circle.
  cv::Point2d xx_, yy_;
  cv::Point2d proj_y;
  cv::Point2d err;
  double dist_;
};

int main(int argc, char **argv)
{

  GFLAGS_NAMESPACE::ParseCommandLineFlags(&argc, &argv, true);
  google::InitGoogleLogging(argv[0]);
  ofstream ofs;
  std::string inputDir;
  std::string outputDir;
  double set_distance;

  if (argc >= 3)
  {
    inputDir = argv[1];
    outputDir = argv[2];
    set_distance = atof(argv[3]);

    std::cout << "---->>> input dir: " << inputDir << std::endl;
    std::cout << "---->>> output dir: " << outputDir << std::endl;
    std::cout << "---->>> set distance: " << set_distance << std::endl;
  }
  else
  {
    std::cout << "./app input output distance" << std::endl;
    return 0;
  }

  std::string radarFile = inputDir + "/test.csv";
  radar_model::RadarProcess radar_process_;
  radar_process_.readDualData(radarFile);

  std::vector<std::vector<cv::Point2f>> dual_radar = radar_process_.getRadarPoints();
  std::vector<cv::Point2d> y_data, x_data; // 数据
  std::cout << "-------------- load data ----------------" << std::endl;
  std::cout << "dual radar size: " << dual_radar.size() << std::endl;

  assert(dual_radar.size() % 2 == 0);

  // left
  std::cout << "----- Left Radar ------" << std::endl;
  for (size_t i = 0; i < dual_radar.size() / 2; ++i)
  {
    for (size_t j = 0; j < dual_radar[i].size(); j++)
    {
      x_data.push_back(dual_radar[i][j]);
      std::cout << dual_radar[i][j] << ";";
    }
    std::cout << std::endl;
  }
  // right
  std::cout << "----- Right Radar ------" << std::endl;
  for (size_t i = dual_radar.size() / 2; i < dual_radar.size(); ++i)
  {
    for (size_t j = 0; j < dual_radar[i].size(); j++)
    {
      y_data.push_back(dual_radar[i][j]);
      std::cout << dual_radar[i][j] << ";";
    }
    std::cout << std::endl;
  }



  // Process
  double x = -2.0;
  double y = 0.0;
  double r = (-65)*PI/180;  // 逆时针为+，顺时针为-

  // Save initial values for comparison.
  double initial_x = x;
  double initial_y = y;
  double initial_r = r;
  // Parameterize r as m^2 so that it can't be negative.
  ceres::Problem problem;
  ceres::LossFunction *loss_function;                           // 损失核函数 
  //loss_function = new ceres::HuberLoss(1.0);
  loss_function = new ceres::CauchyLoss(1.0);                   // 柯西核函数  

  // Add the residuals.
  for (int i = 0; i < x_data.size(); i++)
  {
    CostFunction *cost =
        new AutoDiffCostFunction<DistanceFromCircleCost, 1, 1, 1, 1>(
            new DistanceFromCircleCost(x_data[i], y_data[i], set_distance));
    problem.AddResidualBlock(cost, loss_function, &x, &y, &r);
  }

  // Build and solve the problem.
  Solver::Options options;
  options.max_num_iterations = 20;
  options.linear_solver_type = ceres::DENSE_SCHUR; // ceres::DENSE_QR;
  options.minimizer_progress_to_stdout = true;     // 输出到cout

  Solver::Summary summary;
  Solve(options, &problem, &summary);

  std::cout << summary.BriefReport() << "\n";
  std::cout << "raw x : " << initial_x << " , opt x-> " << x << "\n";
  std::cout << "raw y : " << initial_y << " , opt y-> " << y << "\n";
  std::cout << "raw r : " << initial_r << " , opt r-> " << r << "\n";
  std::cout << "final yaw(deg): " << r * PI_rad << std::endl;


#endif

  return 0;
}
