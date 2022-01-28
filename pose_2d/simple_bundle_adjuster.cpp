//
// Author: keir@google.com (Keir Mierle)
//
// A minimal, self-contained bundle adjuster using Ceres, that reads
// files from University of Washington' Bundle Adjustment in the Large dataset:
// http://grail.cs.washington.edu/projects/bal
//
// This does not use the best configuration for solving; see the more involved
// bundle_adjuster.cc file for details.

#include <cmath>
#include <cstdio>
#include <iostream>

#include "ceres/ceres.h"
#include "ceres/rotation.h"


const int kNumObservations = 67;
// clang-format off
const double data[] = {
0.000000e+00, 1.133898e+00,
7.500000e-02, 1.334902e+00,
1.500000e-01, 1.213546e+00,
2.250000e-01, 1.252016e+00,
3.000000e-01, 1.392265e+00,
3.750000e-01, 1.314458e+00,
4.500000e-01, 1.472541e+00,
5.250000e-01, 1.536218e+00,
6.000000e-01, 1.355679e+00,
6.750000e-01, 1.463566e+00,
7.500000e-01, 1.490201e+00,
8.250000e-01, 1.658699e+00,
9.000000e-01, 1.067574e+00,
9.750000e-01, 1.464629e+00,
1.050000e+00, 1.402653e+00,
1.125000e+00, 1.713141e+00,
1.200000e+00, 1.527021e+00,
1.275000e+00, 1.702632e+00,
1.350000e+00, 1.423899e+00,
1.425000e+00, 5.543078e+00, // Outlier point
1.500000e+00, 5.664015e+00, // Outlier point
1.575000e+00, 1.732484e+00,
1.650000e+00, 1.543296e+00,
1.725000e+00, 1.959523e+00,
1.800000e+00, 1.685132e+00,
1.875000e+00, 1.951791e+00,
1.950000e+00, 2.095346e+00,
2.025000e+00, 2.361460e+00,
2.100000e+00, 2.169119e+00,
2.175000e+00, 2.061745e+00,
2.250000e+00, 2.178641e+00,
2.325000e+00, 2.104346e+00,
2.400000e+00, 2.584470e+00,
2.475000e+00, 1.914158e+00,
2.550000e+00, 2.368375e+00,
2.625000e+00, 2.686125e+00,
2.700000e+00, 2.712395e+00,
2.775000e+00, 2.499511e+00,
2.850000e+00, 2.558897e+00,
2.925000e+00, 2.309154e+00,
3.000000e+00, 2.869503e+00,
3.075000e+00, 3.116645e+00,
3.150000e+00, 3.094907e+00,
3.225000e+00, 2.471759e+00,
3.300000e+00, 3.017131e+00,
3.375000e+00, 3.232381e+00,
3.450000e+00, 2.944596e+00,
3.525000e+00, 3.385343e+00,
3.600000e+00, 3.199826e+00,
3.675000e+00, 3.423039e+00,
3.750000e+00, 3.621552e+00,
3.825000e+00, 3.559255e+00,
3.900000e+00, 3.530713e+00,
3.975000e+00, 3.561766e+00,
4.050000e+00, 3.544574e+00,
4.125000e+00, 3.867945e+00,
4.200000e+00, 4.049776e+00,
4.275000e+00, 3.885601e+00,
4.350000e+00, 4.110505e+00,
4.425000e+00, 4.345320e+00,
4.500000e+00, 4.161241e+00,
4.575000e+00, 4.363407e+00,
4.650000e+00, 4.161576e+00,
4.725000e+00, 4.619728e+00,
4.800000e+00, 4.737410e+00,
4.875000e+00, 4.727863e+00,
4.950000e+00, 4.669206e+00
};
// clang-format on

// Templated pinhole camera model for used with Ceres.  The camera is
// parameterized using 9 parameters: 3 for rotation, 3 for translation, 1 for
// focal length and 2 for radial distortion. The principal point is not modeled
// (i.e. it is assumed be located at the image center).
struct SnavelyReprojectionError {
  SnavelyReprojectionError(double observed_x, double observed_y)
      : observed_x(observed_x), observed_y(observed_y) {}

  template <typename T>
  bool operator()(const T* const camera,
                  const T* const point,
                  T* residuals) const {
    // camera[0,1,2] are the angle-axis rotation.
    T p[3];
    ceres::AngleAxisRotatePoint(camera, point, p);

    // camera[3,4,5] are the translation.
    p[0] += camera[3];
    p[1] += camera[4];
    p[2] += camera[5];

    // Compute the center of distortion. The sign change comes from
    // the camera model that Noah Snavely's Bundler assumes, whereby
    // the camera coordinate system has a negative z axis.
    T xp = -p[0] / p[2];
    T yp = -p[1] / p[2];

    // Apply second and fourth order radial distortion.
    const T& l1 = camera[7];
    const T& l2 = camera[8];
    T r2 = xp * xp + yp * yp;
    T distortion = 1.0 + r2 * (l1 + l2 * r2);

    // Compute final projected point position.
    const T& focal = camera[6];
    T predicted_x = focal * distortion * xp;
    T predicted_y = focal * distortion * yp;

    // The error is the difference between the predicted and observed position.
    residuals[0] = predicted_x - observed_x;

    return true;
  }

  // Factory to hide the construction of the CostFunction object from
  // the client code.
  static ceres::CostFunction* Create(const double observed_x,
                                     const double observed_y) {
    // ceres官方极力推荐用户使用自动求导方式
    return (new ceres::AutoDiffCostFunction<SnavelyReprojectionError, 1, 1, 1>(
        new SnavelyReprojectionError(observed_x, observed_y)));
  }

  double observed_x;
  double observed_y;
};

int main(int argc, char** argv) {
  google::InitGoogleLogging(argv[0]);
  if (argc != 2) {
    std::cerr << "usage: simple_bundle_adjuster <bal_problem>\n";
    return 1;
  }

  double m = 0.0;
  double c = 0.0;

  // Create residuals for each observation in the bundle adjustment problem. The
  // parameters for cameras and points are added automatically.
  ceres::Problem problem;
  for (int i = 0; i < kNumObservations; ++i) {

    ceres::CostFunction* cost_function = SnavelyReprojectionError::Create(
        data[2 * i], data[2 * i + 1]);
    // 传递的参数主要包括代价函数模块、损失函数模块和参数模块
    problem.AddResidualBlock(cost_function,
                             new ceres::CauchyLoss(0.5),
                             &m, &c);
  }

  // Make Ceres automatically detect the bundle structure. Note that the
  // standard solver, SPARSE_NORMAL_CHOLESKY, also works fine but it is slower
  // for standard bundle adjustment problems.
  ceres::Solver::Options options;
  options.linear_solver_type = ceres::DENSE_SCHUR;
  options.minimizer_progress_to_stdout = true;

  ceres::Solver::Summary summary;
  ceres::Solve(options, &problem, &summary);
  std::cout << summary.FullReport() << "\n";
  return 0;
}
