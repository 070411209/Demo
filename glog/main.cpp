#include <iostream>
#include <glog/logging.h>
#include <chrono>
#include <nlopt.hpp>
#include <Eigen/Geometry>

using namespace std;

struct OptData
{
    std::vector<double> r1;
    std::vector<double> r2;
};

class Transform
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    typedef Eigen::Matrix<float, 6, 1> Vector6;
    typedef Eigen::Quaternionf Rotation;
    typedef Eigen::Vector3f Translation;
    typedef Eigen::Matrix<float, 4, 4> Matrix;
    Transform()
    {
        rotation_.setIdentity();
        translation_.setZero();
    }

    Transform(const Translation &translation, const Rotation &rotation)
        : translation_(translation), rotation_(rotation) {}

    static Transform exp(const Vector6 &vector)
    {
        constexpr float kEpsilon = 1e-8;
        const float norm = vector.tail<3>().norm();
        if (norm < kEpsilon)
        {
            return Transform(vector.head<3>(), Rotation::Identity());
        }
        else
        {
            return Transform(vector.head<3>(), Rotation(Eigen::AngleAxisf(
                                                   norm, vector.tail<3>() / norm)));
        }
    }

public:
    Rotation rotation_;
    Translation translation_;
};

class Radar
{
public:
    Radar() { std::cout << "Initial Radar Class" << std::endl; }
    void optimize(const std::vector<double> &lb, const std::vector<double> &ub,
                  OptData *opt_data, std::vector<double> *x);
    static double radarMinimizer(const std::vector<double> &x,
                                 std::vector<double> &grad, void *f_data);

public:
    int max_evals = 200;
    double xtol = 0.0001;
};

double Radar::radarMinimizer(const std::vector<double> &x,
                             std::vector<double> &grad, void *f_data)
{
    OptData *d = static_cast<OptData *>(f_data);

    Eigen::Matrix<double, 6, 1> vec;
    vec.setZero();

    for (size_t i = 0; i < 6; ++i)
    {
        vec[i] = x[i];
    }

    Transform T_o_l = Transform::exp(vec.cast<float>());

    std::cout << "Translation: " << T_o_l.translation_ << std::endl;
    std::cout << "Rotation : " << T_o_l.rotation_.w() << " " << T_o_l.rotation_.x() 
                << " " << T_o_l.rotation_.y()<< " " << T_o_l.rotation_.z()<<std::endl;
    double error = 0.0;

    return error;
}

void Radar::optimize(const std::vector<double> &lb, const std::vector<double> &ub,
                     OptData *opt_data, std::vector<double> *x)
{
    nlopt::opt opt;
    opt = nlopt::opt(nlopt::LN_BOBYQA, x->size());
    opt.set_lower_bounds(lb);
    opt.set_upper_bounds(ub);
    opt.set_maxeval(max_evals);
    opt.set_xtol_abs(xtol);
    opt.set_min_objective(radarMinimizer, opt_data);
    double minf;
    std::vector<double> grad;
    nlopt::result result = opt.optimize(*x, minf);

    if (result)
    {
        std::cout << "\n 目标函数最大值 " << minf << std::endl;
    }
}

int main(int argc, char **argv)
{
    google::InitGoogleLogging(argv[0]);
    google::SetLogFilenameExtension("log_");
    //会输出导致程序结束的信号,和google::InstallFailureWriter(&FatalMessageDump); 配合使用，可以在程序出现严重错误时将详细的错误信息打印出来
    // 通过GFLAGS来设置参数，更多选项可以在logging.cc里面查询
    // 日志等级分为INFO, WARNING, ERROR, FATAL,如果是FATAL级别这直接运行报错
    FLAGS_stderrthreshold = google::INFO; //INFO, WARNING, ERROR都输出，若为google::WARNING，则只输出WARNING, ERROR
    google::SetStderrLogging(google::GLOG_INFO);
    FLAGS_colorlogtostderr = true;          //log为彩色
    FLAGS_logbufsecs = 0;                   // Set log output speed(s)
    FLAGS_max_log_size = 1024;              // Set max log file size
    FLAGS_stop_logging_if_full_disk = true; // If disk is full
    //基本用法：INFO，WAINING，ERROR
    LOG(INFO) << "Hello GLOG";      // << " cookies";
    LOG(WARNING) << "warning test"; // 会输出一个Warning日志
    LOG(ERROR) << "error test";     // 会输出一个Error日志
    // LOG(FATAL) << "fatal";   // Logging a FATAL message terminates the program (after the message is logged)!
    chrono::steady_clock::time_point t1 = chrono::steady_clock::now();
    // process
    Radar test;
    OptData opt_data;
    std::vector<double> x(6, 0.0);
    x[0] = 0.0;
    x[1] = 0.0;
    x[2] = 0.0;
    x[3] = 0.0;
    x[4] = 0.0;
    x[5] = 0.0;

    double translation_range = 1.0;
    double angular_range = 0.5;

    std::vector<double> lb = {
        -translation_range, -translation_range,
        -translation_range, -angular_range,
        -angular_range, -angular_range};
    std::vector<double> ub = {
        translation_range, translation_range,
        translation_range, angular_range,
        angular_range, angular_range};

    for (size_t i = 0; i < 6; ++i)
    {
        lb[i] += x[i];
        ub[i] += x[i];
    }

    test.optimize(lb, ub, &opt_data, &x);
    chrono::steady_clock::time_point t2 = chrono::steady_clock::now();
    chrono::duration<double> time_used =
        chrono::duration_cast<chrono::duration<double>>(t2 - t1);
    std::cout << "solve time cost = " << time_used.count() << " seconds. " << endl;

    google::ShutdownGoogleLogging(); //关闭log服务
    return 0;
}
