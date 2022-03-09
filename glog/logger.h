
//LOG()宏定义如下：
#define LOG(log_rank) \
    Logger(log_rank).start(log_rank, __LINE__, __FUNCTION__)

class Logger
{
    // 指定不同级别日志的输出路径
    friend void initLogger(const std::string &info_log_filename,
                           const std::string &warn_log_filename,
                           const std::string &erro_log_filename);

public:
    //构造函数： 打印级别
    Logger(log_rank_t log_rank) : m_log_rank(log_rank){};

    ~Logger();
    ///
    /// \brief 写入日志信息之前先写入的源代码文件名, 行号, 函数名
    /// \param log_rank 日志的等级
    /// \param line 日志发生的行号
    /// \param function 日志发生的函数
    static std::ostream &start(log_rank_t log_rank,
                               const int line,
                               const std::string &function);

private:
    ///
    /// \brief 根据等级获取相应的日志输出流
    ///
    static std::ostream &getStream(log_rank_t log_rank);

    static std::ofstream m_info_log_file;  ///< 信息日子的输出流
    static std::ofstream m_warn_log_file;  ///< 警告信息的输出流
    static std::ofstream m_error_log_file; ///< 错误信息的输出流
    log_rank_t m_log_rank;                 ///< 日志的信息的等级
};

//Logger类的start函数实现如下：
std::ostream &Logger::start(log_rank_t log_rank,
                            const int line,
                            const std::string &function)
{
    time_t tm;
    time(&tm);
    char time_string[128];
    ctime_r(&tm, time_string);
    return getStream(log_rank) << time_string
                               << "function (" << function << ")"
                               << "line " << line << " "
                               << std::flush;
}

#include <iostream>
// #include "Logger.h"
using namespace std;
int main(int argc, char *argv)
{
    LOG(INFO) << "helloworld" << "and Nice!";
    return 0;
}
