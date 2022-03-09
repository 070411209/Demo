#ifndef _LOGGER_H_
#define _LOGGER_H_

#include <fstream>
#include <iostream>
#include <time.h>

#pragma warning(disable : 4996)

class Logger {
 public:
  enum log_level { debug, info, warning, error };         // 日志等级
  enum log_target { file, terminal, file_and_terminal };  // 日志输出目标
 public:
  Logger();
  Logger(log_target target, log_level level, const std::string& path);
  ~Logger();

  void DEBUG(const std::string& text);
  void INFO(const std::string& text);
  void WARNING(const std::string& text);
  void ERRORS(const std::string& text);

 private:
  std::ofstream m_outfile;  // 将日志输出到文件的流对象
  log_target m_target;      // 日志输出目标
  std::string m_path;       // 日志文件路径
  log_level m_level;        // 日志等级
  void output(const std::string& text, log_level act_level);  // 输出行为
};

#endif  //_LOGGER_H_