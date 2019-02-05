#pragma once

#include <memory>
#ifdef _WIN32
#pragma warning(push, 0)
#endif
#include <spdlog/spdlog.h>
#ifdef _WIN32
#pragma warning(pop)
#endif

namespace spdlog
{
class logger;
};

namespace airt
{
/**
  \class Log

  Logging class for the projects. It is used to log all relevant events ocurred in runtime.
  By default, it will log to a file called logairt.txt. When compiled in debug, it will also
  send the messages to the console.

  Use example:

\code
  #include <log.h>

  using airt::Log;

  ...

  // Simple message
  Log::info("Process started");

  // Formatted message (each '{}' is replaced by the corresponding parameter)
  Log::error("The point cloud has {} points, but the maximum is {}", pc.size(), max_size);
\endcode
  */

class Log
{
public:
  //@{
  /**
      Logs a simple message. The name of the method specifies the log level
      \param m the message
    */
  template <typename T>
  static void trace(const T &m) { instance().logger->trace(m); }
  template <typename T>
  static void debug(const T &m) { instance().logger->debug(m); }
  template <typename T>
  static void info(const T &m) { instance().logger->info(m); }
  template <typename T>
  static void warn(const T &m) { instance().logger->warn(m); }
  template <typename T>
  static void error(const T &m) { instance().logger->error(m); }
  template <typename T>
  static void critical(const T &m) { instance().logger->critical(m); }
  //@}

  //@{
  /**
    Logs a formatted message. The name of the method specifies the log level
    \param fmt the format of the message (a string with '{}' to specify where to insert the 
    arguments
    \param a1 the first argument to insert into the formatted message
    \param args the rest of the arguments
    */
  template <typename Arg1, typename... Args>
  static void trace(const char *fmt, const Arg1 &a1, const Args &... args) { instance().logger->trace(fmt, a1, args...); };
  template <typename Arg1, typename... Args>
  static void debug(const char *fmt, const Arg1 &a1, const Args &... args) { instance().logger->debug(fmt, a1, args...); };
  template <typename Arg1, typename... Args>
  static void info(const char *fmt, const Arg1 &a1, const Args &... args) { instance().logger->info(fmt, a1, args...); };
  template <typename Arg1, typename... Args>
  static void warn(const char *fmt, const Arg1 &a1, const Args &... args) { instance().logger->warn(fmt, a1, args...); };
  template <typename Arg1, typename... Args>
  static void error(const char *fmt, const Arg1 &a1, const Args &... args) { instance().logger->error(fmt, a1, args...); };
  template <typename Arg1, typename... Args>
  static void critical(const char *fmt, const Arg1 &a1, const Args &... args) { instance().logger->critical(fmt, a1, args...); };
  //@}

  /**
    \enum LogLevel
    Specifies a logging level, from the least important (Trace) to the most important (Critical)
    */
  enum LogLevel
  {
    Trace = 0,
    Debug,
    Info,
    Warning,
    Error,
    Critical
  };

  /**
    Sets the logging level.
    \param level only messages with this level or more important will be logged (the rest will be discarded)
    */
  static void set_level(LogLevel level);

  /**
    \returns the current logging level.
    */
  static LogLevel get_level();

  /**
 * Sets the file to write the log to.
 * 
 * \warning Call this method before logging anything. By default, it creates the file log.txt
 */
  static void setFile(const std::string &filename) { Log::filename = filename; }

  /**
  Echoes the logging messages also to the console
  \warning Call this method before logging anything. By default, it only outputs to console in DEBUG
  */
  static void setOutputToConsole(bool console) { Log::echoToConsole = console; }
protected:
  /**
  \returns the only instance of the logging class
  */
  static Log &instance()
  {
    static Log theLog;
    return theLog;
  }

  Log(Log const &) = delete;
  Log(Log &&) = delete;
  Log &operator=(Log const &) = delete;
  Log &operator=(Log &&) = delete;

  template <typename T>
  static void log(const T &m) { instance().logger->log(instance().logger->level(), m); };

  template <typename Arg1, typename... Args>
  static void log(const char *fmt, const Arg1 &a1, const Args &... args)
  {
    instance().logger->log(instance().logger->level(), fmt, a1, args...);
  };

  /** 
	The variable that holds the spdlog logger object.
	*/
  std::shared_ptr<spdlog::logger> logger;
  Log();
  ~Log();

  void configureLog(size_t max_bytes = 1024 * 1024 * 8, size_t num_files = 4);

  static std::string filename;
  static bool echoToConsole;
  
#ifdef TESTING
#include "../../external-libs/googletest/include/gtest/gtest_prod.h"
  FRIEND_TEST(TestingLogClass, MultiTester);
  FRIEND_TEST(TestingLogClass, UnconditionalLog);
  FRIEND_TEST(AeroToolsFCSMessageProcessor, LogAllFCSCommands);
#endif
};
};
