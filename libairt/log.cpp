#include "include/log.h"

#include <vector>
#include <iostream>

using airt::Log;

std::string Log::filename = "log.txt";
bool Log::echoToConsole = 
#ifdef _DEBUG
true
#else
false
#endif
;

static const char *to_string(const spdlog::level::level_enum l) {
  return spdlog::level::level_names[static_cast<int>(l)];
}

void Log::set_level(LogLevel level) { 
  spdlog::level::level_enum spdlevel = static_cast<spdlog::level::level_enum>(level);
  instance().logger->set_level(spdlevel);
  Log::log(std::string("Log level changed to ") + to_string(spdlevel));
}

Log::LogLevel Log::get_level() {
  return static_cast<LogLevel>(instance().logger->level());
}

Log::Log() {
  try
  {
    configureLog();
  }
  catch (const spdlog::spdlog_ex& ex)
  {
    std::cerr << "Log initialization failed: " << ex.what() << std::endl;
  }
  
  logger->info("Logging starts {}. Log level: {} "
  #ifdef NDEBUG
  "NDEBUG"
  #else
  "DEBUG"
  #endif
   , 
  #ifdef _WIN32
    _getpid()
  #else
    getpid()
  #endif
    , to_string(logger->level()) );
}

Log::~Log() {
  Log::info("Logging ends {}", 
#ifdef _WIN32
    _getpid()
#else
    getpid()
#endif
  );
  logger->flush();
}

void Log::configureLog(size_t max_bytes, size_t num_files) {
  std::vector<spdlog::sink_ptr> sinks;

  if (Log::echoToConsole)
    sinks.push_back(std::make_shared<spdlog::sinks::stdout_sink_st>());
  sinks.push_back(std::make_shared<spdlog::sinks::rotating_file_sink_mt>(filename, max_bytes, num_files));
  logger = std::make_shared<spdlog::logger>("theLog", begin(sinks), end(sinks));
}
