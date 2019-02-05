
#include <cctype>
#include <algorithm>
#include <iterator>
#include <fstream>
#include <sstream>
#include <iomanip>
#include <string.h>
#include <dirent.h>
#include <regex>
#include <string>
#include <cstdio>
#include <boost/filesystem.hpp>

#ifdef _WIN32
#include <direct.h>
#include <conio.h>
#else
#include <unistd.h>
#include <signal.h>
#include <sys/select.h>
#include <sys/ioctl.h>
#include <termios.h>
#include <stropts.h>
#endif

#ifdef _WIN32
#include <process.h>
#else
#include <spawn.h>
#include <sys/types.h>
#include <sys/wait.h>
#include <sys/sysinfo.h>
#endif

#ifdef _MSC_VER
#include <tchar.h>
#include <windows.h>
#endif

#include <sys/stat.h>

#include "utils.h"
#include "log.h"

using airt::getExtension;
using airt::loadTextFile;
using std::string;

std::string airt::getExtension(const std::string &filename)
{
  string::size_type dot;

  dot = filename.find_last_of('.');

  if (dot == string::npos)
    return "";

  return filename.substr(dot);
}

bool airt::loadTextFile(const std::string &filename, std::vector<std::string> &content)
{
  std::ifstream file;

  content.clear();
  file.open(filename.c_str());
  if (!file)
    return false;

  while (!file.eof())
  {
    string st;
    getline(file, st);
    content.push_back(st);
  }
  file.close();
  return true;
}

// Devuelve la fecha y hora actuales
long long airt::getCurrentTime()
{
  long long t;
#ifdef _WIN32
  __time64_t long_time;

  // Get time as 64-bit integer.
  _time64(&long_time);
  t = long_time;
#else
  time_t long_time;

  time(&long_time);
  t = long_time;
#endif
  return t;
}

long long airt::getFileModificationTime(const std::string &pathname)
{
  struct stat st;
  int ret = stat(pathname.c_str(), &st);
  if (ret == -1)
  {
    Log::error("Error obteniendo información del fichero " + pathname);
  }
#ifndef __APPLE__
  return st.st_mtime;
#else
  return st.st_mtimespec.tv_sec;
#endif
}

std::string airt::getCurrentDateTimeString()
{

  char buffer[40];
#ifdef _WIN32
  struct tm newtime;
  __time64_t long_time;

  // Get time as 64-bit integer.
  _time64(&long_time);
  _localtime64_s(&newtime, &long_time);
  asctime_s(buffer, sizeof(buffer), &newtime);
#else
  time_t long_time;

  time(&long_time);
  ctime_r(&long_time, buffer);
#endif
  char *remove = buffer + strlen(buffer) - 1;
  // remove the \n at the end
  while (remove >= buffer && *remove < ' ')
  {
    *remove = 0;
    remove--;
  }
  return std::string(buffer);
}

std::string airt::getTimeStamp()
{

  struct tm newtime;
#ifdef _WIN32
  __time64_t long_time;

  // Get time as 64-bit integer.
  _time64(&long_time);
  _localtime64_s(&newtime, &long_time);

#else
  time_t long_time;

  time(&long_time);
  localtime_r(&long_time, &newtime);
#endif
  std::ostringstream os;
  os << newtime.tm_year + 1900 << std::setw(2) << std::setfill('0')
     << newtime.tm_mon + 1 << std::setw(2) << newtime.tm_mday;
  os << "-" << std::setw(2) << std::setfill('0') << newtime.tm_hour
     << std::setw(2) << newtime.tm_min << std::setw(2) << newtime.tm_sec;
  os << std::setw(3) << std::setfill('0')
     << (((unsigned long)clock()) * 1000 / CLOCKS_PER_SEC) % 1000;
  return os.str();
}

std::string airt::hexString(unsigned long n)
{
  std::ostringstream os;
  os << "0x" << std::hex << n;
  return os.str();
}

#ifdef _WIN32
bool airt::keyPressed()
{
  return (_kbhit() != 0);
}

int airt::getChar()
{
  return _getch();
}
#else

bool airt::keyPressed()
{
  static const int STDIN = 0;
  static bool initialized = false;

  if (!initialized)
  {
    // Use termios to turn off line buffering
    termios term;
    tcgetattr(STDIN, &term);
    term.c_lflag &= ~ICANON;
    tcsetattr(STDIN, TCSANOW, &term);
    setbuf(stdin, NULL);
    initialized = true;
  }

  int bytesWaiting;
  ioctl(STDIN, FIONREAD, &bytesWaiting);
  return (bytesWaiting != 0);
}

static struct termios oldt, newt;

/* Initialize new terminal i/o settings */
void initTermios(int echo)
{
  tcgetattr(0, &oldt);     /* grab old terminal i/o settings */
  newt = oldt;             /* make new settings same as old settings */
  newt.c_lflag &= ~ICANON; /* disable buffered i/o */
  if (echo)
  {
    newt.c_lflag |= ECHO; /* set echo mode */
  }
  else
  {
    newt.c_lflag &= ~ECHO; /* set no echo mode */
  }
  tcsetattr(0, TCSANOW, &newt); /* use these new terminal i/o settings now */
}

/* Restore old terminal i/o settings */
void resetTermios(void)
{
  tcsetattr(0, TCSANOW, &oldt);
}

/* Read 1 character - echo defines echo mode */
char getch_(int echo)
{
  char ch;
  initTermios(echo);
  ch = getchar();
  resetTermios();
  return ch;
}

int airt::getChar()
{
  return getch_(0);
}
#endif

std::string airt::getFilenameFromPath(std::string filepath)
{
  std::string::size_type slash1, slash2, slash;

  slash1 = filepath.find_last_of('\\');
  slash2 = filepath.find_last_of('/');

  if (slash1 == std::string::npos)
    slash = slash2;
  else if (slash2 == std::string::npos)
    slash = slash1;
  else if (slash1 > slash2)
    slash = slash1;
  else
    slash = slash2;

  if (slash == std::string::npos)
    return filepath;

  return filepath.substr(slash + 1);
}

std::string airt::getDirectory(std::string filepath)
{
  std::string::size_type slash1, slash2, slash;

  slash1 = filepath.find_last_of('\\');
  slash2 = filepath.find_last_of('/');

  if (slash1 == std::string::npos)
    slash = slash2;
  else if (slash2 == std::string::npos)
    slash = slash1;
  else if (slash1 > slash2)
    slash = slash1;
  else
    slash = slash2;

  if (slash == std::string::npos)
    return "";

  return filepath.substr(0, slash + 1);
}

bool airt::fileExists(const std::string &name)
{
  std::ifstream f(name.c_str());
  return f.good();
}

std::string airt::to_lower(const std::string &s)
{
  std::string data(s);
  std::transform(data.begin(), data.end(), data.begin(), ::tolower);
  return data;
}

/**
Devuelve si la cadena s empieza por b
*/
bool airt::starts_with(const std::string &s, const std::string &b)
{
  return s.compare(0, b.length(), b) == 0;
}

/**
Devuelve si la cadena s acaba por b
*/
bool airt::ends_with(const std::string &s, const std::string &b)
{
  if (s.length() < b.length())
    return 0;
  return s.compare(s.length() - b.length(), b.length(), b) == 0;
}

// Devuelve <posicion del $, longitud del nombre de la variable
std::pair<size_t, size_t> searchVariable(const std::string &line)
{
  std::pair<size_t, size_t> res;

  res.first = line.find('$');
  if (res.first == std::string::npos)
    return res;
  size_t i;
  for (i = res.first + 1; i < line.size(); i++)
    if (!isalnum(line[i]) && line[i] != '_')
      break;
  res.second = i - res.first;
  return res;
}

// trim from start (in place)
void airt::ltrim(std::string &s)
{
  s.erase(s.begin(), std::find_if(s.begin(), s.end(), [](unsigned char ch) {
            return !std::isspace(ch);
          }));
}

// trim from end (in place)
void airt::rtrim(std::string &s)
{
  s.erase(std::find_if(s.rbegin(), s.rend(), [](unsigned char ch) {
            return !std::isspace(ch);
          })
              .base(),
          s.end());
}

// trim from both ends (in place)
void airt::trim(std::string &s)
{
  ltrim(s);
  rtrim(s);
}

#define BYTES_PER_LINE 16

void hexLine(unsigned char *buffer, unsigned int size, unsigned long offset, std::ostream &output)
{
  output << std::setw(7) << std::setbase(10) << offset;
  output << std::hex;
  for (unsigned int i = 0; i < size; i++)
  {
    output << " " << std::setw(2) << (int)buffer[i];
  }
  output << std::endl;
}

void airt::hexdump(unsigned char *buffer, unsigned int size, std::ostream &output)
{
  unsigned long offset = 0;
  while (size >= BYTES_PER_LINE)
  {
    hexLine(buffer, BYTES_PER_LINE, offset, output);
    buffer += BYTES_PER_LINE;
    offset += BYTES_PER_LINE;
    size -= BYTES_PER_LINE;
  }
  if (size > 0)
  {
    hexLine(buffer, size, offset, output);
  }
}

//
//// http://stackoverflow.com/questions/3300419/file-name-matching-with-wildcard
//
//void EscapeRegex(string &regex);
//
//bool MatchTextWithWildcards(const string &text, string wildcardPattern, bool caseSensitive)
//{
//  // Escape all regex special chars
//  EscapeRegex(wildcardPattern);
//
//  // Convert chars '*?' back to their regex equivalents
//  boost::replace_all(wildcardPattern, "\\?", string("."));
//  boost::replace_all(wildcardPattern, "\\*", ".*");
//
//  auto flags = std::regex::basic;
//  if (!caseSensitive) flags |= std::regex::icase;
//
//  std::regex pattern(wildcardPattern, flags);
//
//  return regex_match(text, pattern);
//}
//
//void EscapeRegex(string &regex)
//{
//  boost::replace_all(regex, "\\", "\\\\");
//  boost::replace_all(regex, "^", "\\^");
//  boost::replace_all(regex, ".", "\\.");
//  boost::replace_all(regex, "$", "\\$");
//  boost::replace_all(regex, "|", "\\|");
//  boost::replace_all(regex, "(", "\\(");
//  boost::replace_all(regex, ")", "\\)");
//  boost::replace_all(regex, "[", "\\[");
//  boost::replace_all(regex, "]", "\\]");
//  boost::replace_all(regex, "*", "\\*");
//  boost::replace_all(regex, "+", "\\+");
//  boost::replace_all(regex, "?", "\\?");
//  boost::replace_all(regex, "/", "\\/");
//}
//
//std::vector<std::string> airt::listFiles(const std::string &path, bool recursive) {
//  return listFiles(path, recursive, std::vector<std::string>());
//}
//
// std::vector<std::string> airt::listFiles(const std::string &path, bool recursive, const std::vector<std::string> &patterns)
// {
//   DIR *dir;
//   struct dirent *ent;
//   std::vector<std::string> results;

//   std::string wpath = path;
//   if (path.empty())
//     wpath = "./";
//   else if (path.back() != '/' && path.back() != '\\')
//     wpath += "/";

//   /* Open directory stream */
//   dir = opendir(wpath.c_str());
//   if (dir != NULL)
//   {

//     /* Print all files and directories within the directory */
//     while ((ent = readdir(dir)) != NULL)
//     {
//       switch (ent->d_type)
//       {
//       case DT_REG:
//         if (patterns.empty())
//           results.push_back(wpath + ent->d_name);
//         else
//         {
//           for (auto p : patterns)
//           {
//             if (MatchTextWithWildcards(ent->d_name, p, getPlatform() != Platform::WIN))
//             {
//               results.push_back(wpath + ent->d_name);
//               break;
//             }
//           }
//         }
//         break;

//       case DT_DIR:
//         if (std::string(".") != ent->d_name && std::string("..") != ent->d_name && recursive)
//         {
//           auto tmp = listFiles(wpath + ent->d_name, true, patterns);
//           std::move(tmp.begin(), tmp.end(), std::back_inserter(results));
//         }
//         break;
//       }
//     }
//     closedir(dir);
//   }
//   else
//   {
//     /* Could not open directory */
//     Log::error("No se puede abrir el directorio " + wpath);
//   }
//   return results;
// }

std::vector<std::string> airt::listFiles(const std::string &path, bool recursive, std::function<bool(const std::string &)> matches)
{
  DIR *dir;
  std::vector<std::string> results;

  std::string wpath = path;
  if (path.empty())
    wpath = "./";
  else if (path.back() != '/' && path.back() != '\\')
    wpath += "/";

  /* Open directory stream */
  dir = opendir(wpath.c_str());
  if (dir != NULL)
  {
    struct dirent *ent;

    /* Print all files and directories within the directory */
    while ((ent = readdir(dir)) != NULL)
    {
      switch (ent->d_type)
      {
      case DT_REG:
      case DT_LNK:
      {
        auto filename = wpath + ent->d_name;
        if (matches(filename))
          results.push_back(wpath + ent->d_name);
        break;
      }
      case DT_DIR:
        if (std::string(".") != ent->d_name && std::string("..") != ent->d_name && recursive)
        {
          auto tmp = listFiles(wpath + ent->d_name, true, matches);
          std::move(tmp.begin(), tmp.end(), std::back_inserter(results));
        }
        break;
      }
    }
    closedir(dir);
  }
  else
  {
    /* Could not open directory */
    Log::error("No se puede abrir el directorio " + wpath);
  }
  return results;
}

boost::optional<uint64_t> airt::readIntFromFile(const std::string &filename)
{
  std::ifstream f(filename);

  if (!f.is_open())
  {
    return boost::none;
  }
  uint64_t res;
  f >> res;
  if (f.bad())
    return boost::none;
  return boost::optional<uint64_t>(res);
}

typedef unsigned value_type;

template <typename Iterator>
size_t get_length(Iterator p)
{
  unsigned char c = static_cast<unsigned char>(*p);
  if (c < 0x80)
    return 1;
  else if (!(c & 0x20))
    return 2;
  else if (!(c & 0x10))
    return 3;
  else if (!(c & 0x08))
    return 4;
  else if (!(c & 0x04))
    return 5;
  else
    return 6;
}

template <typename Iterator>
value_type get_value(Iterator p)
{
  size_t len = get_length(p);

  if (len == 1)
    return *p;

  value_type res = static_cast<unsigned char>(
                       *p & (0xff >> (len + 1)))
                   << ((len - 1) * 6);

  for (--len; len; --len)
    res |= (static_cast<unsigned char>(*(++p)) - 0x80) << ((len - 1) * 6);

  return res;
}

std::string airt::utf8ToLatin1(std::string utf8)
{
  std::string s_latin1;
  bool outOfBounds = false;
  for (std::string::iterator p = utf8.begin(); p != utf8.end(); ++p)
  {
    value_type value = get_value<std::string::iterator &>(p);
    if (value > 0xff)
    {
      if (!outOfBounds)
      {
        Log::error("Error conviertiendo de utf8 a latin1: '" + utf8 + "'. Recuerda guardar tu código fuente en utf8");
        outOfBounds = true;
      }
    }
    else
      s_latin1 += static_cast<std::string::value_type>(value);
  }
  return s_latin1;
}

bool airt::deleteFile(const std::string &filename)
{
#ifndef _WIN32
#define _unlink unlink
#endif
  return _unlink(filename.c_str()) == 0;
}

std::string airt::getCurrentWorkingDir()
{
#ifndef _WIN32
#define _getcwd getcwd
#endif
  char path[1024];

  if (_getcwd(path, sizeof(path)) == nullptr)
    Log::error("Error al obtener el directorio actual");

  std::string res = path;
  if (!ends_with(res, "/") && !ends_with(res, "\\"))
    res += "/";
  return res;
}

void airt::changeCurrentDir(const std::string &dir)
{
#ifndef _WIN32
#define _chdir chdir
#endif
  if (_chdir(dir.c_str()))
    Log::error("Error al cambiar el directorio actual a " + dir);
}

std::string airt::removeExtension(const std::string &filename)
{
  string::size_type dot;

  dot = filename.find_last_of('.');

  if (dot == filename.npos)
    return filename;

  if (filename.find('/', dot) != filename.npos || filename.find('\\', dot) != filename.npos)
    return filename;

  return filename.substr(0, dot);
}

bool airt::containsIgnoreCase(const std::string &dondeBuscar, const std::string &queBuscar)
{
  auto it = std::search(
      dondeBuscar.begin(), dondeBuscar.end(),
      queBuscar.begin(), queBuscar.end(),
      [](char ch1, char ch2) { return std::toupper(ch1) == std::toupper(ch2); });
  return (it != dondeBuscar.end());
}

void airt::renameOrMove(const std::string &from, const std::string &to)
{
  std::rename(from.c_str(), to.c_str());
}

size_t airt::repetitionsInFile(const std::string &filename, const std::string &str)
{
  std::ifstream infile(filename);

  std::string line;
  size_t occurrences = 0;
  while (std::getline(infile, line))
  {
    std::string::size_type start = 0;
    while ((start = line.find(str, start)) != std::string::npos)
    {
      ++occurrences;
      start += str.length();
    }
  }
  return occurrences;
}

#ifndef _WIN32
bool airt::fileSize(const std::string &filename, uint64_t &size)
{
  struct stat64 sb;
  int rc = stat64(filename.c_str(), &sb);
  if (rc == 0)
  {
    size = sb.st_size;
    return true;
  }
  return false;
}
#endif

bool airt::to_unsignedlong(const std::string &s, unsigned long &ul)
{
  try
  {
    size_t processed;
    unsigned long temp = stoul(s, &processed, 10);
    if (processed < s.size())
      return false;
    else
    {
      ul = temp;
      return true;
    }
  }
  catch (...)
  {
    return false;
  }
}

#define PROC_DIRECTORY "/proc/"

bool isNumber(const char *str)
{
  while (*str)
  {
    if (!std::isdigit(*str))
      return false;
    str++;
  }
  return true;
}

std::string extractExecutableName(const std::string &cmdline)
{
  size_t firstNUL = cmdline.find('\0');
  if (firstNUL == string::npos)
  {
    // No arguments
    size_t lastSlash = cmdline.find_last_of('/');
    if (lastSlash == string::npos)
      return cmdline;
    else
      return cmdline.substr(lastSlash + 1);
  }
  else
  {
    // Has arguments
    size_t lastSlash = cmdline.rfind('/', firstNUL - 1);
    if (lastSlash == string::npos)
    {
      // No directory: <program>NUL<param1>NUL<param2>...
      return cmdline.substr(0, firstNUL);
    }
    else
    {
      // directory and args: <path>/<program><NUL>param1<NUL>param2...
      return cmdline.substr(lastSlash + 1, firstNUL - lastSlash - 1);
    }
  }
}

#ifndef _WIN32

bool airt::isPIDRunning(const pid_t pid)
{
  return kill(pid, 0) == 0;
}

pid_t airt::getProcessPID(const std::string &processName)
{
  struct dirent *de_DirEntity = NULL;

  DIR *dir_proc = opendir(PROC_DIRECTORY);
  if (dir_proc == NULL)
  {
    Log::error("Couldn't open the " PROC_DIRECTORY " directory");
    return (pid_t)-2;
  }

  // Loop while not NULL
  while ((de_DirEntity = readdir(dir_proc)))
  {
    if (de_DirEntity->d_type == DT_DIR)
    {
      if (isNumber(de_DirEntity->d_name))
      {
        std::string cmdline(PROC_DIRECTORY);
        cmdline += de_DirEntity->d_name;
        cmdline += "/cmdline";
        std::ifstream cmdlineFile(cmdline);
        if (!cmdlineFile)
        {
          Log::error("Error opening {}", cmdline);
        }
        else
        {
          std::string processNameInProc;
          cmdlineFile >> processNameInProc;
          cmdlineFile.close();
          const std::string executable = extractExecutableName(processNameInProc);
          if (executable == processName)
          {
            auto pid = (pid_t)atoi(de_DirEntity->d_name);
            closedir(dir_proc);
            return pid;
          }
        }
      }
    }
  }
  closedir(dir_proc);
  return -1; // Not found
}

bool airt::waitProcessQuitOrKillIt(pid_t pid, unsigned int waitMS, unsigned int timeoutKillms)
{
  if (pid < 0)
    return true;
  if (!waitUntil([pid]() { return !isPIDRunning(pid); }, waitMS))
  {
    // Kill it
    kill(pid, SIGKILL);
    if (!waitUntil([pid]() { return !isPIDRunning(pid); }, timeoutKillms))
    {
      return false;
    }
  }
  return true;
}

int airt::launchIndependentProcess(std::function<void(void)> theFunction)
{
  int pid;
  int Stat;

  pid = fork();
  if (pid < 0)
  {
    return -1;
  }
  if (pid == 0)
  {           // CHILD
    setsid(); // Make this process the session leader of a new session
    pid = fork();
    if (pid < 0)
    {
      return -2;
    }
    if (pid == 0)
    { // GRANDCHILD
      theFunction();
      exit(1);
    }
    exit(0); // SUCCESS (This child is reaped below with waitpid())
  }

  // Reap the child, leaving the grandchild to be inherited by init
  waitpid(pid, &Stat, 0);
  if (WIFEXITED(Stat) && (WEXITSTATUS(Stat) == 0))
    return 0;
  else
    return -3;
}

void airt::demonize(const std::string &currentDir)
{
  pid_t pid, sid;

  /* Fork off the parent process */
  pid = fork();
  if (pid < 0)
  {
    exit(EXIT_FAILURE);
  }
  /* If we got a good PID, then
           we can exit the parent process. */
  if (pid > 0)
  {
    exit(EXIT_SUCCESS);
  }

  /* Change the file mode mask */
  umask(0);

  /* Open any logs here */

  /* Create a new SID for the child process */
  sid = setsid();
  if (sid < 0)
  {
    /* Log any failures here */
    exit(EXIT_FAILURE);
  }

  /* Change the current working directory */

  if (!currentDir.empty() && (chdir(currentDir.c_str())) < 0)
  {
    /* Log any failures here */
    exit(EXIT_FAILURE);
  }

  /* Close out the standard file descriptors */
  close(STDIN_FILENO);
  close(STDOUT_FILENO);
  close(STDERR_FILENO);
}
#endif

std::vector<std::string> airt::splitBySpaces(const std::string &s)
{
  bool inQuote = false;
  std::string temp;
  std::vector<std::string> result;
  std::string trimmed(s);
  trim(trimmed);

  for (auto c : trimmed)
  {
    if (inQuote)
    {
      if (c == '\"')
        inQuote = false;
      else
        temp += c;
    }
    else
    {
      if (c == '\"')
        inQuote = true;
      else
      {
        if (std::isspace(c))
        {
          result.push_back(temp);
          temp.clear();
        }
        else
          temp += c;
      }
    }
  }
  if (!temp.empty())
    result.push_back(temp);
  return result;
}

bool airt::waitUntil(std::function<bool()> condition, unsigned long timeoutMs, unsigned long divisions)
{
  unsigned long elapsed = 0;
  unsigned long period = timeoutMs / divisions;
  if (period == 0)
    period = timeoutMs;

  while (elapsed < timeoutMs && !condition())
  {
    std::this_thread::sleep_for(std::chrono::milliseconds(period));
    elapsed += period;
  }
  return elapsed < timeoutMs;
}

bool airt::diskSpace(const std::string &path, uintmax_t &capacity, uintmax_t &available)
{
  auto info = boost::filesystem::space(path);
  capacity = info.capacity;
  available = info.available;
  return true;
}

#ifndef _WIN32
#define LOADS_SCALE 65536.0f

void airt::systemInfo(SystemInfo &si)
{

  struct sysinfo memInfo;
  sysinfo(&memInfo);

  si.uptime = memInfo.uptime;
  si.oneminuteload = memInfo.loads[0] / LOADS_SCALE;
  si.fiveminuteload = memInfo.loads[1] / LOADS_SCALE;
  si.fifteenminuteload = memInfo.loads[2] / LOADS_SCALE;
  si.freeram = memInfo.freeram * memInfo.mem_unit;
  si.totalram = memInfo.totalram * memInfo.mem_unit;
  si.freeswap = memInfo.freeswap * memInfo.mem_unit;
  si.totalswap = memInfo.totalswap * memInfo.mem_unit;
  si.procs = memInfo.procs;
}
#endif

std::string airt::formatBinaryBuffer(const void *buffer, size_t size, size_t max)
{
  std::ostringstream os;

  os << "[" << std::hex;

#undef min

  size_t realsize = std::min(size, max);
  if (realsize > 0)
    os << "0x" << static_cast<unsigned int>(static_cast<const uint8_t *>(buffer)[0]);
  for (size_t i = 1; i < realsize; i++)
  {
    os << " 0x" << static_cast<unsigned int>(static_cast<const uint8_t *>(buffer)[i]);
  }
  if (realsize < size)
  {
    os << " ... " << std::dec << size - realsize << " more";
  }
  os << "]";
  return os.str();
}

uint64_t airt::msSinceEpoch()
{
  auto now = std::chrono::steady_clock::now();
  return std::chrono::duration_cast<std::chrono::milliseconds>(now.time_since_epoch()).count();
}
