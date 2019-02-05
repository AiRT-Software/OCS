#pragma once

#include <vector>
#include <map>
#include <string>
#include <time.h>
#include <chrono>
#include <functional>
#include <ostream>
#ifdef _WIN32
#include <boost/optional/optional.hpp>
#else
#include <boost/optional.hpp>
#endif

#include <iomanip>
#include <sstream> 

namespace airt
{
/////
// FUNCIONES DE AYUDA SOBRE FICHEROS
/////

/**
  Comprueba que el fichero existe y se puede abrir para lectura
  \param filename Ruta de un fichero
  */
bool fileExists(const std::string &filename);

#ifndef _WIN32
/**
  \param filename file to compute its size
  \param size variable to store the file size
  \return true if the file exists and we could compute its size
*/
bool fileSize(const std::string &filename, uint64_t &size);
#endif

// Devuelve la extensión del fichero, con el punto:
// getExtension("pepe.TxT") -> ".TxT"
std::string getExtension(const std::string &filename);

//! El nombre de fichero indicado, sin extensión
std::string removeExtension(const std::string &filename);

bool loadTextFile(const std::string &filename, std::vector<std::string> &content);

// Devuelve la fecha y hora de modificación del fichero (en segundos desde 1/1/1970)
long long getFileModificationTime(const std::string &pathname);

/**
  Dada una ruta completa de fichero, devuelve el directorio (la parte de la cadena
  antes de la última barra)
  \param filepath Ruta completa de un fichero
  */
std::string getDirectory(std::string filepath);

/**
  Dada una ruta completa de fichero, devuelve el nombre de fichero (la parte de la
  cadena detrás de la última barra)
  \param filepath Ruta completa de un fichero
  */
std::string getFilenameFromPath(std::string filepath);

/**
  Devuelve la lista de ficheros que se encuentran dentro del directorio indicado.
  \param path Directorio
  \param recursive Si es true, hará una búsqueda recursiva
  \return la lista de nombres de fichero
  */
std::vector<std::string> listFiles(const std::string &path, bool recursive);

/**
  Devuelve la lista de ficheros que se encuentran dentro del directorio indicado y
  cuyo nombre coincide con alguno de los patrones indicados.
  \param path Directorio
  \param recursive Si es true, hará una búsqueda recursiva
  \param patterns lista de patrones a buscar (p.e., *.jpg, imagen??.bmp, log*.txt, etc.)
  \return la lista de nombres de fichero
  */
std::vector<std::string> listFiles(const std::string &path, bool recursive, const std::vector<std::string> &patterns);

/**
  Devuelve la lista de ficheros que se encuentran dentro del directorio indicado y
  cuyo nombre hace que la función matches devuelva true.
  \param path Directorio
  \param recursive Si es true, hará una búsqueda recursiva
  \param una función que recibe el nombre de fichero, y devuelve true si tiene que estar en la lista de resultados
  \return la lista de nombres de fichero
  */
std::vector<std::string> listFiles(const std::string &path, bool recursive, std::function<bool(const std::string &)> matches);

/**
  \return El directorio actual (acaba con la /)
  */
std::string getCurrentWorkingDir();

/**
  Borra el fichero indicado
  \return true si se ha podido borrar el fichero
  */
bool deleteFile(const std::string &filename);

/**
  Cambia el directorio actual
  */
void changeCurrentDir(const std::string &dir);

/**
  Cambia el nombre y/o mueve un fichero o directorio
  */
void renameOrMove(const std::string &from, const std::string &to);

/**
  \return el número de apariciones de la cadena indicada
  */
size_t repetitionsInFile(const std::string &filename, const std::string &str);

/**
 * Reads an integer from a file. It is used mainly to parse system proc files (i.e. 
 * /sys/class/net/wlp58s0/statistics/tx_bytes)
 */
boost::optional<uint64_t> readIntFromFile(const std::string &filename);

///////
// FUNCIONES DE AYUDA SOBRE FECHAS
///////

// Devuelve la fecha y hora actuales en el formato Mon Oct 7 10:30:23 2013
std::string getCurrentDateTimeString();
// Devuelve la fecha y hora actuales en el formato 20131023-172213000
// (yyyymmdd-hhmmssnnn), donde n son milisegundos
std::string getTimeStamp();

// Devuelve la fecha y hora actuales (en segundos desde 1/1/1970)
long long getCurrentTime();

class StopWatch
{
public:
  StopWatch() { start = std::chrono::steady_clock::now(); }
  double elapsed()
  {
    std::chrono::duration<double> diff = std::chrono::steady_clock::now() - start;
    return diff.count();
  }

private:
  std::chrono::steady_clock::time_point start;
};

/**
 * \return the number of milliseconds since the epoch
 */
uint64_t msSinceEpoch();

////////
// FUNCIONES DE AYUDA SOBRE EL SISTEMA OPERATIVO
////////

/**
    \return el sistema operativo para el que se compiló la librería
    */
enum class Platform
{
  UNKNOWN,
  WIN,
  LINUX,
  MAC
};
constexpr Platform getPlatform()
{
#ifdef _WIN32
  return Platform::WIN;
#elif __APPLE__
  return Platform::MAC;
#elif __linux__
  return Platform::LINUX;
#else
  return Platform::UNKNOWN;
#endif
}

/*
Prepares the current process to become a Linux daemon

http://www.netzmafia.de/skripten/unix/linux-daemon-howto.html

\param currentDir this directory will be the current working directory
*/
void demonize(const std::string &currentDir = "");

#ifndef _WIN32
pid_t getProcessPID(const std::string &processName);
bool isPIDRunning(const pid_t pid);


/**
	Wait for a process to die naturally, or kill it after a waiting time.
	\param pid process id to kill
	\param waitMS wait this number of miliseconds before killing the process
	\param timeoutKillms after sending the kill signal, wait this number of ms for the process to disappear
	\return true if the process has disappeared, false otherwise
  */
bool waitProcessQuitOrKillIt(pid_t pid, unsigned int waitMS, unsigned int timeoutKillms = 2000);

/**
 *  Executes a function in a completely independent process 
 *  \param theFunction the function to invoke in the separated process (usually a lambda)
 *  \return 0 on success, something different if an error ocurred
 */
int launchIndependentProcess(std::function<void(void)> theFunction);
#endif
/**
 * Espera (bloqueando el thread) hasta que la condition devuelve true o hasta que se cumple el timeout
 * \param condition
 * \param timeoutMs el timeout en milisegundos 
 * \param divisions 
 * \return true si condition ha devuelto true antes del timeout, false en otro caso
 */
bool waitUntil(std::function<bool()> condition, unsigned long timeoutMs, unsigned long divisions = 50);

/**
 * Computes the available space in the specified path
 * \param path directory where the space is queried (e.g., ".")
 * \param capacity returns the total size of the volume (in bytes)
 * \param available returns the available space (in bytes)
 * 
 * \return true if successful
*/
bool diskSpace(const std::string &path, uintmax_t &capacity, uintmax_t &available);

struct SystemInfo
{
  long uptime;                                            /* Seconds since boot */
  float oneminuteload, fiveminuteload, fifteenminuteload; /* 1, 5, and 15 minute load averages */
  uint64_t totalram;                                      /* Total usable main memory size (bytes) */
  uint64_t freeram;                                       /* Available memory size (bytes) */
  uint64_t totalswap;                                     /* Total swap space size (bytes) */
  uint64_t freeswap;                                      /* swap space still available (bytes) */
  unsigned short procs;                                   /* Number of current processes */
};

inline void toCSV(std::ostream &os, const SystemInfo &si)
{
  os << si.uptime << ";" << si.oneminuteload << ";" << si.fiveminuteload << ";" << si.fifteenminuteload << ";" << si.totalram << ";" << si.freeram << ";" << si.totalswap << ";" << si.freeswap << ";" << si.procs;
}

#ifndef _WIN32
/**
 * Returns information about the current usage of memory in the system, uptime, loads and number
 * of processes. \sa airt::SystemInfo
 * \param si the structure where the data will be written to
 * */
void systemInfo(SystemInfo &si);
#endif


  /**
  \return true if the user pressed a key (use airt::getChar to get it)
  */
bool keyPressed();

/**
\return the key pressed by the user
\warn It blocks until the user presses a key (use airt::keyPressed to check if she already did)
*/
int getChar();

////////
// FUNCIONES DE AYUDA SOBRE CADENAS
////////
// Devuelve una representación del número en hexadecimal, en forma de 0xYYY
std::string hexString(unsigned long n);

/**
  Quita los espacios del principio y del final de la cadena.
  */
void trim(std::string &s);

/**
  Remove the leading spaces
  */
void ltrim(std::string &s);

/**
  Remove the trailing spaces
  */
void rtrim(std::string &s);

/**
  Devuelve una copia de la cadena en minúsculas
  */
std::string to_lower(const std::string &s);

/**
  Devuelve si la cadena s empieza por b
  */
bool starts_with(const std::string &s, const std::string &b);

/**
  Devuelve si la cadena s acaba por b, ignorando minúsculas/mayúsculas
  */
bool ends_with(const std::string &s, const std::string &b);

/**
  \return true si la cadena s contiene la cadena b
  http://stackoverflow.com/questions/3152241/case-insensitive-stdstring-find
  */
bool containsIgnoreCase(const std::string &s, const std::string &b);

/**
  Escribe en el flujo un volcado hexadecimal del buffer.
  \param buffer puntero al buffer que contiene los datos
  \param size tamaño en bytes del buffer anterior
  \param output flujo donde escribir el volcado
  */
void hexdump(unsigned char *buffer, unsigned int size, std::ostream &output);

/**
   Convierte una cadena utf8 a latin1
   \param utf8 la cadena codificada en utf8 a convertir
   */
std::string utf8ToLatin1(std::string utf8);

/**
 * Convierte una cadena a entero sin signo.
 * \param s la cadena a transformar
 * \param ul el valor de la cadena, si contenía un número
 * \return true si la cadena se puede interpretar completamente como un número
 */
bool to_unsignedlong(const std::string &s, unsigned long &ul);

/**
 * Separa los componentes de una cadena por los espacios, teniendo en cuenta las comillas.
 * Por ejemplo: splitBySpaces("uno dos \"tres es\" cuatro") -> {"uno", "dos", "tres es", "cuatro"}
 * \param s la cadena a trocear
 * \return un vector con los trozos de la cadena
 * */
std::vector<std::string> splitBySpaces(const std::string &s);


/**
 * Formats a binary blob into a readable form, like:
 * 
 * [0xa 0x1 0x0 0x3c 0xff]
 * [0x10 0x1 0x2 0x3 0x4 0x5 0x10 ... 5 more]  (when the input buffer is greater than the maximum size)
 * 
 * \param buffer pointer to the buffer
 * \param size number of bytes in the buffer
 * \param max maximum number of bytes to output (prints at the end the number of bytes left). By default, 16 
 */
std::string formatBinaryBuffer(const void *buffer, size_t size, size_t max = 16);


/**
 * \return the floating point number with the given number of digits
 */
template<typename F>
std::string to_string(const F value, int precision) {
  std::stringstream ss;

  ss << std::setprecision(precision) << value;
  return ss.str();
}

////////
// FUNCIONES DE AYUDA SOBRE C++
////////

// http://stackoverflow.com/questions/8357240/how-to-automatically-convert-strongly-typed-enum-into-int
template <typename E>
typename std::underlying_type<E>::type to_underlying(E e)
{
  return static_cast<typename std::underlying_type<E>::type>(e);
}

template <typename E, typename T>
constexpr inline typename std::enable_if<std::is_enum<E>::value && std::is_integral<T>::value, E>::type
to_enum(T value) noexcept
{
  return static_cast<E>(value);
}

/**
  \return El número de elementos del array, como una constante en tiempo de compilación
  */
// Effective C++, pp. 16
template <typename T, std::size_t N>
constexpr std::size_t arraySize(T (&)[N]) noexcept { return N; }


template <typename T>
void writeBytesHighToLow(uint8_t *dst, T data) {
  size_t nbytes = sizeof(T);

  for (size_t i = 0; i < nbytes; i++) {
    dst[nbytes - 1 - i] = data & 0xff;
    data >>= 8;
  }
}

template <typename T>
void readBytesHighToLow(uint8_t *data, T &dst) {
  size_t nbytes = sizeof(T);

  dst = data[0];
  for (size_t i = 1; i < nbytes; i++) {
    dst <<= 8;
    dst |= data[i];
  }
}

template <typename T> inline constexpr
int sign(T x, std::false_type) {
    return T(0) < x;
}

template <typename T> inline constexpr
int sign(T x, std::true_type) {
    return (T(0) < x) - (x < T(0));
}

template <typename T> inline constexpr
int sign(T x) {
    return sign(x, std::is_signed<T>());
}

template <typename T>
T truncToRange(const T value, const T min, const T max) {
  if (value < min) 
    return min;
  else if (value > max) 
    return max;
  else return value;
}

/**
 * Binary search a value in a container
 * 
 */
template<class Iter, class T, class Compare>
Iter binary_find(Iter begin, Iter end, T val, Compare comp)
{
    // Finds the lower bound in at most log(last - first) + 1 comparisons
    Iter i = std::lower_bound(begin, end, val, comp);

    if (i != end && !comp(val, *i))
        return i; // found
    else
        return end; // not found
}

}; // namespace 

// http://stackoverflow.com/questions/20631922/expand-macro-inside-string-literal

#ifndef STRINGIFY
#define STRINGIFY2(X) #X
#define STRINGIFY(X) STRINGIFY2(X)
#endif