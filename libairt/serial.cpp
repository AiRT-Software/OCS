#include <unistd.h>


#include "serial.h"
#include "log.h"
#include "utils.h"
using airt::Serial;

/*

Serial Programming Guide for POSIX Operating Systems
www.cmrr.umn.edu/~strupp/serial.html

*/

bool Serial::open(const std::string &port, unsigned int baudrate)
{
    Log::info("Opening port {} @ {} bauds", port, baudrate);

    int baudrateConstant;
    switch (baudrate)
    {
    case 115200:
        baudrateConstant = B115200;
        break;
    case 9600:
        baudrateConstant = B9600;
        break;
    case 38400:
        baudrateConstant = B38400;
        break;
    default:
        Log::error("Unsupported baud rate");
        return false;
    }

    if (fd >= 0)
    {
        Log::warn("Serial port {} was in use. Closing and opening {}", this->device, port);
        close();
    }

    fd = ::open(port.c_str(), O_RDWR | O_NOCTTY | O_NDELAY);
    if (fd < 0)
    {
        Log::error("Failed to open serial port {}", port);
        return false;
    }

    tcgetattr(fd, &oldtio); /* save current port settings */

    // Non-blocking reads
    //fcntl(fd, F_SETFL, FNDELAY);

    struct termios newtio;
    bzero(&newtio, sizeof(newtio));
    newtio.c_cflag = baudrateConstant | CS8 | CLOCAL | CREAD;
    newtio.c_iflag = IGNPAR;
    newtio.c_oflag = 0;

    /* set input mode (non-canonical, no echo,...) */
    newtio.c_lflag = 0;

    newtio.c_cc[VTIME] = 0; 
    newtio.c_cc[VMIN] = 0;  

    tcflush(fd, TCIFLUSH);
    tcsetattr(fd, TCSANOW, &newtio);

    device = port;
    return true;
}

bool Serial::isOpen()
{
    return fd > 0;
}

void Serial::close()
{
    Log::info("Closing serial port {}", device);
    tcsetattr(fd, TCSANOW, &oldtio);
    ::close(fd);
    fd = -1;
}

size_t Serial::read(unsigned char *buffer, size_t max)
{
    auto r = ::read(fd, buffer, max); 
    if (r < 0) r = 0;
    if (r > 0) 
        Log::info("{} READ -> {}", device, airt::formatBinaryBuffer(buffer, r, 32));
    return r;
}

size_t Serial::write(unsigned char *buffer, size_t size)
{
    Log::info("{} <- WRITE {}", device, airt::formatBinaryBuffer(buffer, size, 32));
    auto r = ::write(fd, buffer, size);
    return r;
}
