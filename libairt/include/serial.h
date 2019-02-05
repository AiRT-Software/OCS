#pragma once
#include <string>
#include <termios.h>

namespace airt
{
/**
     * \class Serial
     * Class for working with a serial port.
     * Example:
     * 
     * Serial s;
     * if (!s.open("/dev/ttyS1", 115200)) {
     *      exit(1);
     * }
     * ...
     * unsigned char bufferIn[100], bufferOut[100];
     * size_t t;
     * if ((t = s.read(bufferIn, sizeof(bufferIn))) > 0) {
     *    // procesar t bytes de bufferIn
     * }
     * ...
     * if (s.write(bufferOut, 5) != 5) {
     *      // fallo al escribir
     * }
     * */
class Serial
{
  public:
    Serial() : device("/dev/ttyS1"), baudrate(115200), fd(-1) {};
    /**
     * Opens the given port at the specified baudrate. 
     * It alway uses 8N1 configuration
     * If the port was already open, it will close it and open it again with the new configuration
     * \param port device (e.g. /dev/ttyS1)
     * \param baudrate speed of the connection, in bauds
     * */
    bool open(const std::string &port, unsigned int baudrate);
    /**
     * \return true if the port is open
     * */
    bool isOpen();
    /**
     * Closes the connection and restores the previous configuration of the serial port
     * */
    void close();
    /**
     * Reads up to max characters.
     * This is a non-blocking call. It will return immediately
     * \param buffer where to write the incoming data
     * \param max maximum number of bytes to read
     * \return the number of characters read (it can be 0 if there was no data)
     * */
    size_t read(unsigned char *buffer, size_t max);
    /**
     * Writes the provided data into the serial port
     * \param buffer the data to send
     * \param size number of bytes to send
     * \return the number of bytes successfully sent
     */
    size_t write(unsigned char *buffer, size_t size);

  private:
    std::string device;
    unsigned int baudrate;
    struct termios oldtio;
    int fd;
};
};