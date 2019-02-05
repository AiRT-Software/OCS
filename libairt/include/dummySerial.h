#pragma once
#include <string>
#include <boost/circular_buffer.hpp>
#include "utils.h"
#include "log.h"
namespace airt
{
class Serial
{
  public:
    Serial() : for_read(2048) {};
    bool open(const std::string &port, unsigned int baudrate)
    {
        Log::warn("Opening DUMMY SERIAL PORT {}", port);
        port_open = true;
        uartport = port;
        return port_open;
    };
    bool isOpen() { return port_open; }
    void close() { port_open = false; }
    size_t read(unsigned char *buffer, size_t max)
    {
        if (max > for_read.size())
            max = for_read.size();
        for (size_t i = 0; i < max; i++)
        {
            buffer[i] = for_read.front();
            for_read.pop_front();
        }
        if (max > 0) 
            Log::info("{} READ -> {}", uartport, airt::formatBinaryBuffer(buffer, max, 100));
        return max;
    }
    size_t write(unsigned char *buffer, size_t size)
    {
        if (!port_open) {
            Log::error("Serial port not opened!");
            throw new std::runtime_error("Trying to write to a closed port");
        }
        for (size_t i = 0; i < size; i++)
        {
            written.push_back(buffer[i]);
        }
        Log::info("{} <- WRITE {}", uartport, airt::formatBinaryBuffer(buffer, size, 100));
        return size;
    }

    boost::circular_buffer<uint8_t> for_read;
    std::vector<uint8_t> written;

  private:
    bool port_open = false;
    std::string uartport;
};
};
