#pragma once

#include <string>
#include <memory>
#include <cstdint>

namespace airt
{

/**
\class  ReliableExternalClient

This client will make request to the server. For each request, it will wait a number of milliseconds
before retrying. After a number of retries, it will assume the server is down and it will shutdown.
*/
class Context;
class Message;
class Socket;

class ReliableExternalClient
{
public:
  /**
    Constructor
    \param address address of the server (i.e. "tcp://192.168.1.20:5555")
    \param timeout milliseconds to wait for the response before retrying
    \param retries number of retries before giving up
    */
  ReliableExternalClient(std::shared_ptr<Context> context, const std::string &address, size_t timeout, size_t retries);

  /**
    Sends the mesage to the server. It will wait until it receives the response, or retries on
    timeout. If after the specified number of retries it gets no response, it returns false
    \param data pointer to the data to be sent
    \param size number of bytes of the data
    \param response message for storing the response
    \return true if it sent the message and got the response from the server, and false otherwise
    */
  bool sendAndReceive(const void *data, size_t size, Message &response);
  /**
    Sends the mesage to the server. It will wait until it receives the response, or retries on
    timeout. If after the specified number of retries it gets no response, it returns false
    \param request message to send to the server
    \param response message for storing the response
    \return true if it sent the message and got the response from the server, and false otherwise
    */
  bool sendAndReceive(Message &request, Message &response);
  inline bool sendAndReceive(Message &&request, Message &response)
  {
    return sendAndReceive(request, response);
  }

private:
  void reconnect();
  std::shared_ptr<Context> context;
  std::unique_ptr<Socket> socket;
  std::string srvAddress;
  size_t timeout, retries;
};
};
