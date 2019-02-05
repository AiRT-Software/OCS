#pragma once

#include <memory>
#include <functional>

namespace zmqpp
{
class reactor;
};

namespace airt
{

class Socket;

class Reactor
{
  enum ReactorType
  {
    POLL_IN = 1,
    POLL_OUT = 2,
    POLL_ERROR = 4
  };

public:
  Reactor();
  ~Reactor();

  /**
         * Poll for monitored events and call associated handler when needed.
         *
         * By default this method will block forever or until at least one of the monitored
         * sockets or file descriptors has events.
         *
         * If a timeout is set and was reached then this function returns false.
         *
         * \param timeout milliseconds to timeout.
         * \return true if there is an event..
         */
  bool poll(long timeout_ms = -1);
  /**
         * Add a socket to the reactor, providing a handler that will be called when the monitored events occur.
         *
         * \param socket the socket to monitor.
         * \param callable the function that will be called by the reactor when a registered event occurs on socket.
         * \param event the event flags to monitor on the socket.
         */
  void add(Socket &socket, std::function<void(void)> callback, const short event = POLL_IN);

private:
  std::unique_ptr<zmqpp::reactor> reactor;
};
};