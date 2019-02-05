#pragma once

#include <memory>

namespace zmqpp
{
class poller;
};

namespace airt
{
class Socket;

class Poller
{
  enum PollerType
  {
    POLL_IN = 1,
    POLL_OUT = 2,
    POLL_ERROR = 4
  };

public:
  Poller();
  ~Poller();
  /**
	 * Add a socket to the polling model and set which events to monitor.
	 *
	 * \param socket the socket to monitor.
	 * \param event the event flags to monitor on the socket.
	 */
  void add(Socket &socket, const short event = POLL_IN);

  /**
	 * Poll for monitored events.
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

private:
  std::unique_ptr<zmqpp::poller> poller;
  friend class Socket;
};
};