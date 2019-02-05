#pragma once

#include "message.h"
#include "context.h"

namespace zmqpp
{
class socket;
}

namespace airt
{
class Socket
{
public:
  enum class SocketType
  {
    pair,
    publish,
    subscribe,
    pull,
    push,
    request,
    reply,
    xpublish,
    xsubscribe,
    xrequest,
    xreply,
    stream
  };

  Socket(Context &ctx, SocketType type);
  ~Socket();

  void connect(const std::string &endpoint);
  void bind(const std::string &endpoint);
  bool send(Message &message, bool const dont_block = false);
  bool receive(Message &message, bool const dont_block = false);
  void setLinger(int linger);
  /*
    Subscribe to all messages starting with the 'size' bytes pointed to by 'header'
    \param header the prefix of the messages we want
    \param size number of bytes of the prefix
    It is possible to register several filters calling several times to this method. A message will
    be delivered if it matches any of the registered filters.
  */
  void subscribe(const uint8_t *header, size_t size);
  /**
    Subscribe to all messages
  */
  void subscribeAll();

  void unsubscribe(const uint8_t *header, size_t size);
  /**
    Subscribe to all messages
  */
  void unsubscribeAll();
  /**
	 * Check the socket is still valid
	 * This tests the internal state of the socket.
	 * \return true if the socket is valid
	 */
  operator bool() const;

private:
  std::unique_ptr<zmqpp::socket> sock;
  friend class Poller;
  friend class Reactor;
  friend class ProxySteerable;
};
};