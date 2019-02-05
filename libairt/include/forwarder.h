#pragma once

#include <string>
#include <memory>

/** 
 * 
 * \class Forwarder
 * This class is in charge of implementing a Forwarder device with Zero-mq:
 * http://zguide.zeromq.org/page:all#toc38
 * 
 */

namespace airt
{

class Socket;
class Context;
class ProxySteerable;

class Forwarder
{
public:
  Forwarder(const std::string &receivingPort, const std::string &emittingPort,
            std::shared_ptr<Context> context);
  ~Forwarder();
  void pause();
  void resume();
  void shutdown();
  Socket &getReceivingSocket() const { return *xsub; }
  Socket &getEmittingSocket() const { return *xpub; }
  const std::string &getReceivingPort() const { return receivingP; }
  const std::string &getEmittingPort() const { return emittingP; }

private:
  void run();
  std::unique_ptr<ProxySteerable> proxy;
  std::unique_ptr<Socket> xpub, xsub, ctrl;
  std::shared_ptr<Context> fwdrContext;
  std::thread thread;
  std::string emittingP, receivingP, controlP;
  static std::string produceNewControlPort();
};
};