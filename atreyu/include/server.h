#pragma once

#include <string>
#include <properties.h>
#include <net/socket.h>
#include <net/reactor.h>
#include <forwarder.h>

#include "module.h"
#include <stdMessage.h>

namespace airt
{
class Server
{
public:
  Server();
  ~Server();

  void onStart();
  void onStop();
  void onQuit();
  void onPowerOff();

  void onIdle();
  void run();

protected:
  Server(const Server &other) = delete;
  Server &operator=(Server other) = delete;

  void publish(uint8_t message, size_t size);
  void request(uint8_t message, size_t size);

  void config();
  std::shared_ptr<Context> context;

  void shutdown();

  struct ModuleConnection
  {
    ModuleConnection(std::shared_ptr<Socket> socket, std::shared_ptr<Module> device) : localSocket(socket), device(device){};
    std::shared_ptr<Socket> localSocket;
    std::shared_ptr<Module> device;
  };

  std::map<unsigned char, ModuleConnection> moduleconnections;

private:
  template <typename Device>
  bool installModule(const std::string &portname, StdMessage::Modules commandModule);

  template <typename Device>
  bool installModule(const std::string &cmdportname, const std::string &pubportname, StdMessage::Modules commandModule);

  template <typename Device>
  bool installModule(const std::string &cmdportname, const std::string &pubportname, const std::string &subportname, StdMessage::Modules commandModule);

  bool connectModule(std::shared_ptr<Module> module, const std::string &cmdportname, const std::string &pubportname, StdMessage::Modules commandModule);

  /**
   * There is a message waiting in the socket. Read it and publish it
   * \param socket the communication socket with a module
   * */
  void receiveAndPublishMessage(std::shared_ptr<Socket> socket);

  /**
     * @brief processReq manages the incoming requests from the clients before sending orders to the internal Sources
     */
  void processReq();

  Reactor reactor;
  std::unique_ptr<Forwarder> forwarder;
  Socket requests;

  bool done;

  void dispatchMessageToModules(Message &m);
  void validateAndForward(const std::string &moduleName, Message &msg, uint8_t maxActionCode, airt::Socket &s);
};
}; // namespace airt
