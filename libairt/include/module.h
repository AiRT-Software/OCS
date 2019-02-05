#pragma once

#include <string>
#include <memory>
#include <thread>
#include <chrono>
#include <functional>

#include "taskScheduler.h"

namespace airt
{

/**
\file This class represents an AiRT module. It can generate a flow of information (images, positions,
  timestamps, etc.) and also receive commands.
  It has two ports:
    -a command port (internal, bidirectional)
    -a publishing port
  The command port accepts commands such as
  START, STOP and QUIT that starts sending data, stops sending data and shutdowns the object.

*/

class Socket;
class Context;
class Message;

class Module
{
public:
  /**
   * The server sends command via the cmdport. The module publishes its notifications to all 
   * (other modules and clients) via the pubport, and optionally can subscribe to notifications
   * using the subport.
   * 
   * \param cmdport command port (i.e. "inproc://oscmd.ipc"). Mandatory. 
   * \param pubport publishing port (i.e. "inproc://ospub.ipc"). Mandatory
   * \param subport subscription port (i.e. "inproc://ossub.ipc"). Optional. Empty string if the module does not
   * need a subscription port.
   * */
  Module(const std::string &cmdport, const std::string &pubport, const std::string &subport,
         std::shared_ptr<Context> context);

  virtual ~Module(){};
  /**
    \return the address to connect  in order to communicate with  this object
    */
  const std::string &getCmdPort() const { return commandport; };
  /**
    \return the address of the publisher port
    */
  const std::string &getPubPort() const { return publishport; };

  /**
    \return the address of the subscription port
    */
  const std::string &getSubPort() const { return subscribeport; };

  /**
\return true if the device is on (it is generating data)
*/
  bool isDeviceOn() const { return deviceOn; }

  /**
Override this method and return true when there is data ready to be published
*/
  virtual bool dataReady() = 0;

  /**
Override this method to send the data with a message
*/
  virtual bool sendMessage() = 0;
  /**
Override this method with the code to start the device
\return true if the device started correctly
*/
  virtual bool startDevice() { return true; };

  /**
Override this method with the code to stop the device
*/
  virtual void stopDevice(){};

  /**
Override this method with the code to stop the device
*/
  virtual bool quitDevice() { return true; };

  /**
Override this method to process non-control messages
*/
  virtual void onMessage(const Message &m){};

  /**
 * Override this method to process notifications from other modules
 */
  virtual void onNotification(const Message &m){};

  /**
 * If the module wants to subscribe to other module's notification, override 
 * this method and configure the required subscriptions (you will usually be
 * interested in notifications of a small set of modules)
 **/
  virtual void configureSubscriptions(const std::string &centralPublisher){};

  /**
 * This method is called whenever a cycle didn't produce any event
 * */
  virtual void onIdle() { std::this_thread::sleep_for(std::chrono::milliseconds(1)); }
  
  /**
Call this method for waiting to the thread
*/
  void shutdown();

  /**
   * Schedule a task for later execution. 
   * \warning It is not very precise
   * 
   */
  void schedule(size_t wait_ms, std::function<void (void)> task) { tasks.schedule(wait_ms, task); }
protected:
  template <bool hassubscriber>
  void run();
  void processCmd();
  void processNot();

  std::string commandport, publishport, subscribeport;
  std::shared_ptr<Context> context;
  std::unique_ptr<Socket> cmdsocket, pubsocket, subsocket;

  TaskScheduler tasks;

  std::thread thread;
  bool deviceOn;
  bool done;
};
};
