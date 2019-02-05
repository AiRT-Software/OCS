#pragma once

#include <string>
#include <stdMessage.h>
#include <utils.h>
#include <iosfwd>
#include "module.h"

namespace airt
{
class Context;

class OSModule : public Module
{
public:
#include "interface/osmodule.h"

  OSModule(const std::string &cmdportname, const std::string &pubportname,
           std::shared_ptr<Context> context);

  void onMessage(const Message &m) override;
  bool dataReady() override { return false; }
  bool sendMessage() override { return false; }

private:
  void sendFileError(const std::string &filename, StdMessage::OSFileError error);
  void sendStreamError(const std::string &filename, const std::ios &stream);
  void handleFileRequests(const Message &inmsg, const std::string &filename);
  void handleNetworkStats();
  InterfaceStats getInterfaceStats(const std::string &iface);
};
};
