#pragma once

#include <string>
#include <stdMessage.h>
#include <utils.h>
#include <iosfwd>
#include "module.h"

namespace airt
{
class Context;

class AtreyuModule : public Module
{
public:
#include "interface/atreyumodule.h"

  AtreyuModule(const std::string &cmdportname, const std::string &pubportname,
               std::shared_ptr<Context> context);

  void onMessage(const Message &m) override;
  bool dataReady() override { return false; }
  bool sendMessage() override { return false; }
  bool startDevice() override;
  bool quitDevice() override;

private:
  void heartbeat();
  StdMessage::AtreyuSystemState state;
  unsigned long heartBeatsPeriod_ms;
};
}; // namespace airt
