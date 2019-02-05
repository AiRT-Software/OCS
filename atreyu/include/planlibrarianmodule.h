#pragma once

#include <string>
#include <stdMessage.h>
#include <utils.h>
#include <iosfwd>
#include "module.h"

namespace airt
{
class Context;

class PlanLibrarianModule : public Module
{
public:
#include "interface/planlibrarianmodule.h"

  PlanLibrarianModule(const std::string &cmdportname, const std::string &pubportname,
               std::shared_ptr<Context> context);

  void onMessage(const Message &m) override;
  bool dataReady() override { return false; }
  bool sendMessage() override { return false; }
  void onIdle() override { std::this_thread::sleep_for(std::chrono::milliseconds(150)); }
private:
  std::string baseToMetadataFullpath(const std::string &base) const;
  std::string baseToMissionFullpath(const std::string &base) const;
  std::string libraryDir;
};
};
