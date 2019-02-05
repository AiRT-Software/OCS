#pragma once

#include <string>
#include <stdMessage.h>
#include <utils.h>
#include <iosfwd>
#include <drone.h>
#include "module.h"

namespace airt
{
class Context;

class DroneModule : public Module
{
public:
#include "interface/dronemodule.h"

  DroneModule(const std::string &cmdportname, const std::string &pubportname, const std::string &subportname,
               std::shared_ptr<Context> context);

  void onMessage(const Message &m) override;
  bool dataReady() override { return false; }
  bool sendMessage() override { return false; }
  void configureSubscriptions(const std::string &mainpublisher) override;
  void onNotification(const Message &m) override;
private:
  Drone drone;
  void publishPose();
  void publishGimbalPose();
};
};
