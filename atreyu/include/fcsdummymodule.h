#pragma once

#include <string>
#include <stdMessage.h>
#include <glm/mat4x4.hpp>

#ifdef TESTING
#include "dummySerial.h"
#else
#include "serial.h"
#endif

#include "module.h"
#include "aerotoolsFCSMessageProcessor.h"
namespace airt
{
class Context;

class FCSDummyModule : public Module
{
public:
#include "interface/fcsmodule.h"

  FCSDummyModule(const std::string &cmdportname, const std::string &pubportname, const std::string &subname,
            std::shared_ptr<Context> context);

  bool dataReady() override;
  bool sendMessage() override;
  bool startDevice() override;
  void stopDevice() override;
  void onMessage(const Message &inmsg) override;
  void configureSubscriptions(const std::string &mainpublisher) override;
  void onNotification(const Message &m) override;

private:
  uint8_t battery, motors;
  int requestWP;
  bool flyingMission;
  size_t destinationWP;
  float arrivedDistance;

  std::vector<glm::vec3> waypoints;

  float publishPeriodS;
};
};
