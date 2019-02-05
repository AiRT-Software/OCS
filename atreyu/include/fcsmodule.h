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

class FCSModule : public Module
{
public:
#include "interface/fcsmodule.h"

  FCSModule(const std::string &cmdportname, const std::string &pubportname,
            std::shared_ptr<Context> context);

  bool dataReady() override;
  bool sendMessage() override;
  bool startDevice() override;
  void stopDevice() override;
  void onMessage(const Message &inmsg) override;

private:
  std::string serialport;
  unsigned int baudrate;
  Serial port;
  AeroToolsFCSMessageProcessor messageProcessor;
};
};
