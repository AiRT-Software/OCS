#pragma once

#include <string>
#include <stdMessage.h>
#ifdef TESTING
#include "dummySerial.h"
#else
#include "serial.h"
#endif

#include "module.h"
#include "aerotoolsGimbalMessageProcessor.h"

namespace airt
{
class Context;

class GimbalModule : public Module
{
  public:

  #include "interface/gimbalmodule.h"

    GimbalModule(const std::string &cmdportname, const std::string &pubportname,
              std::shared_ptr<Context> context);

    bool dataReady() override;
    bool sendMessage() override;
    bool startDevice() override;
    void stopDevice() override;
    void onMessage(const Message &inmsg) override;
  private:
    std::string serialport;
    unsigned int baudrate;
    uint64_t publishAnglePeriod_ms; // when the gimbal is moving, publish its position every X ms
    float currentPitch, currentYaw, currentRoll;  // current position (radians)
    float destPitch, destYaw, destRoll; // moving to this position (radians)
    float speedRadsPerSec;  // radians per sec
    uint64_t startTime;  // the gimbal started to move at this time
    float yawOffset;  // add this value to the message to the gimbal
    Serial port;
    AeroToolsGimbalMessageProcessor messageProcessor;

    void startMoving(const GoToAngle *gta);
    void processGoToAngle(float pitch, float yaw, float roll, float speed);
};
};
