#pragma once

#include <string>
#include <stdMessage.h>
#ifdef TESTING
#include "dummySerial.h"
#else
#include "serial.h"
#endif

#include "module.h"
#include "zCamE1MessageProcessor.h"

namespace airt
{
class Context;

class RCamModule : public Module
{
  public:

#include "interface/rcammodule.h"

    RCamModule(const std::string &cmdportname, const std::string &pubportname,
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
    ZCamE1MessageProcessor messageProcessor;
    bool processSetConfigCommand(const Message &inmsg);
};


inline void toCSV(std::ostream &os, const RCamModule::RCamNackNotification &n)
{
    os << n.header.module << ";" << n.header.action << ";" << n.command;
}

inline void toCSV(std::ostream &os, const RCamModule::RCamBatteryLevelNotification &n)
{
    os << n.header.module << ";" << n.header.action << ";" << n.level;
}

inline void toCSV(std::ostream &os, const RCamModule::CameraModeNotification &n)
{
    os << n.header.module << ";" << n.header.action << ";" << static_cast<uint8_t>(n.mode);
}

inline void toCSV(std::ostream &os, const RCamModule::CameraStatusNotification &n)
{
    os << n.header.module << ";" << n.header.action << ";" << static_cast<uint8_t>(n.status);
}

inline void toCSV(std::ostream &os, const RCamModule::WIFIStatusNotification &n)
{
    os << n.header.module << ";" << n.header.action << ";" << static_cast<uint8_t>(n.on);
}

inline void toCSV(std::ostream &os, const RCamModule::SDCardStatusNotification &n)
{
    os << n.header.module << ";" << n.header.action << ";" << static_cast<uint8_t>(n.ready);
}

inline void toCSV(std::ostream &os, const RCamModule::RecordingStatusNotification &n)
{
    os << n.header.module << ";" << n.header.action << ";" << n.recording;
}

inline void toCSV(std::ostream &os, const RCamModule::RecordingTimeLeftNotification &n)
{
    os << n.header.module << ";" << n.header.action << ";" << n.minutes;
}

inline void toCSV(std::ostream &os, const RCamModule::NumberPhotosLeftNotification &n)
{
    os << n.header.module << ";" << n.header.action << ";" << n.photos;
}

inline void toCSV(std::ostream &os, const RCamModule::ChoiceConfigParameter &n)
{
    os << n.header.module << ";" << n.header.action << ";" << n.type << ";" << n.parameter << ";" << n.value << ";" << n.size;
    const uint8_t *aval = reinterpret_cast<const uint8_t *>(&n) + sizeof(RCamModule::ChoiceConfigParameter);
    for (unsigned int i = 0; i < n.size; i++)
    {
        os << ";" << aval[i];
    }
}

inline void toCSV(std::ostream &os, const RCamModule::RangeConfigParameter &n)
{
    os << n.header.module << ";" << n.header.action << ";" << n.type << ";" << n.parameter << ";" << n.value << ";"
       << n.min << ";" << n.max << ";" << n.step;
}
};