#include "gimbalmodule.h"

#include "globalSettings.h"

#include <log.h>
#include <sstream>
#include <limits>
#include <utils.h>

using airt::GimbalModule;
using airt::Log;
using airt::StdMessage;
using airt::StdNotification;

GimbalModule::GimbalModule(const std::string &cmdportname, const std::string &pubportname,
                           std::shared_ptr<Context> context)
    : Module(cmdportname, pubportname, "", context)
{
    assert(context);

    if (!GlobalSettings::getValue("gimbal_uart_port", serialport))
    {
        serialport = "/dev/ttyUSB2";
        Log::warn("Using default value gimbal_uart_port = {}", serialport);
    }

    if (!GlobalSettings::getValue("gimbal_baud_rate", baudrate))
    {
        baudrate = 115200;
        Log::warn("Using default value gimbal_baud_rate = {}", baudrate);
    }

    if (!GlobalSettings::getValue("gimbal_angles_update_period_ms", publishAnglePeriod_ms))
    {
        publishAnglePeriod_ms = 100;
        Log::warn("Using default value for publishAnglePeriod_ms = {}", publishAnglePeriod_ms);
    }

    currentPitch = currentRoll = currentYaw = 0.0f;
    speedRadsPerSec = 0.0f;
    yawOffset = 0.0f;
}

bool GimbalModule::startDevice()
{
    Log::info("GimbalModule starts...");
    if (!port.isOpen() && !port.open(serialport, baudrate))
    {
        Log::critical("Failed to open serial port {} at {} bauds", serialport, baudrate);
        throw new std::runtime_error("Error opening serial port");
    }
    airt::sendNotification(*pubsocket, StdMessage::GIMBAL_MULTIPLEXER_NOTIFICATIONS_MODULE, StdMessage::STARTED);
    return true;
};

void GimbalModule::stopDevice()
{
    if (port.isOpen())
        port.close();
    Log::info("GimbalModule stops...");
    airt::sendNotification(*pubsocket, StdMessage::GIMBAL_MULTIPLEXER_NOTIFICATIONS_MODULE, StdMessage::STOPPED);
};

bool GimbalModule::dataReady()
{
    return false;

    // Currently, the gimbal does not transmit anything. Left here for future use
    // unsigned char buffer[64];
    // size_t bytesread;
    // while ((bytesread = port.read(buffer, sizeof(buffer))) > 0)
    // {
    //     messageProcessor.ingest(buffer, bytesread);
    //     if (bytesread < sizeof(buffer))
    //         break;
    // }
    // return messageProcessor.hasMessages();
}

void GimbalModule::processGoToAngle(float pitch, float yaw, float roll, float speed)
{
    Log::info("Gimbal moving from p: {}, r: {}, y: {} to p: {}, r: {}, y: {} at {} rad/s",
              currentPitch, currentRoll, currentYaw, pitch, roll, yaw, speed);
    messageProcessor.sendGotoAngle(port, pitch, roll, yaw + yawOffset, speed);
    // Store the data for the generation of *simulated* intermediate messages
    destPitch = pitch;
    destYaw = yaw;
    destRoll = roll;
    speedRadsPerSec = speed;
    startTime = msSinceEpoch();

    // Send the notification that we have received the command
    auto gan = GotoAngleNotification();
    gan.pitch = pitch;
    gan.roll = roll;
    gan.yaw = yaw;
    gan.speedRadPerSec = speed;
    airt::sendOnePartMessage(*pubsocket, &gan);

    schedule(publishAnglePeriod_ms, [this]() { sendMessage(); });
}

void GimbalModule::onMessage(const Message &inmsg)
{
    uint8_t command = airt::getMessageAction(inmsg);
    switch (command)
    {
    case StdMessage::GIMBAL_GOTO_ZERO:
        processGoToAngle(0.0f, 0.0f, 0.0f, 50.0 * M_PI / 180.0);
        break;
    case StdMessage::GIMBAL_GOTO_ANGLE:
    {
        const GoToAngle *gta = inmsg.get<const GoToAngle *>(0);
        processGoToAngle(
            airt::truncToRange(gta->pitch, static_cast<float>(-M_PI/2.0), static_cast<float>(M_PI/2)), 
            gta->yaw, 
            airt::truncToRange(gta->roll, static_cast<float>(-M_PI/2.0), static_cast<float>(M_PI/2)), 
            gta->speedRadPerSec);
        break;
    }
    case StdMessage::GIMBAL_MOVE_PITCH:
    case StdMessage::GIMBAL_MOVE_ROLL:
    case StdMessage::GIMBAL_MOVE_YAW:
    case StdMessage::GIMBAL_STOP_PITCH:
    case StdMessage::GIMBAL_STOP_ROLL:
    case StdMessage::GIMBAL_STOP_YAW:
    case StdMessage::GIMBAL_STOP_ALL:
        Log::critical("Unsupported gimbal command: {}", command);
        airt::sendNotification(*pubsocket, StdMessage::GIMBAL_MULTIPLEXER_NOTIFICATIONS_MODULE, StdMessage::UNKNOWN_COMMAND);
        return;
        break;
    case StdMessage::GIMBAL_RC_MODE:
        messageProcessor.sendCommand(port, static_cast<airt::GimbalMultiplexerCommandType>(command));
        airt::sendNotification(*pubsocket, StdMessage::GIMBAL_MULTIPLEXER_NOTIFICATIONS_MODULE, StdMessage::GIMBAL_MODE_CHANGED_TO_RC_NOTIFICATION);
        break;
        break;
    case StdMessage::GIMBAL_RC_FIX_MODE:
        messageProcessor.sendCommand(port, static_cast<airt::GimbalMultiplexerCommandType>(command));
        airt::sendNotification(*pubsocket, StdMessage::GIMBAL_MULTIPLEXER_NOTIFICATIONS_MODULE, StdMessage::GIMBAL_MODE_CHANGED_TO_RC_FIX_MODE_NOTIFICATION);
        break;
    case StdMessage::GIMBAL_RC_NOROLL_MODE:
        messageProcessor.sendCommand(port, static_cast<airt::GimbalMultiplexerCommandType>(command));
        airt::sendNotification(*pubsocket, StdMessage::GIMBAL_MULTIPLEXER_NOTIFICATIONS_MODULE, StdMessage::GIMBAL_MODE_CHANGED_TO_RC_NOROLL_MODE_NOTIFICATION);
        break;
    default:
        Log::critical("Unhandled message in Gimbal module");
        airt::sendNotification(*pubsocket, StdMessage::GIMBAL_MULTIPLEXER_NOTIFICATIONS_MODULE, StdMessage::UNKNOWN_COMMAND);
        return;
    }
    airt::sendNotification(*pubsocket, StdMessage::GIMBAL_MULTIPLEXER_NOTIFICATIONS_MODULE, StdMessage::ACK);
}

bool GimbalModule::sendMessage()
{
    // The gimbal is moving: compute the current position
    // At the given speed, it will take this many seconds to get there

    float deltaPitch = destPitch - currentPitch;
    float deltaYaw = destYaw - currentYaw;
    float deltaRoll = destRoll - currentRoll;

    float largest = std::max(std::fabs(deltaPitch), std::max(std::fabs(deltaYaw), std::fabs(deltaRoll)));
    float time = (largest * 1000.0f)/ speedRadsPerSec;
    float elapsed = static_cast<float>(msSinceEpoch() - startTime);

    GimbalAngleNotification gan;
    if (elapsed > time)
    {
        Log::info("Gimbal arrived to destination");
        speedRadsPerSec = 0.0f;
        gan.pitch = currentPitch = destPitch;
        gan.yaw = currentYaw = destYaw;
        gan.roll = currentRoll = destRoll;
    }
    else
    {
        float advanceSinceStart = speedRadsPerSec * (elapsed / 1000.f);
        if (advanceSinceStart > std::fabs(deltaPitch))
            gan.pitch = destPitch;
        else
            gan.pitch = currentPitch + airt::sign(deltaPitch) * advanceSinceStart;

        if (advanceSinceStart > std::fabs(deltaRoll))
            gan.roll = destRoll;
        else
            gan.roll = currentRoll + airt::sign(deltaRoll) * advanceSinceStart;

        if (advanceSinceStart > std::fabs(deltaYaw))
            gan.yaw = destYaw;
        else
            gan.yaw = currentYaw + airt::sign(deltaYaw) * advanceSinceStart;
        schedule(publishAnglePeriod_ms, [this]() { sendMessage(); });
    }
    airt::sendOnePartMessage(*pubsocket, &gan);
    return true;

    // Currently the gimbal does not generate any message. I leave this code here for the future
    // while (messageProcessor.hasMessages())
    // {
    //     unsigned char buffer[128];
    //     unsigned int size;
    //     if (messageProcessor.popMessage(buffer, sizeof(buffer), size))
    //     {
    //         Message msg;
    //         msg.add_raw(buffer, size);
    //         pubsocket->send(msg);
    //     }
    //     else
    //     {
    //         Log::warn("Supposedly there was a message waiting from the gimbal serial port");
    //     }
    // }
    // return true;
}
