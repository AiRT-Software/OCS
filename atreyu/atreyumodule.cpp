#include "globalSettings.h"
#include "atreyumodule.h"

#include <sstream>
#include <log.h>
#include <stdMessage.h>
#include <utils.h>

using airt::AtreyuModule;
using airt::Log;

AtreyuModule::AtreyuModule(const std::string &cmdportname, const std::string &pubportname,
                           std::shared_ptr<Context> context)
    : Module(cmdportname, pubportname, "", context)
{
    assert(context);

    // Finding the heartbeat period
    double heartBeatsPeriodSeconds;
    if (!GlobalSettings::getValue("heartbeatPeriod", heartBeatsPeriodSeconds))
    {
        Log::error("The configuration file does not specify the heartbeat period (hearbeatPeriod). Setting it to 1.5 seconds");
        heartBeatsPeriod_ms = 1500;
    }
    else
    {
        heartBeatsPeriod_ms = 1000 * heartBeatsPeriodSeconds;
    }

    state = StdMessage::IDLE_STATE;
    Log::info("Atreyu module initialized");
}

void AtreyuModule::onMessage(const Message &inmsg)
{
    auto action = airt::getMessageAction(inmsg);
    switch (action)
    {
    case StdMessage::ATREYU_QUERY_SERVER_VERSION:
    {
        AtreyuVersionNotification m;
        m.major = StdMessage::MAJOR;
        m.minor = StdMessage::MINOR;
        m.patch = StdMessage::PATCH;
        airt::sendOnePartMessage(*pubsocket, &m);
        break;
    }
    case StdMessage::ATREYU_QUERY_UPDATE_PACKAGE_PATHNAME:
    {
        AtreyuUpdatePackagePathnameNotification m;
        airt::sendTwoPartMessage(*pubsocket, &m, airt::installUpdatesPathname);
        break;
    }
    case StdMessage::ATREYU_QUERY_SYSTEM_STATE:
    {
        AtreyuStateChanged s;
        s.newstate = s.oldstate = state;
        airt::sendOnePartMessage(*pubsocket, &s);
        break;
    }
    case StdMessage::ATREYU_ENTER_RECORDING_STATE:
    case StdMessage::ATREYU_ENTER_MAPPING_STATE:
    {
        if (state != StdMessage::IDLE_STATE)
        {
            airt::sendNotification(*pubsocket, StdMessage::ATREYU_NOTIFICATIONS_MODULE, StdMessage::ATREYU_CANNOT_CHANGE_STATE_NOTIFICATION);
        }
        else
        {
            AtreyuStateChanged s;
            s.oldstate = state;
            s.newstate = state = (action == StdMessage::ATREYU_ENTER_RECORDING_STATE ? StdMessage::RECORDING_STATE : StdMessage::MAPPING_STATE);
            airt::sendOnePartMessage(*pubsocket, &s);
        }
        break;
    }

    case StdMessage::ATREYU_RELAY_NOTIFICATION:
    {
        Message outm;
        for (size_t i = 1; i < inmsg.parts(); i++)
            outm.add_raw(inmsg.get<const void *>(i), inmsg.size(i));
        pubsocket->send(outm);
        break;
    }

    // This one is hidden from the clients!!! Only used by other modules
    case StdMessage::ATREYU_LAST_COMMAND + 1:
    {
        AtreyuStateChanged s;
        s.oldstate = state;
        s.newstate = state = StdMessage::IDLE_STATE;
        airt::sendOnePartMessage(*pubsocket, &s);
        break;
    }
    default:
        Log::critical("Unhandled message in atreyu module");
    }
}

bool AtreyuModule::startDevice()
{
    heartbeat();
    return true;
}

bool AtreyuModule::quitDevice()
{
    tasks.cancelAllScheduledTasks();
    return true;
}

void AtreyuModule::heartbeat()
{
    airt::sendNotification(*pubsocket, StdMessage::HEART_BEAT);
    if (!done)
    {
        schedule(heartBeatsPeriod_ms, [this]() {
            heartbeat();
        });
    }
}