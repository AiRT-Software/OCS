#include "log.h"
#include "stdMessage.h"
#include "module.h"
#include "net/message.h"
#include "net/socket.h"
#include "net/reactor.h"

using airt::Context;
using airt::Log;
using airt::Message;
using airt::Module;
using airt::Reactor;
using airt::Socket;

using airt::StdMessage;

Module::Module(const std::string &cmdport, const std::string &pubport, const std::string &subport,
               std::shared_ptr<Context> context) : commandport(cmdport),
                                                   publishport(pubport),
                                                   subscribeport(subport),
                                                   deviceOn(false),
                                                   done(false)
{
    if (!context)
        this->context = std::make_shared<Context>();
    else
        this->context = context;
    cmdsocket = std::unique_ptr<Socket>(new Socket(*(this->context), Socket::SocketType::pair));
    cmdsocket->bind(commandport);

    pubsocket = std::unique_ptr<Socket>(new Socket(*(this->context), Socket::SocketType::publish));
    pubsocket->bind(publishport);

    if (subscribeport.empty())
    {
        thread = std::thread(std::bind(&Module::run<false>, this));
    }
    else
    {
        subsocket = std::unique_ptr<Socket>(new Socket(*(this->context), Socket::SocketType::subscribe));
        subsocket->bind(subscribeport);

        thread = std::thread(std::bind(&Module::run<true>, this));
    }
}

void Module::processCmd()
{
    Message m;
    if (!cmdsocket->receive(m, true))
        return;

    if (airt::isStdCommand(m))
    {
        switch (airt::getMessageAction(m))
        {
        case StdMessage::QUIT:
            done = quitDevice();
            break;
        case StdMessage::START:
            deviceOn = startDevice();
            break;
        case StdMessage::STOP:
            stopDevice();
            deviceOn = false;
            break;
        default:
            Log::error("Module: Unknown standard command message: {}", airt::to_string(m));
            auto um = StdNotification(StdMessage::UNKNOWN_COMMAND).toMsg();
            pubsocket->send(um);
            return;
        }
    }
    else
        this->onMessage(m);
}

void Module::processNot()
{
    Message notification;
    if (subsocket->receive(notification, true))
    {
        onNotification(notification);
    }
}

template <bool hassubscriber>
void Module::run()
{
    if (hassubscriber)
    {
        Log::info("Opening the ports {} (cmd), {} (pub) and {} (sub)", commandport, publishport, subscribeport);
    }
    else
    {
        Log::info("Opening the ports {} (cmd) and {} (pub)", commandport, publishport);
    }

    Reactor reactor;
    if (hassubscriber)
        reactor.add(*subsocket, std::bind(&Module::processNot, this));
    reactor.add(*cmdsocket, std::bind(&Module::processCmd, this));

    while (true)
    {
        bool idle = !reactor.poll(10);
        if (done)
            break;

        // Is the device on and there is a data ready?
        if (deviceOn && dataReady())
        {
            idle = false;
            sendMessage();
        }

        if (tasks.run())
            idle = false;

        if (idle)
        {
            onIdle();
        }
    }

    Log::info("Module in port {} shutting down", commandport);
}

void Module::shutdown()
{
    stopDevice();
    done = true;
    thread.join();
}
