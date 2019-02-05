#include <log.h>
#include <stdMessage.h>
#include <utils.h>

#include <net/socket.h>
#include <net/reactor.h>
#include <forwarder.h>
#include "server.h"
#include "globalSettings.h"
#include "pozyxModule.h"
#include "pozyxDummyModule.h"
#include "pclfileplayer.h"
#include "osmodule.h"
#include "atreyumodule.h"
#include "fcsmodule.h"
#include "fcsdummymodule.h"
#include "rcammodule.h"
#include "gimbalmodule.h"
#include "planlibrarianmodule.h"
#include "planexecutormodule.h"
#include "d415module.h"
#include "fpvmodule.h"
#include "dronemodule.h"
#include "mappermodule.h"

using airt::AtreyuModule;
using airt::DroneModule;
using airt::FCSDummyModule;
using airt::FCSModule;
using airt::FPVModule;
using airt::GimbalModule;
using airt::GlobalSettings;
using airt::MapperModule;
using airt::Message;
using airt::OSModule;
using airt::PCLFilePlayer;
using airt::PlanExecutorModule;
using airt::PlanLibrarianModule;
using airt::PozyxModule;
using airt::RCamModule;
using airt::Server;
using airt::StdMessage;

Server::Server() : context(std::make_shared<Context>()),
                   requests(*context, Socket::SocketType::reply),
                   done(false)
{
    int loglevel;
    if (GlobalSettings::getValue("loglevel", loglevel))
    {
        Log::set_level(static_cast<Log::LogLevel>(loglevel));
    }

    Log::info("Atreyu starting...");
    config();
}

Server::~Server()
{
    Log::info("Atreyu ending...");
    shutdown();
    forwarder->shutdown();
}

template <typename Device>
bool Server::installModule(const std::string &cmdportname, const std::string &pubportname, const std::string &subportname, StdMessage::Modules commandModule)
{
    auto module = std::make_shared<Device>(cmdportname, pubportname, subportname, context);
    auto result = connectModule(module, cmdportname, pubportname, commandModule);
    if (result)
    {
        module->configureSubscriptions(forwarder->getEmittingPort());
    }
    return result;
}

template <typename Device>
bool Server::installModule(const std::string &cmdportname, const std::string &pubportname, StdMessage::Modules commandModule)
{
    auto module = std::make_shared<Device>(cmdportname, pubportname, context);
    return connectModule(module, cmdportname, pubportname, commandModule);
}

bool Server::connectModule(std::shared_ptr<Module> module, const std::string &cmdportname, const std::string &pubportname, StdMessage::Modules commandModule)
{
    auto socket = std::make_shared<Socket>(*context, Socket::SocketType::pair);
    socket->connect(cmdportname);
    if (!(*socket))
    {
        Log::error("Error connecting to module at {}", cmdportname);
        return false;
    }
    reactor.add(*socket, std::bind(&Server::receiveAndPublishMessage, this, socket));
    moduleconnections.emplace(commandModule, ModuleConnection(socket, module));

    forwarder->getReceivingSocket().connect(pubportname);
    Log::info("Successful connecting to module at {}", cmdportname);
    return true;
}

void Server::config()
{
    // Configuring the publishing socket
    std::string portname;
    if (GlobalSettings::getValue("publisherportname", portname))
    {
        Log::info("Binding the publisher to port {}", portname);
        //publisher.bind(portname);
        forwarder = std::unique_ptr<airt::Forwarder>(new airt::Forwarder("inproc://forwarder.ipc", portname, context));
    }
    else
    {
        Log::critical("Publisher port undefined. Check the configuration file");
        exit(1);
    }

    // Atreyu module: manages requests about the server
    if (!installModule<AtreyuModule>("inproc://atreyucmd.ipc", "inproc://atreyupub.ipc", StdMessage::ATREYU_MODULE))
        exit(1);

    // Atreyu module: manages requests about the server
    portname = "inproc://drone";
    if (!installModule<DroneModule>(portname + "cmd.ipc", portname + "pub.ipc", portname + "sub.ipc", StdMessage::DRONE_MODULE))
        exit(1);

    // Plan librarian Module: manages the flight plan library in the server
    if (!installModule<PlanLibrarianModule>("inproc://planlibcmd.ipc", "inproc://planlibpub.ipc", StdMessage::PLAN_LIBRARIAN_MODULE))
        exit(1);

    // OS Module: manages requests of information about the OS
    if (!installModule<OSModule>("inproc://oscmd.ipc", "inproc://ospub.ipc", StdMessage::OS_MODULE))
        exit(1);

    if (GlobalSettings::getValue("d415cam_portname_base", portname))
    {
#ifdef _DEBUG
        rs2::log_to_console(RS2_LOG_SEVERITY_WARN);
#else
        rs2::log_to_file(RS2_LOG_SEVERITY_WARN, "librealsense2.log");
#endif
        if (!installModule<D415Module>(portname + "cmd.ipc", portname + "pub.ipc", portname + "sub.ipc", StdMessage::POINTCLOUD_MODULE))
            exit(1);
    }

    if (GlobalSettings::getValue("fpv_portname_base", portname))
    {
#ifdef _DEBUG
        rs2::log_to_console(RS2_LOG_SEVERITY_WARN);
#else
        rs2::log_to_file(RS2_LOG_SEVERITY_WARN, "librealsense2.log");
#endif
        if (!installModule<FPVModule>(portname + "cmd.ipc", portname + "pub.ipc", StdMessage::FPV_MODULE))
            exit(1);
    }

    if (GlobalSettings::getValue("pclreplay_portname_base", portname))
    {
        auto cam = std::make_shared<PCLFilePlayer>(portname + "cmd.ipc", portname + "pub.ipc", context);
        std::string basename;
        double replayFreq;
        if (GlobalSettings::getValue("pclreplayfilename", basename) && GlobalSettings::getValue("pclreplayfrequency", replayFreq))
        {
            Log::info("PCL files will be replayed from {}", basename);
            cam->setBaseName(basename);
            cam->setReplayFrequency(replayFreq);
        }
        else
        {
            Log::critical("Wrong config of pclreplayer");
            exit(1);
        }
        if (!connectModule(cam, portname + "cmd.ipc", portname + "pub.ipc", StdMessage::POINTCLOUD_MODULE))
            exit(1);
    }

    if (GlobalSettings::getValue("pozyx_portname_base", portname))
    {
        Log::info("Pozyx Module (real) installed");
        if (!installModule<PozyxModule>(portname + "cmd.ipc", portname + "pub.ipc", StdMessage::POSITIONING_MODULE))
            exit(1);
    }
    else
    {
        Log::info("Pozyx Module (dummy) installed");
        portname = "inproc://pozyxdum";
        if (!installModule<PozyxDummyModule>(portname + "cmd.ipc", portname + "pub.ipc", portname + "sub.ipc", StdMessage::POSITIONING_MODULE))
            exit(1);
    }

    if (GlobalSettings::getValue("fcs_portname_base", portname))
    {
        Log::info("FCS Module (real) installed");
        if (!installModule<FCSModule>(portname + "cmd.ipc", portname + "pub.ipc", StdMessage::FCS_MULTIPLEXER_MODULE))
            exit(1);
    }
    else
    {
        Log::info("FCS Module (dummy) installed");
        portname = "inproc://fcs";
        if (!installModule<FCSDummyModule>(portname + "cmd.ipc", portname + "pub.ipc", portname + "sub.ipc", StdMessage::FCS_MULTIPLEXER_MODULE))
            exit(1);
    }

    if (GlobalSettings::getValue("gimbal_portname_base", portname))
    {
        if (!installModule<GimbalModule>(portname + "cmd.ipc", portname + "pub.ipc", StdMessage::GIMBAL_MULTIPLEXER_MODULE))
            exit(1);
    }

    if (GlobalSettings::getValue("rcam_portname_base", portname))
    {
        if (!installModule<RCamModule>(portname + "cmd.ipc", portname + "pub.ipc", StdMessage::RCAM_MODULE))
            exit(1);
    }

    // Plan executor Module: takes a flight plan and executes it via the fcs module
    if (!installModule<PlanExecutorModule>("inproc://planexeccmd.ipc", "inproc://planexecpub.ipc", "inproc://planexecsub.ipc", StdMessage::PLAN_EXECUTOR_MODULE))
        exit(1);

    // Mapper Module: maps the environment
    portname = "inproc://mapper";
    if (!installModule<MapperModule>(portname + "cmd.ipc", portname + "pub.ipc", portname + "sub.ipc", StdMessage::MAPPER_MODULE))
        exit(1);

    // Opening the request/response socket
    if (GlobalSettings::getValue("commandportname", portname))
    {
        requests.bind(portname);
        reactor.add(requests, std::bind(&Server::processReq, this));
    }
    else
    {
        Log::critical("The configuration file does not specify the command port name (commandportname)");
        throw(std::runtime_error("Command port name not specified"));
    }

    std::ostringstream os;
    os << "Installed " << moduleconnections.size() << " modules: ";
    for (const auto &c : moduleconnections)
    {
        os << static_cast<int>(c.first) << " ";
    }
    Log::info(os.str());

    Log::info("Starting all the modules.");
    onStart();
}

void Server::processReq()
{
    Message msg;
    if (!requests.receive(msg, true))
        return;

    // identify the source module
    uint8_t module = airt::getMessageModule(msg);
    if (module >= 128)
    {
        // notifications-modules are not allowed to be received as request
        Log::error("Requests received with a notification code: {}. Ignoring", module);
        airt::sendNotification(requests, StdMessage::UNKNOWN_COMMAND);
        return;
    }

    // identify the action
    uint8_t action = airt::getMessageAction(msg);

    // Is the command adressed to the whole system?
    if (module == StdMessage::STD_COMMANDS_MODULE)
    {
        Log::info("Received STD command {}", airt::to_string(msg));
        switch (action)
        {
        case StdMessage::START:
            onStart();                                                                   // address the command to all modules
            airt::sendNotification(requests, StdMessage::ACK);                           // send confirmation to the client who asked
            airt::sendNotification(forwarder->getEmittingSocket(), StdMessage::STARTED); // publish to other subscribers
            break;
        case StdMessage::STOP:
            onStop();
            airt::sendNotification(requests, StdMessage::ACK);
            airt::sendNotification(forwarder->getEmittingSocket(), StdMessage::STOPPED);
            break;
        case StdMessage::QUIT:
            onQuit();
            airt::sendNotification(requests, StdMessage::ACK);
            airt::sendNotification(forwarder->getEmittingSocket(), StdMessage::QUITTING);
            break;
        case StdMessage::POWEROFF:
            onPowerOff();
            airt::sendNotification(requests, StdMessage::ACK);
            airt::sendNotification(forwarder->getEmittingSocket(), StdMessage::POWERING_OFF);
            break;
        default:
            airt::sendNotification(requests, StdMessage::UNKNOWN_COMMAND);
            break;
        }
        return;
    }

    dispatchMessageToModules(msg);
}

void Server::dispatchMessageToModules(Message &msg)
{
    // Command for a specific module
    auto module = airt::getMessageModule(msg);

    std::shared_ptr<Socket> thesocket;

    auto source = moduleconnections.find(module);
    if (source != moduleconnections.end())
        thesocket = source->second.localSocket;
    else
    {
        Log::error("Command adressed to undefined module: {}. Discarding...", airt::to_string(msg));
        airt::sendNotification(requests, StdMessage::UNDEFINED_MODULE);
        return;
    }

    auto action = airt::getMessageAction(msg);
    // Common action addressed to an specific module
    if (action == StdMessage::START || action == StdMessage::STOP || action == StdMessage::QUIT)
    {
        airt::sendNotification(requests, StdMessage::ACK);                           //confirmation
        airt::sendCommand(*thesocket, static_cast<StdMessage::CommandType>(action)); //redirect to internalsource
    }
    // Specific action addressed to an specific module
    else
    {
        switch (module)
        {
        case StdMessage::POINTCLOUD_MODULE:
            validateAndForward("pointcloud", msg, StdMessage::PCL_LAST_COMMAND, *thesocket);
            break;
        case StdMessage::POSITIONING_MODULE:
            validateAndForward("IPS", msg, StdMessage::IPS_LAST_COMMAND, *thesocket);
            break;
        case StdMessage::RCAM_MODULE:
            validateAndForward("rcam", msg, StdMessage::RCAM_LAST_COMMAND, *thesocket);
            break;
        case StdMessage::FCS_MULTIPLEXER_MODULE:
            validateAndForward("fcs", msg, StdMessage::FCS_LAST_COMMAND, *thesocket);
            break;
        case StdMessage::OS_MODULE:
            validateAndForward("os", msg, StdMessage::OS_LAST_COMMAND, *thesocket);
            break;
        case StdMessage::ATREYU_MODULE:
            validateAndForward("atreyu", msg, StdMessage::ATREYU_LAST_COMMAND, *thesocket);
            break;
        case StdMessage::PLAN_LIBRARIAN_MODULE:
            validateAndForward("plan lib", msg, StdMessage::PLAN_LIB_LAST_COMMAND, *thesocket);
            break;
        case StdMessage::PLAN_EXECUTOR_MODULE:
            validateAndForward("plan exec", msg, StdMessage::PLAN_EXEC_LAST_COMMAND, *thesocket);
            break;
        case StdMessage::FPV_MODULE:
            validateAndForward("fpv", msg, StdMessage::FPV_LAST_COMMAND, *thesocket);
            break;
        case StdMessage::GIMBAL_MULTIPLEXER_MODULE:
            validateAndForward("gimbal", msg, StdMessage::GIMBAL_LAST_COMMAND, *thesocket);
            break;
        case StdMessage::DRONE_MODULE:
            validateAndForward("drone", msg, StdMessage::DRONE_LAST_COMMAND, *thesocket);
            break;
        case StdMessage::MAPPER_MODULE:
            validateAndForward("mapper", msg, StdMessage::MAPPER_LAST_COMMAND, *thesocket);
            break;
        default:
            Log::critical("Uncaught undefined module");
            airt::sendNotification(requests, StdMessage::UNDEFINED_MODULE);
        }
    }
}

void Server::validateAndForward(const std::string &moduleName, Message &msg, uint8_t maxActionCode, airt::Socket &s)
{
    if (airt::getMessageAction(msg) >= maxActionCode)
    {
        Log::error("Unknown {} command received {}", moduleName, airt::to_string(msg));
        airt::sendNotification(requests, StdMessage::UNKNOWN_COMMAND);
        return;
    }
    Log::info("Received {} command {}", moduleName, airt::to_string(msg));
    airt::sendNotification(requests, StdMessage::ACK);
    s.send(msg);
}

void Server::receiveAndPublishMessage(std::shared_ptr<Socket> socket)
{
    Message m;
    if (socket->receive(m, true))
    {
        if (airt::getMessageModule(m) == StdMessage::MESSAGE_BUS_MODULE)
        {
            // This message is directed to another module
            Message out;
            for (size_t i = 1; i < m.parts(); i++)
            {
                out.add_raw(m.get<const void *>(i), m.size(i));
            }

            auto adressee = airt::getMessageModule(out);
            std::shared_ptr<Socket> thesocket;
            auto mconnit = moduleconnections.find(adressee);
            if (mconnit != moduleconnections.end())
            {
                thesocket = mconnit->second.localSocket;
                thesocket->send(out);
            }
            else
            {
                Log::error("Module {} is trying to send a command to an unknown module ({})", airt::getMessageAction(m), adressee);
            }
        }
        else
            forwarder->getEmittingSocket().send(m);
    }
    else
        Log::warn("Supposedly there was a message waiting here...");
}

void Server::onIdle()
{
    //std::cout << "T";
}

void Server::onStart()
{
    for (auto &conn : moduleconnections)
    {
        airt::sendCommand(*conn.second.localSocket, StdMessage::START);
    }
}

void Server::onStop()
{
    for (auto &conn : moduleconnections)
    {
        airt::sendCommand(*conn.second.localSocket, StdMessage::STOP);
    }
}

void Server::onQuit()
{
    shutdown();
    airt::sendNotification(forwarder->getEmittingSocket(), StdMessage::QUITTING);
}

void Server::onPowerOff()
{
    shutdown();
    airt::sendNotification(forwarder->getEmittingSocket(), StdMessage::POWERING_OFF);
}

void Server::shutdown()
{
    for (auto &conn : moduleconnections)
    {
        airt::sendCommand(*conn.second.localSocket, StdMessage::QUIT);
        conn.second.device->shutdown();
    }
    moduleconnections.clear();

    done = true;
}

void Server::run()
{
    Log::info("Atreyu configured. Waiting for clients");

    while (!done)
    {
        reactor.poll();
    }
}
