#include "log.h"
#include "forwarder.h"
#include "net/socket.h"
#include "net/proxysteerable.h"
#include "stdMessage.h"
#include "utils.h"

using airt::Context;
using airt::Forwarder;
using airt::Socket;

Forwarder::Forwarder(const std::string &receivingPort, const std::string &emittingPort,
                     std::shared_ptr<Context> context) : fwdrContext(context), emittingP(emittingPort), receivingP(receivingPort)
{
    if (!context)
        fwdrContext = std::make_shared<Context>();

    xpub = std::unique_ptr<Socket>(new Socket(*fwdrContext, Socket::SocketType::xpublish));
    xpub->bind(emittingPort);

    xsub = std::unique_ptr<Socket>(new Socket(*fwdrContext, Socket::SocketType::xsubscribe));
    xsub->bind(receivingPort);

    ctrl = std::unique_ptr<Socket>(new Socket(*fwdrContext, Socket::SocketType::pair));
    controlP = Forwarder::produceNewControlPort();
    ctrl->bind(controlP);

    if (airt::starts_with(emittingP, "tcp://*")) {
        std::string connectTo = "tcp://localhost:" + emittingP.substr(emittingP.rfind(':') + 1);
        emittingP = connectTo;
    }

    thread = std::thread(std::bind(&Forwarder::run, this));
}

Forwarder::~Forwarder()
{
}

void Forwarder::run()
{
    Log::info("Opening a proxy from {} to {} controlled by {}", receivingP, emittingP, controlP);

    proxy = std::unique_ptr<airt::ProxySteerable>(new airt::ProxySteerable(*xsub, *xpub, *ctrl));

    Log::info("Finishing a proxy from {} to {} controlled by {}", receivingP, emittingP, controlP);
}

void Forwarder::pause()
{
    Socket tmp(*fwdrContext, Socket::SocketType::pair);
    tmp.connect(controlP);

    airt::sendOnePartMessage(tmp, "PAUSE", 5);
}

void Forwarder::resume()
{
    Socket tmp(*fwdrContext, Socket::SocketType::pair);
    tmp.connect(controlP);

    airt::sendOnePartMessage(tmp, "RESUME", 6);
}

void Forwarder::shutdown()
{
    Socket tmp(*fwdrContext, Socket::SocketType::pair);
    tmp.connect(controlP);

    airt::sendOnePartMessage(tmp, "TERMINATE", 9);
    thread.join();
}

std::string Forwarder::produceNewControlPort()
{
    static int count = 0;
    return "inproc://forwarderctrl" + std::to_string(count) + ".ipc";
}