#include "net/reactor.h"
#include "net/socket.h"

#include <zmqpp.hpp>

using airt::Reactor;
using airt::Socket;

Reactor::Reactor()
{
    reactor = std::unique_ptr<zmqpp::reactor>(new zmqpp::reactor());
}

Reactor::~Reactor()
{
}

bool Reactor::poll(long timeout_ms)
{
    return reactor->poll(timeout_ms);
}

void Reactor::add(Socket &socket, std::function<void(void)> callback, const short event)
{
    short opts = 0;

    opts |= ((event & POLL_IN) ? zmqpp::poller::poll_in : 0);
    opts |= ((event & POLL_OUT) ? zmqpp::poller::poll_out : 0);
    opts |= ((event & POLL_ERROR) ? zmqpp::poller::poll_error : 0);

    reactor->add(*socket.sock, callback, opts);
}
