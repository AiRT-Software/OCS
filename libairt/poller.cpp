#include "net/poller.h"
#include "net/socket.h"

#include <zmqpp.hpp>


using airt::Poller;
using airt::Socket;

Poller::Poller() {
    poller = std::unique_ptr<zmqpp::poller>(new zmqpp::poller());
}

Poller::~Poller() {
    
}

void Poller::add(Socket& socket, const short event) {
    short opts = 0;

    opts |= ((event & POLL_IN) ? zmqpp::poller::poll_in : 0);
    opts |= ((event & POLL_OUT) ? zmqpp::poller::poll_out : 0);
    opts |= ((event & POLL_ERROR) ? zmqpp::poller::poll_error : 0);
    
    poller->add(*socket.sock, opts);
}

bool Poller::poll(long timeout_ms)
{
    return poller->poll(timeout_ms);
}