#include "net/proxysteerable.h"
#include "net/socket.h"

#include <zmqpp.hpp>


using airt::Socket;
using airt::ProxySteerable;

ProxySteerable::ProxySteerable(Socket &recv, Socket &send, Socket &ctrl) : 
    proxy(std::unique_ptr<zmqpp::proxy_steerable>(new zmqpp::proxy_steerable(*recv.sock, *send.sock, *ctrl.sock)))
{
}

ProxySteerable::~ProxySteerable() {

};