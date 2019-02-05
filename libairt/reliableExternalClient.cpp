#include "net/socket.h"
#include "net/poller.h"
#include "reliableExternalClient.h"
#include "log.h"
#include "stdMessage.h"

using airt::ReliableExternalClient;
using airt::Log;
using airt::Poller;
using airt::Context;
using airt::Message;

ReliableExternalClient::ReliableExternalClient(std::shared_ptr<Context> context, const std::string &address, size_t timeout, size_t retries)
 : context(context), srvAddress(address), timeout(timeout), retries(retries)
 {};

void ReliableExternalClient::reconnect()
{
    Log::info("Connecting to server {}", srvAddress);
    socket = std::unique_ptr<Socket>(new Socket(*context, Socket::SocketType::request));
    socket->connect(srvAddress);

    //  Configure socket to not wait at close time
    socket->setLinger(0);
}

bool ReliableExternalClient::sendAndReceive(Message &request, Message &response)
{
    if (!socket)
        reconnect();

    for (size_t retries_left = 0; retries_left < retries; retries_left++)
    {
        Log::info("Sending request {}...", airt::to_string(request));
        Message copy = request.copy();

        if (!socket->send(copy))
            return false;

        Poller poller;
        poller.add(*socket);

        if (poller.poll(timeout)) {
            return socket->receive(response, true);
        }

        Log::warn("No response from server {}, retrying", srvAddress);
        reconnect();
    }
    Log::error("Server {} seems to be offline, abandoning", srvAddress);
    return false;
}

bool ReliableExternalClient::sendAndReceive(const void *data, size_t size, Message &response)
{
    Message req;
    req.add_raw(data, size);
    return sendAndReceive(req, response);
}
