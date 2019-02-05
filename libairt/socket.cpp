#include "net/socket.h"

#include <zmqpp.hpp>

using airt::Socket;
using airt::Context;
using airt::Message;

Socket::Socket(Context &ctx, SocketType type)
{
    zmqpp::socket_type st;
    switch (type)
    {
    case SocketType::pair:
        st = zmqpp::socket_type::pair;
        break;
    case SocketType::publish:
        st = zmqpp::socket_type::publish;
        break;
    case SocketType::subscribe:
        st = zmqpp::socket_type::subscribe;
        break;
    case SocketType::pull:
        st = zmqpp::socket_type::pull;
        break;
    case SocketType::push:
        st = zmqpp::socket_type::push;
        break;
    case SocketType::request:
        st = zmqpp::socket_type::request;
        break;
    case SocketType::reply:
        st = zmqpp::socket_type::reply;
        break;
    case SocketType::xpublish:
        st = zmqpp::socket_type::xpublish;
        break;
    case SocketType::xsubscribe:
        st = zmqpp::socket_type::xsubscribe;
        break;
    case SocketType::xrequest:
        st = zmqpp::socket_type::xrequest;
        break;
    case SocketType::xreply:
        st = zmqpp::socket_type::xreply;
        break;
    case SocketType::stream:
        st = zmqpp::socket_type::stream;
        break;
    default:
        throw(new std::runtime_error("Unknown socket type"));
    }
    sock = std::unique_ptr<zmqpp::socket>(new zmqpp::socket(*ctx.context, st));
}

Socket::~Socket()
{
}

bool Socket::send(Message &message, bool const dont_block)
{
    return sock->send(*message.msg, dont_block);
}

bool Socket::receive(Message &message, bool const dont_block)
{
    return sock->receive(*message.msg, dont_block);
}

void Socket::bind(const std::string &endpoint)
{
    sock->bind(endpoint);
}

void Socket::connect(const std::string &endpoint)
{
    sock->connect(endpoint);
}

void Socket::setLinger(int linger)
{

    sock->set(zmqpp::socket_option::linger, 0);
}

void Socket::subscribe(const uint8_t *header, size_t size)
{
    sock->set(zmqpp::socket_option::subscribe, reinterpret_cast<const char *>(header), size);
}

void Socket::subscribeAll()
{
    subscribe(nullptr, 0);
}

void Socket::unsubscribe(const uint8_t *header, size_t size)
{
    sock->set(zmqpp::socket_option::unsubscribe, reinterpret_cast<const char *>(header), size);
}

void Socket::unsubscribeAll()
{
    unsubscribe(nullptr, 0);
}

Socket::operator bool() const
{
    return sock->operator bool();
}
