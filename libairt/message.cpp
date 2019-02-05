#include "net/message.h"

#include <zmqpp.hpp>

using airt::Message;

Message::Message()
{
    msg = std::unique_ptr<zmqpp::message>(new zmqpp::message());
}

Message::Message(Message &&o)
{
    msg = std::move(o.msg);
}

Message::~Message()
{
}

size_t Message::size(size_t const part) const
{
    return msg->size(part);
}

Message Message::copy() {
    Message c;
    *c.msg = msg->copy();
    return c;
}