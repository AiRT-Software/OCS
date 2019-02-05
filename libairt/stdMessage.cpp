#include "stdMessage.h"
#include <sstream>

using airt::Message;
using airt::StdMessage;
using airt::AIRT_Message_Header;

std::string to_string(const Message &m);
std::string to_string(const StdMessage &m);

// std::string airt::debugString(const uint8_t *bytes, size_t size)
// {
//     std::ostringstream ss;
//     for (size_t i = 0; i < size; i++)
//     {
//         if (bytes[i] >= ' ' && bytes[i] < 127)
//         {
//             ss << bytes[i];
//         }
//         else
//             ss << "<" << (uint)bytes[i] << ">";
//     }
//     return ss.str();
// }

std::string airt::to_string(const Message &m)
{
    std::ostringstream ss;
    auto header = m.get<const AIRT_Message_Header *>(0);
    ss << "msg{" << header->airt << ", " << (int)header->module << ", " << (int)header->action << "}";
    return ss.str();
}

std::string airt::to_string(const StdMessage &m)
{
    std::ostringstream ss;
    ss << "msg{" << m.airt << ", " << (int)m.module << ", " << (int)m.action << "}";
    return ss.str();
    
}
