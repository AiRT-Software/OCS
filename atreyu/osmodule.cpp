#include "globalSettings.h"
#include "osmodule.h"

#include <sstream>
#include <log.h>
#include <stdMessage.h>
#include <utils.h>

using airt::Log;
using airt::OSModule;

OSModule::OSModule(const std::string &cmdportname, const std::string &pubportname,
                   std::shared_ptr<Context> context)
    : Module(cmdportname, pubportname, "", context)
{
    assert(context);

    Log::info("OS module initialized");
}

void OSModule::sendFileError(const std::string &filename, StdMessage::OSFileError error)
{
    FileErrorNotification n;
    n.error = error;

    Message m;
    m.add_raw(&n, sizeof(FileErrorNotification));
    m.add_raw(filename.c_str(), filename.size() + 1);
    pubsocket->send(m);
}

void OSModule::sendStreamError(const std::string &filename, const std::ios &stream)
{
    // Actually the stream could have more than one of the conditions, but we have put the most importants first
    // See https://gehrcke.de/2011/06/reading-files-in-c-using-ifstream-dealing-correctly-with-badbit-failbit-eofbit-and-perror/
    if (stream.bad())
    {
        sendFileError(filename, StdMessage::OS_FILE_IO_ERROR);
    }
    else if (stream.eof())
    {
        sendFileError(filename, StdMessage::OS_FILE_END_OF_FILE);
    }
    else if (stream.fail())
    {
        sendFileError(filename, StdMessage::OS_FILE_LOGICAL_ERROR);
    }
}

void OSModule::onMessage(const Message &inmsg)
{
    Message outmsg;
    auto action = airt::getMessageAction(inmsg);
    switch (action)
    {
    case StdMessage::OS_QUERY_AVAILABLE_DISKSPACE:
    {
        AvailableDiskSpaceNotification m;
        uintmax_t avail, capacity;
        airt::diskSpace(".", capacity, avail);
        m.availableBytes = avail;
        m.capacityBytes = capacity;
        outmsg.add_raw(&m, sizeof(AvailableDiskSpaceNotification));
        pubsocket->send(outmsg);
        break;
    }
    case StdMessage::OS_QUERY_SYSTEM_INFO:
    {
        SystemInfoNotification s;
        airt::systemInfo(s.si);
        outmsg.add_raw(&s, sizeof(SystemInfoNotification));
        pubsocket->send(outmsg);
        break;
    }
    case StdMessage::OS_QUERY_NETWORK_STATS:
        handleNetworkStats();
        break;
    case StdMessage::OS_ACCEPT_BLOB:
    {
        auto inblob = inmsg.get<const AcceptBlobCmd *>(0);
        Log::info("Request to accept a blob of {} bytes", inmsg.size(1));

        BlobReceivedNotification blob;
        blob.id = inblob->id;
        airt::sendOnePartMessage(*pubsocket, &blob);
        break;
    }
    case StdMessage::OS_REQUEST_BLOB:
    {
        auto requestedBlob = inmsg.get<const RequestBlobCmd *>(0);
        Log::info("Received request for blob of {} bytes", requestedBlob->size);

        BlobSentNotification blob;
        blob.id = requestedBlob->id;

        std::vector<uint8_t> trash;
        trash.resize(requestedBlob->size);
        airt::sendTwoPartMessage(*pubsocket, &blob, &trash[0], requestedBlob->size);
        break;
    }

    case StdMessage::OS_DELETE_FILE:
    case StdMessage::OS_CREATE_FILE:
    case StdMessage::OS_APPEND_DATA_FILE:
    case StdMessage::OS_REQUEST_FILE_CONTENT:
    case StdMessage::OS_REQUEST_FILE_SIZE:
    {
        std::string filename(inmsg.get<const char *>(1));
        if (filename.size() == 0 || filename.size() > 100)
        {
            sendFileError(filename, StdMessage::OS_FILE_INVALID_NAME);
            Log::critical("Error creating a file named {}", filename);
            break;
        }
        handleFileRequests(inmsg, filename);
        break;
    }
    default:
        Log::critical("Unhandled message in os module");
    }
}

void OSModule::handleFileRequests(const Message &inmsg, const std::string &filename)
{
    auto action = airt::getMessageAction(inmsg);
    if (action != StdMessage::OS_CREATE_FILE &&
        action != StdMessage::OS_APPEND_DATA_FILE &&
        !fileExists(filename))
    {
        Log::error("File not found {}", filename);
        sendFileError(filename, StdMessage::OS_FILE_NOT_FOUND);
        return;
    }

    Message outmsg;

    switch (action)
    {
    case StdMessage::OS_DELETE_FILE:
        if (airt::deleteFile(filename))
        {
            Log::info("File {} deleted", filename);
            AIRT_Message_Header header(StdMessage::OS_NOTIFICATIONS_MODULE, StdMessage::OS_FILE_DELETED_NOTIFICATION);
            airt::sendTwoPartMessage(*pubsocket, &header, filename);
        }
        else
        {
            sendFileError(filename, StdMessage::OS_FILE_DELETE_FAILED);
            Log::error("Error deleting file {}", filename);
        }
        return;

    case StdMessage::OS_CREATE_FILE:
    {
        std::ofstream newfile(filename, std::ofstream::binary | std::ofstream::trunc);
        if (newfile.is_open())
        {
            Log::info("File {} created", filename);
            AIRT_Message_Header header(StdMessage::OS_NOTIFICATIONS_MODULE, StdMessage::OS_FILE_CREATED_NOTIFICATION);
            airt::sendTwoPartMessage(*pubsocket, &header, filename);
        }
        else
        {
            sendFileError(filename, StdMessage::OS_FILE_CREATION_FAILED);
            Log::error("Error creating file {}", filename);
        }
        return;
    }
    case StdMessage::OS_APPEND_DATA_FILE:
    {
        std::ofstream newfile(filename, std::ofstream::app | std::ofstream::binary);
        if (newfile.is_open())
        {
            Log::info("Opened file {} for append", filename);
            newfile.write(inmsg.get<const char *>(2), inmsg.size(2));
            if (!newfile.good())
            {
                Log::error("Error writing to {}", filename);
                sendStreamError(filename, newfile);
            } else {
                FileChunkWrittenNotification n;
                n.size = inmsg.size(2);
                airt::sendTwoPartMessage(*pubsocket, &n, filename);
            }
        }
        else
        {
            sendFileError(filename, StdMessage::OS_FILE_OPEN_FAILED);
            Log::error("Error opening the file {} for append", filename);
        }
        return;
    }
    case StdMessage::OS_REQUEST_FILE_CONTENT:
    {
        uint64_t filesize;
        airt::fileSize(filename, filesize);
        const RequestFileContentCmd *cmd = inmsg.get<const RequestFileContentCmd *>(0);
        if (cmd->offset > filesize)
        {
            Log::error("Trying to read beyond an EOF {}", filename);
            sendFileError(filename, StdMessage::OS_FILE_OUT_OF_BOUNDS);
            return;
        }
        uint64_t adjustedSize = cmd->size;
        if (cmd->offset + cmd->size > filesize)
        {
            adjustedSize = filesize - cmd->offset;
        }
        std::ifstream newfile(filename, std::ofstream::binary);
        if (newfile.is_open())
        {
            Log::info("Opened file {} for reading", filename);
            newfile.seekg(cmd->offset);
            std::unique_ptr<char[]> buffer(new (std::nothrow) char[adjustedSize]);
            if (!buffer)
            {
                Log::critical("Not enough memory for creating a buffer");
                airt::sendNotification(*pubsocket, StdMessage::OS_NOTIFICATIONS_MODULE, StdMessage::OS_INSUFFICIENT_MEMORY_NOTIFICATION);
                return;
            }
            newfile.read(buffer.get(), adjustedSize);
            if (!newfile.good())
            {
                Log::error("Error reading file {}", filename);
                sendStreamError(filename, newfile);
            }
            FileContentNotification n;
            n.offset = cmd->offset;
            n.size = adjustedSize;
            airt::sendThreePartMessage(*pubsocket, &n, filename, buffer.get(), adjustedSize);
        }
        else
        {
            sendFileError(filename, StdMessage::OS_FILE_OPEN_FAILED);
            Log::error("Error opening the file {} for reading", filename);
        }
        return;
    }
    case StdMessage::OS_REQUEST_FILE_SIZE:
    {
        FileSizeNotification n;
        airt::fileSize(filename, n.size);
        airt::sendTwoPartMessage(*pubsocket, &n, filename);
        return;
    }
    }
}


static const std::string network_stats_prefix {"/sys/class/net"};


void OSModule::handleNetworkStats() {
    auto ifs = airt::listFiles(network_stats_prefix, false, [](const std::string &) { return true; });
    auto msg = StdMessage(StdMessage::OS_NOTIFICATIONS_MODULE, StdMessage::OS_NETWORK_STATS_NOTIFICATION).toMsg();
    for (const auto &fname : ifs) {
        auto name = airt::getFilenameFromPath(fname);
        msg.add_raw(name.c_str(), name.size() + 1);
        auto ifstats = getInterfaceStats(fname);
        msg.add_raw(&ifstats, sizeof(OSModule::InterfaceStats));
    }
    pubsocket->send(msg);
}

OSModule::InterfaceStats OSModule::getInterfaceStats(const std::string &iface) {
    OSModule::InterfaceStats stats;
    
    stats.rx_bytes = airt::readIntFromFile(iface + "/statistics/rx_bytes").value_or(0);
    stats.rx_packets = airt::readIntFromFile(iface + "/statistics/rx_packets").value_or(0);
    stats.rx_errors = airt::readIntFromFile(iface + "/statistics/rx_errors").value_or(0);

    stats.tx_bytes = airt::readIntFromFile(iface + "/statistics/tx_bytes").value_or(0);
    stats.tx_packets = airt::readIntFromFile(iface + "/statistics/tx_packets").value_or(0);
    stats.tx_errors = airt::readIntFromFile(iface + "/statistics/tx_errors").value_or(0);

    return stats;
}
