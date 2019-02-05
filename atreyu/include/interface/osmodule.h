
#pragma pack(push, 1)
    // ACTIONS
    // Non payload actions are built using a single AiRT command packet:
    // { 'A', OS_MODULE, <ACTION>}
    // where <ACTION> can be any of the \sa StdMessage::OSCommandType that do not require a parameter

    // For other actions:

    // Creating a file (2-part message):
    // { 'A', OS_MODULE, OS_CREATE_FILE} {byte sequence with the filename (zero-terminated), better if in ASCII}

    // Appending data to the end of a file (3-part message):
    // {'A', OS_MODULE, OS_APPEND_DATA_FILE} {filename, as before} {data bytes}

    // Deletes a file (2-part message):
    // {'A', OS_MODULE, OS_DELETE_FILE} {filename, as before}

    // Request a chunk of a file (2-part message) (if there are less bytes available than requested, it just returns those)
    // {'A', OS_MODULE, OS_REQUEST_FILE_CONTENT, uint64_t offset, uint64_t size} {filename, as before}
    struct RequestFileContentCmd {
        const AIRT_Message_Header header{StdMessage::OS_MODULE, StdMessage::OS_REQUEST_FILE_CONTENT};
        uint64_t offset, size;
    };

    // Request the size of a file in the server (2-part message):
    // {'A', OS_MODULE, OS_REQUEST_FILE_SIZE} {filename, as before}


    // Ask atreyu to accept a blob (used for bandwidth testing) 2-part message. The second part contains the payload, that will be ignored
    struct AcceptBlobCmd {
        const AIRT_Message_Header header{StdMessage::OS_MODULE, StdMessage::OS_ACCEPT_BLOB};
        uint8_t space; // not used
        uint64_t id; // used for acknowledgement
    };

    // Ask atreyu to make up a message of the given size and send it over (used for bandwith measures). 
    struct RequestBlobCmd {
        const AIRT_Message_Header header{StdMessage::OS_MODULE, StdMessage::OS_REQUEST_BLOB};
        uint8_t space; // not used
        uint64_t id; // used for acknowledgement
        uint64_t size; // size
    };



    // NOTIFICATIONS

    // Two-part message.
    struct FileCreatedNotification {
        const AIRT_Message_Header header{StdMessage::OS_NOTIFICATIONS_MODULE, StdMessage::OS_FILE_CREATED_NOTIFICATION};
    };
    // The second part has the filename in ascii-z

    // Two-part message
    struct FileChunkWrittenNotification {
        const AIRT_Message_Header header{StdMessage::OS_NOTIFICATIONS_MODULE, StdMessage::OS_FILE_CHUNK_WRITTEN_NOTIFICATION};
        size_t size;    // Number of bytes written
    };
    // The second part has the filename in ascii-z
    

    struct AvailableDiskSpaceNotification
    {
        const AIRT_Message_Header header{StdMessage::OS_NOTIFICATIONS_MODULE, StdMessage::OS_AVAILABLE_DISKSPACE_NOTIFICATION};
        uint64_t availableBytes; // free space
        uint64_t capacityBytes;  // free space + used space
    };

    struct SystemInfoNotification
    {
        const AIRT_Message_Header header{StdMessage::OS_NOTIFICATIONS_MODULE, StdMessage::OS_SYSTEM_INFO_NOTIFICATION};
        SystemInfo si;
    };

    // This is a three-part message. The second part contains the filename (in ASCIIZ). The third part has 'size' bytes, 
    // with the contents of the file
    struct FileContentNotification 
    {
        const AIRT_Message_Header header{StdMessage::OS_NOTIFICATIONS_MODULE, StdMessage::OS_FILE_CONTENT_NOTIFICATION};
        uint64_t offset;  // offset of the first byte within the file
        uint64_t size;      // number of bytes provided
    };

    // This is a two-part message. The second part contains the filename (in ASCIIZ). 
    struct FileSizeNotification
    {
        const AIRT_Message_Header header{StdMessage::OS_NOTIFICATIONS_MODULE, StdMessage::OS_FILE_SIZE_NOTIFICATION};
        uint64_t size;
    };

    // This is a two-part message. The second part contains the filename (in ASCIIZ).
    struct FileErrorNotification 
    {
        const AIRT_Message_Header header{StdMessage::OS_NOTIFICATIONS_MODULE, StdMessage::OS_FILE_ERROR_NOTIFICATION};
        StdMessage::OSFileError error;
    };
    
    // This is a 2n + 1 part message, where n is the number of network interfaces in the system. The part 2i+1 contains the ascii-z name of the 
    // i-th network interface (lo, wlp58s0, eth0...), starting from zero, and the part 2(i+1) has its stats
    struct NetworkStatsHeader 
    {
        const AIRT_Message_Header header{StdMessage::OS_NOTIFICATIONS_MODULE, StdMessage::OS_NETWORK_STATS_NOTIFICATION};
    };

    struct InterfaceStats // this is taken from /sys/class/net/<dev>/statistics/
    {
        uint64_t rx_bytes; // bytes received
        uint64_t rx_packets; // packets received
        uint64_t tx_bytes;  // bytes sent
        uint64_t tx_packets;    // packets sent
        uint64_t rx_errors; 
        uint64_t tx_errors;
    };

    // Acknowledgement of the reception of a blob
    struct BlobReceivedNotification {
        const AIRT_Message_Header header{StdMessage::OS_NOTIFICATIONS_MODULE, StdMessage::OS_BLOB_RECEIVED_NOTIFICATION};
        uint8_t space;
        uint64_t id;    // id of the received blob
    };

    // This is the requested blob. 2-part message. The second part will have random data
    struct BlobSentNotification {
        const AIRT_Message_Header header{StdMessage::OS_NOTIFICATIONS_MODULE, StdMessage::OS_BLOB_SENT_NOTIFICATION};
        uint8_t space;
        uint64_t id;    // id of the requested blob
    };


inline void toCSV(std::ostream &os, const AvailableDiskSpaceNotification &n)
{
    os << n.header.module << ";" << n.header.action << ";" << n.availableBytes << ";" << n.capacityBytes;
}
inline void toCSV(std::ostream &os, const SystemInfoNotification &n)
{
    os << n.header.module << ";" << n.header.action << ";";
    airt::toCSV(os, n.si);
}
#pragma pack(pop)