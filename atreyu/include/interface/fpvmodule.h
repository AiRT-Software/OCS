#pragma pack(push, 1)

struct FPVResolutionNotification
{
    const AIRT_Message_Header header{StdMessage::FPV_NOTIFICATIONS_MODULE, StdMessage::FPV_RESOLUTION_NOTIFICATION};
    int width, height;
};

// 1st part
struct FPVImageJpegHeader
{
    const AIRT_Message_Header header{StdMessage::FPV_NOTIFICATIONS_MODULE, StdMessage::FPV_IMAGE_JPEG_NOTIFICATION};
    int frameNumber;
};

// 2nd part: it will be a unsigned char buffer

#pragma pack(pop)
