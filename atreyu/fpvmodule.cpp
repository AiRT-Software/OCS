#include "fpvmodule.h"
#include "globalSettings.h"

#include <log.h>

using airt::Log;
using airt::FPVModule;


FPVModule::FPVModule(const std::string &cmdportname, const std::string &pubportname,
                         std::shared_ptr<Context> context)
    : Module(cmdportname, pubportname, "", context)
{
    assert(context);

    std::ostringstream info;
    grabber.printRSContextInfo(info);
    Log::info("{}", info.str());
}

bool FPVModule::init()
{
    grabber.configureRSStreams(true, false);
    if(grabber.color_enabled)
    {
        Log::info("FPVModule: enabled color stream: {}x{}@{}",
                  grabber.color_w, grabber.color_h, grabber.camera_fps);
    }
    if(grabber.depth_enabled)
    {
        Log::info("FPVModule: enabled depth stream: {}x{}@{}",
                  grabber.depth_w, grabber.depth_h, grabber.camera_fps);
    }

    max_jpeg_size = grabber.color_w * grabber.color_h * 3;
    image_jpeg = std::unique_ptr<unsigned char []>(new unsigned char[max_jpeg_size]);

    currentFrame = 0;
    fpv_module_is_streaming = false;

    return true;
}

bool FPVModule::startDevice()
{
    return true;
}

void FPVModule::stopDevice()
{
}

bool FPVModule::dataReady()
{
    if(!fpv_module_is_streaming)
    {
        return false;
    }

    size_image_jpeg = max_jpeg_size;
    return grabber.getJpegImage(image_jpeg.get(), size_image_jpeg);
}

bool FPVModule::quitDevice()
{
    return true;
}

bool FPVModule::startStreaming()
{
    if(!grabber.startStreaming())
    {
        Log::error("FPVModule: start streaming has failed");
        auto st = StdMessage(StdMessage::FPV_NOTIFICATIONS_MODULE, StdMessage::FPV_LAST_COMMAND_HAS_FAILED).toMsg();
        pubsocket->send(st);

        return false;
    }
    else
    {
        Log::info("FPVModule: start streaming. Current frame: {}", currentFrame);
        auto st = StdMessage(StdMessage::FPV_NOTIFICATIONS_MODULE, StdMessage::FPV_STREAMING_STARTED).toMsg();
        pubsocket->send(st);

        fpv_module_is_streaming = true;
        return true;
    }
}

bool FPVModule::stopStreaming()
{
    if(!grabber.stopStreaming())
    {
        Log::error("FPVModule: stop streaming has failed");
        auto st = StdMessage(StdMessage::FPV_NOTIFICATIONS_MODULE, StdMessage::FPV_LAST_COMMAND_HAS_FAILED).toMsg();
        pubsocket->send(st);

        return false;
    }
    else
    {
        Log::info("FPVModule: stop streaming. Current frame: {}", currentFrame);
        auto st = StdMessage(StdMessage::FPV_NOTIFICATIONS_MODULE, StdMessage::FPV_STREAMING_STOPPED).toMsg();
        pubsocket->send(st);

        currentFrame = 0;
        fpv_module_is_streaming = false;
        return true;
    }
}

void FPVModule::onMessage(const Message &inmsg)
{
    auto action = airt::getMessageAction(inmsg);
    switch (action)
    {
    case StdMessage::FPV_GET_RESOLUTION:
    {
        Message outmsg;

        FPVResolutionNotification res;
        res.width = grabber.color_w;
        res.height = grabber.color_h;
        outmsg.add_raw(&res, sizeof(FPVResolutionNotification));
        pubsocket->send(outmsg);
    }
    break;

    case StdMessage::FPV_START_STREAMING:
        init();
        startStreaming();
    break;

    case StdMessage::FPV_STOP_STREAMING:
        stopStreaming();
    break;

    default:
        Log::critical("FPVModule: received unhandled command {}", action);
    }
}

bool FPVModule::sendMessage()
{
    std::chrono::milliseconds frame_period_ms(0);
    if(currentFrame > 0)
    {
        std::chrono::duration<double> frame_elapsed_secs = std::chrono::steady_clock::now() - lastFrameTS;
        frame_period_ms = std::chrono::duration_cast<std::chrono::milliseconds>(frame_elapsed_secs);
    }
    lastFrameTS = std::chrono::steady_clock::now();

    currentFrame++;

    airt::FPVModule::FPVImageJpegHeader hdr;
    hdr.frameNumber = currentFrame;

    Message msg;
    msg.add_raw(&hdr, sizeof(hdr));
    msg.add_raw(image_jpeg.get(), size_image_jpeg);

    pubsocket->send(msg);

    if((currentFrame % grabber.camera_fps) == 0)
    {
        Log::info("FPVModule: totalFramesSent {}, last-jpeg-bytes {} last-frame-period {} ms",
                  currentFrame, size_image_jpeg, frame_period_ms.count());
    }

    return true;
}
