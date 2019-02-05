#include <sstream>
#include <fstream>
#include <glm/gtc/matrix_transform.hpp>

#include <stdMessage.h>
#include <log.h>
#include <utils.h>

#include <glmutils.h>

#include "d415module.h"
#include "globalSettings.h"

using airt::AIRT_Message_Header;
using airt::StdMessage;

#include "interface/dronemodule.h"

using airt::D415Module;
using airt::Log;

D415Module::D415Module(const std::string &cmdportname, const std::string &pubportname, const std::string &subname,
                       std::shared_ptr<Context> context)
    : Module(cmdportname, pubportname, subname, context)
{
    assert(context);

    std::ostringstream info;

    grabber.printRSContextInfo(info);
    Log::info("{}", info.str());

    cameraOffset = glm::vec4(0.0f);
}

D415Module::~D415Module()
{
}

bool D415Module::init()
{
    grabber.configureRSStreams(true, true);
    if (grabber.color_enabled)
    {
        Log::info("D415Module: enabled color stream: {}x{}@{}",
                  grabber.color_w, grabber.color_h, grabber.camera_fps);
    }
    if (grabber.depth_enabled)
    {
        Log::info("D415Module: enabled depth stream: {}x{}@{}",
                  grabber.depth_w, grabber.depth_h, grabber.camera_fps);
    }

    max_num_points = grabber.depth_w * grabber.depth_h;
    points_rgb = std::unique_ptr<RS_PointRGB[]>(new RS_PointRGB[max_num_points]);

    filter.clearQueues();
    currentFrame = 0;
    is_capturing = false;

    currentPositionIPS = glm::vec4(0.0, 0.0, 0.0, 1.0f);
    rollIPS = 0.0;
    yawIPS = 0.0;
    pitchIPS = 0.0;
    timestampIPS = 0;

    numReceivedIPSFrames = 0;

    return true;
}

void D415Module::configureSubscriptions(const std::string &mainpublisher)
{
    assert(subsocket);
    subsocket->connect(mainpublisher);
    uint8_t dronesignature[]{'A', StdMessage::DRONE_NOTIFICATIONS_MODULE};
    subsocket->subscribe(dronesignature, airt::arraySize(dronesignature));
}

bool D415Module::startDevice()
{
    // Request the z camera offset
    MsgToOtherModule header(StdMessage::POINTCLOUD_MODULE);
    StdMessage getCameraOffset(StdMessage::DRONE_MODULE, StdMessage::DRONE_GET_ZCAMERA_OFFSET);
    airt::sendTwoPartMessage(*cmdsocket, &header, &getCameraOffset, sizeof(getCameraOffset));

    return true;
}

void D415Module::stopDevice()
{
}

bool D415Module::dataReady()
{
    if (!is_capturing || !grabber.is_streaming)
    {
        return false;
    }

    num_points_captured = 0;
    if (!grabber.getPointsRGB(points_rgb.get(), num_points_captured))
    {
        return false;
    }

    pcl::PointCloud<PointT>::Ptr cloud(new pcl::PointCloud<PointT>());
    generatePcl(points_rgb.get(), num_points_captured, cloud);

    // filtering
    filter.enqueuePcl(cloud);
    return filter.dataReady();
}

bool D415Module::quitDevice()
{
    return true;
}

bool D415Module::startCapturing()
{
    if (!grabber.startStreaming())
    {
        Log::error("D415Module: start capturing has failed");
        auto st = StdMessage(StdMessage::POINTCLOUD_NOTIFICATIONS_MODULE, StdMessage::PCL_LAST_COMMAND_HAS_FAILED).toMsg();
        pubsocket->send(st);

        return false;
    }
    else
    {
        Log::info("D415Module: start capturing. Current frame: {}", currentFrame);
        auto st = StdMessage(StdMessage::POINTCLOUD_NOTIFICATIONS_MODULE, StdMessage::PCL_CAPTURING_STARTED).toMsg();
        pubsocket->send(st);

        is_capturing = true;
        return true;
    }
}

bool D415Module::stopCapturing()
{
    if (!grabber.stopStreaming())
    {
        Log::error("D415Module: stop capturing has failed");
        auto st = StdMessage(StdMessage::POINTCLOUD_NOTIFICATIONS_MODULE, StdMessage::PCL_LAST_COMMAND_HAS_FAILED).toMsg();
        pubsocket->send(st);

        return false;
    }
    else
    {
        Log::info("D415Module: stop capturing. Current frame: {}", currentFrame);
        auto st = StdMessage(StdMessage::POINTCLOUD_NOTIFICATIONS_MODULE, StdMessage::PCL_CAPTURING_STOPPED).toMsg();
        pubsocket->send(st);

        filter.clearQueues();

        currentFrame = 0;
        is_capturing = false;
        return true;
    }
}

void D415Module::generatePcl(RS_PointRGB *points_rgb, const size_t num_points, pcl::PointCloud<pcl::PointXYZRGB>::Ptr outcloud)
{
    outcloud->resize(num_points);
    outcloud->header.frame_id = "D415";
    outcloud->is_dense = true; //it is false when certain points can contain inf/nan values

    pcl::PointCloud<pcl::PointXYZRGB>::iterator it = outcloud->begin();
    for (size_t i = 0; i < num_points; i++, it++)
    {
        it->x = points_rgb[i].vertex.x;
        it->y = points_rgb[i].vertex.y;
        it->z = points_rgb[i].vertex.z;

        it->r = points_rgb[i].r;
        it->g = points_rgb[i].g;
        it->b = points_rgb[i].b;
    }
}

void D415Module::onMessage(const Message &inmsg)
{
    uint8_t module = airt::getMessageModule(inmsg);
    uint8_t action = airt::getMessageAction(inmsg);

    if (module != StdMessage::POINTCLOUD_MODULE)
    {
        return;
    }

    switch (action)
    {
    case StdMessage::PCL_START_CAPTURING:
        Log::info("D415Module: received PCL_START_CAPTURING");
        init();

        startCapturing();
        break;

    case StdMessage::PCL_STOP_CAPTURING:
        Log::info("D415Module: received PCL_STOP_CAPTURING");
        stopCapturing();
        break;

    default:
        Log::critical("D415Module: reveived unhandled command {}", action);
    }
}

void D415Module::onNotification(const Message &m)
{
    // even when this module is stopped, it will be receiving and updating the currentPositionIPS

    uint8_t module = airt::getMessageModule(m);
    uint8_t action = airt::getMessageAction(m);

    if (module != StdMessage::DRONE_NOTIFICATIONS_MODULE)
    {
        Log::error("Unwanted notification in D415Module {}", module);
        return;
    }

    switch (action)
    {
    case StdMessage::DRONE_ZCAMERA_OFFSET_NOTIFICATION:
    {
        auto dzo = m.get<const DroneZCamOffsetNotification *>(0);
        cameraOffset = glm::vec4{dzo->offsetX, dzo->offsetY, dzo->offsetZ, 1.0f};
    }
    break;

    case StdMessage::DRONE_POSITION_POSE_NOTIFICATION:
    {
        auto dps = m.get<const DronePositionPose *>(0);
        currentPositionIPS = glm::vec4(dps->x, dps->y, dps->z, 1.0f);
        rollIPS = dps->roll; // radians
        pitchIPS = dps->pitch;
        yawIPS = dps->yaw;
        timestampIPS = dps->timestamp;
        numReceivedIPSFrames++;
    }
    break;
    }
}

bool D415Module::sendMessage()
{
    std::chrono::milliseconds pclframe_period_ms(0);
    if (currentFrame > 0)
    {
        std::chrono::duration<double> pclframe_elapsed_secs = std::chrono::steady_clock::now() - lastPclFrameTS;
        pclframe_period_ms = std::chrono::duration_cast<std::chrono::milliseconds>(pclframe_elapsed_secs);
    }
    lastPclFrameTS = std::chrono::steady_clock::now();

    pcl::PointCloud<PointNormalT>::Ptr withnormals(new pcl::PointCloud<PointNormalT>());
    if (!filter.getFilteredCloud(withnormals))
    {
        return false;
    }

    const size_t totalPoints = withnormals->width * withnormals->height;
    if (totalPoints < 5)
    {
        Log::error("D415Module: not enough points to send. Not sending message.");
        return false;
    }

#ifdef _DEBUG
    if (cameraOffset.w == 0.0f)
    {
        Log::error("The camera offset was not set!!!");
    }
#endif

    airt::D415Module::pointCloudHeader pch;
    pch.numPoints = totalPoints;

    // changing coordinates from realsense camera to pozyx
    glm::vec4 camPos = glm::rotate(glm::mat4(), yawIPS, glm::vec3(0.0f, 0.0f, 1.0f)) *
                       glm::rotate(glm::mat4(), pitchIPS, glm::vec3(1.0f, 0.0f, 0.0f)) *
                       cameraOffset;
    pch.x = currentPositionIPS.x + camPos.x;
    pch.y = currentPositionIPS.y + camPos.y;
    pch.z = currentPositionIPS.z + camPos.z;
    pch.pitch = pitchIPS;
    pch.roll = rollIPS;
    pch.yaw = yawIPS;

    Message msg;
    msg.add_raw(&pch, sizeof(pch));

    std::vector<airt::D415Module::pointNormal> points;
    points.reserve(pch.numPoints);
    for (size_t j = 0; j < pch.numPoints; j++)
    {
        airt::D415Module::pointNormal p;
        auto pclpoint = withnormals->points[j];

        // changing coordinates from realsense camera to pozyx
        p.x = pclpoint.x;
        p.y = pclpoint.z;
        p.z = -pclpoint.y;

        p.r = pclpoint.r;
        p.g = pclpoint.g;
        p.b = pclpoint.b;
        p.a = pclpoint.a;

        // changing coordinates from realsense camera to pozyx
        p.normal_x = pclpoint.normal_x;
        p.normal_y = pclpoint.normal_z;
        p.normal_z = -pclpoint.normal_y;

        points.emplace_back(p);
    }
    msg.add_raw(&points[0], sizeof(airt::D415Module::pointNormal) * points.size());
    pubsocket->send(msg);

    Log::info("D415Module: sent pcl_{} numpoints {}, last-pclframe-period {} ms, num-received-IPSframes {}",
              currentFrame, points.size(), pclframe_period_ms.count(), numReceivedIPSFrames);

    currentFrame++;

    return true;
}
