#include "zr300module.h"
#include "globalSettings.h"
#include "server.h"

#include <sstream>
#include <log.h>
#include <stdMessage.h>
#include <utils.h>

using airt::Log;
using airt::ZR300Source;

void ZR300Source::motion_callback(rs::motion_data entry)
{
    // only accelerometer frames
    if ((rs::event)entry.timestamp_data.source_id == rs::event::event_imu_accel)
    {
        // call external listener
        externalIMUlistener(entry.timestamp_data.timestamp, entry.axes[0], entry.axes[1], entry.axes[2]);

        if (acceleratorLogger)
        {
            acceleratorLogger << entry.timestamp_data.timestamp << ";" << entry.axes[0] << ";" << entry.axes[1] << ";" << entry.axes[2] << "\n";
        }
    }
}

ZR300Source::ZR300Source(const std::string &portname,
                         std::shared_ptr<Context> context)
    : InternalSource(portname, context)
{
    assert(context);

    std::ostringstream info;
    grabber.printRSContextInfo(info);
    Log::info("{}", info.str());

    if (!GlobalSettings::getValue("zr300maxpointstosend", maxPointsToSend))
    {
        maxPointsToSend = 20000;
        Log::warn("Using default value: zr300maxpointstosend = {}", maxPointsToSend);
    }

    grabber.configureRSStreams();

    // set motion callback for enabling receiving data from accelerometer
    grabber.setMotionCallback(std::bind(&ZR300Source::motion_callback, this, std::placeholders::_1));

    filter.clearQueues();
    currentFrame = 0;
}

bool ZR300Source::startDevice()
{
    Log::info("ZR300Source starts... Current frame: {}", currentFrame);
    
    auto st = StdMessage(StdMessage::POINTCLOUD_NOTIFICATIONS_MODULE, StdMessage::STARTED).toMsg();
    socket->send(st);

    return grabber.startStreaming();
}

void ZR300Source::stopDevice()
{
    Log::info("ZR300Source stops... Current frame: {}", currentFrame);
    auto st = StdMessage(StdMessage::POINTCLOUD_NOTIFICATIONS_MODULE, StdMessage::STOPPED).toMsg();
    socket->send(st);

    grabber.stopStreaming();
    grabber.closeRSStreams();

    if (acceleratorLogger)
        acceleratorLogger.close();
    filter.clearQueues();

    currentFrame = 0;
}

bool ZR300Source::dataReady()
{
    pcl::PointCloud<PointT>::Ptr cloud = pcl::PointCloud<PointT>::Ptr(new pcl::PointCloud<PointT>);
    if (!grabber.generatePointCloud(cloud))
        return false;

    // filtering
    filter.enqueuePcl(cloud);
    return filter.dataReady();

    return true;
}

void ZR300Source::onMessage(const Message &m)
{
    switch (airt::getMessageAction(m))
    {
    case StdMessage::MCAM_START_LOGGING_IMU:
        if (grabber.isStreaming()) {
            Log::error("It is not allowed to start logging the IMU data while streaming is on.");
            auto m = StdMessage(StdMessage::POINTCLOUD_NOTIFICATIONS_MODULE, StdMessage::INVALID_STATUS).toMsg();
            socket->send(m);
        } else {
            std::string filename = "imudata-" + airt::getTimeStamp() + ".csv";
            acceleratorLogger.open(filename, std::ios_base::out);
            if (!acceleratorLogger) {
                Log::error("Cound not open file {} for writing");
                auto m= StdMessage(StdMessage::POINTCLOUD_NOTIFICATIONS_MODULE, StdMessage::INVALID_STATUS).toMsg();
                socket->send(m);
            }
            auto m = StdMessage(StdMessage::POINTCLOUD_NOTIFICATIONS_MODULE, StdMessage::MCAM_IMULOGGING_STARTED).toMsg();
            socket->send(m);
        }
        break;
    }
}

bool ZR300Source::sendMessage()
{
    pcl::PointCloud<PointNormalT>::Ptr withnormals(new pcl::PointCloud<PointNormalT>());
    if (!filter.getFilteredCloud(withnormals))
    {
        return false;
    }

    const size_t totalPoints = withnormals->width * withnormals->height;
    if(totalPoints < 5)
    {
        Log::error("ZR300Source: not enough points to send. Not sending message.");
        return false;
    }

    //splits if cloud_size > maxPointsToSend
    for (size_t sent = 0, part = 0; sent < totalPoints; part++)
    {
        airt::ZR300Source::pointCloudHeader pch;
        pch.numPoints = std::min(maxPointsToSend, totalPoints - sent);
        pch.i = pch.j = pch.k = currentFrame;
        Message msg;
        msg.add_raw(&pch, sizeof(pch));

        std::vector<airt::ZR300Source::pointNormal> points;
        points.reserve(pch.numPoints);
        for (size_t j = 0; j < pch.numPoints; j++)
        {
            airt::ZR300Source::pointNormal p;
            auto pclpoint = withnormals->points[j + sent];
            p.x = pclpoint.x;
            p.y = pclpoint.y;
            p.z = pclpoint.z;
            p.r = pclpoint.r;
            p.g = pclpoint.g;
            p.b = pclpoint.b;
            p.a = pclpoint.a;
            //p.color = pclpoint.r + (pclpoint.g << 8) + (pclpoint.b << 16);
            p.normal_x = pclpoint.normal_x;
            p.normal_y = pclpoint.normal_y;
            p.normal_z = pclpoint.normal_z;

            points.emplace_back(p);
        }
        msg.add_raw(&points[0], sizeof(airt::ZR300Source::pointNormal) * points.size());
        socket->send(msg);

        sent += pch.numPoints;
    }
    currentFrame++;

    return true;
}
