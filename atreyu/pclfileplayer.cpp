#include "pclfileplayer.h"

#include "globalSettings.h"
#include "stdMessage.h"
//#include "zr300module.h"
#include "d415module.h"
#include <log.h>
#include <sstream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <limits>

using airt::PCLFilePlayer;
using airt::Log;
using airt::StdNotification;
using airt::StdMessage;

PCLFilePlayer::PCLFilePlayer(const std::string &cmdportname, const std::string &pubportname,
                             std::shared_ptr<Context> context)
    : Module(cmdportname, pubportname, "", context), pclFilename(""), replayFreq(1.), publishing(false),
      currentFrame(0), cloud(new pcl::PointCloud<PointNormalT>)
{
    assert(context);
    if (!GlobalSettings::getValue("pclreplayermaxpointstosend", maxPointsToSend))
    {
        maxPointsToSend = 20000;
        Log::warn("Using default value pclreplayermaxpointstosend = {}", maxPointsToSend);
    }
    if (!GlobalSettings::getValue("pclreplaytotalframes", totalFramesToPublish)) {
        Log::warn("The config file has no pclreplaytotalframes. We will publish all the available frames");
        totalFramesToPublish = std::numeric_limits<std::size_t>::max();
    }
}

bool PCLFilePlayer::startDevice()
{
    Log::info("PCLFilePlayer starts... Current frame: {}", currentFrame);
    publishing = true;

    auto st = StdMessage(StdMessage::POINTCLOUD_NOTIFICATIONS_MODULE, StdMessage::STARTED).toMsg();
    pubsocket->send(st);
    return true;    
};

void PCLFilePlayer::stopDevice()
{
    Log::info("PCLFilePlayer stops... Current frame: {}", currentFrame);
    publishing = false;
    auto st = StdMessage(StdMessage::POINTCLOUD_NOTIFICATIONS_MODULE, StdMessage::STOPPED).toMsg();
    pubsocket->send(st);

    //reset
    currentFrame = 0;
};

bool PCLFilePlayer::dataReady()
{
    if (!publishing)
        return false;
    static std::chrono::time_point<std::chrono::steady_clock> prev = std::chrono::steady_clock::now();

    std::chrono::time_point<std::chrono::steady_clock> now = std::chrono::steady_clock::now();
    std::chrono::duration<double> elapsed = now - prev;
    if (elapsed.count() > /*0.03333333*/ 1.0 / replayFreq)
    {
        prev = now;
        return true;
    }
    return false;
}

bool PCLFilePlayer::sendMessage()
{
    if (currentFrame >= totalFramesToPublish || !readFile(pclFilename, currentFrame))
    {
        stopDevice();
        return false;
    }

    const size_t totalPoints = cloud->width * cloud->height;
    for (size_t sent = 0; sent < totalPoints;)
    {
        airt::D415Module::pointCloudHeader pch;
        pch.numPoints = std::min(maxPointsToSend, totalPoints - sent);

        Message msg;
        msg.add_raw(&pch, sizeof(pch));

        std::vector<airt::D415Module::pointNormal> points;
        points.reserve(pch.numPoints);
        for (size_t j = 0; j < pch.numPoints; j++)
        {
            airt::D415Module::pointNormal p;
            auto pclpoint = cloud->points[j + sent];
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
        msg.add_raw(&points[0], sizeof(airt::D415Module::pointNormal) * points.size());
        pubsocket->send(msg);

        sent += pch.numPoints;
    }
    currentFrame++;

    return true;
}

bool PCLFilePlayer::readFile(const std::string &basename, size_t frame)
{
    std::ostringstream ss;

    ss << basename << frame << ".pcd";
    if (pcl::io::loadPCDFile<PointNormalT>(ss.str(), *cloud) == -1) //* load the file
    {
        Log::error("Couldn't read file {}", ss.str());
        return false;
    }
    else
    {
        Log::info("{} read correctly", ss.str());
    }
    /*
    std::cout << "Loaded "
              << cloud->width * cloud->height
              << " data points from test_pcd.pcd with the following fields: "
              << std::endl;
    for (size_t i = 0; i < cloud->points.size(); ++i)
        std::cout << "    " << cloud->points[i].x
                  << " " << cloud->points[i].y
                  << " " << cloud->points[i].z << std::endl;

    return (0);
    */
    return true;
}
