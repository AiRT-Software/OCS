#include "grabber.h"

#include <pcl/io/pcd_io.h>
#include <pcl/common/transforms.h>

#include <log.h>
#include <iostream>
#include <sstream>

using namespace std;
using airt::Log;

rs::context Grabber::ctx;

// Constructor
Grabber::Grabber(unsigned int cameraIndex, Resolution depthImage, Resolution colorImage, bool fastestFPS)
: depth_scale(1.0f)
{
    auto numCameras = ctx.get_device_count();
    if (numCameras == 0)
    {
        Log::critical("No Realsense device detected. Is it plugged in?");
        throw std::runtime_error("No Realsense device detected. Is it plugged in?");
    }
    else if (cameraIndex >= static_cast<unsigned int>(numCameras))
    {
        Log::critical("Camera {} does not exists", cameraIndex);
        throw std::runtime_error("Camera does not exists");
    }

    Log::info("Connecting to camera {} of {}", cameraIndex, numCameras);
    dev = ctx.get_device(cameraIndex);

    switch (depthImage)
    {
    case Resolution::Small:
        depth_w = 320;
        depth_h = 240;
        break;
    case Resolution::Medium:
        depth_w = 480;
        depth_h = 360;
        break;
    case Resolution::Large:
        depth_w = 640;
        depth_h = 480;
        break;
    }

    switch (colorImage)
    {
    case Resolution::Small:
        color_w = 320;
        color_h = 240;
        break;
    case Resolution::Medium:
        color_w = 640;
        color_h = 480;
        break;
    case Resolution::Large:
        color_w = 1920;
        color_h = 1080;
        break;
    }

    if (fastestFPS)
        camera_fps = 60;
    else
        camera_fps = 30;

    this->motion_callback = NULL;
}

bool Grabber::printRSContextInfo(std::ostream &info)
{
    auto numCameras = ctx.get_device_count();
    info << "There are " << numCameras << " connected RealSense devices\n";

    if (numCameras == 0)
    {
        return false;
    }

    for (int i = 0; i < numCameras; i++)
    {
        auto dev = ctx.get_device(i);
        info << "Device " << i << ", an " << dev->get_name() << "\n";
        info << "    Serial number: " << dev->get_serial() << "\n";
        info << "    Firmware version: " << dev->get_firmware_version() << "\n";
    }

    return true;
}

bool Grabber::configureRSStreams()
{
    dev->enable_stream(rs::stream::depth, depth_w, depth_h, rs::format::z16, camera_fps);
    dev->enable_stream(rs::stream::color, color_w, color_h, rs::format::rgb8, camera_fps);

    if (!dev->is_stream_enabled(rs::stream::depth) ||
        !dev->is_stream_enabled(rs::stream::color))
    {
        return false;
    }

    // After enable depth/color streams we can retrieve camera parameters for build the pointcloud
    depth_intrin = dev->get_stream_intrinsics(rs::stream::depth);
    depth_to_color = dev->get_extrinsics(rs::stream::depth, rs::stream::color);
    color_intrin = dev->get_stream_intrinsics(rs::stream::color);
    depth_scale = dev->get_depth_scale();

    return true;
}

bool Grabber::setMotionCallback(std::function<void(rs::motion_data)> &mcb)
{
    // accelerometer & gyroscope frames catched in callback as rs::motion_data
    if (!dev->supports(rs::capabilities::motion_events))
    {
        Log::critical("This rs::device does not support motion tracking");
        throw std::runtime_error("This rs::device does not support motion tracking");
    }
    else
    {
        this->motion_callback = mcb;  // save motion_callback for next start commans
        dev->enable_motion_tracking(this->motion_callback);
    }

    return true;
}

bool Grabber::startStreaming()
{
    if (!dev->is_stream_enabled(rs::stream::depth) ||
        !dev->is_stream_enabled(rs::stream::color))
    {
        configureRSStreams();
    }

    if (!dev->is_streaming())
    {
        if(this->motion_callback != NULL)
        {
            dev->enable_motion_tracking(this->motion_callback);
            dev->start(rs::source::all_sources);
        }
        else
            dev->start(rs::source::video);
    }

    return dev->is_streaming();
}

bool Grabber::isStreaming() {
    return dev->is_streaming();
}

bool Grabber::stopStreaming()
{
    if (dev->is_streaming())
    {
        if(this->motion_callback != NULL)
            dev->stop(rs::source::all_sources);
        else
            dev->stop(rs::source::video);
    }

    // disable motion after stopping streaming
    if(this->motion_callback != NULL)
        dev->disable_motion_tracking();

    return !dev->is_streaming();
}

bool Grabber::closeRSStreams()
{
    stopStreaming();

    if (dev->is_stream_enabled(rs::stream::depth))
    {
        dev->disable_stream(rs::stream::depth);
    }
    if (dev->is_stream_enabled(rs::stream::color))
    {
        dev->disable_stream(rs::stream::color);
    }

    return (!dev->is_stream_enabled(rs::stream::depth) &&
            !dev->is_stream_enabled(rs::stream::color)
            );
}

bool Grabber::generatePointCloud(pcl::PointCloud<PointT>::Ptr cloud, bool organized)
{
    /*
     * @brief Test: the driver gives always the last frame, there is no buffering
     * */
    /*
    while(1)
    {
        while (dev->poll_for_frames())
        {
            Log::info("Grabber: frame: {}, timestamp: {} ms",
                      dev->get_frame_number(rs::stream::depth),
                      dev->get_frame_timestamp(rs::stream::depth));
        }

        int ms = 1000;
        Log::info("Grabber: sleeping {} ms", ms);
        std::this_thread::sleep_for(std::chrono::milliseconds(ms));
    }
    */


    if (!dev->is_streaming() || !dev->poll_for_frames())
    {
        return false;
    }

    //drop first bad-frames
    if (dev->get_frame_number(rs::stream::depth) < 20)
    {
        return false;
    }

    const uint8_t *color_image = (const uint8_t *)dev->get_frame_data(rs::stream::color);
    const rs::float3 *points = reinterpret_cast<const rs::float3 *>(dev->get_frame_data(rs::stream::points));

    //set up pointcloud values
    cloud->clear();
    cloud->header.frame_id = "ZR300";
    cloud->header.seq = dev->get_frame_number(rs::stream::depth);
    cloud->header.stamp = std::lround( dev->get_frame_timestamp(rs::stream::depth) );
    cloud->is_dense = true; //it is false when certain points can contain inf/nan values

    //cloud->sensor_origin_
    //cloud->sensor_orientation_

    if(organized)
    {
        cloud->resize(depth_intrin.width * depth_intrin.height);
        cloud->width = depth_intrin.width;
        cloud->height = depth_intrin.height;

        //generate organized pointcloud (width*height)
        pcl::PointCloud<PointT>::iterator it = cloud->begin();
        for (int i = 0; i < depth_intrin.height * depth_intrin.width; ++i, ++points, ++it)
        {
            it->x = points->x;
            it->y = points->y;
            it->z = points->z;
            if (points->z > 0.1 && points->z < 3.5)
            {
                rs::float2 color_pixel = color_intrin.project(depth_to_color.transform(*points));

                //because different frame resolutions
                int cx = (int)std::round(color_pixel.x);
                int cy = (int)std::round(color_pixel.y);

                if (cx >= 0 && cy >= 0 && cx < color_intrin.width && cy < color_intrin.height)
                {
                    const uint8_t *pixel_offset = (color_image + (cy * color_intrin.width + cx) * 3);
                    it->r = *(pixel_offset);
                    it->g = *(pixel_offset + 1);
                    it->b = *(pixel_offset + 2);
                }
                else
                {
                    it->r = 255;
                    it->g = 255;
                    it->b = 255;
                }
            }
            else
            {
                //when not valid data the driver writes z = 0 (nan/inf values not allowed)
                it->r = 255;
                it->g = 0;
                it->b = 0;
            }
        }
    }
    else
    {
        //generate non-organized pointcloud (width*1)
        for (int i = 0; i < depth_intrin.height * depth_intrin.width; ++i, ++points)
        {
            PointT p;

            //only points within the valid range
            if (points->z > 0.1 && points->z < 3.5)
            {
                p.x = points->x;
                p.y = points->y;
                p.z = points->z;

                rs::float2 color_pixel = color_intrin.project(depth_to_color.transform(*points));

                //because different frame resolutions
                int cx = (int)std::round(color_pixel.x);
                int cy = (int)std::round(color_pixel.y);

                if (cx >= 0 && cy >= 0 && cx < color_intrin.width && cy < color_intrin.height)
                {
                    const uint8_t *pixel_offset = (color_image + (cy * color_intrin.width + cx) * 3);
                    p.r = *(pixel_offset);
                    p.g = *(pixel_offset + 1);
                    p.b = *(pixel_offset + 2);
                }
                else
                {
                    p.r = 255;
                    p.g = 255;
                    p.b = 255;
                }
                cloud->push_back(p);
            }
        }
    }

    return true;
}

rs::context &Grabber::getRSContext()
{
    return ctx;
}

rs::device *Grabber::getRSDevice()
{
    return dev;
}
