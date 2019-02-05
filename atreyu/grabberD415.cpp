#include <log.h>
using airt::Log;


#include "grabberD415.h"
#include <librealsense2/rsutil.h>
#include <cmath>
#include <turbojpeg.h>

using namespace std;
rs2::context GrabberD415::ctx;

#include <thread>


// Constructor
GrabberD415::GrabberD415(Resolution depthImage, Resolution colorImage, bool fastestFPS, bool use_queues_, int queue_capacity)
 : depth_enabled(false), color_enabled(false) {
    // sensor-stream indexes have been got using rs-sensor-control utility from librealsense2

    if (fastestFPS)
        camera_fps = 60;
    else
        camera_fps = 30;

    switch (depthImage)
    {
    case Resolution::Small:
        depth_w = 480;
        depth_h = 270;
        depth_sensor_stream_index = (camera_fps == 30)? 228 : 227;
        break;
    case Resolution::Medium:
        depth_w = 640;
        depth_h = 480;
        depth_sensor_stream_index = (camera_fps == 30)? 218 : 217;
        break;
    case Resolution::Large:
        depth_w = 848;
        depth_h = 480;
        depth_sensor_stream_index = (camera_fps == 30)? 213 : 212;
        break;
    }

    switch (colorImage)
    {
    case Resolution::Small:
        color_w = 424;
        color_h = 240;
        color_sensor_stream_index = (camera_fps == 30)? 143 : 137;
        break;
    case Resolution::Medium:
        color_w = 640;
        color_h = 480;
        color_sensor_stream_index = (camera_fps == 30)? 95 : 89;
        break;
    case Resolution::Large:
        color_w = 960;
        color_h = 540;
        color_sensor_stream_index = (camera_fps == 30)? 47 : 41;
        break;
    }

    std::vector<rs2::device> devs = getRS2Devices();
    if(devs.size() == 0)
    {
        throw std::runtime_error("No Realsense2 device detected. Is it plugged in?");
    }
    dev = std::make_unique<rs2::device>(devs[devs.size()-1]);

//    dev = std::make_unique<rs2::device>(hwreset(*dev.get()));  //error when quitting atreyu

    std::vector<rs2::sensor> ss = dev->query_sensors();
    for(size_t i=0; i < ss.size(); i++)
    {
        sensors.push_back( ss[i] );
    }

    use_queues = use_queues_;
    if(use_queues)
    {
        queue_depth = rs2::frame_queue(queue_capacity);
        queue_color = rs2::frame_queue(queue_capacity);
    }
}

GrabberD415::~GrabberD415()
{
}

rs2::device GrabberD415::hwreset(rs2::device & d)
{
    d.hardware_reset();
    std::this_thread::sleep_for(std::chrono::seconds(2));
    rs2::context ctx;
    rs2::device_hub hub(ctx);
    return hub.wait_for_device();
}

std::string GrabberD415::getRS2DeviceName(const rs2::device &cam)
{
    std::string name = "Unknown Device"; // api can find non-realsense cams

    // ensure it is a rs2 camera checking if the device supports these parameters
    if (cam.supports(RS2_CAMERA_INFO_NAME) &&
        cam.supports(RS2_CAMERA_INFO_SERIAL_NUMBER) &&
        cam.supports(RS2_CAMERA_INFO_FIRMWARE_VERSION))
    {
        std::string tmp = cam.get_info(RS2_CAMERA_INFO_NAME);

        size_t index_found = tmp.find("RealSense");
        if(index_found != std::string::npos)
        {
            name = tmp;
        }
    }

    return name;
}

std::vector<rs2::device> GrabberD415::getRS2Devices()
{
    rs2::context ctx;
    rs2::device_list alldevices = ctx.query_devices(); // even no-realsense cameras
    std::vector<rs2::device> rs2cams;

    for (size_t i = 0; i < alldevices.size(); i++)
    {
        if (getRS2DeviceName(alldevices[i]) != "Unknown Device")
        {
            rs2cams.push_back(alldevices[i]);
        }
    }

    return rs2cams;
}

bool GrabberD415::printRSContextInfo(std::ostream &info)
{
    std::vector<rs2::device> rs2cams = getRS2Devices();

    size_t numDevices = rs2cams.size();
    info << "There are " << numDevices << " RealSense2 cameras connected\n";

    if (numDevices == 0)
    {
        return false;
    }

    for (size_t i = 0; i < numDevices; i++)
    {
        rs2::device cam = rs2cams[i];

        info << "RS2-Device id " << cam.get_info(RS2_CAMERA_INFO_PRODUCT_ID) << ", " << cam.get_info(RS2_CAMERA_INFO_NAME) << "\n";
        info << "  Serial number: " << cam.get_info(RS2_CAMERA_INFO_SERIAL_NUMBER) << "\n";
        info << "  Firmware version: " << cam.get_info(RS2_CAMERA_INFO_FIRMWARE_VERSION) << "\n";

        std::vector<rs2::sensor> ss = cam.query_sensors();
        int sensor_idx = 0;

        info << "    Sensors: ";
        for (rs2::sensor s : ss)
        {
            info << " " << sensor_idx << " " << s.get_info(RS2_CAMERA_INFO_NAME) << " ";
            sensor_idx++;
        }
        info << "\n";
    }

    return true;
}

bool GrabberD415::configureRSStreams(bool enable_color_, bool enable_depth_)
{
    color_enabled = enable_color_;
    depth_enabled = enable_depth_;

    if(is_streaming)
    {
        stopStreaming();
    }

    std::string serial_number(dev->get_info(RS2_CAMERA_INFO_SERIAL_NUMBER));
    config.enable_device(serial_number);

    if (depth_enabled)
    {
        config.enable_stream(RS2_STREAM_DEPTH, depth_w, depth_h, RS2_FORMAT_Z16, camera_fps);
        Log::info("Grabberd415: enabled RS2_STREAM_DEPTH");
    }

    if (color_enabled)
    {
        config.enable_stream(RS2_STREAM_COLOR, color_w, color_h, RS2_FORMAT_RGB8, camera_fps);
        Log::info("Grabberd415: enabled RS2_STREAM_COLOR");
    }

    profile = pipe.start(config);
    is_streaming = true;

    // TODO: add new librealsense2 filters in config object

    if (color_enabled)
    {
        rs2::video_stream_profile vsp_color = profile.get_stream(RS2_STREAM_COLOR).as<rs2::video_stream_profile>();

        // After enable depth/color streams we can retrieve camera parameters for building the pointcloud
        color_intrin = vsp_color.get_intrinsics();

        Log::info("Grabberd415: got color_intrin from profile");
    }

    if (depth_enabled)
    {
        rs2::video_stream_profile vsp_depth = profile.get_stream(RS2_STREAM_DEPTH).as<rs2::video_stream_profile>();
        depth_intrin = vsp_depth.get_intrinsics();

        Log::info("Grabberd415: got depth_intrin from profile");
    }

    if(use_queues)
    {
        pipe.stop();
        is_streaming = false;

        Log::info("Grabberd415:  configuring sensors");
        for(size_t i=0; i < sensors.size(); i++)
        {
            std::string sensor_name(sensors[i].get_info(RS2_CAMERA_INFO_NAME));

            if(color_enabled && sensor_name == "RGB Camera")
            {
                std::vector<rs2::stream_profile> sensor_profiles = sensors[i].get_stream_profiles();
                sensors[i].open(sensor_profiles[color_sensor_stream_index]);
                Log::info("Grabberd415: color-sensor openned");
            }

            if(depth_enabled && sensor_name == "Stereo Module")
            {
                std::vector<rs2::stream_profile> sensor_profiles = sensors[i].get_stream_profiles();
                sensors[i].open(sensor_profiles[depth_sensor_stream_index]);
                Log::info("Grabberd415: depth-sensor openned");
            }
        }
    }

    return true;
}

bool GrabberD415::startStreaming()
{    
    if(!is_streaming)
    {
        if(!use_queues)
        {
            profile = pipe.start(config);
            Log::info("GrabberD415: rs2 streams started");
        }
        else
        {
            for(size_t i=0; i < sensors.size(); i++)
            {
                std::string sensor_name(sensors[i].get_info(RS2_CAMERA_INFO_NAME));

                if(color_enabled && sensor_name == "RGB Camera")
                {
                    sensors[i].start([this](rs2::frame f)
                    {
                        Log::info("Grabberd415: COLOR-frame arrived {} {}",
                                  f.get_frame_number(), f.get_timestamp() * 0.001);

                        queue_color.enqueue(std::move(f));
                    } );

                    Log::info("Grabberd415: color callback attached and started");
                }

                if(depth_enabled && sensor_name == "Stereo Module")
                {
                    sensors[i].start([this](rs2::frame f)
                    {
                        Log::info("Grabberd415: DEPTH-frame arrived: {} {}",
                                  f.get_frame_number(), f.get_timestamp() * 0.001);

                        queue_depth.enqueue(std::move(f));
                    } );

                    Log::info("Grabberd415: depth callback attached and started");
                }
            }
        }
        is_streaming = true;
    }
    else
    {
        Log::warn("GrabberD415: rs2 streams already started");
    }

    return true;
}

bool GrabberD415::stopStreaming()
{
    if (is_streaming)
    {
        if(!use_queues)
        {
            pipe.stop(); // stops and closes streams
            Log::info("GrabberD415: rs2 streams stopped");
            closeRSStreams();
            Log::info("GrabberD415: rs2 streams closed");
        }
        else
        {
            for(rs2::sensor s : sensors)
            {
                s.stop();  // will block until all pending callbacks return
                Log::info("GrabberD415: stopped sensor: {}", s.get_info(RS2_CAMERA_INFO_NAME));
            }
        }
        is_streaming = false;
    }
    else
    {
        Log::warn("GrabberD415: rs2 streams already stopped");
    }

    return true;
}

bool GrabberD415::closeRSStreams()
{
    if(!use_queues)
    {
        config.disable_all_streams();
        Log::info("GrabberD415: rs2 streams stopped");
    }
    else
    {
        for(rs2::sensor s : sensors)
        {
            s.close();  // will block until all pending callbacks return
            Log::info("GrabberD415: closed sensor: {}", s.get_info(RS2_CAMERA_INFO_NAME));
        }
    }
    return true;
}

bool GrabberD415::dataReady(rs2::frameset & frames)
{
    if(!is_streaming)
    {
        return false;
    }

//    if(!pipe.poll_for_frames(&frames))
//    {
//        return false;
//    }

    frames = pipe.wait_for_frames(1000);

    return true;
}

bool GrabberD415::getFrameSet(rs2::frameset & frameset)
{
    if(!dataReady(frameset))
    {
        return false;
    }

    // drop first bad frames
    if (frameset.get_frame_number() < 8)
    {
        return false;
    }

    return true;
}

bool GrabberD415::getJpegImage(unsigned char *compressed_image, size_t &bytes_written)
{
    if(!is_streaming)
    {
        return false;
    }

    rs2::frameset frameset;
    rs2::frame fcolor;

    if(!use_queues)
    {
        if(!getFrameSet(frameset))
        {
            return false;
        }
        fcolor = frameset.get_color_frame();
    }
    else
    {
        if(!queue_color.poll_for_frame(&fcolor))
        {
            return false;
        }
    }

    rs2::video_frame frame_color = fcolor.as<rs2::video_frame>();

    uint8_t * image_color = reinterpret_cast<uint8_t *>(const_cast<void *>(frame_color.get_data()));

    // JPEG compression
    const int JPEG_QUALITY = 70;
    const int _width = frame_color.get_width();
    const int _height = frame_color.get_height();

    tjhandle _jpegCompressor = tjInitCompress();

    tjCompress2(_jpegCompressor, image_color, _width, 0, _height, TJPF_RGB,
                &compressed_image, &bytes_written, TJSAMP_411, JPEG_QUALITY,
                TJFLAG_FASTDCT);

    tjDestroy(_jpegCompressor);

    /*
    std::ofstream of;
    std::string filepath = "test.jpeg";
    of.open(filepath, std::ios_base::out | std::ios_base::binary);

    if (!of.is_open())
    {
        return false;
    }
    of.write(reinterpret_cast<const char *>(compressed_image), bytes_written);
    of.close();
    */

    return true;
}

bool GrabberD415::getPointsRGB(RS_PointRGB* points_rgb, size_t &num_points)
{
    if(!is_streaming)
    {
        return false;
    }

    rs2::frameset frameset;
    rs2::frame fdepth;
    rs2::frame fcolor;

    if(!use_queues)
    {
        if(!getFrameSet(frameset))
        {
            return false;
        }
        fdepth = frameset.get_depth_frame();
        fcolor = frameset.get_color_frame();
    }
    else
    {
        if(!queue_color.poll_for_frame(&fcolor))
        {
            Log::info("GrabberD415: no color frame arrived\n");
            return false;
        }

        if(!queue_depth.poll_for_frame(&fdepth))
        {
            Log::info("GrabberD415: no depth frame arrived");
            return false;
        }
    }

    rs2::video_frame frame_color = fcolor.as<rs2::video_frame>();
    rs2::depth_frame frame_depth = fdepth.as<rs2::depth_frame>();

    rs2_points = rs2_pointcloud.calculate(frame_depth);
    rs2_pointcloud.map_to(frame_color);

    const rs2::vertex * vertices = rs2_points.get_vertices();
    const rs2::texture_coordinate * uvs = rs2_points.get_texture_coordinates();

    const uint8_t * image_color = reinterpret_cast<const uint8_t *>(const_cast<void*>(frame_color.get_data()));

    // generate non-organized pointcloud (width*1)
    int current_index = 0;

    for (int i=0; i < depth_intrin.width * depth_intrin.height; i++, vertices++, uvs++)
    {
        if(vertices->z > 0.1 && vertices->z < 8.0)
        {
            RS_PointRGB & p = points_rgb[current_index];

            p.vertex.x = vertices->x;
            p.vertex.y = vertices->y;
            p.vertex.z = vertices->z;

            int cx = std::round(uvs->u * color_intrin.width);
            int cy = std::round(uvs->v * color_intrin.height);

            if (cx >= 0 && cy >= 0 && cx < color_intrin.width && cy < color_intrin.height)
            {
                const uint8_t *pixel_offset = (image_color + (cy * color_intrin.width + cx) * 3);
                p.r = *(pixel_offset);
                p.g = *(pixel_offset + 1);
                p.b = *(pixel_offset + 2);

                current_index++;
            }
            else
            {
                continue;
            }
        }
    }
    num_points = current_index;

    return true;
}

rs2::device *GrabberD415::getRSDevice()
{
    return dev.get();
}
