#ifndef RSGRABBERD415_H
#define RSGRABBERD415_H

#ifdef _DEBUG
#undef _DEBUG
#include <librealsense2/rs.hpp>
#define _DEBUG
#else
#include <librealsense2/rs.hpp>
#endif

#include <ostream>


class RS_PointRGB
{
public:
    rs2::vertex vertex;
    uint8_t r, g, b;
};


class GrabberD415
{
public:
    enum class Resolution
    {
        Small,
        Medium,
        Large
    };

    static rs2::context ctx;

    // RSStreams: Resolution and FPS values
    int depth_w;
    int depth_h;
    int color_w;
    int color_h;
    int camera_fps;

    bool is_streaming = false;
    bool depth_enabled, color_enabled;

    /**
    * GrabberD415 constructor. GrabberD415 wraps the code for accessing the camera
    * \param cameraIndex índice de la cámara a abrir (en caso de que haya más de una). Por defecto usa la 0
    * \param depth_w width of depth image ()
    * \param depthImage: Resolution::Small (424x240), Resolution::Medium (640x480), Resolution::Large (1280x720)
    * \param colorImage: Resolution::Small (320x240), Resolution::Medium (640x480), Resoluton::Large (1920x1080, 30fps only)
    * \param fastestFPS: use fastests refresh rate available (60 fps), or not (30 fps)
    */
    GrabberD415(Resolution depthImage = Resolution::Small, Resolution colorImage = Resolution::Medium, bool fastestFPS = false, bool use_queues_ = false, int queue_capacity = 5);
    ~GrabberD415();

    rs2::device hwreset(rs2::device &d);

    /**
     * @brief getRS2DeviceName gets a realsense camera name
     * @param cam a realsense2 device reference
     * @return the name of the realsense2 camera or "Unknown device"
     */
    std::string getRS2DeviceName(const rs2::device & cam);

    /**
     * @brief getRS2Devices looks for realsense cameras using RS2Device name
     * @return a list of found realsense2 devices
     */
    std::vector<rs2::device> getRS2Devices();

    /**
    * @brief printRSContextInfo dumps information about the connected realsense devices over the indicated flow
    * @param info flow in which the information will be dumped
    * @return false if no cameras detected, true otherwise
    */
    bool printRSContextInfo(std::ostream &info);

    /**
    * @brief configureRSStreams configures depth and color streams and it also
    * gets intrinsic and extrinsic values from the sensors for permofing transformations
    * for calculating the 3d location of the points of the pointcloud
    * @return
    */
    bool configureRSStreams(bool enable_color_ = true, bool enable_depth_ = false);

    /**
     * @brief dataReady
     * @param frames structure to store the frames in
     * @return true when frames are ready to be caught
     */
    bool dataReady(rs2::frameset & frames);

    /**
     * @brief getFrameSet gets the pair color-depth frames
     * @param frameset structure of frames to be filled
     * @return true when frames are ready, false otherwise
     */
    bool getFrameSet(rs2::frameset & frameset);

    /**
     * @brief getJpegImage compress the raw color image into a jpeg buffer
     * @param compressed_image pointer to the buffer in which the compressed jpeg image
     * will be stored
     * @param bytes_written is an input/output value. As input value it represents
     * the total number of bytes of the compressed_image buffer. As output value it
     * will we overwriten with the number of bytes of the compressed jpeg image
     * @return true when success, false otherwise
     */
    bool getJpegImage(unsigned char *compressed_image, size_t &bytes_written);

    /**
     * @brief getPointsRGB
     * @param points_rgb reserved buffer in which the points will be stored
     * @param num_points number of points written into the buffer
     * @return true when pointsrgb have been written, false otherwise
     */
    bool getPointsRGB(RS_PointRGB* points_rgb, size_t &num_points);

    /**
    * @brief startStreaming starts the enabled streams. Commonly depth and color.
    * If motion_callback has been set it also starts the motion tracking (accelerometer & gyroscope)
    * @return true if done, false otherwise
    */
    bool startStreaming();

    /**
    * @brief stopStreaming stops the active streams. Commonly depth and color.
    * If motion_callback has been set it also stops the motion tracking (acceleromter & gyroscope)
    * @return true if done, false otherwise
    */
    bool stopStreaming();

    /**
    * @brief closeRSStreams disables all enabled streams.
    * If motion_callback has been set it also disables motion tracking.
    * @return
    */
    bool closeRSStreams();

    rs2::device * getRSDevice();

private:
    std::unique_ptr<rs2::device> dev;
    int depth_sensor_stream_index;
    int color_sensor_stream_index;

    // RealSense2 pipeline, encapsulating the actual device and sensors
    rs2::pipeline pipe;
    rs2::config config;
    rs2::pipeline_profile profile;

    // After enable depth/color streams we can retrieve camera parameters for generating the pointcloud
    rs2_intrinsics depth_intrin;
    rs2_intrinsics color_intrin;

    // Realsense2 pointcloud structure
    rs2::pointcloud rs2_pointcloud;
    rs2::points rs2_points;

    // sensors and queues
    bool use_queues;
    rs2::frame_queue queue_depth;
    rs2::frame_queue queue_color;
    std::vector<rs2::sensor> sensors;
};

#endif
