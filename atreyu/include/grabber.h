#ifndef RSGRABBER_H
#define RSGRABBER_H

#include <ostream>
#include <librealsense/rs.hpp>

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>


typedef pcl::PointXYZRGB PointT;

class Grabber
{
private:
  Grabber(const Grabber &other) = delete;
  Grabber &operator=(Grabber other) = delete;

  // RealSenseContext owns the handles to all connected realsense devices
  static rs::context ctx;
  rs::device *dev;

  // After enable depth/color streams we can retrieve camera parameters for generating the pointcloud
  rs::intrinsics depth_intrin;
  rs::extrinsics depth_to_color;
  rs::intrinsics color_intrin;
  float depth_scale;

  // RSStreams: Resolution and FPS values
  int depth_w;
  int depth_h;
  int color_w;
  int color_h;
  int camera_fps;

  /**
   * @brief motion_callback is used for saving the motion_callback for receiving
   * frames from ZR300 accelerometer and gyroscope. It is assigned NULL in the
   * Grabber constructor. You must set it with setMotionCallaback for enabling
   * motion_tracking before the start command.
   */
  std::function<void(rs::motion_data)> motion_callback;

public:
  enum class Resolution
  {
    Small,
    Medium,
    Large
  };

  /**
   * Grabber constructor. Grabber wraps the code for accessing the camera 
   * \param cameraIndex índice de la cámara a abrir (en caso de que haya más de una). Por defecto usa la 0
   * \param depth_w width of depth image ()
   * \param depthImage: Resolution::Small (320x240), Resolution::Medium (480x360), Resolution::Large (640x480)
   * \param colorImage: Resolution::Small (320x240), Resolution::Medium (640x480), Resoluton::Large (1920x1080, 30fps only)
   * \param fastestFPS: use fastests refresh rate available (60 fps), or not (30 fps)
   */
  Grabber(unsigned int cameraIndex = 0, Resolution depthImage = Resolution::Medium, Resolution colorImage = Resolution::Medium, bool fastestFPS = false);

  /**
   * @brief printRSContextInfo dumps information about the connected realsense devices over the indicated flow
   * @param info flow in which the information will be dumped
   * @return false if no cameras detected, true otherwise
   */
  static bool printRSContextInfo(std::ostream &info);

  /**
   * @brief configureRSStreams configures depth and color streams and it also
   * gets intrinsic and extrinsic values from the sensors for permofing transformations
   * for calculating the 3d location of the points of the pointcloud
   * @return
   */
  bool configureRSStreams();

  /**
   * @brief setMotionCallback saves the callback funtion to be executed when a new
   * motion_data (accelerometer or gyroscope) has arrived from ZR300.
   * You can not set motion_callback after starting streaming.
   * @param mcb function to be executed
   * @return true if done, false otherwise
   */
  bool setMotionCallback(std::function<void(rs::motion_data)> &mcb);

  /**
   * @brief generatePointCloud generates a pointcloud using the depth and color images received from the depth camera
   * @param cloud pointcloud in which the points will be saved
   * @param timestamp time stamp in milliseconds at which the depth-frame was captured.
   * @param organized true for saving all points, false for saving only points within the valid depth range [0.1, 3.5] in meters. False by default.
   * @return true when a pointcloud ends up being generated, false otherwise
   */
  bool generatePointCloud(pcl::PointCloud<PointT>::Ptr cloud, bool organized = false);

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
 * \return true if streaming is on
 * */
  bool isStreaming();
  /**
   * @brief closeRSStreams disables all enabled streams.
   * If motion_callback has been set it also disables motion tracking.
   * @return
   */
  bool closeRSStreams();

  rs::context &getRSContext();
  rs::device *getRSDevice();
};

#endif
