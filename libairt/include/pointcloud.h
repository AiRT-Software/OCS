#pragma once

#include <glm/vec3.hpp>
#include <vector>

#include "stdMessage.h"

using airt::AIRT_Message_Header;
using airt::StdMessage;

#include "../../atreyu/include/interface/d415module.h"

namespace airt
{
class PointCloud
{
public:
  PointCloud(const glm::vec3 &camera_pos, float pitch, float roll, float yaw);

  /**
     * \return the number of points in the cloud
     */
  size_t size() const { return points.size(); }

  /**
     * \return the position of the camera when the cloud was captured, in mm
     */
  glm::vec3 position() const { return cpos; };

  /**
     * \return the yaw of the camera when the cloud was captured, in radians
     */
  float yaw() const { return cyaw; };

  /**
     * \return the pitch of the camera when the cloud was captured, in radians
     */
  float pitch() const { return cpitch; };
  /**
     * \return the roll of the camera when the cloud was captured, in radians
     */
  float roll() const { return croll; };

  void setPoints(const pointNormal *data, size_t numPoints);

  const pointNormal *getPoints() { return &points[0]; };

  /**
     * Writes the point cloud into the given stream (probably a file stream)
     * \param stream the output stream where to write the cloud
     * \return true if it succeded 
     */
  bool serialize(std::ostream &stream);

  /**
     * Reads a point cloud from the input stream and returns a new object
     * \param istream the stream that contains the point cloud
     * \return a shared pointer with the read point cloud. The shared pointer will be null if something failed
     */
  static std::shared_ptr<PointCloud> deserialize(std::istream &stream);

  /**
   * Returns the number of ms since epoch of when the point cloud was created 
   */
  uint64_t timestamp() const { return timestamp_ms; }
private:
  std::vector<pointNormal> points;
  glm::vec3 cpos;            // Position of the camera when this point cloud was captured (see drone.h for an explanation)
  float cpitch, croll, cyaw; // Orientation of the camera when this point cloud was captured
  uint64_t timestamp_ms;

#ifdef TESTING
#include "../../external-libs/googletest/include/gtest/gtest_prod.h"
  FRIEND_TEST(PointCloudDatabase, RejectMalformedPointClouds);
  FRIEND_TEST(PointCloudDatabase, AcceptPC);
  FRIEND_TEST(PointCloudDatabase, GetPC);
  FRIEND_TEST(PointCloudDatabase, RemovePC);
  FRIEND_TEST(PointCloudDatabase, RemoveBlock);
  FRIEND_TEST(PointCloudDatabase, NonUnitBlocks);
  FRIEND_TEST(PointCloudDatabase, VisitAll);
  FRIEND_TEST(PointCloudDatabase, GetPCId);
  FRIEND_TEST(PointCloudDatabase, GetPCId_2m);
#endif
};
}; // namespace airt