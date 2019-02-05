#pragma once

#include <unordered_map>
#include <memory>
#include <functional>
#include <glm/vec3.hpp>

namespace airt
{

class PointCloud;


/**
 * 
 * \class PointCloudDatabase
 * This class is used during the mapping phase. It will contain all the point clouds captured during the flight.
 * The client of this class will provide all the point clouds captured by the camera, and this class will decide which
 * point clouds will store and which clouds will ignore.
 * There are mainly two parameters for rejecting a cloud: the number of points must be within a range (not too many, not too few)
 * and the position of the camera should be centered in a block (the distance to the border of a block should be greater than a 
 * threshold).
 */
class PointCloudDatabase
{
  public:
  /**
   * Constructor of the database 
   * \param minPointsPerCloud the minimum number of points a cloud should have to consider it valid
   * \param maxPointsPerCloud the maximum number of points a cloud should have to be valid
   * \param blocksize the size in meters of each block in which the space is quantized (e.g., 1.0 -> blocks of 1x1x1 m)
   * \param minDistanceToBorder the position of the camera should be at least blocksize*minDistanceToBorder m from a border to be valid
   */
    PointCloudDatabase(size_t minPointsPerCloud = 100, size_t maxPointsPerCloud = 20000, float blocksize_meters = 1.0f, float minDistanceToBorder = 0.01f, uint32_t expirationTime_ms = 3000);

    enum class PCDB_AcceptResult
    {
        Inserted,    // there was no equivalent point cloud in the database and it was inserted
        Replaced, // there was a point cloud in the database from the same view point, but we replaced it with the new one
        Ignored,    // there was a point cloud in the database from the samae view point, so the new one was ignored
        Rejected    // the provided point cloud is rejected because we consider it low quality
    };
    /**
     * Use this method to try to insert a new point clound in the database.
     * The method will check if there was a point cloud in the database and, it will decide if it keeps the provided point cloud or not.
     * The possible results of this method are:
     *  -the new point cloud was not in the DDBB, so it was inserted
     *  -the new point cloud was in the DDBB, and it was better that the existing one, so it was replaced
     *  -the new point cloud was in the DDBB, but it was not better than the existing one, so the new cloud was ignored
     *  -the new point cloud is considered to be low quality
     * 
     */
    PCDB_AcceptResult accept(std::shared_ptr<PointCloud> pc);

    /**
     * Indicates the main horizontal direction (as given by the yaw)
     */
    enum Heading
    {
        NORTH = 0,
        WEST = 63,
        SOUTH = 98,
        EAST = 125
    };

    /**
     * Identifies a point cloud in the database
     */
    struct PointCloudId
    {
        int i, j, k;
        uint8_t heading;
        bool operator ==(const PointCloudId &other) const {
            return other.i == i && other.j == j && other.k == k && other.heading == heading;
        };
    };

    /**
     * Discards all the clouds
     */
    void clear();
    /**
     * Remove all point clouds associated with this block
     * \param pcid point cloud id of the block (the heading is ignored)
     * \return the number of clouds deleted
     */
    size_t removePointCloudsInBlock(const PointCloudId &pcid);

    /**
     * Remove a point cloud from a block
     * \param point cloud id of the point cloud to remove
     * \return true if the cloud was in the database and was removed
     */
    bool removePointCloud(const PointCloudId &pcid);

    /**
     * \return the id of the given point cloud, taken into account its position and yaw
     * \param pc the point cloud
     */
    PointCloudId getPointCloudId(std::shared_ptr<PointCloud> pc) const;

    /**
     * \return the id of the given point cloud, taken into account its position and yaw
     */
    PointCloudId getPointCloudId(const glm::vec3 &position, float yaw) const;

    /**
     * \return the requested point cloud (or null if there is none with the requested id)
     */
    std::shared_ptr<PointCloud> getPointCloud(const PointCloudId &pcid) const;

    /**
     * \return the number of point clouds in the database
     */
    size_t size() const { return ddbb.size(); }

    /**
     * Invokes the given function on all the point clouds in the database (in no particular order)
     * \param callback the function that will receive one by one all the clouds in the database. If the function returns false, no 
     * other point cloud will be visited
     * \return true if all the callbacks return true, false otherwise
     */
    bool visitAll(std::function<bool (std::shared_ptr<PointCloud> pc)> callback);

    /**
     * Save the database to a file with the given file name. 
     */
    bool save(const std::string &filename);

    /**
     * Load the database from the given base name (destroys the current database)
     */
    bool load(const std::string &filename);

  private:
    size_t minPointsInCloud, maxPointsInCloud; // clouds with less than the minimum or more than the maximum number of points will be discarded
    float blockSizeIn_mm, minDistanceToBorders;
    uint32_t expirationPeriod_ms;
    /**
     * Applies validation checks to reject non acceptable clouds 
     * \return true if the point cloud is valid
     */
    bool isValid(std::shared_ptr<PointCloud> pc) const;

    /**
     * \return true if newPC  is "better" than oldPC
     */
    bool isBetterThan(std::shared_ptr<PointCloud> newPC, std::shared_ptr<PointCloud> oldPC);

    class PointCloudIdHash
    {
      public:
        size_t operator()(const PointCloudId &pcid) const
        {
            return  pcid.i ^ (~pcid.j) ^ pcid.k ^ (pcid.heading << 8 | pcid.heading);
        }
    };

    std::unordered_map < PointCloudId, std::shared_ptr<PointCloud>, PointCloudIdHash> ddbb;
};

}; // namespace airt