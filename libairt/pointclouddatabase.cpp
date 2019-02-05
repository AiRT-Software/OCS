#include <fstream>

#include "pointclouddatabase.h"
#include "log.h"

#include <stdMessage.h>
#include <utils.h>

using airt::StdMessage;

#include "pointcloud.h"
#include "glm/trigonometric.hpp"
#include "glm/common.hpp"

using airt::Log;
using airt::PointCloud;
using airt::PointCloudDatabase;

// If the new point cloud has 5% more points than the existing one, we replace it
#define SIZE_INCREASE_FOR_REPLACE 1.05

PointCloudDatabase::PointCloudDatabase(size_t minPointsPerCloud, size_t maxPointsPerCloud, float blocksize_meters, float minDistanceToBorder, uint32_t expirationTime_ms)
    : minPointsInCloud(minPointsPerCloud),
      maxPointsInCloud(maxPointsPerCloud),
      blockSizeIn_mm(blocksize_meters * 1000.0f),
      minDistanceToBorders(minDistanceToBorder),
      expirationPeriod_ms(expirationTime_ms)
{
    ddbb.reserve(1000);
};

bool PointCloudDatabase::isValid(std::shared_ptr<PointCloud> pc) const
{
    // Does it have the required number of points?
    if (pc->size() < minPointsInCloud || pc->size() > maxPointsInCloud)
    {
        Log::info("Cloud rejected because its size {}", pc->size());
        return false;
    }

    // Is the position of the camera well into the center of the block?
    auto f = glm::fract(pc->position() / blockSizeIn_mm);
    auto b = glm::lessThan(f, glm::vec3(minDistanceToBorders));
    if (glm::any(b))
    {
        Log::info("Cloud rejected because it is too close to a block border");
        return false;
    }
    b = glm::greaterThan(f, glm::vec3(1.0f - minDistanceToBorders));
    if (glm::any(b))
    {
        Log::info("Cloud rejected because it is too close to a block border");
        return false;
    }
    return true;
}

PointCloudDatabase::PCDB_AcceptResult PointCloudDatabase::accept(std::shared_ptr<PointCloud> pc)
{
    // Early reject conditions
    if (!isValid(pc))
        return PCDB_AcceptResult::Rejected;

    auto id = getPointCloudId(pc);
    auto old = getPointCloud(id);
    if (!old)
    {
        // It wasn't in the ddbb: insert it
        ddbb[id] = pc;
        return PCDB_AcceptResult::Inserted;
    }
    else
    {
        // It was in the ddbb: check if this cloud is "better" than the previous one
        if (isBetterThan(pc, old))
        {
            // replace
            ddbb[id] = pc;
            return PCDB_AcceptResult::Replaced;
        }
        else
        {
            // ignore
            return PCDB_AcceptResult::Ignored;
        }
    }
}

/**
 * For the moment, we will consider a cloud better than other, if it has more points.
 * We should also consider how aligned with the main direction are. Then, we have to combine both metrics to
 * decide whether to keep the old cloud or replace it with the new one
 */
bool PointCloudDatabase::isBetterThan(std::shared_ptr<PointCloud> newPC, std::shared_ptr<PointCloud> oldPC)
{
    return (oldPC->timestamp() < airt::msSinceEpoch() - expirationPeriod_ms);
}
// bool PointCloudDatabase::isBetterThan(std::shared_ptr<PointCloud> newPC, std::shared_ptr<PointCloud> oldPC)
// {
//     return oldPC->size() * SIZE_INCREASE_FOR_REPLACE < newPC->size() ||
//            (oldPC->timestamp() < airt::msSinceEpoch() - expirationPeriod_ms &&
//             newPC->size() >= (oldPC->size() * 8) / 10);
// }

PointCloudDatabase::PointCloudId PointCloudDatabase::getPointCloudId(const glm::vec3 &position, float yaw) const
{
    PointCloudDatabase::PointCloudId id;

    auto p = glm::floor(position / blockSizeIn_mm);
    id.i = static_cast<int>(p.x);
    id.j = static_cast<int>(p.y);
    id.k = static_cast<int>(p.z);

    if (yaw < 0.0f)
        yaw += M_PI * 2.0;

    if (yaw > glm::radians(45.0f) && yaw <= glm::radians(135.0f))
        id.heading = PointCloudDatabase::WEST;
    else if (yaw > glm::radians(135.0f) && yaw <= glm::radians(225.0f))
        id.heading = PointCloudDatabase::SOUTH;
    else if (yaw > glm::radians(225.0f) && yaw <= glm::radians(315.0f))
        id.heading = PointCloudDatabase::EAST;
    else
        id.heading = PointCloudDatabase::NORTH;

    return id;
}

PointCloudDatabase::PointCloudId PointCloudDatabase::getPointCloudId(std::shared_ptr<PointCloud> pc) const
{
    return getPointCloudId(pc->position(), pc->yaw());
}

std::shared_ptr<PointCloud> PointCloudDatabase::getPointCloud(const PointCloudId &pcid) const
{
    auto pc = ddbb.find(pcid);
    if (pc == ddbb.end())
    {
        return std::shared_ptr<PointCloud>();
    }
    else
    {
        return pc->second;
    }
}

void PointCloudDatabase::clear()
{
    ddbb.clear();
}

bool PointCloudDatabase::removePointCloud(const PointCloudId &pcid)
{
    return ddbb.erase(pcid) > 0;
}

size_t PointCloudDatabase::removePointCloudsInBlock(const PointCloudId &pcid)
{
    size_t result = 0;
    auto id = pcid;
    id.heading = NORTH;
    if (removePointCloud(id))
        ++result;
    id.heading = WEST;
    if (removePointCloud(id))
        ++result;
    id.heading = SOUTH;
    if (removePointCloud(id))
        ++result;
    id.heading = EAST;
    if (removePointCloud(id))
        ++result;

    return result;
}

bool PointCloudDatabase::visitAll(std::function<bool(std::shared_ptr<PointCloud> pc)> callback)
{
    for (auto pc : ddbb)
    {
        if (!callback(pc.second))
            return false;
    }
    return true;
}

bool PointCloudDatabase::save(const std::string &filename)
{
    std::ofstream file;

    file.open(filename, std::ofstream::binary | std::ofstream::out);
    if (!file.is_open())
        return false;

    if (!file.write("APCL", 4))
        return false;
    if (!file.write(reinterpret_cast<const char *>(&minPointsInCloud), sizeof(minPointsInCloud)))
        return false;
    if (!file.write(reinterpret_cast<const char *>(&maxPointsInCloud), sizeof(maxPointsInCloud)))
        return false;
    if (!file.write(reinterpret_cast<const char *>(&blockSizeIn_mm), sizeof(blockSizeIn_mm)))
        return false;
    if (!file.write(reinterpret_cast<const char *>(&minDistanceToBorders), sizeof(minDistanceToBorders)))
        return false;

    auto size = ddbb.size();
    if (!file.write(reinterpret_cast<const char *>(&size), sizeof(size)))
        return false;

    return visitAll([&file](std::shared_ptr<airt::PointCloud> pc) {
        return pc->serialize(file);
    });
}

bool PointCloudDatabase::load(const std::string &filename)
{
    std::ifstream file;

    file.open(filename, std::ifstream::binary | std::ofstream::in);
    if (!file.is_open())
        return false;

    char signature[4];
    if (!file.read(signature, sizeof(signature)))
        return false;
    if (signature[0] != 'A' || signature[1] != 'P' || signature[2] != 'C' || signature[3] != 'L')
        return false;

    if (!file.read(reinterpret_cast<char *>(&minPointsInCloud), sizeof(minPointsInCloud)))
        return false;
    if (!file.read(reinterpret_cast<char *>(&maxPointsInCloud), sizeof(maxPointsInCloud)))
        return false;
    if (!file.read(reinterpret_cast<char *>(&blockSizeIn_mm), sizeof(blockSizeIn_mm)))
        return false;
    if (!file.read(reinterpret_cast<char *>(&minDistanceToBorders), sizeof(minDistanceToBorders)))
        return false;

    size_t size;
    if (!file.read(reinterpret_cast<char *>(&size), sizeof(size)))
        return false;

    for (size_t i = 0; i < size; i++)
    {
        auto pc = PointCloud::deserialize(file);
        if (pc)
        {
            auto id = getPointCloudId(pc);
            ddbb[id] = pc;
        }
        else
            return false;
    }
    return true;
}