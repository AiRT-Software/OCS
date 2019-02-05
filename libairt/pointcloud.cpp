#include <utils.h>
#include "pointcloud.h"


using airt::PointCloud;


  PointCloud::PointCloud(const glm::vec3 &camera_pos, float pitch, float roll, float yaw) : 
    cpos(camera_pos), cpitch(pitch), croll(roll), cyaw(yaw), timestamp_ms(airt::msSinceEpoch()) {};


bool PointCloud::serialize(std::ostream &stream) {

    if (!stream.write("_PCL", 4))
        return false;

    if (!stream.write(reinterpret_cast<const char *>(&cpos.x), sizeof(cpos.x))) 
        return false;
    if (!stream.write(reinterpret_cast<const char *>(&cpos.y), sizeof(cpos.y))) 
        return false;
    if (!stream.write(reinterpret_cast<const char *>(&cpos.z), sizeof(cpos.z))) 
        return false;
    if (!stream.write(reinterpret_cast<const char *>(&cpitch), sizeof(cpitch))) 
        return false;
    if (!stream.write(reinterpret_cast<const char *>(&croll), sizeof(croll))) 
        return false;
    if (!stream.write(reinterpret_cast<const char *>(&cyaw), sizeof(cyaw))) 
        return false;

    size_t size = points.size();
    if (!stream.write(reinterpret_cast<const char *>(&size), sizeof(size))) 
        return false;
    
    if (!stream.write(reinterpret_cast<const char *>(&points[0]), sizeof(pointNormal)*size))
        return false;
    return true;
}

std::shared_ptr<PointCloud> PointCloud::deserialize(std::istream &stream) {
    
    std::shared_ptr<PointCloud> result;
    char signature[4];
    if (!stream.read(signature, sizeof(signature)))
        return result;
    if (signature[0] != '_' || signature[1] != 'P' || signature[2] != 'C' || signature[3] != 'L')
        return result;

    glm::vec3 cpos;
    float cpitch, croll, cyaw;
    if (!stream.read(reinterpret_cast<char *>(&cpos.x), sizeof(cpos.x))) 
        return result;
    if (!stream.read(reinterpret_cast<char *>(&cpos.y), sizeof(cpos.y))) 
        return result;
    if (!stream.read(reinterpret_cast<char *>(&cpos.z), sizeof(cpos.z))) 
        return result;
    if (!stream.read(reinterpret_cast<char *>(&cpitch), sizeof(cpitch))) 
        return result;
    if (!stream.read(reinterpret_cast<char *>(&croll), sizeof(croll))) 
        return result;
    if (!stream.read(reinterpret_cast<char *>(&cyaw), sizeof(cyaw))) 
        return result;

    size_t size;
    if (!stream.read(reinterpret_cast<char *>(&size), sizeof(size)))
        return result;

    result = std::make_shared<PointCloud>(cpos, cpitch, croll, cyaw);
    result->points.resize(size);

    if(!stream.read(reinterpret_cast<char *>(&result->points[0]), sizeof(pointNormal)*size))
        return std::shared_ptr<PointCloud>();

    return result;
}

void PointCloud::setPoints(const pointNormal *data, size_t numPoints) {
    points.resize(numPoints);
    memcpy(&points[0], data, sizeof(pointNormal)* numPoints);
}