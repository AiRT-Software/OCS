#pragma once
#include <iostream>
#include <vector>
#include <boost/optional.hpp>

namespace MissionContainer
{

enum WaypointType
{
    TAKEOFF = 0,
    WAYPOINT,
    STOP,
    LAND
};

struct WayPoint
{
    int id;
    float x, y, z;  // meters
    float speed;  // m/s
    WaypointType type;
    uint32_t stopTime_ms;  // if (type == STOP), wait in this wp for stopTime_ms ms
};

struct GimbalParameters
{
    uint32_t wpId;     
    float x_pitch, y_roll, z_yaw; // poi position in map coordinates, or direction in map coordinates (radians)
    uint8_t mode;   // see GimbalMode below
};

struct GimbalWaypoint 
{
    float pitch, roll, yaw;
    float speed;
};


enum GimbalMode {
    LOOK_AT_POI,    // always looks at a given coordinate
    LOOK_AHEAD,     // follows the tangent of the trajectory
    LOOK_AHEAD_FIX_PITCH,   // follows the tangent of the trajectory, but the pitch is fixed at the given value
    BLOCK_DIRECTION     // locks the gimbal at the given direction
};

/*
 See atreyu/include/interface/reccammodule.h for the definition of the commands
The commands in here have no signature and module. 
Example:

{'A', RCAM_MODULE, RCAM_SWITCH_TO_REC} -> {RCAM_SWITCH_TO_REC}

{'A', RCAM_MODULE, RCAM_SET_CONFIG, RCAM_CONFIG_WB, 254} -> {RCAM_SET_CONFIG, RCAM_CONFIG_WB, 254}

*/
struct RecCamParameters
{
    uint32_t wpId;
    std::vector<std::vector<uint8_t>> commands;
};



struct AirtDateTime
{
    int year;
    uint8_t month;
    uint8_t day;
    uint8_t hours;
    uint8_t minutes;
    uint8_t seconds;
};

struct PathMetadata
{
    std::string path_name;
    std::string author_name;
    AirtDateTime modification_date;
    std::string user_notes;
    int version_major, version_minor, version_patch;

    PathMetadata & operator =(const PathMetadata & other)
    {
        path_name = other.path_name;
        author_name = other.author_name;
        modification_date = other.modification_date;
        user_notes = other.user_notes;
        version_major = other.version_major;
        version_minor = other.version_minor;
        version_patch = other.version_patch;

        return *this;
    }
};

struct Path
{
    PathMetadata metadata;
    std::vector<WayPoint> waypoints;
    std::vector<RecCamParameters> rcamParams;
    std::vector<GimbalParameters> gimbalParams;
};

enum MapType
{
    PointCloud = 0,
    Model3D,
    EmptyBox
};

struct MapMetadata
{
    std::string guid;  // globally unique identifier generated into the AiRT-application
    std::string name;
    std::string location;  // city name
    AirtDateTime modification_date;
    unsigned long bytes_size;
    MapType type;
    std::string filepath;

    // if map_type == emptybox use emptyboxsize
    float emptyBoxSize_x, emptyBoxSize_y, emptyBoxSize_z;
};

struct Map
{
    std::string guid;  //same as MapMetadata::guid
    std::vector<Path> paths;

    Map(std::string guid_ = "", std::vector<Path> paths_ = {})
        : guid(guid_), paths{std::move(paths_)}
    {};

    const WayPoint &getWaypoint(uint32_t planIndex, uint32_t wpId);
    /**
     * Are there reccam parameters set for this waypoint?
     * \param planIndex which plan in the mission
     * \param wpId the waypoint
     * \return the index of the parameter, or -1 if there is no specific commands (and the previous configuratio should be kept)
     */ 
    int findRecCamParameters(uint32_t planIndex, uint32_t wpId);

    const RecCamParameters &getRecCamParameters(uint32_t planIndex, uint32_t rcamIdx);
    /**
     * Are there gimbal parameters set for this waypoint?
     * \param planIndex which plan in the mission
     * \param wpId the waypoint
     * \return the index of the parameter, or -1 if there is no specific commands (and the previous configuratio should be kept)
     */ 
    int findGimbalParameters(uint32_t planIndex, uint32_t wpId);

    /**
     * Provides the gimbal parameters associated to a given index
     *    
     * 
     */
    const GimbalParameters &getGimbalParameters(uint32_t planIndex, uint32_t gimbalIdx);


    /**
     * Returns a vector with the same number of elements as waypoints.
     * Each element in the vector specifies the configuration of the gimbal for that waypoint.
     * \param planIndex plan index
     * \return the gimbal flight plan (one-to-one with the waypoints)
     */
    std::vector<GimbalWaypoint>  compileGimbalFlightPlan(uint32_t planIndex);

    void effectiveGimbalPosition(uint32_t planIndex, uint32_t gimbalIdx, uint32_t waypointIdx, float &pitch, float &roll, float &yaw);
};
}
