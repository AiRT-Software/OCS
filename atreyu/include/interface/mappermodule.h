
#pragma pack(push, 1)

/*
    This module is in charge of mapping the environment. It will capture the point clouds from the zcamera
    and it will decide which to keep and which to discard, in order to capture the maximum amount of volume in the scene,
    while keeping the redundancy to a minimum.

    The steps for using this module are:

    1. Change the status of atreyu to MAPPING_STATE. (Sending anything to this module in other state will result in an INVALID_STATUS error)
    2. Ask the mapper module to create a map for a mission MAPPER_CREATE_MAP(guid), or MAPPER_LOAD_MAP(guid). With the second option, we will
    be able to continue a previous session.
    3. In this state the system is ready to start capturing data.
    4. Send a MAPPER_START_MAPPING to begin the capture. During the session, you can issue as many MAPPER_PAUSE_MAPPING and 
        MAPPER_START_MAPPING as necessary. 
    5. Whenever the system accepts a new point cloud, it will publish it to the clients MAPPER_POINTCLOUD_NOTIFICATION(i, j, k, h, cloud). 
    Point clouds are identified by 3 int32_t numbers (the index of the block where the camera was located) and one int8_t number 
    (the orientation of the camera). i, j, k can be negative. h is always positive.
    6. Point clouds can be replaced. When the system decides that a new point cloud is better than the existing one, it will publish a notification
    MAPPER_POINTCLOUD_NOTIFICATION(i, j, k, h, cloud). The clients should discard the old cloud and use the new one.
    7. When the process is done, the client should send a MAPPER_DONE_MAPPING (from the PAUSED state). The system will save the map and 
    it will make atreyu go back to IDLE_STATE. The map will appear in the list of maps. If there was an error writing the map, it will send a 
    MAPPER_ERROR_SAVING_DDBB_NOTIFICATION and it will stay in the Ready state.

    There are also managing commands:
    -MAPPER_GET_NUMBER_OF_POINTCLOUDS will return the total number of point clouds
    -MAPPER_GET_POINTCLOUDS_IDS will return a list with the ids of the captured point clouds
    -MAPPER_GET_POINTCLOUD: will publish the required point cloud (or MAPPER_NON_EXISTENT_POINTCLOUD_NOTIFICATION)
    -MAPPER_GET_ALL_POINTCLOUDS: will publish all the existing point clouds (use with caution)
    -MAPPER_DELETE_POINT_CLOUD will delete the given cloud
    -MAPPER_DELETE_ALL_POINT_CLOUDS will discard all the captured clouds
*/

// ACTIONS

// Create a new map for mapping. If the name of the map is in use, it will issue an INVALID_STATUS notification.
// Two-part command.
struct CreateMapForMapping
{
    const AIRT_Message_Header header{StdMessage::MAPPER_MODULE, StdMessage::MAPPER_CREATE_MAP};
};
// Second part: ascii-z with the guid

// Load an existing map for adding more point clouds. If the map does not exists or fails to load, it will issue an INVALID_STATUS notification.
// Two-part command.
struct LoadMapForMapping
{
    const AIRT_Message_Header header{StdMessage::MAPPER_MODULE, StdMessage::MAPPER_LOAD_MAP};
};
// Second part: ascii-z with the guid

// Request a point cloud
struct RequestPointCloud
{
    const AIRT_Message_Header header{StdMessage::MAPPER_MODULE, StdMessage::MAPPER_GET_POINTCLOUD};
    uint8_t space; // not used
    int32_t i, j, k;
    uint8_t heading;
};

// Request a point cloud
struct DeletePointCloud
{
    const AIRT_Message_Header header{StdMessage::MAPPER_MODULE, StdMessage::MAPPER_GET_POINTCLOUD};
    uint8_t space; // not used
    int32_t i, j, k;
    uint8_t heading;
};

// NOTIFICATIONS

// No-payload messages
// {'A', MAPPER_NOTIFICATIONS_MODULE, MAPPER_STARTED_MAPPING_NOTIFICATION}
// {'A', MAPPER_NOTIFICATIONS_MODULE, MAPPER_PAUSE_MAPPING_NOTIFICATION}
// {'A', MAPPER_NOTIFICATIONS_MODULE, MAPPER_DONE_MAPPING_NOTIFICATION}
// {'A', MAPPER_NOTIFICATIONS_MODULE, MAPPER_ALL_POINT_CLOUDS_DELETED_NOTIFICATION}
// {'A', MAPPER_NOTIFICATIONS_MODULE, MAPPER_NON_EXISTENT_POINTCLOUD_NOTIFICATION}

// The map has been created or has been loaded and is ready to map
// Two-part message
struct MapReadyForMapping
{
    const AIRT_Message_Header header{StdMessage::MAPPER_NOTIFICATIONS_MODULE, StdMessage::MAPPER_MAP_READY_NOTIFICATION};
};
// Second part: asciiz with the guid

struct NumberOfPointcloudsInMap
{
    const AIRT_Message_Header header{StdMessage::MAPPER_NOTIFICATIONS_MODULE, StdMessage::MAPPER_NUMBER_POINTCLOUDS_IN_MAP_NOTIFICATION};
    uint8_t space;      // not used
    uint32_t numClouds; // number of point clouds
};

// List of point cloud ids. Two-part message
struct ListOfPointcloudIDs
{
    const AIRT_Message_Header header{StdMessage::MAPPER_NOTIFICATIONS_MODULE, StdMessage::MAPPER_LIST_POINTCLOUD_IDS_NOTIFICATION};
    uint8_t space;      // not used
    uint32_t numClouds; // number of point clouds
};
// The second part has an array of numClouds of the following structures,
struct PointCloudID
{
    int32_t i, j, k;
    uint8_t heading;
};

// The indicated point cloud was deleted from the database
struct PointCloudDeleted
{
    const AIRT_Message_Header header{StdMessage::MAPPER_NOTIFICATIONS_MODULE, StdMessage::MAPPER_DELETED_POINTCLOUD_NOTIFICATION};
    uint8_t heading;
    int32_t i, j, k;
};

// A point cloud. Two-part message
// First part
struct PointCloudMessage
{
    const AIRT_Message_Header header{StdMessage::MAPPER_NOTIFICATIONS_MODULE, StdMessage::MAPPER_POINTCLOUD_NOTIFICATION};
    uint8_t space;
    uint32_t numPoints;
    float x, y, z, pitch, roll, yaw;
    struct PointCloudID id;
};
// Second part
// WATCH OUT!!! This should be exactly the same as struct pointNormal in d415module.h
struct PointNormal
{
    float x, y, z;
    uint8_t r, g, b, a;
    float normal_x, normal_y, normal_z;
};

// Two-part message. Provides the guid of the current map
struct CurrentGuid
{
    const AIRT_Message_Header header{StdMessage::MAPPER_NOTIFICATIONS_MODULE, StdMessage::MAPPER_CURRENT_GUID_NOTIFICATION};
};
// Second part: asciiz with the guid

#pragma pack(pop)
