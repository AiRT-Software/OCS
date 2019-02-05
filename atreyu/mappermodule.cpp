#include <log.h>
#include <utils.h>
#include <stdMessage.h>
#include <glm/vec3.hpp>

#include "pointclouddatabase.h"
#include "mappermodule.h"
#include "globalSettings.h"

using airt::AIRT_Message_Header;
using airt::StdMessage;

#include "interface/atreyumodule.h"
//#include "interface/d415module.h"

#include "pointcloud.h"

using airt::GlobalSettings;
using airt::Log;
using airt::MapperModule;
using airt::Message;

MapperModule::MapperModule(const std::string &portname, const std::string &pubname, const std::string &subname,
                           std::shared_ptr<Context> context)
    : Module(portname, pubname, subname, context)
{
    assert(context);

    uint32_t minPointsPerCloud;
    if (!GlobalSettings::getValue("mapper_min_points_per_cloud", minPointsPerCloud))
    {
        minPointsPerCloud = 100;
        Log::warn("Minimum number of points per cloud not defined (mapper_min_points_per_cloud). Set to {} points", minPointsPerCloud);
    }

    uint32_t maxPointsPerCloud;
    if (!GlobalSettings::getValue("mapper_max_points_per_cloud", maxPointsPerCloud))
    {
        maxPointsPerCloud = 20000;
        Log::warn("Maximum number of points per cloud not defined (mapper_max_points_per_cloud). Set to {} points", minPointsPerCloud);
    }

    double metersPerBlock;
    if (!GlobalSettings::getValue("mapper_meters_per_block", metersPerBlock))
    {
        metersPerBlock = 1.0;
        Log::warn("Mapper blocks size not defined (mapper_meters_per_block). Set to {} s", metersPerBlock);
    }

    double minDistanceToBorder;
    if (!GlobalSettings::getValue("mapper_min_distance_to_border", minDistanceToBorder))
    {
        minDistanceToBorder = 0.1;
        Log::warn("Minimum distance to border not defined (mapper_min_distance_to_border). Set to {}*blockSize", minDistanceToBorder);
    }

    state = State::NonInitialized;
    ddbb = std::unique_ptr<airt::PointCloudDatabase>(new airt::PointCloudDatabase(minPointsPerCloud, maxPointsPerCloud, metersPerBlock, minDistanceToBorder));
    Log::info("Mapper module module initialized");
}

MapperModule::~MapperModule()
{
}

void MapperModule::configureSubscriptions(const std::string &mainpublisher)
{
    assert(subsocket);
    subsocket->connect(mainpublisher);
    uint8_t zcamsignature[]{'A', StdMessage::POINTCLOUD_NOTIFICATIONS_MODULE};
    subsocket->subscribe(zcamsignature, airt::arraySize(zcamsignature));
    uint8_t atreyusignature[]{'A', StdMessage::ATREYU_NOTIFICATIONS_MODULE};
    subsocket->subscribe(atreyusignature, airt::arraySize(atreyusignature));
}

void MapperModule::publishPointCloud(const glm::vec3 &pos, float roll, float pitch, float yaw, size_t numPoints, const void *points)
{
    PointCloudMessage pm;

    pm.numPoints = numPoints;
    pm.x = pos.x;
    pm.y = pos.y;
    pm.z = pos.z;
    pm.pitch = pitch;
    pm.roll = roll;
    pm.yaw = yaw;
    auto id = ddbb->getPointCloudId(pos, yaw);
    pm.id.i = id.i;
    pm.id.j = id.j;
    pm.id.k = id.k;
    pm.id.heading = id.heading;

    airt::sendTwoPartMessage(*pubsocket, &pm, points, sizeof(pointNormal) * numPoints);
}

void MapperModule::onNotification(const Message &m)
{
    uint8_t module = airt::getMessageModule(m);
    uint8_t action = airt::getMessageAction(m);

    if (state == State::NonInitialized)
    {
        // the module is not active
        if (module == StdMessage::ATREYU_NOTIFICATIONS_MODULE && action == StdMessage::ATREYU_SYSTEM_STATE_CHANGE_NOTIFICATION)
        {
            // are we going to map?
            const auto *sc = m.get<const AtreyuStateChanged *>(0);
            if (sc->newstate != StdMessage::MAPPING_STATE)
                return;
        }
        else
            return;
    }

    switch (module)
    {

    case StdMessage::ATREYU_NOTIFICATIONS_MODULE:
        if (action == StdMessage::ATREYU_SYSTEM_STATE_CHANGE_NOTIFICATION)
        {
            const auto *sc = m.get<const AtreyuStateChanged *>(0);
            if (sc->newstate == StdMessage::IDLE_STATE && state != State::Idle)
            {
                Log::info("Mapper module is done");
                reset();
            }
            else if (sc->newstate == StdMessage::MAPPING_STATE && state == State::NonInitialized)
            {
                Log::info("Mapping starts");
                state = State::Idle;
                airt::sendNotification(*pubsocket, StdMessage::MAPPER_NOTIFICATIONS_MODULE, StdMessage::MAPPER_IDLE_NOTIFICATION);
            }
        }
        break;
    case StdMessage::POINTCLOUD_NOTIFICATIONS_MODULE:
    {
        switch (action)
        {
        case StdMessage::PCL_SINGLE:
        {
            if (state != State::Mapping)
                return;
            const auto *pc = m.get<const pointCloudHeader *>(0);
            glm::vec3 pos{pc->x, pc->y, pc->z};
            auto pointcloud = std::make_shared<airt::PointCloud>(pos, pc->pitch, pc->roll, pc->yaw);
            pointcloud->setPoints(m.get<const pointNormal *>(1), pc->numPoints);
            auto result = ddbb->accept(pointcloud);
            switch (result)
            {
            case PointCloudDatabase::PCDB_AcceptResult::Replaced:
            case PointCloudDatabase::PCDB_AcceptResult::Inserted:
            {
                // republish
                publishingQueue.push_back(pointcloud);
                Log::info("Point cloud accepted. Total number of clouds: {}", ddbb->size());
            }
            break;
            case PointCloudDatabase::PCDB_AcceptResult::Ignored:
                Log::info("Point cloud ignored");
                break;
            case PointCloudDatabase::PCDB_AcceptResult::Rejected:
                Log::info("Point cloud rejected");
                break;
            }
        }
        break;
        case StdMessage::PCL_CAPTURING_STARTED:
        {
            if (state != State::Ready)
            {
                Log::error("Who started the zcam???");
                return;
            }
            state = State::Mapping;
        }
        break;
        case StdMessage::PCL_STOP_CAPTURING:
        {

            if (state != State::Mapping || state != State::Paused)
            {
                Log::error("Who stopped the zcam???");
                return;
            }
        }
        break;
        }
    }
    break;
    default:
        Log::critical("Error: message from unexpected module");
    }
}

void MapperModule::reset()
{
    state = State::NonInitialized;

    guid.clear();
    ddbb->clear();
}

bool MapperModule::checkDDBB()
{
    if (!ddbb)
    {
        airt::sendNotification(*pubsocket, StdMessage::MAPPER_NOTIFICATIONS_MODULE, StdMessage::INVALID_STATUS);
        Log::error("There are no point cloud database");
        return false;
    }
    return true;
}

void MapperModule::onMessage(const Message &inmsg)
{
    if (state == State::NonInitialized)
    {
        Log::error("Message received when not in MAPPING mode {}", airt::to_string(inmsg));
        airt::sendNotification(*pubsocket, StdMessage::MAPPER_NOTIFICATIONS_MODULE, StdMessage::INVALID_STATUS);
        return;
    }
    auto action = airt::getMessageAction(inmsg);
    switch (action)
    {
    case StdMessage::MAPPER_CREATE_MAP:
    {
        // Only in idle
        if (state != State::Idle)
        {
            airt::sendNotification(*pubsocket, StdMessage::MAPPER_NOTIFICATIONS_MODULE, StdMessage::INVALID_STATUS);
            Log::error("Cannot create a new map if there is an existing session");
            return;
        }
        // The second part of the message has the guid
        guid = inmsg.get<const char *>(1);
        state = State::Ready;
        MapReadyForMapping mr;
        airt::sendTwoPartMessage(*pubsocket, &mr, guid);
        Log::info("New map created for mapping: {}", guid);
    }
    break;
    case StdMessage::MAPPER_LOAD_MAP:
    {
        // Only in idle
        if (state != State::Idle)
        {
            airt::sendNotification(*pubsocket, StdMessage::MAPPER_NOTIFICATIONS_MODULE, StdMessage::INVALID_STATUS);
            Log::error("Cannot load a map if there is an existing session");
            return;
        }
        std::string guid = inmsg.get<const char *>(1);

        std::string plibdir;
        GlobalSettings::getValue("plan_lib_library_dir", plibdir);
        std::string filename = plibdir + "/" + guid + ".dpl.map";
        Log::info("Loading map {} {}", guid, filename);
        if (!ddbb->load(filename))
        {
            airt::sendNotification(*pubsocket, StdMessage::MAPPER_NOTIFICATIONS_MODULE, StdMessage::INVALID_STATUS);
            Log::error("Error loading map {}", filename);
            return;
        }
        state = State::Ready;
        MapReadyForMapping mr;
        airt::sendTwoPartMessage(*pubsocket, &mr, guid);
        Log::info("Load map created and ready to map: {}", guid);
    }
    break;
    case StdMessage::MAPPER_START_MAPPING:
    {
        if (state != State::Ready)
        {
            airt::sendNotification(*pubsocket, StdMessage::MAPPER_NOTIFICATIONS_MODULE, StdMessage::INVALID_STATUS);
            Log::error("The map has to be created before start mapping");
            return;
        }
        // Send the camera the message to start and change the status when the camera acknowledges
        MsgToOtherModule header(StdMessage::MAPPER_MODULE);
        StdMessage startCapturing(StdMessage::POINTCLOUD_MODULE, StdMessage::PCL_START_CAPTURING);
        airt::sendTwoPartMessage(*cmdsocket, &header, &startCapturing, sizeof(startCapturing));

        Log::info("Started mapping");
        airt::sendNotification(*pubsocket, StdMessage::MAPPER_NOTIFICATIONS_MODULE, StdMessage::MAPPER_STARTED_MAPPING_NOTIFICATION);
    }
    break;
    case StdMessage::MAPPER_PAUSE_MAPPING:
    {
        if (state != State::Mapping)
        {
            airt::sendNotification(*pubsocket, StdMessage::MAPPER_NOTIFICATIONS_MODULE, StdMessage::INVALID_STATUS);
            Log::error("We cannot pause mapping because we weren't mapping");
            return;
        }
        Log::info("Pausing mapping");
        airt::sendNotification(*pubsocket, StdMessage::MAPPER_NOTIFICATIONS_MODULE, StdMessage::MAPPER_PAUSE_MAPPING_NOTIFICATION);
        state = State::Paused;
    }
    break;
    case StdMessage::MAPPER_RESUME_MAPPING:
    {
        if (state != State::Paused)
        {
            airt::sendNotification(*pubsocket, StdMessage::MAPPER_NOTIFICATIONS_MODULE, StdMessage::INVALID_STATUS);
            Log::error("We cannot resume mapping because we weren't paused");
            return;
        }
        Log::info("Resuming mapping");
        airt::sendNotification(*pubsocket, StdMessage::MAPPER_NOTIFICATIONS_MODULE, StdMessage::MAPPER_STARTED_MAPPING_NOTIFICATION);
        state = State::Mapping;
    }
    break;

    case StdMessage::MAPPER_DONE_MAPPING:
    {
        if (state != State::Paused)
        {
            airt::sendNotification(*pubsocket, StdMessage::MAPPER_NOTIFICATIONS_MODULE, StdMessage::INVALID_STATUS);
            Log::error("We cannot finish mapping because we weren't paused");
            return;
        }
        // Send the camera the message to stop
        MsgToOtherModule header(StdMessage::MAPPER_MODULE);
        StdMessage stopCapturing(StdMessage::POINTCLOUD_MODULE, StdMessage::PCL_STOP_CAPTURING);
        airt::sendTwoPartMessage(*cmdsocket, &header, &stopCapturing, sizeof(stopCapturing));

        std::string plibdir;
        GlobalSettings::getValue("plan_lib_library_dir", plibdir);
        std::string filename = plibdir + "/" + guid + ".dpl.map";
        Log::info("Saving map {} {}", guid, filename);

        if (!ddbb->save(filename))
        {
            airt::sendNotification(*pubsocket, StdMessage::MAPPER_NOTIFICATIONS_MODULE, StdMessage::MAPPER_ERROR_SAVING_DDBB_NOTIFICATION);
            Log::error("We couldn't save the database");
            state = State::Ready;
            return;
        }

        Log::info("Mapper: done");
        airt::sendNotification(*pubsocket, StdMessage::MAPPER_NOTIFICATIONS_MODULE, StdMessage::MAPPER_DONE_MAPPING_NOTIFICATION);

        // Sent atreyu to the IDLE_STATE
        StdMessage gotoIdle(StdMessage::ATREYU_MODULE, StdMessage::ATREYU_LAST_COMMAND + 1);
        airt::sendTwoPartMessage(*cmdsocket, &header, &gotoIdle, sizeof(gotoIdle));

        reset();
    }
    break;
    case StdMessage::MAPPER_GET_NUMBER_OF_POINTCLOUDS:
    {
        if (!checkDDBB())
            return;

        NumberOfPointcloudsInMap npc;
        npc.numClouds = ddbb->size();
        airt::sendOnePartMessage(*pubsocket, &npc);
        Log::info("Publishing the number of clouds in database ({})", ddbb->size());
    }
    break;
    case StdMessage::MAPPER_GET_POINTCLOUDS_IDS:
    {
        if (!checkDDBB())
            return;

        std::vector<PointCloudID> pcids;
        ddbb->visitAll([&pcids, this](std::shared_ptr<airt::PointCloud> cloud) {
            auto id = ddbb->getPointCloudId(cloud);
            pcids.push_back({id.i, id.j, id.k, id.heading});
            return true;
        });

        ListOfPointcloudIDs lids;
        lids.numClouds = pcids.size();
        airt::sendTwoPartMessage(*pubsocket, &lids, &pcids[0], sizeof(PointCloudID) * pcids.size());
        Log::info("Publishing the ids of the point clouds");
    }
    break;
    case StdMessage::MAPPER_GET_POINTCLOUD:
    {
        if (!checkDDBB())
            return;
        auto rpc = inmsg.get<const RequestPointCloud *>(0);
        PointCloudDatabase::PointCloudId id{rpc->i, rpc->j, rpc->k, rpc->heading};

        auto pc = ddbb->getPointCloud(id);
        if (!pc)
        {
            Log::error("The requested point cloud does not exists");
            airt::sendNotification(*pubsocket, StdMessage::MAPPER_NOTIFICATIONS_MODULE, StdMessage::MAPPER_NON_EXISTENT_POINTCLOUD_NOTIFICATION);
            return;
        }
        else
        {
            publishingQueue.push_back(pc);
            Log::info("Requested cloud published");
        }
    }
    break;
    case StdMessage::MAPPER_GET_ALL_POINTCLOUDS:
    {
        if (!checkDDBB())
            return;
        Log::info("Publishing all clouds");
        publishingQueue.clear();
        ddbb->visitAll([this](std::shared_ptr<airt::PointCloud> cloud) {
            publishingQueue.push_back(cloud);
            return true;
        });
    }
    break;
    case StdMessage::MAPPER_DELETE_POINT_CLOUD:
    {
        if (!checkDDBB())
            return;
        const auto del = inmsg.get<const DeletePointCloud *>(0);
        PointCloudDatabase::PointCloudId id{del->i, del->j, del->k, del->heading};
        if (ddbb->removePointCloud(id))
        {
            Log::info("Point cloud {}, {}, {}, {} deleted", del->i, del->j, del->k, del->heading);
            PointCloudDeleted pcd;
            pcd.heading = del->heading;
            pcd.i = del->i;
            pcd.j = del->j;
            pcd.k = del->k;
            airt::sendOnePartMessage(*pubsocket, &pcd);
        }
        else
        {
            Log::error("Could not delete point cloud {}, {}, {}, {}. It does not exists", del->i, del->j, del->k, del->heading);
        }
    }
    break;

    case StdMessage::MAPPER_REQUEST_CURRENT_GUID:
    {
        if (!ddbb || state == State::NonInitialized || state == State::Idle)
        {
            Log::error("Requested the map guid when not mapping");
            airt::sendNotification(*pubsocket, StdMessage::MAPPER_NOTIFICATIONS_MODULE, StdMessage::INVALID_STATUS);
            return;
        }
        CurrentGuid cguid;
        Log::info("Publishing the current guid");
        airt::sendTwoPartMessage(*pubsocket, &cguid, guid);
    }
    break;
    case StdMessage::MAPPER_REQUEST_CURRENT_STATE:
    {
        if (!ddbb || state == State::NonInitialized)
        {
            Log::error("Requested mapping state when not mapping");
            airt::sendNotification(*pubsocket, StdMessage::MAPPER_NOTIFICATIONS_MODULE, StdMessage::INVALID_STATUS);
            return;
        }
        switch (state)
        {
        case State::Idle:
            airt::sendNotification(*pubsocket, StdMessage::MAPPER_NOTIFICATIONS_MODULE, StdMessage::MAPPER_IDLE_NOTIFICATION);
            break;
        case State::Ready:
            airt::sendNotification(*pubsocket, StdMessage::MAPPER_NOTIFICATIONS_MODULE, StdMessage::MAPPER_MAP_READY_NOTIFICATION);
            break;
        case State::Mapping:
            airt::sendNotification(*pubsocket, StdMessage::MAPPER_NOTIFICATIONS_MODULE, StdMessage::MAPPER_STARTED_MAPPING_NOTIFICATION);
            break;
        case State::Paused:
            airt::sendNotification(*pubsocket, StdMessage::MAPPER_NOTIFICATIONS_MODULE, StdMessage::MAPPER_PAUSE_MAPPING_NOTIFICATION);
            break;
        default:
            airt::sendNotification(*pubsocket, StdMessage::MAPPER_NOTIFICATIONS_MODULE, StdMessage::INVALID_STATUS);
            break;
        }
    }
    break;

    case StdMessage::MAPPER_DELETE_ALL_POINT_CLOUDS:
        if (!checkDDBB())
            return;

        ddbb->clear();
        airt::sendNotification(*pubsocket, StdMessage::MAPPER_NOTIFICATIONS_MODULE, StdMessage::MAPPER_ALL_POINT_CLOUDS_DELETED_NOTIFICATION);
        Log::info("All point clouds deleted");
        break;
    }
}

void MapperModule::onIdle()
{
    if (publishingQueue.empty())
    {
        Module::onIdle();
    }
    else
    {
        // Limit the number of clouds published at a time (for not stalling the thread)
        for (size_t i = 0; !publishingQueue.empty() && i < maxNumberCloudsPublishedPerCycle; i++)
        {
            auto pc = publishingQueue.front();
            publishingQueue.pop_front();
            publishPointCloud(pc->position(), pc->roll(), pc->pitch(), pc->yaw(), pc->size(), pc->getPoints());
        }
    }
}
