#include "missionLoader.h"
#include "missionContainer.h"
#include <log.h>

using airt::Log;
using airt::MissionLoader;

#include <rapidjson/document.h>
#include <rapidjson/istreamwrapper.h>
#include <rapidjson/stringbuffer.h>
#include <rapidjson/prettywriter.h>
#include <rapidjson/ostreamwrapper.h>
#include <fstream>

using namespace rapidjson;

bool MissionLoader::loadMapMetada(const std::string &filepath, MissionContainer::MapMetadata &map_metadata)
{
    std::ifstream ifs(filepath);
    IStreamWrapper isw(ifs);

    Document doc;
    doc.ParseStream(isw);

    if (doc.HasParseError())
    {
        Log::error("MissionLoader: loadMapMetada: parse error in {}. Maybe file does not exists.", filepath);
        return false;
    }

    Log::info("MissionLoader: reading {} json file", filepath);

    map_metadata.guid = doc["guid"].GetString();
    map_metadata.name = doc["name"].GetString();
    map_metadata.location = doc["location"].GetString();

    const Value &mod_date = doc["modification_date"];
    map_metadata.modification_date.year = mod_date["year"].GetInt();
    map_metadata.modification_date.month = (uint8_t)mod_date["month"].GetInt();
    map_metadata.modification_date.day = (uint8_t)mod_date["day"].GetInt();
    map_metadata.modification_date.hours = (uint8_t)mod_date["hours"].GetInt();
    map_metadata.modification_date.minutes = (uint8_t)mod_date["minutes"].GetInt();
    map_metadata.modification_date.seconds = (uint8_t)mod_date["seconds"].GetInt();

    map_metadata.bytes_size = doc["bytes_size"].GetUint64();
    map_metadata.type = (MissionContainer::MapType)doc["map_type"].GetUint();

    map_metadata.filepath = doc["file_path"].GetString(); // airt application

    map_metadata.emptyBoxSize_x = doc["box_scale"]["x"].GetFloat();
    map_metadata.emptyBoxSize_y = doc["box_scale"]["y"].GetFloat();
    map_metadata.emptyBoxSize_z = doc["box_scale"]["z"].GetFloat();

    return true;
}

bool MissionLoader::loadMap(const std::string &filepath, MissionContainer::Map &map)
{
    std::ifstream ifs(filepath);
    IStreamWrapper isw(ifs);

    Document doc;
    doc.ParseStream(isw);

    if (doc.HasParseError())
    {
        Log::error("MissionLoader: loadMap: parse error in {}. Maybe file does not exists.", filepath);
        return false;
    }

    Log::info("MissionLoader: reading {}", filepath);

    map.guid = doc["guid"].GetString();

    const Value &paths_json = doc["paths"];
    map.paths.resize(paths_json.Size());

    for (size_t pathIdx = 0; pathIdx < map.paths.size(); pathIdx++)
    {
        const Value &p_metadata = paths_json[pathIdx]["path_metadata"];

        map.paths[pathIdx].metadata.author_name = p_metadata["author_name"].GetString();
        map.paths[pathIdx].metadata.path_name = p_metadata["path_name"].GetString();

        const Value &mod_date = p_metadata["modification_date"];
        map.paths[pathIdx].metadata.modification_date.year = mod_date["year"].GetInt();
        map.paths[pathIdx].metadata.modification_date.month = mod_date["month"].GetInt();
        map.paths[pathIdx].metadata.modification_date.day = mod_date["day"].GetInt();
        map.paths[pathIdx].metadata.modification_date.hours = mod_date["hours"].GetInt();
        map.paths[pathIdx].metadata.modification_date.minutes = mod_date["minutes"].GetInt();
        map.paths[pathIdx].metadata.modification_date.seconds = mod_date["seconds"].GetInt();

        map.paths[pathIdx].metadata.user_notes = p_metadata["user_notes"].GetString();
        map.paths[pathIdx].metadata.version_major = p_metadata["version_major"].GetInt();
        map.paths[pathIdx].metadata.version_minor = p_metadata["version_minor"].GetInt();
        map.paths[pathIdx].metadata.version_patch = p_metadata["version_patch"].GetInt();

        const Value &waypoints = paths_json[pathIdx]["waypoints"];
        map.paths[pathIdx].waypoints.resize(waypoints.Size());

        for (size_t wp = 0; wp < map.paths[pathIdx].waypoints.size(); wp++)
        {
            map.paths[pathIdx].waypoints[wp].id = waypoints[wp]["id"].GetInt();
            map.paths[pathIdx].waypoints[wp].speed = waypoints[wp]["speed"].GetFloat();
            map.paths[pathIdx].waypoints[wp].type = (MissionContainer::WaypointType)waypoints[pathIdx]["point_type"].GetInt();
            map.paths[pathIdx].waypoints[wp].x = waypoints[wp]["coordinates"]["x"].GetFloat();
            map.paths[pathIdx].waypoints[wp].y = waypoints[wp]["coordinates"]["y"].GetFloat();
            map.paths[pathIdx].waypoints[wp].z = waypoints[wp]["coordinates"]["z"].GetFloat();
            map.paths[pathIdx].waypoints[wp].stopTime_ms = waypoints[wp]["stopTime"].GetInt();
        }

        const Value &reccamparameters = paths_json[pathIdx]["rcam_update_parameters"];
        map.paths[pathIdx].rcamParams.resize(reccamparameters.Size());

        for (size_t wpup = 0; wpup < map.paths[pathIdx].rcamParams.size(); wpup++)
        {
            MissionContainer::RecCamParameters &wp_update = map.paths[pathIdx].rcamParams[wpup];
            wp_update.wpId = reccamparameters[wpup]["id_pointer"].GetInt();

            const Value &params = reccamparameters[wpup]["reccam_parameters"];
            assert(params.IsArray());

            // reccam command
            wp_update.commands.clear();

            for(size_t rcparam_idx = 0; rcparam_idx < params.Size(); rcparam_idx++)
            {
                const Value & rcparam = params[rcparam_idx]["array"];
                assert(rcparam.IsArray());

                // reccam command bytes
                std::vector<uint8_t> rcbytes;
                for(size_t rcparam_byte = 0; rcparam_byte < rcparam.Size(); rcparam_byte++)
                {
                    rcbytes.push_back((uint8_t)rcparam[rcparam_byte].GetInt());
                }

                wp_update.commands.push_back(rcbytes);
            }
        }

        const Value &gimbalparameters = paths_json[pathIdx]["gimbal_update_parameters"];
        map.paths[pathIdx].gimbalParams.resize(gimbalparameters.Size());
        
        for (size_t wpup = 0; wpup < gimbalparameters.Size(); wpup++)
        {
            MissionContainer::GimbalParameters &wp_update = map.paths[pathIdx].gimbalParams[wpup];

            wp_update.wpId = gimbalparameters[wpup]["id_pointer"].GetInt();
            wp_update.mode = (uint8_t)gimbalparameters[wpup]["mode"].GetInt();

            const Value & gimbal = gimbalparameters[wpup];

            switch (wp_update.mode) {
            case MissionContainer::GimbalMode::LOOK_AT_POI:
                wp_update.x_pitch = gimbal["poi_or_angles"]["x"].GetFloat();
                wp_update.y_roll = gimbal["poi_or_angles"]["y"].GetFloat();
                wp_update.z_yaw = gimbal["poi_or_angles"]["z"].GetFloat();
                break;

            case MissionContainer::GimbalMode::LOOK_AHEAD:
                wp_update.x_pitch = 0.0;
                wp_update.y_roll = 0.0;
                wp_update.z_yaw = 0.0;
                break;

            case MissionContainer::GimbalMode::LOOK_AHEAD_FIX_PITCH:
                wp_update.x_pitch = gimbal["poi_or_angles"]["x"].GetFloat();
                wp_update.y_roll = 0.0;
                wp_update.z_yaw = 0.0;
                break;

            case MissionContainer::GimbalMode::BLOCK_DIRECTION:
                wp_update.x_pitch = gimbal["poi_or_angles"]["x"].GetFloat();  //degrees
                wp_update.y_roll = gimbal["poi_or_angles"]["y"].GetFloat();
                wp_update.z_yaw = gimbal["poi_or_angles"]["z"].GetFloat();
                break;

            default:
                Log::error("MissionLoader: unknown GimbalMode {}", wp_update.mode);
                return false;
            }
        }
    }

    return true;
}
