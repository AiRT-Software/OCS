#pragma once
#include <string>

namespace MissionContainer
{
    struct Map;
    struct MapMetadata;
    struct Path;
    struct PathMetadata;
    struct WayPoint;
}

namespace airt
{

class MissionLoader
{
public:
    MissionLoader() = delete;
    
    static bool loadMapMetada(const std::string & filepath, MissionContainer::MapMetadata & map_metadata);
    static bool loadMap(const std::string & filepath, MissionContainer::Map & map);
};

};
