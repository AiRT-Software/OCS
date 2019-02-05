#include "pozyxMessageProcessor.h"
#include "pozyxResponses.h"
#include "pozyxPayloads.h"

#define MIN_NUM_ANCHORS 8
#define MIN_NUM_TAGS 4
using airt::PozyxMessageProcessor;

#include <rapidjson/document.h>
#include <rapidjson/stringbuffer.h>
#include <rapidjson/istreamwrapper.h>
#include <rapidjson/prettywriter.h>
#include <rapidjson/error/en.h>
#include <fstream>
using namespace rapidjson;

#include <log.h>
using airt::Log;


PozyxMessageProcessor::PozyxMessageProcessor(std::string ip_, std::string httpport_)
    : ip(ip_), httpport(httpport_)
{
    setEndpoints(ip, httpport);
}

PozyxMessageProcessor::~PozyxMessageProcessor()
{
}

void PozyxMessageProcessor::setEndpoints(const std::string & ip, const std::string & httpport)
{
    endpoint_system = "http://" + ip + ":" + httpport + "/v1.0/system";
    endpoint_discover = "http://" + ip + ":" + httpport + "/v1.0/discover";
    endpoint_discoverfull = "http://" + ip + ":" + httpport + "/v1.0/discover/full";
    endpoint_autocalibrate = "http://" + ip + ":" + httpport + "/v1.0/autocalibrate";
    endpoint_updatesettings = "http://" + ip + ":" + httpport + "/v1.0/update-settings";
    endpoint_start = "http://" + ip + ":" + httpport + "/v1.0/start";
    endpoint_stop = "http://" + ip + ":" + httpport + "/v1.0/stop";
    endpoint_kill = "http://" + ip + ":" + httpport + "/v1.0/kill";
}

bool PozyxMessageProcessor::requestSystem(PozyxResponses::System & resp_system)
{
    std::string reply = "";
    if(!rest.httpGet(reply, endpoint_system))
        return false;

    Document doc;
    ParseResult result = doc.Parse(reply.c_str());

    if(!result)
    {
        Log::error("PozyxMessageProcessor: error when parsing requestSystem-response: {}, {}",
                   rapidjson::GetParseError_En(result.Code()), result.Offset());
        return false;
    }

    resp_system.status = doc["status"].GetString();
    if(resp_system.status == "NOT_CONNECTED")
    {
        Log::error("PozyxMessageProcessor: local-tag not connected");
        return false;
    }
    if((resp_system.status == "IDLE" || resp_system.status == "POSITIONING")
            && doc["device"].IsNull())
    {
        Log::error("PozyxMessageProcessor: local-tag is not responding");
        return false;
    }

    resp_system.name = doc["name"].GetString();
    resp_system.version = doc["version"].GetString();

    resp_system.device.id = doc["device"]["id"].GetString();
    resp_system.device.hexId = doc["device"]["hexId"].GetString();
    resp_system.device.fwVersion = doc["device"]["fwVersion"].GetString();
    resp_system.device.hwVersion = doc["device"]["hwVersion"].GetString();
    resp_system.device.distance = doc["device"]["distance"].GetInt();
    resp_system.device.rss = doc["device"]["rss"].GetInt();

    resp_system.device.settings_uwb.channel = doc["device"]["settings"]["uwb"]["channel"].GetInt();
    resp_system.device.settings_uwb.bitrate = doc["device"]["settings"]["uwb"]["bitrate"].GetString();
    resp_system.device.settings_uwb.prf = doc["device"]["settings"]["uwb"]["prf"].GetString();
    resp_system.device.settings_uwb.plen = doc["device"]["settings"]["uwb"]["plen"].GetInt();
    resp_system.device.settings_uwb.gain = doc["device"]["settings"]["uwb"]["gain"].GetFloat();

    return true;
}

bool PozyxMessageProcessor::requestDiscover(PozyxResponses::Discover & resp_discover)
{
    std::string reply = "";

    if(!rest.httpGet(reply, endpoint_discover))
        return false;

    Document doc;
    ParseResult result = doc.Parse(reply.c_str());

    if(!result)
    {
        Log::error("PozyxMessageProcessor: error when parsing requestDiscover-response: {}, {}",
                   rapidjson::GetParseError_En(result.Code()), result.Offset());
        return false;
    }

    resp_discover.success = doc["success"].GetBool();
    if(!resp_discover.success)
    {
        Log::error("PozyxMessageProcessor: requestDiscover had no success");
        return false;
    }

    Log::info("PozyxMessageProcessor: iterating discovered tags");

    const Value & tags = doc["tags"];
    resp_discover.tags.clear();

    for(size_t i=0; i < tags.Size(); i++)
    {
        PozyxPayloads::Tag tag;

        if(tags[i].HasMember("local") && !tags[i]["local"].IsNull())
        {
            tag.isLocal = tags[i]["local"].GetBool();
        }

        if(tags[i].HasMember("id") && !tags[i]["id"].IsNull())
        {
            tag.id = tags[i]["id"].GetString();
        }

        // sometimes pozyx-server does not add this parameter
        if(tags[i].HasMember("rss") && !tags[i]["rss"].IsNull())
        {
            tag.rss = tags[i]["rss"].GetInt();
        }

        if(tags[i].HasMember("distance") && !tags[i]["distance"].IsNull())
        {
            tag.distance = tags[i]["distance"].GetInt();
        }

        if(tags[i].HasMember("hexId") && !tags[i]["hexId"].IsNull())
        {
            tag.hexId = tags[i]["hexId"].GetString();
        }

        // sometimes pozyx-server inserts this value as null
        if(tags[i].HasMember("hwVersion") && !tags[i]["hwVersion"].IsNull())
        {
            tag.hwVersion = tags[i]["hwVersion"].GetString();
        }

        if(tags[i].HasMember("fwVersion") && !tags[i]["fwVersion"].IsNull())
        {
            tag.fwVersion = tags[i]["fwVersion"].GetString();
        }

        if(tags[i].HasMember("settings") && !tags[i]["settings"].IsNull() &&
                tags[i]["settings"].HasMember("uwb") && !tags[i]["settings"]["uwb"].IsNull())
        {
            const Value & tag_uwb = tags[i]["settings"]["uwb"];

            if(tag_uwb.HasMember("channel") && !tag_uwb["channel"].IsNull())
            {
                tag.settings_uwb.channel = tag_uwb["channel"].GetInt();
            }

            if(tag_uwb.HasMember("bitrate") && !tag_uwb["bitrate"].IsNull())
            {
                tag.settings_uwb.bitrate = tag_uwb["bitrate"].GetString();
            }

            if(tag_uwb.HasMember("prf") && !tag_uwb["prf"].IsNull())
            {
                tag.settings_uwb.prf = tag_uwb["prf"].GetString();
            }

            if(tag_uwb.HasMember("plen") && !tag_uwb["plen"].IsNull())
            {
                tag.settings_uwb.plen = tag_uwb["plen"].GetInt();
            }

            if(tag_uwb.HasMember("gain") && !tag_uwb["gain"].IsNull())
            {
                tag.settings_uwb.gain = tag_uwb["gain"].GetFloat();
            }
        }

        resp_discover.tags.push_back(tag);
    }

    Log::info("PozyxMessageProcessor: iterating discovered anchors");

    const Value & anchors = doc["anchors"];
    resp_discover.anchors.clear();

    for(size_t i=0; i < anchors.Size(); i++)
    {
        PozyxPayloads::Anchor anchor;
        if(anchors[i].HasMember("id") && !anchors[i]["id"].IsNull())
        {
            anchor.id = anchors[i]["id"].GetString();
        }

        // sometimes pozyx-server does not add this parameter
        if(anchors[i].HasMember("rss") && !anchors[i]["rss"].IsNull())
        {
            anchor.rss = anchors[i]["rss"].GetInt();
        }

        if(anchors[i].HasMember("distance") && !anchors[i]["distance"].IsNull())
        {
            anchor.distance = anchors[i]["distance"].GetInt();
        }

        if(anchors[i].HasMember("hexId") && !anchors[i]["hexId"].IsNull())
        {
            anchor.hexId = anchors[i]["hexId"].GetString();
        }

        if(anchors[i].HasMember("hwVersion") && !anchors[i]["hwVersion"].IsNull())
        {
            anchor.hwVersion = anchors[i]["hwVersion"].GetString();
        }

        if(anchors[i].HasMember("fwVersion") && !anchors[i]["fwVersion"].IsNull())
        {
            anchor.fwVersion = anchors[i]["fwVersion"].GetString();
        }

        if(anchors[i].HasMember("settings") && !anchors[i]["settings"].IsNull() &&
                anchors[i]["settings"].HasMember("uwb") && !anchors[i]["settings"]["uwb"].IsNull())
        {
            const Value & anchor_uwb = anchors[i]["settings"]["uwb"];

            if(anchor_uwb.HasMember("channel") && !anchor_uwb["channel"].IsNull())
            {
                anchor.settings_uwb.channel = anchor_uwb["channel"].GetInt();
            }

            if(anchor_uwb.HasMember("bitrate") && !anchor_uwb["bitrate"].IsNull())
            {
                anchor.settings_uwb.bitrate = anchor_uwb["bitrate"].GetString();
            }

            if(anchor_uwb.HasMember("prf") && !anchor_uwb["prf"].IsNull())
            {
                anchor.settings_uwb.prf = anchor_uwb["prf"].GetString();
            }

            if(anchor_uwb.HasMember("plen") && !anchor_uwb["plen"].IsNull())
            {
                anchor.settings_uwb.plen = anchor_uwb["plen"].GetInt();
            }

            if(anchor_uwb.HasMember("gain") && !anchor_uwb["gain"].IsNull())
            {
                anchor.settings_uwb.gain = anchor_uwb["gain"].GetFloat();
            }
        }

        resp_discover.anchors.push_back(anchor);
    }

    Log::info("PozyxMessageProcessor: finished extracting values from discover response");

    return true;
}

bool PozyxMessageProcessor::requestDiscoverFull(PozyxResponses::Discover & resp_discoverfull)
{
    std::string reply = "";
    if(!rest.httpGet(reply, endpoint_discoverfull))
        return false;

    Document doc;
    ParseResult result = doc.Parse(reply.c_str());

    if(!result)
    {
        Log::error("PozyxMessageProcessor: error when parsing requestDiscoverFull-response: {}, {}",
                   rapidjson::GetParseError_En(result.Code()), result.Offset());
        return false;
    }

    resp_discoverfull.success = doc["success"].GetBool();
    if(!resp_discoverfull.success)
    {
        Log::error("PozyxMessageProcessor: requestDiscoverFull had no success");
        return false;
    }

    Log::info("PozyxMessageProcessor: iterating discovered tags");

    const Value & tags = doc["tags"];
    resp_discoverfull.tags.clear();

    for(size_t i=0; i < tags.Size(); i++)
    {
        PozyxPayloads::Tag tag;

        if(tags[i].HasMember("local") && !tags[i]["local"].IsNull())
        {
            tag.isLocal = tags[i]["local"].GetBool();
        }

        if(tags[i].HasMember("id") && !tags[i]["id"].IsNull())
        {
            tag.id = tags[i]["id"].GetString();
        }

        // sometimes pozyx-server does not add this parameter
        if(tags[i].HasMember("rss") && !tags[i]["rss"].IsNull())
        {
            tag.rss = tags[i]["rss"].GetInt();
        }

        if(tags[i].HasMember("distance") && !tags[i]["distance"].IsNull())
        {
            tag.distance = tags[i]["distance"].GetInt();
        }

        if(tags[i].HasMember("hexId") && !tags[i]["hexId"].IsNull())
        {
            tag.hexId = tags[i]["hexId"].GetString();
        }

        // sometimes pozyx-server inserts this value as null
        if(tags[i].HasMember("hwVersion") && !tags[i]["hwVersion"].IsNull())
        {
            tag.hwVersion = tags[i]["hwVersion"].GetString();
        }

        if(tags[i].HasMember("fwVersion") && !tags[i]["fwVersion"].IsNull())
        {
            tag.fwVersion = tags[i]["fwVersion"].GetString();
        }

        if(tags[i].HasMember("settings") && !tags[i]["settings"].IsNull() &&
                tags[i]["settings"].HasMember("uwb") && !tags[i]["settings"]["uwb"].IsNull())
        {
            const Value & tag_uwb = tags[i]["settings"]["uwb"];

            if(tag_uwb.HasMember("channel") && !tag_uwb["channel"].IsNull())
            {
                tag.settings_uwb.channel = tag_uwb["channel"].GetInt();
            }

            if(tag_uwb.HasMember("bitrate") && !tag_uwb["bitrate"].IsNull())
            {
                tag.settings_uwb.bitrate = tag_uwb["bitrate"].GetString();
            }

            if(tag_uwb.HasMember("prf") && !tag_uwb["prf"].IsNull())
            {
                tag.settings_uwb.prf = tag_uwb["prf"].GetString();
            }

            if(tag_uwb.HasMember("plen") && !tag_uwb["plen"].IsNull())
            {
                tag.settings_uwb.plen = tag_uwb["plen"].GetInt();
            }

            if(tag_uwb.HasMember("gain") && !tag_uwb["gain"].IsNull())
            {
                tag.settings_uwb.gain = tag_uwb["gain"].GetFloat();
            }
        }

        resp_discoverfull.tags.push_back(tag);
    }

    Log::info("PozyxMessageProcessor: iterating discovered anchors");

    const Value & anchors = doc["anchors"];
    resp_discoverfull.anchors.clear();

    for(size_t i=0; i < anchors.Size(); i++)
    {
        PozyxPayloads::Anchor anchor;
        if(anchors[i].HasMember("id") && !anchors[i]["id"].IsNull())
        {
            anchor.id = anchors[i]["id"].GetString();
        }

        // sometimes pozyx-server does not add this parameter
        if(anchors[i].HasMember("rss") && !anchors[i]["rss"].IsNull())
        {
            anchor.rss = anchors[i]["rss"].GetInt();
        }

        if(anchors[i].HasMember("distance") && !anchors[i]["distance"].IsNull())
        {
            anchor.distance = anchors[i]["distance"].GetInt();
        }

        if(anchors[i].HasMember("hexId") && !anchors[i]["hexId"].IsNull())
        {
            anchor.hexId = anchors[i]["hexId"].GetString();
        }

        if(anchors[i].HasMember("hwVersion") && !anchors[i]["hwVersion"].IsNull())
        {
            anchor.hwVersion = anchors[i]["hwVersion"].GetString();
        }

        if(anchors[i].HasMember("fwVersion") && !anchors[i]["fwVersion"].IsNull())
        {
            anchor.fwVersion = anchors[i]["fwVersion"].GetString();
        }

        if(anchors[i].HasMember("settings") && !anchors[i]["settings"].IsNull() &&
                anchors[i]["settings"].HasMember("uwb") && !anchors[i]["settings"]["uwb"].IsNull())
        {
            const Value & anchor_uwb = anchors[i]["settings"]["uwb"];

            if(anchor_uwb.HasMember("channel") && !anchor_uwb["channel"].IsNull())
            {
                anchor.settings_uwb.channel = anchor_uwb["channel"].GetInt();
            }

            if(anchor_uwb.HasMember("bitrate") && !anchor_uwb["bitrate"].IsNull())
            {
                anchor.settings_uwb.bitrate = anchor_uwb["bitrate"].GetString();
            }

            if(anchor_uwb.HasMember("prf") && !anchor_uwb["prf"].IsNull())
            {
                anchor.settings_uwb.prf = anchor_uwb["prf"].GetString();
            }

            if(anchor_uwb.HasMember("plen") && !anchor_uwb["plen"].IsNull())
            {
                anchor.settings_uwb.plen = anchor_uwb["plen"].GetInt();
            }

            if(anchor_uwb.HasMember("gain") && !anchor_uwb["gain"].IsNull())
            {
                anchor.settings_uwb.gain = anchor_uwb["gain"].GetFloat();
            }
        }

        resp_discoverfull.anchors.push_back(anchor);
    }

    Log::info("PozyxMessageProcessor: finished extracting values from discover-full response");

    return true;
}

bool PozyxMessageProcessor::requestAutocalibrate(const PozyxPayloads::Autocalib & autocalib, PozyxResponses::Autocalibrate & resp_autocalibrate)
{
    std::string reply = "";
    if(!rest.httpPost(reply, endpoint_autocalibrate, autocalibToString(autocalib)))
        return false;

    Document doc;
    ParseResult result = doc.Parse(reply.c_str());

    if(!result)
    {
        Log::error("PozyxMessageProcessor: error when parsing requestAutocalibrate-response: {}, {}",
                   rapidjson::GetParseError_En(result.Code()), result.Offset());
        return false;
    }

    resp_autocalibrate.success = doc["success"].GetBool();
    if(!resp_autocalibrate.success)
    {
        Log::error("PozyxMessageProcessor: requestAutocalibrate: failed");
        return false;
    }

    resp_autocalibrate.score = doc["score"].GetInt();
    resp_autocalibrate.connectivity = doc["connectivity"].GetInt();

    const Value & anchors = doc["anchors"];

    if(anchors.Size() != autocalib.anchors.size())
    {
        Log::error("PozyxMessageProcessor: requestAutocalibrate: autocalibrate response ({} anchors) does not have the same num of anchors as autocalib payload ({} anchors)",
                   anchors.Size(), autocalib.anchors.size());
        return false;
    }

    resp_autocalibrate.anchors.resize(anchors.Size());

    for(size_t i=0; i < resp_autocalibrate.anchors.size(); i++)
    {
        resp_autocalibrate.anchors[i].id = anchors[i]["id"].GetString();
        resp_autocalibrate.anchors[i].coordinates.x = anchors[i]["coordinates"]["x"].GetInt();
        resp_autocalibrate.anchors[i].coordinates.y = anchors[i]["coordinates"]["y"].GetInt();
        resp_autocalibrate.anchors[i].coordinates.z = anchors[i]["coordinates"]["z"].GetInt();

        // assign the order comparing the ids with autocalib-payload
        size_t j=0;
        for( ; j < autocalib.anchors.size(); j++)
        {
            if(resp_autocalibrate.anchors[i].id == autocalib.anchors[j].id)
            {
                // id found
                resp_autocalibrate.anchors[i].order = autocalib.anchors[j].order;
                break;
            }
        }

        // id not found
        if(j == autocalib.anchors.size())
        {
            Log::critical("PozyxMessageProcessor: requestAutocalibrate: resp_autocalibrate does not contain the anchor id {} that was requested",
                          resp_autocalibrate.anchors[i].id);
            return false;
        }
    }

    // re-ordering
    std::vector<PozyxResponses::AutocalibratedAnchor> tmp;
    tmp.resize(resp_autocalibrate.anchors.size());

    for(size_t i=0; i < tmp.size(); i++)
    {
        tmp[ resp_autocalibrate.anchors[i].order ] = resp_autocalibrate.anchors[i];
    }

    resp_autocalibrate.anchors.clear();
    resp_autocalibrate.anchors = tmp;

    // fix mirrored coordinates
    for(size_t i=0; i < resp_autocalibrate.anchors.size(); i++)
    {
        if(resp_autocalibrate.anchors[i].coordinates.x < 0)
        {
            Log::info("PozyxMessageProcessor: requestAutocalibrate: fixing mirrored coordinate X: from {} to {}, anchor-id {}",
                      resp_autocalibrate.anchors[i].coordinates.x,
                      resp_autocalibrate.anchors[i].coordinates.x * -1,
                      resp_autocalibrate.anchors[i].id);
            resp_autocalibrate.anchors[i].coordinates.x *= -1;
        }
        if(resp_autocalibrate.anchors[i].coordinates.y < 0)
        {
            Log::info("PozyxMessageProcessor: requestAutocalibrate: fixing mirrored coordinate Y: from {} to {}, anchor-id {}",
                      resp_autocalibrate.anchors[i].coordinates.y,
                      resp_autocalibrate.anchors[i].coordinates.y * -1,
                      resp_autocalibrate.anchors[i].id);
            resp_autocalibrate.anchors[i].coordinates.y *= -1;
        }
    }

    return true;
}

bool PozyxMessageProcessor::requestUpdateSettings(const PozyxPayloads::Settings & settings, PozyxResponses::UpdateSettings & resp_updatesettings)
{
    /**
      Devices that are already in the target uwb will not appear in the result arrays
      If a device could not be changed will have "success: false", else "success: true"
     */

    std::string reply = "";
    if(!rest.httpPost(reply, endpoint_updatesettings, settingsToString(settings, true)))
        return false;

    Document doc;
    ParseResult result = doc.Parse(reply.c_str());

    if(!result)
    {
        Log::error("PozyxMessageProcessor: error when parsing requestUpdateSettings-response: {}, {}",
                   rapidjson::GetParseError_En(result.Code()), result.Offset());
        return false;
    }

    resp_updatesettings.success = doc["success"].GetBool();
    if(!resp_updatesettings.success)
    {
        resp_updatesettings.error_general = doc["error"].GetString();
        Log::error("PozyxMessageProcessor: failed update-positioning settings. Probably the settings.json has a value the server does not recognize. Error: {}", resp_updatesettings.error_general);
        return false;
    }

    bool allok = true;

    const Value & tags_positioning = doc["tags"]["positioning"];
    resp_updatesettings.tags_updated.resize(tags_positioning.Size());
    for(size_t i=0; i < tags_positioning.Size(); i++)
    {
        resp_updatesettings.tags_updated[i].id = tags_positioning[i]["id"].GetString();
        resp_updatesettings.tags_updated[i].success = tags_positioning[i]["success"].GetBool();
        if(!resp_updatesettings.tags_updated[i].success)
        {
            resp_updatesettings.tags_updated[i].error = tags_positioning[i]["error"].GetString();
            Log::error("PozyxMessageProcessor: failed update-settings for tag id {}. Error: {}", resp_updatesettings.tags_updated[i].id, resp_updatesettings.tags_updated[i].error);
            allok = false;
        }
    }

    const Value & anchors_uwb = doc["anchors"]["uwb"];
    resp_updatesettings.anchors_updated.resize(anchors_uwb.Size());
    for(size_t i=0; i < anchors_uwb.Size(); i++)
    {
        resp_updatesettings.anchors_updated[i].id = anchors_uwb[i]["id"].GetString();
        resp_updatesettings.anchors_updated[i].success = anchors_uwb[i]["success"].GetBool();
        if(!resp_updatesettings.anchors_updated[i].success)
        {
            resp_updatesettings.anchors_updated[i].error = anchors_uwb[i]["error"].GetString();
            allok = false;
            Log::error("PozyxMessageProcessor: failed update-settings for anchor id {}. Error: {}", resp_updatesettings.anchors_updated[i].id, resp_updatesettings.anchors_updated[i].error);
        }
    }

    return allok;
}

bool PozyxMessageProcessor::requestShortCommand(const std::string & endpoint, PozyxResponses::ShortCommandResp & cmdresp)
{
    std::string reply = "";
    std::string payload = "";
    if(!rest.httpPost(reply, endpoint, payload))
        return false;

    Document doc;
    ParseResult result = doc.Parse(reply.c_str());

    if(!result)
    {
        Log::error("PozyxMessageProcessor: error when parsing requestShortCommand-response: {}, {}",
                   rapidjson::GetParseError_En(result.Code()), result.Offset());
        return false;
    }

    cmdresp.success = doc["success"].GetBool();
    if(!cmdresp.success)
    {
        cmdresp.error = doc["error"].GetString();
        return false;
    }

    return true;
}

bool PozyxMessageProcessor::requestStart(PozyxResponses::ShortCommandResp & start_resp)
{
    return requestShortCommand(endpoint_start, start_resp);
}

bool PozyxMessageProcessor::requestStop(PozyxResponses::ShortCommandResp & stop_resp)
{
    return requestShortCommand(endpoint_stop, stop_resp);
}

bool PozyxMessageProcessor::requestKill(PozyxResponses::ShortCommandResp & kill_resp)
{
    return requestShortCommand(endpoint_kill, kill_resp);
}

bool PozyxMessageProcessor::extractPositioningFrame(const std::string & ipsframe_json, PozyxResponses::PositioningFrame & ipsframe)
{
//    Log::info("PozyxMessageProcessor: extractPositioningFrame: {}", ipsframe_json);

    Document doc;
    doc.Parse(ipsframe_json.c_str());

    ParseResult result = doc.Parse(ipsframe_json.c_str());
    if(!result)
    {
        Log::error("PozyxMessageProcessor: extractPositioningFrame: error when parsing IPS-frame: {}, {}",
                   rapidjson::GetParseError_En(result.Code()), result.Offset());
        return false;
    }

    if(!doc.HasMember("success") || doc["success"].IsNull())
    {
        Log::error("PozyxMessageProcessor: extractPositioningFrame: no 'success' field encountered");
        return false;
    }

    ipsframe.success = doc["success"].GetBool();

    if(doc.HasMember("timestamp") && !doc["timestamp"].IsNull())
    {
        ipsframe.timestamp = doc["timestamp"].GetDouble();
    }

    if(!ipsframe.success)
    {
        if(doc.HasMember("errorCode") && !doc["errorCode"].IsNull())
        {
            ipsframe.error = doc["errorCode"].GetString();
        }
        Log::error("PozyxMessageProcessor: not-valid positioning-frame: timestamp {} error {}",
                   ipsframe.timestamp, ipsframe.error);

        return false;
    }

//    ipsframe.version = doc["version"].GetString();

    if(!doc.HasMember("data") || doc["data"].IsNull())
    {
        Log::error("PozyxMessageProcessor: extractPositioningFrame: no 'data' field");
        return false;
    }

    if(!doc["data"].HasMember("coordinates") || doc["data"]["coordinates"].IsNull())
    {
        Log::error("PozyxMessageProcessor: extractPositioningFrame: no 'coordinates' field");
        return false;
    }

    if(!doc["data"].HasMember("orientation") || doc["data"]["orientation"].IsNull())
    {
        Log::error("PozyxMessageProcessor: extractPositioningFrame: no 'orientation' field");
        return false;
    }

    // extract pos
    if(doc["data"]["coordinates"].HasMember("x") && !doc["data"]["coordinates"]["x"].IsNull())
    {
        ipsframe.coordinates.x = doc["data"]["coordinates"]["x"].GetFloat();
    }

    if(doc["data"]["coordinates"].HasMember("y") && !doc["data"]["coordinates"]["y"].IsNull())
    {
        ipsframe.coordinates.y = doc["data"]["coordinates"]["y"].GetFloat();
    }

    if(doc["data"]["coordinates"].HasMember("z") && !doc["data"]["coordinates"]["z"].IsNull())
    {
        ipsframe.coordinates.z = doc["data"]["coordinates"]["z"].GetFloat();
    }

    /*
    ipsframe.velocity.x = doc["data"]["velocity"]["x"].GetFloat();
    ipsframe.velocity.y = doc["data"]["velocity"]["y"].GetFloat();
    ipsframe.velocity.z = doc["data"]["velocity"]["z"].GetFloat();
    */

    // extract yaw
    if(doc["data"]["orientation"].HasMember("roll") && !doc["data"]["orientation"]["roll"].IsNull())
    {
        ipsframe.roll = doc["data"]["orientation"]["roll"].GetFloat();
    }

    if(doc["data"]["orientation"].HasMember("pitch") && !doc["data"]["orientation"]["pitch"].IsNull())
    {
        ipsframe.pitch = doc["data"]["orientation"]["pitch"].GetFloat();
    }

    if(doc["data"]["orientation"].HasMember("yaw") && !doc["data"]["orientation"]["yaw"].IsNull())
    {
        ipsframe.yaw = doc["data"]["orientation"]["yaw"].GetFloat();
    }

    /*
    ipsframe.north.x = doc["data"]["north"]["x"].GetFloat();
    ipsframe.north.y = doc["data"]["north"]["y"].GetFloat();
    ipsframe.north.z = doc["data"]["north"]["z"].GetFloat();

    ipsframe.ranges.resize(doc["data"]["ranges"].Size());
    for(size_t i=0; i < ipsframe.ranges.size(); i++)
    {
        ipsframe.ranges[i].anchorId = doc["data"]["ranges"][i]["anchorId"].GetString();
        ipsframe.ranges[i].range = doc["data"]["ranges"][i]["range"].GetInt();  // mm
    }

    const Value & metrics = doc["data"]["metrics"];
    ipsframe.latency = metrics["latency"].GetFloat();
    ipsframe.rate_update = metrics["rates"]["update"].GetFloat();
    ipsframe.rate_success = metrics["rates"]["success"].GetFloat();
    ipsframe.positioning_latency = metrics["extra"]["positioningLatency"].GetFloat();
    ipsframe.drone_calculation_latency = metrics["extra"]["droneCalculationLatency"].GetFloat();

    ipsframe.tags_ids.resize(metrics["extra"]["tags"].Size());
    for(size_t i=0; i < ipsframe.tags_ids.size(); i++)
    {
        ipsframe.tags_ids[i] = std::stoi(metrics["extra"]["tags"][i].GetString());
    }
    */

    return true;
}

bool PozyxMessageProcessor::loadSettings(std::string file_path, PozyxPayloads::Settings & settings)
{
    std::ifstream ifs(file_path);
    IStreamWrapper isw(ifs);

    Document doc;
    doc.ParseStream(isw);
    if(doc.HasParseError())
    {
        Log::error("PozyxMessageProcessor: parse error in loadSettings {}. Maybe file does not exists", file_path);
        return false;
    }

    // consult main members
    const Value & positioning = doc["settings"]["positioning"];
    const Value & drone = doc["settings"]["drone"];
    const Value & uwb = doc["settings"]["uwb"];
    const Value & tags = doc["tags"];
    const Value & anchors = doc["anchors"];

    // extract values
    settings.positioning.enabled = positioning["enabled"].GetBool();
    settings.positioning.algorithm = positioning["diy"]["algorithm"].GetString();
    settings.positioning.dimension = positioning["diy"]["dimension"].GetString();
    settings.positioning.height = positioning["diy"]["height"].GetInt();
    settings.positioning.rangingProtocol = positioning["diy"]["rangingProtocol"].GetString();
    settings.positioning.filterType = positioning["diy"]["filterType"].GetString();
    settings.positioning.filterStrength = positioning["diy"]["filterStrength"].GetInt();

    settings.drone.droneWidthmm = drone["droneWidthmm"].GetInt();
    settings.drone.droneHeightmm = drone["droneHeightmm"].GetInt();
    settings.drone.options_mode = drone["options"]["mode"].GetString();
    settings.drone.options_onFail = drone["options"]["onFail"].GetString();
    settings.drone.options_orientationDimension = drone["options"]["orientationDimension"].GetInt();
    settings.drone.options_filter_updatePeriod = drone["options"]["filter"]["updatePeriod"].GetFloat();
    settings.drone.options_filter_movementFreedom = drone["options"]["filter"]["movementFreedom"].GetInt();
    settings.drone.tag_sw = drone["tags"]["SW"].GetString();
    settings.drone.tag_nw = drone["tags"]["NW"].GetString();
    settings.drone.tag_ne = drone["tags"]["NE"].GetString();
    settings.drone.tag_se = drone["tags"]["SE"].GetString();

    settings.uwb.channel = uwb["channel"].GetInt();
    settings.uwb.bitrate = uwb["bitrate"].GetString();
    settings.uwb.prf = uwb["prf"].GetString();
    settings.uwb.plen = uwb["plen"].GetInt();
    settings.uwb.gain = uwb["gain"].GetFloat();

    settings.tags.resize(tags.Size());
    for(size_t i=0; i < settings.tags.size(); i++)
    {
        settings.tags[i].id = tags[i]["id"].GetString();
        settings.tags[i].inUse = tags[i]["inUse"].GetBool();
        settings.tags[i].settings_uwb.channel = tags[i]["settings"]["uwb"]["channel"].GetInt();
        settings.tags[i].settings_uwb.bitrate = tags[i]["settings"]["uwb"]["bitrate"].GetString();
        settings.tags[i].settings_uwb.prf = tags[i]["settings"]["uwb"]["prf"].GetString();
        settings.tags[i].settings_uwb.plen = tags[i]["settings"]["uwb"]["plen"].GetInt();
        settings.tags[i].settings_uwb.gain = tags[i]["settings"]["uwb"]["gain"].GetFloat();
    }

    settings.anchors.resize(anchors.Size());
    for(size_t i=0; i < settings.anchors.size(); i++)
    {
        settings.anchors[i].id = anchors[i]["id"].GetString();
        settings.anchors[i].inUse = anchors[i]["inUse"].GetBool();
        settings.anchors[i].settings_uwb.channel = anchors[i]["settings"]["uwb"]["channel"].GetInt();
        settings.anchors[i].settings_uwb.bitrate = anchors[i]["settings"]["uwb"]["bitrate"].GetString();
        settings.anchors[i].settings_uwb.prf = anchors[i]["settings"]["uwb"]["prf"].GetString();
        settings.anchors[i].settings_uwb.plen = anchors[i]["settings"]["uwb"]["plen"].GetInt();
        settings.anchors[i].settings_uwb.gain = anchors[i]["settings"]["uwb"]["gain"].GetFloat();

        settings.anchors[i].coordinates.x = anchors[i]["coordinates"]["x"].GetInt();
        settings.anchors[i].coordinates.y = anchors[i]["coordinates"]["y"].GetInt();
        settings.anchors[i].coordinates.z = anchors[i]["coordinates"]["z"].GetInt();
    }

    Log::info("PozyxMessageProcessor: settings loaded from {}", file_path);
    return true;
}

bool endsWith(const std::string & s, const std::string & suffix)
{
    return s.rfind(suffix) == (s.size()-suffix.size());
}

bool PozyxMessageProcessor::saveSettings(std::string file_path, const PozyxPayloads::Settings & settings)
{
    // saving
    std::ofstream of;
    of.open(file_path, std::ios_base::out);
    if(!of.is_open())
    {
        Log::error("PozyxMessageProcessor: could not open {} for saving settings", file_path);
        return false;
    }

    of << settingsToString(settings, !endsWith(file_path, ".bak"));
    of.close();

    Log::info("pozyxMessageProcessor: settings saved to {}", file_path);

    return true;
}

bool PozyxMessageProcessor::checkSettings(const PozyxPayloads::Settings &settings)
{
    bool ok = true;

    if(settings.tags.size() < MIN_NUM_TAGS)
    {
        ok = false;
        Log::error("PozyxMessageProcessor: you must especify at least {} tags in settings. Specified: {}", MIN_NUM_TAGS, settings.tags.size());
    }

    if(settings.anchors.size() < MIN_NUM_ANCHORS)
    {
        ok = false;
        Log::error("PozyxMessageProcessor: you must especify at least {} anchors in settings. Specified: {}", MIN_NUM_ANCHORS, settings.anchors.size());
    }

    for(size_t i=0; i < settings.tags.size(); i++)
    {
        if(settings.tags[i].id == "0")
        {
            Log::error("PozyxMessageProcessor: tag[{}] has invalid id {} in settings", i, settings.tags[i].id);
            ok = false;
        }
    }

    for(size_t i=0; i < settings.anchors.size(); i++)
    {
        if(settings.anchors[i].id == "0")
        {
            Log::error("PozyxMessageProcessor: anchor[{}] has uninitialized id {} in settings", i, settings.anchors[i].id);
            ok = false;
        }

        if(settings.anchors[i].coordinates.x == 0 && settings.anchors[i].coordinates.y == 0 && settings.anchors[i].coordinates.z == 0)
        {
            Log::error("PozyxMessageProcessor: anchor[{}] has uninitialized coordinates {} {} {} in settings",
                       i, settings.anchors[i].coordinates.x, settings.anchors[i].coordinates.y, settings.anchors[i].coordinates.z);
            ok = false;
        }
    }

    int ntags_ok = 0;
    for(size_t i=0; i < settings.tags.size(); i++)
    {
        if(settings.tags[i].id == settings.drone.tag_sw ||
                settings.tags[i].id == settings.drone.tag_nw ||
                settings.tags[i].id == settings.drone.tag_ne ||
                settings.tags[i].id == settings.drone.tag_se)
        {
            ntags_ok++;
        }
    }
    if(ntags_ok != 4)
    {
        Log::critical("PozyxMessageProcessor: sendDroneTags: settings is malformed dronetags (sw,nw,ne,se) ids are not the same as tags[] array");
        ok = false;
    }

    return ok;
}

PozyxPayloads::Settings PozyxMessageProcessor::buildSettings(const PozyxPayloads::Positioning & positioning, const PozyxPayloads::Drone & drone,
                                                             const PozyxPayloads::Uwb & uwb_target, const PozyxResponses::Discover & discovered)
{
    PozyxPayloads::Settings newsettings;

    // keep these values
    newsettings.positioning = positioning;
    newsettings.drone = drone;

    // update new values
    newsettings.uwb = uwb_target;

    newsettings.tags.resize(discovered.tags.size());
    for(size_t i=0; i < discovered.tags.size(); i++)
    {
        newsettings.tags[i] = discovered.tags[i];
    }

    newsettings.anchors.resize(discovered.anchors.size());
    for(size_t i=0; i < discovered.anchors.size(); i++)
    {
        newsettings.anchors[i] = discovered.anchors[i];
    }

    return newsettings;
}

std::string PozyxMessageProcessor::settingsToString(const PozyxPayloads::Settings & settings, bool printToLog)
{
    Document doc;
    doc.SetObject();
    Document::AllocatorType & allocator = doc.GetAllocator();

    Value settingsgroup;

    // filling settings.positioning member
    Value positioning, enabled, diy, algorithm, dimension, height, rangingProtocol, filterType, filterStrength;

    enabled.SetBool(settings.positioning.enabled);
    algorithm.SetString(settings.positioning.algorithm.c_str(), settings.positioning.algorithm.length());
    dimension.SetString(settings.positioning.dimension.c_str(), settings.positioning.dimension.length());
    height.SetInt(settings.positioning.height);
    rangingProtocol.SetString(settings.positioning.rangingProtocol.c_str(), settings.positioning.rangingProtocol.length());
    filterType.SetString(settings.positioning.filterType.c_str(), settings.positioning.filterType.length());
    filterStrength.SetInt(settings.positioning.filterStrength);

    // adding to json
    diy.SetObject();
    diy.AddMember("algorithm", algorithm, allocator);
    diy.AddMember("dimension", dimension, allocator);
    diy.AddMember("height", height, allocator);
    diy.AddMember("rangingProtocol", rangingProtocol, allocator);
    diy.AddMember("filterType", filterType, allocator);
    diy.AddMember("filterStrength", filterStrength, allocator);

    positioning.SetObject();
    positioning.AddMember("enabled", enabled, allocator);
    positioning.AddMember("diy", diy, allocator);

    settingsgroup.SetObject();
    settingsgroup.AddMember("positioning", positioning, allocator);

    // filling settings.drone member
    Value drone, tagsdrone, SW, NW, NE, SE, droneWidthmm, droneHeightmm,
            options, mode, onFail, orientationDimension,
            filter, updatePeriod, movementFreedom;

    SW.SetString(settings.drone.tag_sw.c_str(), settings.drone.tag_sw.length());
    NW.SetString(settings.drone.tag_nw.c_str(), settings.drone.tag_nw.length());
    NE.SetString(settings.drone.tag_ne.c_str(), settings.drone.tag_ne.length());
    SE.SetString(settings.drone.tag_se.c_str(), settings.drone.tag_se.length());
    droneWidthmm.SetInt(settings.drone.droneWidthmm);
    droneHeightmm.SetInt(settings.drone.droneHeightmm);
    mode.SetString(settings.drone.options_mode.c_str(), settings.drone.options_mode.length());
    onFail.SetString(settings.drone.options_onFail.c_str(), settings.drone.options_onFail.length());
    orientationDimension.SetInt(settings.drone.options_orientationDimension);
    updatePeriod.SetFloat(settings.drone.options_filter_updatePeriod);
    movementFreedom.SetInt(settings.drone.options_filter_movementFreedom);

    // adding to json
    tagsdrone.SetObject();
    tagsdrone.AddMember("SW", SW, allocator);
    tagsdrone.AddMember("NW", NW, allocator);
    tagsdrone.AddMember("NE", NE, allocator);
    tagsdrone.AddMember("SE", SE, allocator);

    filter.SetObject();
    filter.AddMember("updatePeriod", updatePeriod, allocator);
    filter.AddMember("movementFreedom", movementFreedom, allocator);

    options.SetObject();
    options.AddMember("mode", mode, allocator);
    options.AddMember("onFail", onFail, allocator);
    options.AddMember("orientationDimension", orientationDimension, allocator);
    options.AddMember("filter", filter, allocator);

    drone.SetObject();
    drone.AddMember("tags", tagsdrone, allocator);
    drone.AddMember("droneWidthmm", droneWidthmm, allocator);
    drone.AddMember("droneHeightmm", droneHeightmm, allocator);
    drone.AddMember("options", options, allocator);

    settingsgroup.AddMember("drone", drone, allocator);

    // filling settings.uwb member
    Value uwb, channel, bitrate, prf, plen, gain;

    channel.SetInt(settings.uwb.channel);
    bitrate.SetString(settings.uwb.bitrate.c_str(), settings.uwb.bitrate.length());
    prf.SetString(settings.uwb.prf.c_str(), settings.uwb.prf.length());
    plen.SetInt(settings.uwb.plen);
    gain.SetFloat(settings.uwb.gain);

    // adding to json
    uwb.SetObject();
    uwb.AddMember("channel", channel, allocator);
    uwb.AddMember("bitrate", bitrate, allocator);
    uwb.AddMember("prf", prf, allocator);
    uwb.AddMember("plen", plen, allocator);
    uwb.AddMember("gain", gain, allocator);

    settingsgroup.AddMember("uwb", uwb, allocator);

    doc.AddMember("settings", settingsgroup, allocator);

    // TAGS & ANCHORS
    // filling tags array
    Value tags;
    tags.SetArray();

    for(size_t i=0; i < settings.tags.size(); i++)
    {
        Value tag, id, inUse;

        id.SetString(settings.tags[i].id.c_str(), settings.tags[i].id.length());
        inUse.SetBool(settings.tags[i].inUse);

        tag.SetObject();
        tag.AddMember("id", id, allocator);
        tag.AddMember("inUse", inUse, allocator);

        Value device_settings, device_uwb, device_channel, device_bitrate, device_prf, device_plen, device_gain;

        device_channel.SetInt(settings.tags[i].settings_uwb.channel);
        device_bitrate.SetString(settings.tags[i].settings_uwb.bitrate.c_str(), settings.tags[i].settings_uwb.bitrate.length());
        device_prf.SetString(settings.tags[i].settings_uwb.prf.c_str(), settings.tags[i].settings_uwb.prf.length());
        device_plen.SetInt(settings.tags[i].settings_uwb.plen);
        device_gain.SetFloat(settings.tags[i].settings_uwb.gain);

        device_uwb.SetObject();
        device_uwb.AddMember("channel", device_channel, allocator);
        device_uwb.AddMember("bitrate", device_bitrate, allocator);
        device_uwb.AddMember("prf", device_prf, allocator);
        device_uwb.AddMember("plen", device_plen, allocator);
        device_uwb.AddMember("gain", device_gain, allocator);

        device_settings.SetObject();
        device_settings.AddMember("uwb", device_uwb, allocator);

        tag.AddMember("settings", device_settings, allocator);
        tags.PushBack(tag.Move(), allocator);
    }
    doc.AddMember("tags", tags, allocator);

    // filling anchors array
    Value anchors;
    anchors.SetArray();

    for(size_t i=0; i < settings.anchors.size(); i++)
    {
        Value anchor, id, inUse, coordinates;

        id.SetString(settings.anchors[i].id.c_str(), settings.anchors[i].id.length());
        inUse.SetBool(settings.anchors[i].inUse);

        anchor.SetObject();
        anchor.AddMember("id", id, allocator);
        anchor.AddMember("inUse", inUse, allocator);

        Value device_settings, device_uwb, device_channel, device_bitrate, device_prf, device_plen, device_gain;

        device_channel.SetInt(settings.anchors[i].settings_uwb.channel);
        device_bitrate.SetString(settings.anchors[i].settings_uwb.bitrate.c_str(), settings.anchors[i].settings_uwb.bitrate.length());
        device_prf.SetString(settings.anchors[i].settings_uwb.prf.c_str(), settings.anchors[i].settings_uwb.prf.length());
        device_plen.SetInt(settings.anchors[i].settings_uwb.plen);
        device_gain.SetFloat(settings.anchors[i].settings_uwb.gain);

        device_uwb.SetObject();
        device_uwb.AddMember("channel", device_channel, allocator);
        device_uwb.AddMember("bitrate", device_bitrate, allocator);
        device_uwb.AddMember("prf", device_prf, allocator);
        device_uwb.AddMember("plen", device_plen, allocator);
        device_uwb.AddMember("gain", device_gain, allocator);

        device_settings.SetObject();
        device_settings.AddMember("uwb", device_uwb, allocator);
        anchor.AddMember("settings", device_settings, allocator);

        Value x, y, z;
        x.SetInt(settings.anchors[i].coordinates.x);
        y.SetInt(settings.anchors[i].coordinates.y);
        z.SetInt(settings.anchors[i].coordinates.z);

        coordinates.SetObject();
        coordinates.AddMember("x", x, allocator);
        coordinates.AddMember("y", y, allocator);
        coordinates.AddMember("z", z, allocator);

        anchor.AddMember("coordinates", coordinates, allocator);

        anchors.PushBack(anchor.Move(), allocator);
    }
    doc.AddMember("anchors", anchors, allocator);

    // converting jsondoc to string
    StringBuffer sbuffer;
    PrettyWriter<StringBuffer> pwriter(sbuffer);
    doc.Accept(pwriter);

    if(printToLog)
    {
        Log::info("PozyxMessageProcessor: settingsToString: {}", sbuffer.GetString());
    }

    return sbuffer.GetString();
}

std::string PozyxMessageProcessor::autocalibToString(const PozyxPayloads::Autocalib &autocalib)
{
    Document doc;
    doc.SetObject();
    Document::AllocatorType & allocator = doc.GetAllocator();

    Value settings, matchPrevious, testLocalizability, method, dimension, estimator, name;

    matchPrevious.SetBool(autocalib.settings.matchPrevious);
    testLocalizability.SetBool(autocalib.settings.testLocalizability);
    method.SetString(autocalib.settings.method.c_str(), autocalib.settings.method.length());
    dimension.SetString(autocalib.settings.dimension.c_str(), autocalib.settings.dimension.length());

    estimator.SetObject();
    name.SetString(autocalib.settings.estimator_name.c_str(), autocalib.settings.estimator_name.length());
    estimator.AddMember("name", name, allocator);

    settings.SetObject();
    settings.AddMember("matchPrevious", matchPrevious, allocator);
    settings.AddMember("testLocalizability", testLocalizability, allocator);
    settings.AddMember("method", method, allocator);
    settings.AddMember("dimension", dimension, allocator);
    settings.AddMember("estimator", estimator, allocator);

    doc.AddMember("settings", settings, allocator);

    Value anchors;
    anchors.SetArray();

    for(size_t i=0; i < autocalib.anchors.size(); i++)
    {
        Value anchorconfig, id, coordinates, x, y, z;

        id.SetString(autocalib.anchors[i].id.c_str(), autocalib.anchors[i].id.length());

        anchorconfig.SetObject();
        anchorconfig.AddMember("id", id, allocator);

        if(autocalib.anchors[i].coordinates.x_ToBeAutocalibrated)
            x.SetString("NaN");
        else
            x.SetInt(autocalib.anchors[i].coordinates.x);

        if(autocalib.anchors[i].coordinates.y_ToBeAutocalibrated)
            y.SetString("NaN");
        else
            y.SetInt(autocalib.anchors[i].coordinates.y);

        if(autocalib.anchors[i].coordinates.z_ToBeAutocalibrated)
            z.SetString("NaN");
        else
            z.SetInt(autocalib.anchors[i].coordinates.z);

        coordinates.SetObject();
        coordinates.AddMember("x", x, allocator);
        coordinates.AddMember("y", y, allocator);
        coordinates.AddMember("z", z, allocator);

        anchorconfig.AddMember("coordinates", coordinates, allocator);

        anchors.PushBack(anchorconfig.Move(), allocator);
    }
    doc.AddMember("anchors", anchors, allocator);

    Value oldAnchors;
    oldAnchors.SetArray();

    for(size_t i=0; i < autocalib.oldAnchors.size(); i++)
    {
        Value oldAnchorconfig, id, coordinates, x, y, z;

        id.SetString(autocalib.oldAnchors[i].id.c_str(), autocalib.oldAnchors[i].id.length());

        oldAnchorconfig.SetObject();
        oldAnchorconfig.AddMember("id", id, allocator);

        if(autocalib.oldAnchors[i].coordinates.x_ToBeAutocalibrated)
            x.SetString("Nan");
        else
            x.SetInt(autocalib.oldAnchors[i].coordinates.x);

        if(autocalib.oldAnchors[i].coordinates.y_ToBeAutocalibrated)
            y.SetString("Nan");
        else
            y.SetInt(autocalib.oldAnchors[i].coordinates.y);

        if(autocalib.oldAnchors[i].coordinates.z_ToBeAutocalibrated)
            z.SetString("Nan");
        else
            z.SetInt(autocalib.oldAnchors[i].coordinates.z);

        coordinates.SetObject();
        coordinates.AddMember("x", x, allocator);
        coordinates.AddMember("y", y, allocator);
        coordinates.AddMember("z", z, allocator);

        oldAnchorconfig.AddMember("coordinates", coordinates, allocator);

        oldAnchors.PushBack(oldAnchorconfig.Move(), allocator);
    }
    doc.AddMember("oldAnchors", oldAnchors, allocator);

    // converting jsondoc to string
    StringBuffer sbuffer;
    PrettyWriter<StringBuffer> pwriter(sbuffer);
    doc.Accept(pwriter);

    Log::info("PozyxMessageProcessor: autocalibToString: {}", sbuffer.GetString());

    return sbuffer.GetString();
}

bool PozyxMessageProcessor::buildAutocalib(const PozyxPayloads::Settings & settings, PozyxPayloads::Autocalib & autocalib)
{
    if(settings.anchors.size() < MIN_NUM_ANCHORS)
    {
        Log::error("PozyxMessageProcessor: buildAutocalib: settings have {} anchors, min num anchors is {}", settings.anchors.size(), MIN_NUM_ANCHORS);
        return false;
    }

    autocalib.settings.matchPrevious = false;
    autocalib.settings.testLocalizability = true;
    autocalib.settings.method = "intrarange";
    autocalib.settings.dimension = "2D";
    autocalib.settings.estimator_name = "nlls";

    autocalib.anchors.resize(settings.anchors.size());

    for(size_t i=0; i < settings.anchors.size(); i++)
    {
        autocalib.anchors[i].id = settings.anchors[i].id;

        // origin (bottom and top)
        if(i==0 || i == 4)
        {
            autocalib.anchors[i].coordinates.x = settings.anchors[i].coordinates.x;
            autocalib.anchors[i].coordinates.y = settings.anchors[i].coordinates.y;
            autocalib.anchors[i].coordinates.z = settings.anchors[i].coordinates.z;
        }

        // x axis
        else if(i==1 || i==5)
        {
            autocalib.anchors[i].coordinates.x_ToBeAutocalibrated = true;
            autocalib.anchors[i].coordinates.y = settings.anchors[i].coordinates.y;
            autocalib.anchors[i].coordinates.z = settings.anchors[i].coordinates.z;
        }

        // y axis
        else if(i==3 || i==7)
        {
            autocalib.anchors[i].coordinates.x = settings.anchors[i].coordinates.x;
            autocalib.anchors[i].coordinates.y_ToBeAutocalibrated = true;
            autocalib.anchors[i].coordinates.z = settings.anchors[i].coordinates.z;
        }

        // the rest of anchors
        else
        {
            autocalib.anchors[i].coordinates.x_ToBeAutocalibrated = true;
            autocalib.anchors[i].coordinates.y_ToBeAutocalibrated = true;
            autocalib.anchors[i].coordinates.z = settings.anchors[i].coordinates.z;
        }
    }

    return true;
}

bool PozyxMessageProcessor::buildAutocalib(const std::vector<PozyxPayloads::AutocalibAnchorConfig> & anchors_tobe_autocalibrated, PozyxPayloads::Autocalib & autocalib)
{
    if(anchors_tobe_autocalibrated.size() < MIN_NUM_ANCHORS)
    {
        Log::error("PozyxMessageProcessor: buildAutocalib: anchors_tobe_autocalibrated have {} anchors, min num anchors is {}", anchors_tobe_autocalibrated.size(), MIN_NUM_ANCHORS);
        return false;
    }

    autocalib.settings.matchPrevious = false;
    autocalib.settings.testLocalizability = true;
    autocalib.settings.method = "intrarange";
    autocalib.settings.dimension = "2D";
    autocalib.settings.estimator_name = "nlls";

    autocalib.anchors.clear();

    for(size_t i=0; i < anchors_tobe_autocalibrated.size(); i++)
    {
        PozyxPayloads::AutocalibAnchorConfig acac;

        acac.id = anchors_tobe_autocalibrated[i].id;
        acac.order = anchors_tobe_autocalibrated[i].order;

        const uint8_t & order = anchors_tobe_autocalibrated[i].order;

        // origin (bottom and top)
        if(order==0 || order == 4)
        {
            acac.coordinates.x = anchors_tobe_autocalibrated[i].coordinates.x;
            acac.coordinates.y = anchors_tobe_autocalibrated[i].coordinates.y;
            acac.coordinates.z = anchors_tobe_autocalibrated[i].coordinates.z;
        }

        // x axis
        else if(order==1 || order==5)
        {
            acac.coordinates.x_ToBeAutocalibrated = true;
            acac.coordinates.y = anchors_tobe_autocalibrated[i].coordinates.y;
            acac.coordinates.z = anchors_tobe_autocalibrated[i].coordinates.z;
        }

        // y axis
        else if(order==3 || order==7)
        {
            acac.coordinates.x = anchors_tobe_autocalibrated[i].coordinates.x;
            acac.coordinates.y_ToBeAutocalibrated = true;
            acac.coordinates.z = anchors_tobe_autocalibrated[i].coordinates.z;
        }

        // the rest of anchors
        else
        {
            acac.coordinates.x_ToBeAutocalibrated = true;
            acac.coordinates.y_ToBeAutocalibrated = true;
            acac.coordinates.z = anchors_tobe_autocalibrated[i].coordinates.z;
        }

        autocalib.anchors.push_back(acac);
    }

    return true;
}

bool PozyxMessageProcessor::setAnchorConfig(int id, uint8_t order, int x, int y, int z, PozyxPayloads::Settings &settings, const std::vector<PozyxPayloads::Anchor> & discovered_anchors)
{
    if(settings.anchors.size() < MIN_NUM_ANCHORS)
    {
        Log::error("PozyxMessageProcessor: setAnchorConfig: settings have only {} anchors, min num anchors is {}", settings.anchors.size(), MIN_NUM_ANCHORS);
    }

    if(order < 0 || order >= settings.anchors.size())
    {
        Log::error("PozyxMessageProcessor: setAnchorConfig: order {} out of range", order);
        return false;
    }

    PozyxPayloads::Anchor & a = settings.anchors[order];
    a.id = std::to_string(id);
    a.coordinates.x = x;
    a.coordinates.y = y;
    a.coordinates.z = z;

//    Log::info("PozyxMessageProcessor: setAnchorConfig: id {} order {} x {} y {} z {}",
//              a.id, order, a.coordinates.x, a.coordinates.y, a.coordinates.z);

    // for moving from current (discovered) channel to target channel
    // we have to specify the current channel in the array
    if(discovered_anchors.size() > 0)
    {
        for(size_t da=0; da < discovered_anchors.size(); da++)
        {
            if(a.id == discovered_anchors[da].id)
            {
                a.settings_uwb = discovered_anchors[da].settings_uwb;
            }
        }
    }
    // else use default from pozyx_settings.json

    return true;
}

bool PozyxMessageProcessor::getSyntheticAutocalibrateResponse(const PozyxPayloads::Autocalib & autocalib, PozyxResponses::Autocalibrate & autocalibrate)
{
    if(autocalib.anchors.size() < MIN_NUM_ANCHORS)
    {
        Log::error("PozyxMessageProcessor: getSyntheticAutocalibrateResponse: autocalib payload has {} anchors, min num anchors is {}",
                   autocalib.anchors.size(), MIN_NUM_ANCHORS);
        return false;
    }

    // synthetic coordinates
    std::vector<PozyxPayloads::Int3> coordinates;
    coordinates.push_back( PozyxPayloads::Int3(0, 0, 875));
    coordinates.push_back( PozyxPayloads::Int3(4289, 0, 875));
    coordinates.push_back( PozyxPayloads::Int3(4289, 4370, 875));
    coordinates.push_back( PozyxPayloads::Int3(0, 4370, 875));

    coordinates.push_back( PozyxPayloads::Int3(0, 0, 2515));
    coordinates.push_back( PozyxPayloads::Int3(4289, 0, 2515));
    coordinates.push_back( PozyxPayloads::Int3(4289, 4370, 2515));
    coordinates.push_back( PozyxPayloads::Int3(0, 4370, 2515));

    autocalibrate.anchors.resize(autocalib.anchors.size());
    for(size_t i=0; i < autocalibrate.anchors.size(); i++)
    {
        autocalibrate.anchors[i].id = autocalib.anchors[i].id;
        autocalibrate.anchors[i].order = autocalib.anchors[i].order;
        autocalibrate.anchors[i].coordinates = coordinates[i];
    }

    autocalibrate.success = true;
    autocalibrate.connectivity = 25;
    autocalibrate.score = 30;

    return true;
}

bool PozyxMessageProcessor::setDroneTagsConfig(int drone_width_mm, int drone_height_mm, int sw_id, int nw_id, int ne_id, int se_id, PozyxPayloads::Settings &settings, const std::vector<PozyxPayloads::Tag> & discovered_tags)
{
    if(settings.tags.size() != 4)
    {
        Log::critical("PozyxMessageProcessor: setDroneTagsConfig: num of tags in settings tags array have to be 4, found {}", settings.tags.size());
        return false;
    }

    settings.drone.droneWidthmm = drone_width_mm;
    settings.drone.droneHeightmm = drone_height_mm;

    settings.drone.tag_sw = std::to_string(sw_id);
    settings.drone.tag_nw = std::to_string(nw_id);
    settings.drone.tag_ne = std::to_string(ne_id);
    settings.drone.tag_se = std::to_string(se_id);

    settings.tags[0].id = settings.drone.tag_sw;
    settings.tags[1].id = settings.drone.tag_nw;
    settings.tags[2].id = settings.drone.tag_ne;
    settings.tags[3].id = settings.drone.tag_se;

    // for moving from current (discovered) channel to target channel
    // we have to specify the current channel in the array
    if(discovered_tags.size() > 0)
    {
        for(size_t i = 0; i < settings.tags.size(); i++)
        {
            for(size_t dt=0; dt < discovered_tags.size(); dt++)
            {
                if(settings.tags[i].id == discovered_tags[dt].id)
                {
                    settings.tags[i].settings_uwb = discovered_tags[dt].settings_uwb;
                }
            }
        }
    }
    // else use default from pozyx_settings.json

    return true;
}

bool PozyxMessageProcessor::setDroneFilter(float updatePeriod_secs, int movementFreedom, PozyxPayloads::Settings & settings)
{
    settings.drone.options_filter_updatePeriod = updatePeriod_secs;
    settings.drone.options_filter_movementFreedom = movementFreedom;
    return true;
}

bool PozyxMessageProcessor::clearSettings(PozyxPayloads::Settings & settings)
{
    std::string zerostr = std::to_string(0);

    settings.drone.tag_sw = zerostr;
    settings.drone.tag_nw = zerostr;
    settings.drone.tag_ne = zerostr;
    settings.drone.tag_se = zerostr;
    settings.drone.droneWidthmm = 0;
    settings.drone.droneHeightmm = 0;

    for(size_t i=0; i < settings.tags.size(); i++)
    {
        settings.tags[i].id = zerostr;
    }

    for(size_t i=0; i < settings.anchors.size(); i++)
    {
        settings.anchors[i].id = zerostr;
        settings.anchors[i].coordinates.x = 0;
        settings.anchors[i].coordinates.y = 0;
        settings.anchors[i].coordinates.z = 0;
    }

    return true;
}

bool PozyxMessageProcessor::setAnchorsAndTagsToTargetUwb(const PozyxPayloads::Uwb & target_uwb, PozyxPayloads::Settings & settings)
{
    for(size_t i=0; i < settings.tags.size(); i++)
    {
        settings.tags[i].settings_uwb = target_uwb;
    }

    for(size_t i=0; i < settings.anchors.size(); i++)
    {
        settings.anchors[i].settings_uwb = target_uwb;
    }

    return true;
}
