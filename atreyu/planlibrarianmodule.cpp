#include "globalSettings.h"
#include "planlibrarianmodule.h"

#include <sstream>
#include <log.h>
#include <stdMessage.h>
#include <utils.h>

using airt::Log;
using airt::PlanLibrarianModule;

PlanLibrarianModule::PlanLibrarianModule(const std::string &cmdportname, const std::string &pubportname,
                                         std::shared_ptr<Context> context)
    : Module(cmdportname, pubportname, "", context)
{
    assert(context);

    if (!GlobalSettings::getValue("plan_lib_library_dir", libraryDir))
    {
        libraryDir = "/usr/share/airt/library";
        Log::warn("Using default value plan_lib_library_dir = {}", libraryDir);
    }

    Log::info("Plan librarian module initialized");
}

void PlanLibrarianModule::onMessage(const Message &inmsg)
{
    Message outmsg;
    auto action = airt::getMessageAction(inmsg);
    switch (action)
    {
    case StdMessage::PLAN_LIB_REQUEST_NUMBER_PLANS_IN_DDBB:
    {
        auto ddbb = airt::listFiles(libraryDir, false, [](const std::string &fname) { return airt::ends_with(fname, ".metadata"); });
        NumberOfPlansNotifications n;
        n.numberOfPlans = ddbb.size();
        airt::sendOnePartMessage(*pubsocket, &n);
        break;
    }
    case StdMessage::PLAN_LIB_REQUEST_BASE_NAME:
    {
        auto ddbb = airt::listFiles(libraryDir, false, [](const std::string &fname) { return airt::ends_with(fname, ".metadata"); });
        const RequestPlanBaseName *cmd = inmsg.get<const RequestPlanBaseName *>(0);
        size_t dotidx;
        if (cmd->index >= ddbb.size())
        {
            MissionIndexNotFoundNotification n;
            n.index = cmd->index;
            airt::sendOnePartMessage(*pubsocket, &n);
            return;
        }
        std::string filename = airt::getFilenameFromPath(ddbb[cmd->index]);
        if ((dotidx = filename.find('.')) == std::string::npos)
        {
            MissionIndexNotFoundNotification n;
            n.index = cmd->index;
            airt::sendOnePartMessage(*pubsocket, &n);
            Log::error("This mission file name is not right: {}", ddbb[cmd->index]);
        }
        else
        {
            BaseNameNotification n;
            n.index = cmd->index;
            auto result = filename.substr(0, dotidx);
            airt::sendTwoPartMessage(*pubsocket, &n, result);
        }
        break;
    }
    case StdMessage::PLAN_LIB_REQUEST_LIBRARY_PATH:
    {
        MissionLibraryPathNotification n;
        airt::sendTwoPartMessage(*pubsocket, &n, libraryDir);
        break;
    }
    case StdMessage::PLAN_LIB_REQUEST_THUMBNAIL:
    {
        std::string base = inmsg.get<const char *>(1);
        std::string fullbase = libraryDir + "/" + base;
        auto result = airt::listFiles(libraryDir, false, [fullbase](const std::string &fname) {
            return airt::starts_with(fname, fullbase) && airt::ends_with(fname, ".thumbnail");
        });
        if (result.size() == 0)
        {
            FileNotFound f;
            std::string t = base + ".<ext>.thumbnail";
            airt::sendTwoPartMessage(*pubsocket, &f, t);
            return;
        }
        else
        {
            ThumbnailNotification n;
            airt::sendThreePartMessage(*pubsocket, &n, base, result[0]);
        }
        break;
    }
    case StdMessage::PLAN_LIB_REQUEST_MISSION:
    {
        std::string base = inmsg.get<const char *>(1);
        std::string filename = baseToMissionFullpath(base);
        if (!airt::fileExists(filename))
        {
            FileNotFound f;
            airt::sendTwoPartMessage(*pubsocket, &f, filename);
            return;
        }
        else
        {
            MissionNotification n;
            airt::sendThreePartMessage(*pubsocket, &n, base, filename);
        }
        break;
    }

    case StdMessage::PLAN_LIB_REQUEST_METADATA:
    {
        std::string base = inmsg.get<const char *>(1);
        std::string filename = baseToMetadataFullpath(base);
        if (!airt::fileExists(filename))
        {
            FileNotFound f;
            airt::sendTwoPartMessage(*pubsocket, &f, filename);
            return;
        }
        else
        {
            MetadataNotification n;
            airt::sendThreePartMessage(*pubsocket, &n, base, filename);
        }
        break;
    }

    case StdMessage::PLAN_LIB_REQUEST_MAP:
    {
        std::string base = inmsg.get<const char *>(1);
        std::string fullbase = libraryDir + "/" + base;

        auto result = airt::listFiles(libraryDir, false, [fullbase](const std::string &fname) {
            return airt::starts_with(fname, fullbase) && airt::ends_with(fname, ".map");
        });
        if (result.size() == 0)
        {
            FileNotFound f;
            std::string t = fullbase + ".<ext>.map";
            airt::sendTwoPartMessage(*pubsocket, &f, t);
            return;
        }
        else
        {
            MapNotification n;
            airt::sendThreePartMessage(*pubsocket, &n, base, result[0]);
        }
        break;
    }
    case StdMessage::PLAN_LIB_DELETE_MISSION:
    {
        std::string base =  inmsg.get<const char *>(1);
        std::string fullbase = libraryDir + "/" + base;
        auto result = airt::listFiles(libraryDir, false, [fullbase](const std::string &fname) {
            return airt::starts_with(fname, fullbase);
        });
        auto deletedFiles = result.size();
        for (const auto & f : result) {
            airt::deleteFile(f);
        }
        Log::info("Mission {} deleted ({} files)", base, deletedFiles);
        DeletedMissionNotification n;
        airt::sendTwoPartMessage(*pubsocket, &n, base);
        break;
    }

    default:
        Log::critical("Unhandled message in plan librarian module");
    }
}

std::string PlanLibrarianModule::baseToMetadataFullpath(const std::string &base) const {
    return libraryDir + "/" + base + ".json.metadata";
}

std::string PlanLibrarianModule::baseToMissionFullpath(const std::string &base) const {
    return libraryDir + "/" + base + ".json.mission";
}