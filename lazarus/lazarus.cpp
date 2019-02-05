
#include "lazarus.h"
#include "globalSettings.h"

#include <log.h>
#include <stdMessage.h>
#include <utils.h>
#include <wait.h>
#include <linux/reboot.h>
#include <sys/reboot.h>

using airt::GlobalSettings;
using airt::Lazarus;

Lazarus::Lazarus() : context(std::make_shared<Context>()), subscriber(*context, Socket::SocketType::subscribe), poweroff(false)
{
    Log::info("Lazarus starting on directory {}", airt::getCurrentWorkingDir());
    if (airt::fileExists(airt::installUpdatesPathname))
    {
        Log::info("Deleting an update package left from a previous session");
        airt::deleteFile(airt::installUpdatesPathname);
    }
    config();
}

void Lazarus::config()
{
    std::string portname;
    if (GlobalSettings::getValue("publisherportname", portname))
    {
        subscriber.connect(portname);
        auto nots = StdNotification(0).toMsg();
        subscriber.subscribe(nots.get<const uint8_t *>(0), sizeof(StdNotification) - 1);
    }
    else
    {
        Log::critical("Config file does not specify the address of the publisher");
        exit(-1);
    }

    if (GlobalSettings::getValue("requestportname", portname))
    {
        reqSrv = std::unique_ptr<airt::ReliableExternalClient>(new airt::ReliableExternalClient(context, portname, 1000, 3));
    }
    else
    {
        Log::critical("Config file does not specify the address of the request/reply port (requestportname)");
        exit(-1);
    }

    if (!GlobalSettings::getValue("atreyuexecname", atreyuProcessName) ||
        !GlobalSettings::getValue("atreyuexecdir", atreyuExecPath) ||
        !GlobalSettings::getValue("atreyuargs", atreyuArgs))
    {
        Log::critical("Config file does not specify the atreyu executable parameters (atreyuexecname, atreyuexecdir, atreyuargs)");
        exit(-1);
    }
    atreyuExeFullname = atreyuExecPath + "/" + atreyuProcessName;

    if (!airt::fileExists(atreyuExeFullname))
    {
        Log::critical("Could not find atreyu's executable ({})", atreyuExeFullname);
        exit(-1);
    }

    if (!GlobalSettings::getValue("atreyuworkingdir", atreyuWorkingDir))
    {
        Log::info("Variable atreyuworkingdir not set. Using current dir");
    }

    if (!GlobalSettings::getValue("timeouttocheckpidms", timeoutToCheckPID) ||
        !GlobalSettings::getValue("timeouttokillms", timeoutToKill))
    {
        Log::critical("Timeouts not defined in the config file (timeouttocheckpidms, timeouttokillms)");
        exit(-1);
    }

    installUpdatesDir = airt::getDirectory(installUpdatesPathname);
}

void Lazarus::processNotification(const Message &msg)
{
    lastMessageTS = std::chrono::steady_clock::now();
    switch (airt::getMessageAction(msg))
    {
    case StdMessage::STARTED:
        Log::info("The server has started");
        break;
    case StdMessage::STOPPED:
        Log::info("Received the stop command");
        break;
    case StdMessage::HEART_BEAT:
        break;
    case StdMessage::QUITTING:
    {
        Log::info("Received the quit command");
        if (!airt::waitProcessQuitOrKillIt(airt::getProcessPID(atreyuProcessName), 5000, timeoutToKill))
        {
            Log::error("Could not kill atreyu process {}", airt::getProcessPID(atreyuProcessName));
            done = true;
            return;
        }

        if (airt::fileExists(airt::installUpdatesPathname))
        {
            Log::info("Found and update file ({})! Installing...", airt::installUpdatesPathname);
            if (installUpdate())
            {
                Log::info("Update correctly scheduled. Shutting lazarus down...");
                done = true;
            }
            else
            {
                Log::error("Update failed. Restarting atreyu...");
                launchAtreyu();
            }
        }
        else
            done = true;
    }
    break;
    case StdMessage::POWERING_OFF:
        Log::info("Received the poweroff command. Shutting down the machine");
        airt::waitProcessQuitOrKillIt(airt::getProcessPID(atreyuProcessName), 5000, timeoutToKill);
        poweroff = done = true;
        break;
    default:
        Log::warn("Lazarus: Unknown command message {}", airt::to_string(msg));
        break;
    }
}

bool Lazarus::isAtreyuRunning()
{
    return airt::getProcessPID(atreyuProcessName) > 0;
}

void runExecv(const std::string &filepath, const std::vector<std::string> &args)
{
    const char *argArray[args.size() + 2];
    argArray[0] = filepath.c_str();
    for (size_t i = 0; i < args.size(); i++)
    {
        argArray[i + 1] = args[i].c_str();
    }
    argArray[args.size() + 1] = nullptr;

    char *const *ptr = static_cast<char *const *>(static_cast<void *>(argArray));
            ///### REMOVE FROM HERE
            std::ofstream kk("/tmp/lip5.txt");
            kk << getpid() << "\n";
            kk << filepath.c_str() << " ";
            auto pp = ptr;
            for (int i = 0; pp[i] != nullptr; i++) {
                kk << pp[i] << "\n";
            }
            kk.close();
            /// ### TO HERE


    execv(filepath.c_str(), ptr); // Only returns on error
}
//https://stackoverflow.com/questions/17599096/how-to-spawn-child-processes-that-dont-die-with-parent
void Lazarus::launchAtreyu()
{
    int pid;
    int Stat;

    Log::info("Launching atreyu...");

    pid = fork();
    if (pid < 0)
    {
        Log::critical("Fork failed while launching atreyu");
        exit(-2);
    }
    if (pid == 0)
    {             // CHILD
        setsid(); // Make this process the session leader of a new session
        pid = fork();
        if (pid < 0)
        {
            exit(1);
        }
        if (pid == 0)
        { // GRANDCHILD
            if (!atreyuWorkingDir.empty())
            {
                airt::changeCurrentDir(atreyuWorkingDir);
            }
            runExecv(atreyuExeFullname, airt::splitBySpaces(atreyuArgs));
            exit(1);
        }
        exit(0); // SUCCESS (This child is reaped below with waitpid())
    }

    // Reap the child, leaving the grandchild to be inherited by init
    waitpid(pid, &Stat, 0);
    if (WIFEXITED(Stat) && (WEXITSTATUS(Stat) == 0))
    {
        // Wait for atreyu's pid to show up
        bool alive = waitUntil(std::bind(&Lazarus::isAtreyuRunning, this), timeoutToCheckPID);
        if (alive)
            return;
    }
    Log::critical("Failed to spawn orphan launching atreyu. Giving up");
    exit(-2);
}

void Lazarus::run()
{
    Log::info("Lazarus ready");

    if (isAtreyuRunning())
    {
        Log::error("Atreyu was running when Lazarus started. Who launched it?");
    }
    else
    {
        launchAtreyu();
    }

    Log::info("Atreyu up and running");
    done = false;
    int launchAtreyuAttempts = 0;
    // Last time we received something from the server
    lastMessageTS = std::chrono::steady_clock::now();
    while (!done)
    {
        Message msg;
        if (subscriber.receive(msg, true))
        {
            if (airt::isStdNotification(msg))
                processNotification(msg);
            else
                Log::error("Unknown message received in subscription: " + to_string(msg));
        }
        else
        {
            auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now() - lastMessageTS);
            if (elapsed.count() > timeoutToCheckPID)
            {
                Log::warn("Connection to the server lost for {} ms", elapsed.count());
                auto apid = airt::getProcessPID(atreyuProcessName);
                if (apid <= 0)
                {
                    Log::error("Atreyu's gone. Restarting it...");
                    launchAtreyu();
                    launchAtreyuAttempts++;
                    if (launchAtreyuAttempts > 10) {
                        Log::critical("Couldn't launch atreyu. Giving up...");
                        done = true;
                    }
                } 
                lastMessageTS = std::chrono::steady_clock::now();
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(50));
        }
    }
    Log::info("Lazarus shuting down");
    if (poweroff)
    {
        Log::info("Powering the machine off");
        sync();
        reboot(LINUX_REBOOT_CMD_POWER_OFF);
    }
}

// dpkg -i /usr/share/airt/airt.deb > /usr/share/airt/install.log 2>&1 ; rm -f /usr/share/airt/airt.deb
bool Lazarus::installUpdate()
{
    std::ostringstream os;

    // Install the package

    os << "dpkg -i " << airt::installUpdatesPathname;
    os << " > " << installUpdatesDir << "install.log 2>&1";

    auto cmdline = os.str();
    auto logfile = installUpdatesDir + "install.log";

    Log::info("Scheduling an update of atreyu. Logging to {}", logfile);

    auto res = airt::launchIndependentProcess(
        [cmdline, logfile]() {
            std::this_thread::sleep_for(std::chrono::milliseconds(2000));
            int fd = open(logfile.c_str(), O_RDWR | O_CREAT, S_IRUSR | S_IWUSR);
            dup2(fd, 1); // make stdout go to file
            dup2(fd, 2); // make stderr go to file
            close(fd);
            runExecv("/usr/bin/dpkg", {"-i", airt::installUpdatesPathname});
        });

    if (res != 0)
    {
        Log::critical("Failed to launch the update process. Error code: {}", res);
        return false;
    }
    else
        return true;
}
