
#include <utils.h>
#include <log.h>

#include "globalSettings.h"
#include "lazarus.h"

using airt::Lazarus;

std::string iniFilename = "lazarus.ini";

int main(int argc, char **argv) {
    if (argc > 1) {
        iniFilename = argv[1];
    }

    airt::demonize();
    airt::Log::setFile("lazarus.log");
    Lazarus lazarus;
    lazarus.run();
}