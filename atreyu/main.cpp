
/*

AiRT Software
http://www.airt.eu


This project has received funding from the European Union's Horizon 2020 research
and innovation programme under grant agreement No 732433.

Copyright Universitat Politecnica de Valencia 2017-2018. 
All rights reserved.

Instituto de Automática e Informática Industrial (ai2)
http://www.ai2.upv.es

Contact: Paco Abad (fjabad@dsic.upv.es)

*/


#include <log.h>

#include "server.h"
#include "globalSettings.h"

using namespace airt;

int main(int argc, char **argv)
{
    Log::setFile("atreyu.log");
    
    // Remove for final release
    Log::setOutputToConsole(true);
    // 

    for (int i = 1; i < argc; i++)
    {
        std::string opt(argv[i]);
        if (opt == "-l" && (i + 1 < argc))
        {
            Log::setFile(argv[++i]);
        }
        else if (opt == "-i" && (i + 1 < argc))
        {
            GlobalSettings::setIniFile(argv[++i]);
        }
        else if (opt == "-h" || opt == "--help") 
        {
            std::cerr << argv[0] << " [-l logfile] [-i initfile] [-h]\n";
            return -2;
        }
    }

    Server server;
    server.run();
    return 0;
}
