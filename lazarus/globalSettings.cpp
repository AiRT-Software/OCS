#include "globalSettings.h"

using airt::GlobalSettings;
using airt::Properties;

extern std::string iniFilename;

GlobalSettings::GlobalSettings()
{
    props = std::unique_ptr<Properties>(new Properties(iniFilename));
    if (props->empty())
    {
        // No hay fichero de propiedades. Dar valores por defecto
        props->setValue("publisherportname", "tcp://localhost:5555");
        props->setValue("requestportname", "tcp://localhost:5556");
        props->setValue("atreyuexecname", "atreyu");
        props->setValue("atreyuexecdir", "/usr/local/bin");
        props->setValue("atreyuargs", "-i /usr/share/airt/atreyu.ini");
        props->setValue("atreyuworkingdir", "/usr/share/airt");
        props->setValue("timeouttocheckpidms", 1000);
        props->setValue("timeouttokillms", 1000);
        props->save();
    }
}

GlobalSettings &GlobalSettings::instance()
{
    static GlobalSettings globalSettings;
    return globalSettings;
}
