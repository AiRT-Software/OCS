#include <properties.h>
#include <utils.h>

#include "globalSettings.h"

using airt::GlobalSettings;
using airt::Properties;

std::string GlobalSettings::iniFilename = "atreyu.ini";

GlobalSettings::GlobalSettings()
{
    props = std::unique_ptr<Properties>(new Properties(iniFilename));

    if (props->empty())
    {
        props->setValue("loglevel", airt::to_underlying(Log::LogLevel::Debug));
        props->setValue("publisherportname", "tcp://*:5555");
        props->setValue("commandportname", "tcp://*:5556");

        props->setValue("pozyx_portname_base", "inproc://pozyx");
        props->setValue("pozyx_ip", "10.42.0.100");
        props->setValue("pozyx_httpport", 5000);
        props->setValue("pozyx_udpport", 2000);
        props->setValue("pozyx_settings_file", "/usr/share/airt/pozyx_settings.json");
        
        props->setValue("fcs_portname_base", "inproc://fcs");
        props->setValue("fcs_uart_port", "/dev/ttyUSB1");
        props->setValue("fcs_baud_rate", 115200);
        props->setValue("fcs_origin_lon", 5.0);
        props->setValue("fcs_origin_lat", 50.0);


        props->setValue("gimbal_portname_base", "inproc://gimbal");
        props->setValue("gimbal_uart_port", "/dev/ttyUSB2");
        props->setValue("gimbal_baud_rate", 115200);

        props->setValue("rcam_portname_base", "inproc://rcam");
        props->setValue("rcam_uart_port", "/dev/ttyUSB3");
        props->setValue("rcam_baud_rate", 115200);

        props->setValue("plan_lib_library_dir", "/usr/share/airt/library");
        
        props->setValue("fpv_portname_base", "inproc://fpv");
        
        props->setValue("d415cam_portname_base", "inproc://d415cam");
        props->setValue("d415cam_maxpointstosend", 20000);
        props->setValue("d415cam_params_file", "/usr/share/airt/mappingcam_params.txt");
        props->setValue("d415cam_distancemm", 405);
        
        props->setValue("plan_exec_preflight_check_timeout_seconds", 3.0);
        props->setValue("plan_exec_motor_power_to_arm", 20.0);
        props->setValue("plan_exec_bypass_preflight_checks", false);

        props->save();

        std::ofstream fprop(iniFilename, std::ofstream::app);
        fprop << "# pclreplayfilename = \"../data/pointclouds/gazebokitchennormals/cloud_\"\n";
        fprop << "# pclreplay_portname_base = \"inproc://pclreplay\"\n";
        fprop << "# pclreplayfrequency = 3\n";
        fprop << "# pclreplaytotalframes = 50\n";
        fprop << "# zr300portname = inproc://zr300.ipc";
        fprop << "# zr300maxpointstosend = 20000";
    }
}

GlobalSettings &GlobalSettings::instance()
{
    static GlobalSettings globalSettings;
    return globalSettings;
}
