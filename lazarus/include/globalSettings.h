#pragma once

#include <memory>
#include <properties.h>

namespace airt
{

class GlobalSettings
{
  public:
    template <typename T>
    static bool getValue(const std::string &name, T &value)
    {
        return instance().props->getValue(name, value);
    }

    static Properties &getProperties() { return *(instance().props);}

  private:
    GlobalSettings();
    static GlobalSettings &instance();
    std::unique_ptr<Properties> props;
};
};
