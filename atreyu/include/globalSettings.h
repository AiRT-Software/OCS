#pragma once

#include <memory>
#include <properties.h>
#include <log.h>

namespace airt
{
class GlobalSettings
{
public:
  template <typename T>
  static bool getValue(const std::string &name, T &value)
  {
    auto res = instance().props->getValue(name, value);
    if (!res)
    {
      Log::warn("Undefined property: {}", name);
    }
    return res;
  }

  template <typename T>
  static void setValue(const std::string &name, T &value)
  {
    instance().props->setValue(name, value);
    Log::info("Property {} set to {}", name, value);
  }


  template <typename T>
  static bool getArrayValues(const std::string &name, std::vector<T> &values)
  {
    auto res = instance().props->getArrayValues(name, values);
    if (!res)
    {
      Log::warn("Undefined property: {}", name);
    }
    return res;
  }

	template <typename T>
	static void setArrayValues(const std::string &name, const std::vector<T> &values)
  {
    instance().props->setArrayValues(name, values);
  }


  static void save() {
    instance().props->save();
    Log::info("Properties saved to file {}", iniFilename);
  }

  static void setIniFile(const std::string &path) { iniFilename = path; }

private:
  GlobalSettings();
  static GlobalSettings &instance();
  static std::string iniFilename;
  std::unique_ptr<Properties> props;
};
};
