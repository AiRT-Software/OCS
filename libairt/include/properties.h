#pragma once

#include <string>
#include <memory>
#include <cpptoml.h>

namespace airt
{

/**
\class Properties
This class allows storing pairs of key=value in files. Used for storing properties.
By default, when the object goes out of scope, it will write its properties to file. 
If this behavior is not desired, invoke Properties::setAutoSave with false.

Use example:

\code
  Properties pp("airt.ini");

	if (pp.empty())
	{

		std::cout << "There was no airt.ini\n";
		pp.setValue("Hello", "there!");
		pp.setValue("double", 1.234);
		pp.save();
	}
	else
	{
		std::cout << "There was an airt.ini!\n";
		std::string val;
		if (pp.getValue("Hello", val)) {
			std::cout << "Hello=" << val << "\n";
		} else {
			std::cout << "There was no value for the key Hello\n";
		}
	}
	\endcode

  */

class Properties
{
  public:
	/**
	Load the properties stored in the file properties.ini in the current directory.
	*/
	Properties();
	/**
	Destructor
	*/
	~Properties();
	/**
	Loads the properties stored in the given filename. If you modify any property, unless it is saved with
	the save method, it will be lost when the object goes out of scope.
	*/
	explicit Properties(const std::string &filename);
	/**
	Stores the properties to the given file, or the file it was loaded from if no filename is provided
	\param filename file path to store the properties to, or empty for using the same file the properties were 
		loaded from
	*/
	void save(const std::string &filename = "");
	/**
	Request the value related with a given key
	\param name key name
	\param value variable to store the read value in. If the key did not exist, the variable is not written to
	\return true if there was a value associated with the given key, false otherwise
	*/
	template <typename T>
	bool getValue(const std::string &name, T &value)
	{
		auto v = table->get_qualified_as<T>(name);
		if (v)
		{
			value = *v;
			return true;
		}
		else
			return false;
	}

	/**
	Request the array related with a given key
	\param name key name
	\param value variable to store the read values in. If the key did not exist, the variable is not written to
	\return true if there was a value associated with the given key, false otherwise
	\warning Only the following types are allowed! std::string, int64_t, double, bool, local_date, local_time,
                local_datetime, offset_datetime
	*/
	template <typename T>
	bool getArrayValues(const std::string &name, std::vector<T> &values)
	{
		auto v = table->get_array_of<T>(name);
		if (v)
		{
			values = *v;
			return true;
		}
		else
			return false;
	}

	/**
	Sets a value for the given key (an array of values)
	\param name key
	\param value value to store in the property
	\warning If the properties are not saved with the method Properties::save, any change in the object will be lost
	*/
	template <typename T>
	void setArrayValues(const std::string &name, const std::vector<T> &values)
	{
		auto theArray = cpptoml::make_array();
		for (const T &v : values)
		{
			theArray->push_back(v);
		}
		table->insert(name, theArray);
		dirty = true;
	}

	/**
	Sets a value for the given key.
	\param name key
	\param value value to store in the property
	\warning If the properties are not saved with the method Properties::save, any change in the object will be lost
	*/
	template <typename T>
	void setValue(const std::string &name, const T &value)
	{
		table->insert(name, value);
		dirty = true;
	}

	/**
	\return true if there are no properties
	*/
	bool empty() const;
	/**
	If a Properties file has its autosave variable set, it will write its contents to the file on destruction. 
	*/
	void setAutoSave(bool autosave) { this->autosave = autosave; }

	static constexpr const char *DEFAULT_PROPERTIES_FILENAME = "properties.ini";

  private:
	std::string filename;
	Properties(const Properties &) = delete;
	Properties &operator=(const Properties &) = delete;
	void load(const std::string &filename);
	std::shared_ptr<cpptoml::table> table;
	bool dirty, autosave;
};

template <>
inline bool Properties::getValue<float>(const std::string &name, float &value)
{
	double tmp;
	bool result = getValue(name, tmp);
	if (result)
		value = static_cast<float>(tmp);
	return result;
}

template <>
inline void Properties::setValue<float>(const std::string &name, const float &value)
{
	double tmp = value;
	table->insert(name, tmp);
	dirty = true;
}

};
