
#include "properties.h"
#include "log.h"
#include "utils.h"

using airt::Properties;

Properties::Properties() : Properties(DEFAULT_PROPERTIES_FILENAME)
{
}

Properties::Properties(const std::string &filename) : dirty(false), autosave(true)
{
	load(filename);
}

Properties::~Properties()
{
	if (dirty && autosave)
		save();
}

void Properties::load(const std::string &filename)
{
	// Even if we fail to read the file, we store its pathname in case the user wants to create it later
	this->filename = filename;
	if (!airt::fileExists(filename)) {
		Log::error("The properties file {} does not exists", filename);
		table = cpptoml::make_table();
		return;
	}
	try
	{
		table = cpptoml::parse_file(filename);
		Log::info("Config file {} loaded", filename);
	}
	catch (const cpptoml::parse_exception &e)
	{
		// Create an empty table
		table = cpptoml::make_table();
		Log::error("Failed to parse {}: {}", filename, e.what());
	}
}

void Properties::save(const std::string &filename)
{
	std::string actualFilename = filename;
	if (filename.empty())
		actualFilename = this->filename;

	std::ofstream f(actualFilename);
	if (!f)
	{
		Log::error("Error writing the config file {}", actualFilename);
		return;
	}
	f << *table;
	dirty = false;
}

bool Properties::empty() const
{
	return table->empty();
}
