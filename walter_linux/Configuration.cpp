#include "Configuration.h"
#include <cstring>
//#include <Windows.h>
#include <fstream>
#include <map>

Configuration::Configuration()
{
	m_configFilePath = std::string();
	m_entries = std::vector<ConfigurationEntry>();
}

Configuration::Configuration(std::vector<ConfigurationEntry> entries)
{
	m_configFilePath = std::string();
	m_entries = entries;
}

Configuration::~Configuration()
{
}

void Configuration::AddEntry(ConfigurationEntry entry)
{
	m_entries.emplace_back(entry);
}

void Configuration::AddEntries(std::vector<ConfigurationEntry> entries)
{
	//TODO: test this...
	m_entries.reserve(m_entries.size() + entries.size());
	memcpy(&m_entries[m_entries.size()], &entries, entries.size());
	//m_entries.emplace_back(entries);
}

std::vector<ConfigurationEntry> Configuration::GetEntries()
{
	return m_entries;
}
