#pragma once
#include <vector>
#include <string>

struct ConfigurationEntry
{
	unsigned int m_nChannelNumber;
	unsigned int m_nMinPulse;
	unsigned int m_nNeutralPulse;
	unsigned int m_nMaxPulse;
	unsigned int m_nStartPulse;
	ConfigurationEntry
	(
		unsigned int channelNumber,
		unsigned int minPulse,
		unsigned int neutralPulse,
		unsigned int maxPulse,
		unsigned int startPulse
	)
	{
		m_nChannelNumber = channelNumber;
		m_nMinPulse = minPulse;
		m_nNeutralPulse = neutralPulse;
		m_nMaxPulse = maxPulse;
		m_nStartPulse = startPulse;
	}

	ConfigurationEntry()
	{
		m_nChannelNumber = 0;
		m_nMinPulse = 500;
		m_nNeutralPulse = 1500;
		m_nMaxPulse = 2500;
		m_nStartPulse = m_nNeutralPulse;
	}
};


class Configuration
{
private:
	std::string m_configFilePath;
	std::vector<ConfigurationEntry> m_entries;

public:

	Configuration();
	Configuration(std::vector<ConfigurationEntry> entries);
	~Configuration();

	void AddEntry(ConfigurationEntry entry);
	void AddEntries(std::vector<ConfigurationEntry> entries);
	std::vector<ConfigurationEntry> GetEntries();
};