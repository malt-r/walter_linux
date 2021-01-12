#include "ConfigurationHandler.h"
#include <fstream>
#include <map>
#include <iostream>

Configuration ConfigurationHandler::LoadConfiguration(std::string fileName)
{
	Configuration retConfig;

	std::ifstream fileInput;
	fileInput.open(fileName, std::ifstream::in);

	std::vector<ConfigurationEntry> entriesFromConfig;
	unsigned int nChannelNumberFromFile;
	unsigned int nMinPosFromFile;
	unsigned int nNeutralPosFromFile;
	unsigned int nMaxPosFromFile;
	unsigned int nStartPosFromFile;
	char comma;
	int peeked = 0;
	while (-1 != peeked)
	{
		fileInput >> nChannelNumberFromFile >> comma;
		fileInput >> nMinPosFromFile >> comma;
		fileInput >> nNeutralPosFromFile >> comma;
		fileInput >> nMaxPosFromFile >> comma;
		fileInput >> nStartPosFromFile >> comma;
		peeked = fileInput.peek();
		entriesFromConfig.emplace_back
		(
			ConfigurationEntry
			{
				nChannelNumberFromFile,
				nMinPosFromFile,
				nNeutralPosFromFile,
				nMaxPosFromFile,
				nStartPosFromFile
			}
		);
	}
	fileInput.close();

	retConfig = Configuration(entriesFromConfig);
	return retConfig;
}

bool ConfigurationHandler::SaveConfiguration(std::string fileName, Configuration config)
{
	bool bRet = true;
	std::ofstream outdata;
	outdata.open(fileName, std::ofstream::trunc);
	if (!outdata.is_open())
	{
		bRet = false;
	}
	else
	{
		//servo data
		for (int i = 0; i < config.GetEntries().size(); i++)
		{
			if (i != 0)
			{
				outdata << std::endl;
			}
			ConfigurationEntry currEntry = config.GetEntries()[i];
			outdata << currEntry.m_nChannelNumber << ",";
			outdata << currEntry.m_nMinPulse << ",";
			outdata << currEntry.m_nNeutralPulse << ",";
			outdata << currEntry.m_nMaxPulse << ",";
			outdata << currEntry.m_nStartPulse << ",";
		}
		outdata.close();
	}
	return bRet;
}

Configuration ConfigurationHandler::GetConfigurationFromServoVector
(
	std::vector<Servo> servos
)
{
	Configuration configToReturn = Configuration();

	ConfigurationEntry configEntryToAdd;
	for (Servo servo : servos)
	{
		configEntryToAdd
			= ConfigurationEntry
			(
				servo.GetChannel(),
				servo.GetMinPulse(),
				servo.GetNeutralPulse(),
				servo.GetMaxPulse(),
				servo.GetStartPulse()
			);
		configToReturn.AddEntry(configEntryToAdd);
	}
	return configToReturn;
}

int ConfigurationHandler::ApplyConfigurationToServoVector
(
	std::map<int, Servo*> servoPtrMap, 
	Configuration config
)
{
	int returnVal = 0;
	//construct configuration-map
	auto configMap = std::map<int, ConfigurationEntry>();
	for (ConfigurationEntry entry : config.GetEntries())
	{
		bool noEntryWithChannelNumberInMap
			= configMap.find(entry.m_nChannelNumber) == configMap.end();
		if (noEntryWithChannelNumberInMap)
		{
			configMap.emplace
			(
				std::pair<int, ConfigurationEntry>
				(
					entry.m_nChannelNumber,
					entry
					)
			);
		}
	}

	////construct servo-map
	//auto servoMap = std::map<int, Servo*>();
	//for (Servo* servoPtr : servoPtrs)
	//{
	//	bool noServoWithChannelNumberInMap
	//		= servoMap.find(servoPtr->GetChannel()) == servoMap.end();
	//	if (noServoWithChannelNumberInMap)
	//	{
	//		servoMap.emplace
	//		(
	//			std::pair<int, Servo*>
	//			(
	//				servoPtr->GetChannel(),
	//				servoPtr
	//				)
	//		);
	//	}
	//}

	int nNumChannels = 32;
	for (int nChannel = 0; nChannel < nNumChannels; nChannel++)
	{
		auto configIterator = configMap.find(nChannel);
		auto servoIterator = servoPtrMap.find(nChannel);

		if (configMap.end() != configIterator &&
			servoPtrMap.end() != servoIterator)
		{
			Servo* servoPtr = servoIterator->second;
			ConfigurationEntry configEntry = configIterator->second;
			servoPtr->ApplyConfiguration(configEntry);
		}
	}
	return returnVal;
}

int ConfigurationHandler::SetMinPosition(Servo* servoPtr, int minPulseToSet)
{
	int maxPulse = servoPtr->GetMaxPulse();
	if (minPulseToSet > maxPulse)
	{
		std::cout << "not set new minpulse, pulse " << minPulseToSet;
		std::cout << "is bigger than maxpulse " << maxPulse << std::endl;
	}
	else
	{
		int neutralPulse = servoPtr->GetNeutralPulse();
		int startPulse = servoPtr->GetStartPulse();
		ConfigurationEntry tempEntry
			= ConfigurationEntry
			(
				servoPtr->GetChannel(),
				minPulseToSet,
				neutralPulse,
				maxPulse,
				startPulse
			);
		servoPtr->ApplyConfiguration(tempEntry);
	}
	return 0;
}

int ConfigurationHandler::SetNeutralPosition(Servo* servoPtr, int neutralPulseToSet)
{
	int minPulse = servoPtr->GetMinPulse();
	int maxPulse = servoPtr->GetMaxPulse();
	if (neutralPulseToSet > maxPulse)
	{
		std::cout << "not set new neutral pulse, pulse " << neutralPulseToSet;
		std::cout << "is bigger than maxpulse " << maxPulse << std::endl;
	}
	else if (neutralPulseToSet < minPulse)
	{
		std::cout << "not set new neutral pulse, pulse " << neutralPulseToSet;
		std::cout << "is smaller than minpulse" << minPulse << std::endl;
	}
	else
	{
		int startPulse = servoPtr->GetStartPulse();
		ConfigurationEntry tempEntry
			= ConfigurationEntry
			(
				servoPtr->GetChannel(),
				minPulse,
				neutralPulseToSet,
				maxPulse,
				startPulse
			);
		servoPtr->ApplyConfiguration(tempEntry);
	}
	return 0;
}

int ConfigurationHandler::SetMaxPosition(Servo* servoPtr, int maxPulseToSet)
{
	int minPulse = servoPtr->GetMinPulse();
	if (maxPulseToSet < minPulse)
	{
		std::cout << "not set new maxpulse, pulse " << maxPulseToSet;
		std::cout << "is bigger than minpulse " << minPulse << std::endl;
	}
	else
	{
		int neutralPulse = servoPtr->GetNeutralPulse();
		int startPulse = servoPtr->GetStartPulse();
		ConfigurationEntry tempEntry
			= ConfigurationEntry
			(
				servoPtr->GetChannel(),
				minPulse,
				neutralPulse,
				maxPulseToSet,
				startPulse
			);
		servoPtr->ApplyConfiguration(tempEntry);
	}
	return 0;
}

int ConfigurationHandler::SetStartPosition(Servo* servoPtr, int startPulseToSet)
{
	int minPulse = servoPtr->GetMinPulse();
	int maxPulse = servoPtr->GetMaxPulse();
	if (startPulseToSet > maxPulse)
	{
		std::cout << "not set new start pulse, pulse " << startPulseToSet;
		std::cout << "is bigger than maxpulse " << maxPulse << std::endl;
	}
	else if (startPulseToSet < minPulse)
	{
		std::cout << "not set new start pulse, pulse " << startPulseToSet;
		std::cout << "is smaller than minpulse" << minPulse << std::endl;
	}
	else
	{
		int neutralPulse = servoPtr->GetNeutralPulse();
		ConfigurationEntry tempEntry
			= ConfigurationEntry
			(
				servoPtr->GetChannel(),
				minPulse,
				neutralPulse,
				maxPulse,
				startPulseToSet
			);
		servoPtr->ApplyConfiguration(tempEntry);
	}
	return 0;
}
