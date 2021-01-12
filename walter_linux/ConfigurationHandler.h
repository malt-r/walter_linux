#pragma once
#include <vector>
#include <map>
#include "Configuration.h"
#include "Servo.h"

class ConfigurationHandler
{
public:
	static Configuration LoadConfiguration(std::string fileName);
	static bool SaveConfiguration(std::string fileName, Configuration config);

	static Configuration GetConfigurationFromServoVector(std::vector<Servo> servos);
	static int ApplyConfigurationToServoVector(std::map<int, Servo*> servoPtrMap, Configuration config);

	static int SetMinPosition(Servo* servoPtr, int minPosition);
	static int SetNeutralPosition(Servo* servoPtr, int neutralPosition);
	static int SetMaxPosition(Servo* servoPtr, int maxPosition);
	static int SetStartPosition(Servo* servoPtr, int startPosition);
};