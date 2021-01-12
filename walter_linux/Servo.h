#pragma once
#include <string>
#include "Configuration.h"

class Servo
{
public:
	enum ServoKind
	{
		unitialized = -1,
		hipJoint = 0,
		upperlegJoint = 1,
		lowerlegJoint = 2
	};
private:

	int m_nChannel;
	unsigned int m_nCurrentPulse;
	unsigned int m_nNeutralPulse;
	unsigned int m_nMaxPulse;
	unsigned int m_nMinPulse;
	unsigned int m_nStartPulse;
	double m_fNeutralAngle; //As Degree
    double m_fCurrentAngle; //AS Degree
    double m_fMaxAngle;
    double m_fMinAngle;
	ServoKind m_eKindOfSelf;
    bool m_bTurnDirectionIsFlipped;

public:
	Servo();
	Servo(int nChannel, ServoKind kindOfSelf = ServoKind::unitialized, unsigned int nStartPulse = 0);
	~Servo();

	int GetChannel();
	void SetChannel(int nIndex);

    double GetAngleForPulseWidth(unsigned int pulseWidth);
    unsigned int GetPulseWidthForAngle(double fAngle, bool bAsDegree = true);

    void SetAngle(double fAngle, bool asDegree = true, bool bOverrideBoundaries = false);

	unsigned int GetMaxPulse();
	unsigned int GetNeutralPulse();
	unsigned int GetMinPulse();
	unsigned int GetStartPulse();
	unsigned int GetCurrentPulse();
	unsigned int GetReachablePulse(unsigned int nWantedPulse, bool bOverrideBoundaries = false);

	ServoKind GetKindOfSelf();

    double GetMaxAngle(bool AsDegree = true);
    double GetNeutralAngle(bool AsDegree = true);
    double GetMinAngle(bool AsDegree = true);
    double GetCurrentAngle(bool AsDegree = true);
    double GetReachableAngle(double fWantedAngle, bool bOverrideBoundaries = false);

    void SetPulse(unsigned int nPulse, bool bOverrideBoundaries = false);

    //void SetAngle(double fAngle, bool asDegree = true);

	bool ApplyConfiguration(ConfigurationEntry config);

    std::string GetServoPulseStringWithAngle
    (
        double fAngle, 
        double* fReachableAngle, 
        bool bOverrideBoundaries = false
    );
	std::string GetServoPulseString
	(
		unsigned int nPulseWidth, 
		unsigned int* nReachablePulseWidth = nullptr,
		bool bOverrideBoundaries = false
	);
};