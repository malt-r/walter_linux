#include "Servo.h"
//#include <Windows.h>
#include <iostream>
#include <string>
#include "ServoConfigurationDefaults.h"
#include "MathEngine.h"

Servo::Servo()
{
	m_nChannel = -1;
	m_nCurrentPulse = 1500;
	m_nNeutralPulse = 1500;
	m_nMaxPulse = 3000;
	m_nMinPulse = 10;
	m_nStartPulse = 1500;
	m_eKindOfSelf = unitialized;
    m_fNeutralAngle = GetAngleForPulseWidth(m_nNeutralPulse);
    m_fMaxAngle = GetAngleForPulseWidth(m_nMaxPulse);
    m_fMinAngle = GetAngleForPulseWidth(m_nMinPulse);
    m_bTurnDirectionIsFlipped = false;
}

//Servo::Servo(int nIndex, unsigned int nNeutralAnglePulse)
//{
//    //TODO: remove this constructor, is not needed
//	m_nChannel = nIndex;
//	m_nCurrentPulse = nNeutralAnglePulse;
//	m_nNeutralPulse = nNeutralAnglePulse;
//	m_nMaxPulse = 3000;
//	m_nMinPulse = 10;
//	m_nStartPulse = 1500;
//	m_eKindOfSelf = unitialized;
//    m_bTurnDirectionIsFlipped = false;
//
//    m_fNeutralAngle = GetAngleForPulseWidth(m_nNeutralPulse);
//    m_fMaxAngle = GetAngleForPulseWidth(m_nMaxPulse);
//    m_fMinAngle = GetAngleForPulseWidth(m_nMinPulse);
//}

Servo::Servo(int nIndex, ServoKind kindOfSelf, unsigned int nStartPulse)
{
	m_nChannel = nIndex;
	m_eKindOfSelf = kindOfSelf;
	m_nStartPulse = nStartPulse;
	//set neutral position depending on kindOfSelf
	if (ServoKind::unitialized == kindOfSelf)
	{
		m_nNeutralPulse = 1500;
		m_nMaxPulse = 2000;
		m_nMinPulse = 1000;
        m_fNeutralAngle = 0;
        m_bTurnDirectionIsFlipped = false;
	}
	else if (ServoKind::hipJoint == kindOfSelf)
	{
		m_nNeutralPulse = anNeutralPulses[ServoKind::hipJoint];
		m_nMaxPulse = anMaxPulses[ServoKind::hipJoint];
		m_nMinPulse = anMinPulses[ServoKind::hipJoint];
        m_fNeutralAngle = afNeutralAngles[ServoKind::hipJoint];
        m_bTurnDirectionIsFlipped = true;
	}
	else if (ServoKind::upperlegJoint == kindOfSelf)
	{
		m_nNeutralPulse = anNeutralPulses[ServoKind::upperlegJoint];
		m_nMaxPulse = anMaxPulses[ServoKind::upperlegJoint];
		m_nMinPulse = anMinPulses[ServoKind::upperlegJoint];
        m_fNeutralAngle = afNeutralAngles[ServoKind::upperlegJoint];
        m_bTurnDirectionIsFlipped = false;
	}
	else if (ServoKind::lowerlegJoint == kindOfSelf)
	{
		m_nNeutralPulse = anNeutralPulses[ServoKind::lowerlegJoint];
		m_nMaxPulse = anMaxPulses[ServoKind::lowerlegJoint];
		m_nMinPulse = anMinPulses[ServoKind::lowerlegJoint];
        m_fNeutralAngle = afNeutralAngles[ServoKind::lowerlegJoint];
        m_bTurnDirectionIsFlipped = true;
	}

    if (!m_bTurnDirectionIsFlipped)
    {
        m_fMaxAngle = GetAngleForPulseWidth(m_nMaxPulse);
        m_fMinAngle = GetAngleForPulseWidth(m_nMinPulse);
    }
    else
    {
        m_fMaxAngle = GetAngleForPulseWidth(m_nMinPulse);
        m_fMinAngle = GetAngleForPulseWidth(m_nMaxPulse);
    }
}

int Servo::GetChannel()
{
	return m_nChannel;
}


void Servo::SetChannel(int nIndex)
{
	m_nChannel = nIndex;
}


unsigned int Servo::GetCurrentPulse()
{
	return m_nCurrentPulse;
}

double Servo::GetCurrentAngle(bool AsDegree)
{
    double retVal = 0.0;

    if (AsDegree)
    {
        retVal = m_fCurrentAngle;
    }
    else
    {
        retVal = MathEngine::FromDegToRad(m_fCurrentAngle);
    }

    return retVal;
}

void Servo::SetPulse(unsigned int nPulse, bool bOverrideBoundaries)
{
    if (bOverrideBoundaries)
    {
        m_nCurrentPulse = nPulse;
    }
    else
    {
        if (nPulse < m_nMinPulse)
        {
            m_nCurrentPulse = m_nMinPulse;
        }
        else if (nPulse > m_nMaxPulse)
        {
            m_nCurrentPulse = m_nMaxPulse;
        }
        else
        {
            m_nCurrentPulse = nPulse;
        }
    }
    SetAngle(GetAngleForPulseWidth(m_nCurrentPulse));
}

//not sure, if this works
double Servo::GetAngleForPulseWidth(unsigned int pulseWidth)
{
    double fAngle = 0;
    int nDifferencePulse = pulseWidth - m_nNeutralPulse;
    double fDifferenceAngle = nDifferencePulse * servoConst_fAnglePerPulse;
    if (m_bTurnDirectionIsFlipped)
    {
        fAngle = m_fNeutralAngle - fDifferenceAngle;
    }
    else
    {
        fAngle = m_fNeutralAngle + fDifferenceAngle;
    }
    return fAngle;
}

unsigned int Servo::GetPulseWidthForAngle(double fAngle, bool bAsDegree)
{
    unsigned int nPulseWidth = 1500;

    if (!bAsDegree)
    {
        fAngle = MathEngine::FromRadToDeg(fAngle);
    }

    double fDifferenceAngle = fAngle - m_fNeutralAngle;
    int nDifferencePulses = fDifferenceAngle / servoConst_fAnglePerPulse;

    //not sure about that
    if (m_bTurnDirectionIsFlipped)
    {
        nPulseWidth = m_nNeutralPulse - nDifferencePulses;
    }
    else
    {
        //flip direction of calculation for lowerLegJoint
        nPulseWidth = m_nNeutralPulse + nDifferencePulses;
    }

    return nPulseWidth;
}

void Servo::SetAngle(double fAngle, bool asDegree, bool bOverrideBoundaries)
{
    if (!isnan(fAngle))
    {
        //TODO: add boundaries -> calculate from max- min pulses...
        double tempAngle = 0.0;
        if (asDegree)
        {
            tempAngle = fAngle;
        }
        else
        {
            tempAngle = MathEngine::FromRadToDeg(fAngle);
        }

        if (bOverrideBoundaries)
        {
            m_fCurrentAngle = tempAngle;
        }
        else
        {
            if (tempAngle < m_fMinAngle)
            {
                m_fCurrentAngle = m_fMinAngle;
            }
            else if (tempAngle > m_fMaxAngle)
            {
                m_fCurrentAngle = m_fMaxAngle;
            }
            else
            {
                m_fCurrentAngle = tempAngle;
            }
        }
        if (isnan(m_fCurrentAngle))
        {
            bool debug = true;
        }
        //TODO: figure out which part of the pulseWidth-Handling is obsolete..
        m_nCurrentPulse = GetPulseWidthForAngle(m_fCurrentAngle);
    }
}

unsigned int Servo::GetMaxPulse()
{
	return m_nMaxPulse;
}

unsigned int Servo::GetMinPulse()
{
	return m_nMinPulse;
}

unsigned int Servo::GetStartPulse()
{
	return m_nStartPulse;
}

Servo::ServoKind Servo::GetKindOfSelf()
{
	return m_eKindOfSelf;
}

unsigned int Servo::GetNeutralPulse()
{
	return m_nNeutralPulse;
}

double Servo::GetNeutralAngle(bool AsDegree)
{
    if (AsDegree)
    {
        return m_fNeutralAngle;
    }
    else
    {
        double retVal = MathEngine::FromDegToRad(m_fNeutralAngle);
        return retVal;
    }
}

double Servo::GetMaxAngle(bool AsDegree)
{
    if (AsDegree)
    {
        return m_fMaxAngle;
    }
    else
    {
        double retVal = MathEngine::FromDegToRad(m_fMaxAngle);
        return retVal;
    }
}

double Servo::GetMinAngle(bool AsDegree)
{
    if (AsDegree)
    {
        return m_fMinAngle;
    }
    else
    {
        double retVal = MathEngine::FromDegToRad(m_fMinAngle);
        return retVal;
    }
}

unsigned int Servo::GetReachablePulse(unsigned int nWantedPulse, bool bOverrideBoundaries)
{
	unsigned int nReachablePulse = nWantedPulse;
	if (!bOverrideBoundaries)
	{
		if (nWantedPulse < m_nMinPulse)
		{
			std::cout << "nPulse was out of range: " << nWantedPulse << std::endl;
			std::cout << "Moving to minPulse " << m_nMinPulse << std::endl;
			nReachablePulse = m_nMinPulse;
		}
		else if (nWantedPulse > m_nMaxPulse)
		{
			std::cout << "nPulse was out of range: " << nWantedPulse << std::endl;
			std::cout << "Moving to maxPulse " << m_nMaxPulse << std::endl;
			nReachablePulse = m_nMaxPulse;
		}
	}
	return nReachablePulse;
}

double Servo::GetReachableAngle(double fWantedAngle, bool bOverrideBoundaries)
{
    double fReachableAngle = fWantedAngle;
    if (!bOverrideBoundaries)
    {
        if (fWantedAngle < m_fMinAngle)
        {
            std::cout << "nPulse was out of range: " << fWantedAngle << std::endl;
            std::cout << "Moving to minPulse " << m_fMinAngle << std::endl;
            fReachableAngle = m_fMinAngle;
        }
        else if (fWantedAngle > m_fMaxAngle)
        {
            std::cout << "nPulse was out of range: " << fWantedAngle << std::endl;
            std::cout << "Moving to maxPulse " << m_fMaxAngle << std::endl;
            fReachableAngle = m_fMaxAngle;
        }
    }
    return fReachableAngle;
}

bool Servo::ApplyConfiguration(ConfigurationEntry config)
{
	m_nMinPulse = config.m_nMinPulse;
	m_nNeutralPulse = config.m_nNeutralPulse;
	m_nMaxPulse = config.m_nMaxPulse;
	m_nStartPulse = config.m_nStartPulse;
	return false;
}

std::string Servo::GetServoPulseStringWithAngle
(
    double fAngle,
    double* fReachableAngle,
    bool bOverrideBoundaries
)
{
    double localReachableAngle = GetReachableAngle(fAngle, bOverrideBoundaries);
    if (nullptr != fReachableAngle)
    {
        *fReachableAngle = localReachableAngle;
    }
    unsigned int nPulse = GetPulseWidthForAngle(fAngle);
    
    //generate serial command
        //format: # <ch> P <pw> ?S??<spd_opt>??T? <time_opt> <cr>
    std::string sCommand("#");
    sCommand.append(std::to_string(m_nChannel));

    //take in account for maximum pulsewidths
    sCommand.append("P");
    sCommand.append(std::to_string(nPulse));
    return sCommand;
}

std::string Servo::GetServoPulseString
(
	unsigned int nPulseWidth, 
	unsigned int* nReachablePulseWidth,
	bool config
)
{
	//generate serial command
		//format: # <ch> P <pw> ?S??<spd_opt>??T? <time_opt> <cr>
	std::string sCommand("#");
	sCommand.append(std::to_string(m_nChannel));

	//take in account for maximum pulsewidths
	sCommand.append("P");
	unsigned int nLocalReachablePulseWidth 
		= GetReachablePulse(nPulseWidth, config);
	if (nReachablePulseWidth != nullptr)
	{
		*nReachablePulseWidth = nLocalReachablePulseWidth;
	}
	sCommand.append(std::to_string(nPulseWidth));
	return sCommand;
}

Servo::~Servo()
{
}

